/*
 * The realme 8i's Mali-G57 MC2 before any driver: what LK leaves (boot-arg mp_gpuprobe=1, link `gpu probe`) and the GPU
 * powered as LK left its supply (mp_gpuprobe=2, link `gpu power`): MFG0-3 on, mfgcfg set up, GPU_ID and SHADER_PRESENT
 * read, then its interrupts masked and the domains powered down again (MFGPLL and the rails stay as LK left them).
 * Never a PMIC write: VGPU and VSRAM_GPU stay at LK's levels, MFGPLL at LK's 300 MHz. Anything not as expected
 * refuses before the first write.
 *
 * Registers and order from MediaTek's mt6781 clock/power-domain sources and the realme 8i device tree (read for
 * addresses and order only) and from ios-viva's VivaGPU.cpp.
 */
#include <stdint.h>
#include <string.h>
#include <kern/clock.h>
#include <kern/misc_protos.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

#if DEVELOPMENT || DEBUG        /* a bring-up tool: not in a release kernel */
extern int mtk_pwrap_read(uint32_t addr, uint32_t *value);                          /* MTKPwrap.cpp */
extern void delay(int usec);

#define SPM             0x10006000ull
#define SPM_KEY_REG     0x000
#define SPM_KEY         0x0b160001u
#define PWR_STATUS      0x16c
#define PWR_STATUS_2ND  0x170
#define MFG_PWR_CON(n)  (0x308 + 4 * (n))
#define MFG_BIT(n)      (1u << (2 + (n)))
#define PWR_RST_B       (1u << 0)
#define PWR_ISO         (1u << 1)
#define PWR_ON          (1u << 2)
#define PWR_ON_2ND      (1u << 3)
#define PWR_CLK_DIS     (1u << 4)
#define PWR_SRAM_PDN    (1u << 8)
#define PWR_SRAM_ACK    (1u << 12)

#define TOPCK_CLK_CFG_0 0x10000040ull           /* mfg_sel bits 17:16 (1 = mfgpll), bit 23 clock off */
#define MFGPLL_CON0     0x1000c250ull           /* bit 0 enable */
#define MFGPLL_CON1     0x1000c254ull           /* posdiv 26:24, PCW 21:0 */
#define MFGPLL_PWR_CON0 0x1000c25cull           /* bit 0 power on, bit 1 isolation */
#define MFGPLL_300MHZ   0x031713b1u

#define INFRA           0x10001000ull
#define PROT_SET        0x2a0
#define PROT_CLR        0x2a4
#define PROT_STA1       0x228
#define PROT1_SET       0x2a8
#define PROT1_CLR       0x2ac
#define PROT1_STA1      0x258
#define ACP_CON         0x290

#define MFGCFG          0x13fbf000ull
#define MALI            0x13000000ull
#define GPU_ID          0x000
#define GPU_IRQ_RAWSTAT 0x020
#define GPU_IRQ_CLEAR   0x024
#define GPU_IRQ_MASK    0x028
#define JOB_IRQ_RAWSTAT 0x1000
#define JOB_IRQ_CLEAR   0x1004
#define JOB_IRQ_MASK    0x1008
#define MMU_IRQ_RAWSTAT 0x2000
#define MMU_IRQ_CLEAR   0x2004
#define MMU_IRQ_MASK    0x2008
#define SHADER_PRESENT  0x100
#define GPU_ID_G57_MC2  0x90930010u             /* Android's kbase: "arch 9.0.9 r0p1" */
#define SHADERS_SPACED  0x5u                    /* cores 0 and 2 (Android: shader_present=5) */

/* VGPU and VSRAM_GPU through the PMIC wrapper: enable, enable as applied, voltage as applied (DA) */
#define VGPU_EN         0x1508
#define VGPU_DA_EN      0x1520
#define VGPU_DA_VOSEL   0x151e                  /* 6:0, 500 mV + 6.25 mV steps */
#define VSRAMG_EN       0x1bc8
#define VSRAMG_DA_EN    0x1be4
#define VSRAMG_DA_VOSEL 0x1be2                  /* 14:8 */
#define VGPU_LK         0x40                    /* 900 mV */
#define VSRAMG_LK       0x50                    /* 1000 mV */

static inline uint32_t rd(uint64_t pa) { return ml_phys_read_word_64(pa); }
static inline void wr(uint64_t pa, uint32_t v) { ml_phys_write_word_64(pa, v); }

static int gProbe = -1;                         /* mp_gpuprobe=, read once */

/* the reply so far is *p; a line that does not fit is cut, never written past end */
#define SAY(...) do { \
	int n_ = snprintf(*p, (size_t)(end - *p), __VA_ARGS__); \
	*p += n_ < 0 ? 0 : (n_ < end - *p ? n_ : end - *p - 1); \
} while (0)

static bool
poll_ms(uint64_t pa, uint32_t mask, uint32_t want)
{
	for (int i = 0; i < 1000; i++) {
		if ((rd(pa) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return false;
}

/* Step 1. true when everything is as LK should leave it for step 2: rails on at LK's levels, MFGPLL on at 300 MHz
 * and selected, nothing of the GPU powered. */
static bool
gpu_probe(char **p, char *end, bool *mfgOn)
{
	uint32_t ven = 0, vden = 0, vsel = 0, sen = 0, sden = 0, ssel = 0;
	bool pmic = mtk_pwrap_read(VGPU_EN, &ven) == 0 && mtk_pwrap_read(VGPU_DA_EN, &vden) == 0 &&
	    mtk_pwrap_read(VGPU_DA_VOSEL, &vsel) == 0 && mtk_pwrap_read(VSRAMG_EN, &sen) == 0 &&
	    mtk_pwrap_read(VSRAMG_DA_EN, &sden) == 0 && mtk_pwrap_read(VSRAMG_DA_VOSEL, &ssel) == 0;
	uint32_t v = vsel & 0x7f, s = (ssel >> 8) & 0x7f;
	uint32_t ck = rd(TOPCK_CLK_CFG_0), p0 = rd(MFGPLL_CON0), p1 = rd(MFGPLL_CON1), pp = rd(MFGPLL_PWR_CON0);
	uint32_t st = rd(SPM + PWR_STATUS), st2 = rd(SPM + PWR_STATUS_2ND);
	bool rails = pmic && (ven & 1) && (vden & 1) && (sen & 1) && (sden & 1) && v == VGPU_LK && s == VSRAMG_LK;
	bool pll = (p0 & 1) && (p1 & 0x07ffffff) == MFGPLL_300MHZ && (pp & 3) == 1 && ((ck >> 16) & 3) == 1 &&
	    !(ck & (1u << 23));
	uint32_t on = 0, on2 = 0;
	for (int n = 0; n < 4; n++) {
		on |= (st & MFG_BIT(n)) ? 1u << n : 0;
		on2 |= (st2 & MFG_BIT(n)) ? 1u << n : 0;
	}
	*mfgOn = on == 0xf && on2 == 0xf;
	SAY(
	    "gpu probe (read only):\n"
	    "  VGPU %s, %u mV (vsel 0x%02x, LK's 0x%02x); VSRAM_GPU %s, %u mV (vsel 0x%02x, LK's 0x%02x): %s\n"
	    "  MFGPLL con0 0x%08x con1 0x%08x pwr 0x%08x, mfg_sel %u%s: %s\n"
	    "  MFG0-3 power status 0x%x / 0x%x (pwr_con 0x%08x 0x%08x 0x%08x 0x%08x)\n"
	    "  infra bus protection 0x%08x / 0x%08x, ACP 0x%08x\n",
	    (ven & vden & 1) ? "on" : "OFF", 500 + v * 25 / 4, v, VGPU_LK, (sen & sden & 1) ? "on" : "OFF",
	    500 + s * 25 / 4, s, VSRAMG_LK, !pmic ? "PMIC not readable" : rails ? "as LK leaves them" : "NOT as expected",
	    p0, p1, pp, (ck >> 16) & 3, (ck & (1u << 23)) ? " (off)" : "", pll ? "300 MHz, selected" : "NOT as expected",
	    on, on2, rd(SPM + MFG_PWR_CON(0)), rd(SPM + MFG_PWR_CON(1)), rd(SPM + MFG_PWR_CON(2)),
	    rd(SPM + MFG_PWR_CON(3)), rd(INFRA + PROT_STA1), rd(INFRA + PROT1_STA1), rd(INFRA + ACP_CON));
	if (*mfgOn) {
		SAY("  Mali GPU_ID 0x%08x, SHADER_PRESENT 0x%x\n", rd(MALI + GPU_ID),
		    rd(MALI + SHADER_PRESENT));
	}
	return rails && pll && (on == 0 || *mfgOn) && on == on2;
}

/* MFG1's bus protection steps, set in this order before it powers off (released in reverse when it powers on) */
static const struct { uint32_t set, sta, mask; } gProt[4] = {
	{ PROT1_SET, PROT1_STA1, 0x18000000 }, { PROT_SET, PROT_STA1, 0x00600000 },
	{ PROT_SET, PROT_STA1, 0x02000000 }, { PROT1_SET, PROT1_STA1, 0x20000000 },
};

/* The domains off again, MFG3 to MFG0, as MappleGPU's mfgOff without its PLL and rail steps. false: something was left on. */
static bool
gpu_down(char **p, char *end)
{
	wr(SPM + SPM_KEY_REG, SPM_KEY);
	wr(MFGCFG + 0x0b4, 1);                                          /* QCHANNEL_CON */
	wr(MFGCFG + 0x170, 3);                                          /* BUS_IDLE_CON */
	if (!poll_ms(MFGCFG + 0x178, 1u << 2, 1u << 2)) {
		SAY("  GPU bus not idle (0x%08x)\n", rd(MFGCFG + 0x178));
	}
	wr(MFGCFG + 0x004, 1u << 0);                                    /* CG_SET: BG3D clock off */
	for (int n = 3; n >= 0; n--) {
		uint64_t con = SPM + MFG_PWR_CON(n);
		if (n == 1) {
			for (int i = 0; i < 4; i++) {
				wr(INFRA + gProt[i].set, gProt[i].mask);
				if (!poll_ms(INFRA + gProt[i].sta, gProt[i].mask, gProt[i].mask)) {
					SAY("  MFG1: bus protection step %d not acknowledged (0x%08x): MFG0-1 left on\n", i + 1,
					    rd(INFRA + gProt[i].sta));
					return false;
				}
			}
		}
		wr(con, rd(con) | PWR_SRAM_PDN);
		if (!poll_ms(con, PWR_SRAM_ACK, PWR_SRAM_ACK)) {
			SAY("  MFG%d: no SRAM power-down ack\n", n);
		}
		wr(con, rd(con) | PWR_ISO);
		wr(con, rd(con) | PWR_CLK_DIS);
		wr(con, rd(con) & ~PWR_RST_B);
		wr(con, rd(con) & ~PWR_ON);
		wr(con, rd(con) & ~PWR_ON_2ND);
		if (!poll_ms(SPM + PWR_STATUS, MFG_BIT(n), 0) || !poll_ms(SPM + PWR_STATUS_2ND, MFG_BIT(n), 0)) {
			SAY("  MFG%d: still on after power off\n", n);
			return false;
		}
	}
	return true;
}

/* Step 2: the four domains on (as MediaTek's power-domain driver and MappleGPU's mtcmosOn), then mfgcfg as gpufreq sets
 * it up, then the Mali's identity. */
static void
gpu_power(char **p, char *end)
{
	bool mfgOn = false;
	if (!gpu_probe(p, end, &mfgOn)) {
		SAY("gpu power: refused, not as expected above\n");
		return;
	}
	if (!mfgOn) {
		wr(SPM + SPM_KEY_REG, SPM_KEY);
		for (int n = 0; n < 4; n++) {
			uint64_t con = SPM + MFG_PWR_CON(n);
			wr(con, rd(con) | PWR_ON);
			wr(con, rd(con) | PWR_ON_2ND);
			if (!poll_ms(SPM + PWR_STATUS, MFG_BIT(n), MFG_BIT(n)) || !poll_ms(SPM + PWR_STATUS_2ND, MFG_BIT(n), MFG_BIT(n))) {
				SAY("gpu power: MFG%d no power ack (pwr_con 0x%08x): stopped, reboot to undo\n", n, rd(con));
				return;
			}
			wr(con, rd(con) & ~PWR_CLK_DIS);
			wr(con, rd(con) & ~PWR_ISO);
			wr(con, rd(con) | PWR_RST_B);
			wr(con, rd(con) & ~PWR_SRAM_PDN);
			if (!poll_ms(con, PWR_SRAM_ACK, 0)) {
				SAY("gpu power: MFG%d no SRAM ack (pwr_con 0x%08x): stopped, reboot to undo\n", n, rd(con));
				return;
			}
			if (n == 1) {           /* MFG1's bus protection, released in reverse of the order it is set */
				wr(INFRA + PROT1_CLR, 0x20000000);
				wr(INFRA + PROT_CLR, 0x02000000);
				wr(INFRA + PROT_CLR, 0x00600000);
				wr(INFRA + PROT1_CLR, 0x18000000);
			}
			SAY("  MFG%d on (pwr_con 0x%08x)\n", n, rd(con));
		}
		wr(MFGCFG + 0x008, 1u << 0);                                    /* CG_CLR: BG3D clock */
		wr(MFGCFG + 0x020, rd(MFGCFG + 0x020) | (0xfu << 22));          /* ASYNC_CON */
		wr(MFGCFG + 0x0b0, rd(MFGCFG + 0x0b0) & ~((1u << 10) | (1u << 9)));     /* GLOBAL_CON */
		wr(MFGCFG + 0x0b4, rd(MFGCFG + 0x0b4) | (1u << 4));             /* QCHANNEL_CON */
		wr(MFGCFG + 0x0b0, rd(MFGCFG + 0x0b0) & ~((1u << 19) | (1u << 8)));
		wr(MFGCFG + 0x024, rd(MFGCFG + 0x024) | 1u);                    /* ASYNC_CON_1 */
		wr(MFGCFG + 0xf60, rd(MFGCFG + 0xf60) | 1u);                    /* I2M protector */
		wr(MFGCFG + 0x130, 1);                                          /* timestamp */
		wr(INFRA + ACP_CON, rd(INFRA + ACP_CON) | (1u << 9));
	}
	uint32_t id = rd(MALI + GPU_ID), sh = rd(MALI + SHADER_PRESENT), cg = rd(MFGCFG + 0x000);
	bool ok = id == GPU_ID_G57_MC2 && sh == SHADERS_SPACED && !(cg & 1);
	SAY("gpu power: mfgcfg cg 0x%08x; GPU_ID 0x%08x (want 0x%08x), SHADER_PRESENT 0x%x (want 0x%x): %s\n",
	    cg, id, GPU_ID_G57_MC2, sh, SHADERS_SPACED, ok ? "the Mali-G57 MC2 as expected" : "MISMATCH");
	/* nothing services the Mali's interrupts here: all masked and cleared before the domains go off */
	SAY("  irq raw/mask before: gpu 0x%08x/0x%08x job 0x%08x/0x%08x mmu 0x%08x/0x%08x\n", rd(MALI + GPU_IRQ_RAWSTAT),
	    rd(MALI + GPU_IRQ_MASK), rd(MALI + JOB_IRQ_RAWSTAT), rd(MALI + JOB_IRQ_MASK), rd(MALI + MMU_IRQ_RAWSTAT),
	    rd(MALI + MMU_IRQ_MASK));
	wr(MALI + GPU_IRQ_MASK, 0);
	wr(MALI + JOB_IRQ_MASK, 0);
	wr(MALI + MMU_IRQ_MASK, 0);
	wr(MALI + GPU_IRQ_CLEAR, 0xffffffff);
	wr(MALI + JOB_IRQ_CLEAR, 0xffffffff);
	wr(MALI + MMU_IRQ_CLEAR, 0xffffffff);
	bool down = gpu_down(p, end);
	SAY("gpu power: %s; MFG0-3 status 0x%x / 0x%x, MFGPLL con0 0x%08x (rails untouched)\n",
	    down ? "powered down again" : "NOT all powered down (reboot to undo)", (rd(SPM + PWR_STATUS) >> 2) & 0xf,
	    (rd(SPM + PWR_STATUS_2ND) >> 2) & 0xf, rd(MFGPLL_CON0));
}

void mtk_gpu_probe_link(const char *args, char *out, size_t len);

void
mtk_gpu_probe_link(const char *args, char *out, size_t len)
{
	char *p = out, *end = out + len;
	if (gProbe < 0) {
		int v = 0;
		PE_parse_boot_argn_alias("mp_gpuprobe", "gpuprobe", &v, sizeof(v));
		gProbe = v;
	}
	while (*args == ' ') {
		args++;
	}
	bool mfgOn;
	if (strncmp(args, "probe", 5) == 0 && gProbe >= 1) {
		gpu_probe(&p, end, &mfgOn);
	} else if (strncmp(args, "power", 5) == 0 && gProbe >= 2) {
		gpu_power(&p, end);
	} else {
		snprintf(p, end - p, "gpu: driver not running; gpu probe (boot-arg mp_gpuprobe=1), gpu power (mp_gpuprobe=2)\n");
	}
}
#endif /* DEVELOPMENT || DEBUG */
