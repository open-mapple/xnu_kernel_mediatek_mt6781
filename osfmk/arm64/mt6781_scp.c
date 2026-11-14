/*
 * MT6781: the SCP, the Cortex-M ("tinysys") that runs the sensor hub, for bsd/dev/mtk_scp.c.
 *
 * LK loads its firmware into the SCP's SRAM; Android's kernel puts its clock (scp_sel) on clk26m and lets it run with
 * one write to cfg+0. Messages go through two 288-byte slots in its SRAM, {u32 id, u32 len, 8, 272 bytes}, and a
 * doorbell each way. Addresses from the realme 8i's device tree (scp@10500000: SRAM, cfg; SPI 205) and kernel
 * (scp/cm4: scp_reg.h, scp_helper.h, v01/scp_ipi.c, v01/scp_irq.c, scp_awake.c; clk-mt6781.c). Every write goes
 * through scp_write() and its list: these registers and values only.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/clock.h>
#include <kern/misc_protos.h>
#include <machine/machine_routines.h>

bool mt6781_scp_map(void);
uint32_t mt6781_scp_cfg(uint32_t off);
uint32_t mt6781_scp_sram(uint32_t off);
uint32_t mt6781_scp_clock(void);
bool mt6781_scp_clock_26m(void);
bool mt6781_scp_run(void);
bool mt6781_scp_send(uint32_t id, const void *data, uint32_t len);
bool mt6781_scp_recv(uint32_t *id, void *data, uint32_t *len, uint32_t max);

#define TOPCKGEN        0x10000000u
#define INFRACFG_AO     0x10001000u
#define SCP_SRAM        0x10500000u
#define SCP_SRAM_SIZE   0xc0000u
#define SCP_CFG         0x105c0000u

#define CLK_CFG_UPDATE  (TOPCKGEN + 0x004)      /* bit 1: scp_sel's new setting taken */
#define CLK_CFG_0       (TOPCKGEN + 0x040)      /* [10:8] scp_sel (0 = clk26m); bit 15 its power-down, never written */
#define CLK_CFG_0_CLR   (TOPCKGEN + 0x048)
#define SCP_SEL         (7u << 8)
#define INFRA_IRQ_SET   (INFRACFG_AO + 0xb14)   /* awake handshake: 0xA0 | bit 0 lock, bit 1 unlock */
#define INFRA_IRQ_CLR   (INFRACFG_AO + 0xb18)
#define SCP_RUN         (SCP_CFG + 0x000)
#define SCP_TO_HOST     (SCP_CFG + 0x01c)       /* bit 0 a message, bit 8 its watchdog; 1 clears */
#define SCP_TO_SPM      (SCP_CFG + 0x020)
#define SCP_GIPC_IN     (SCP_CFG + 0x028)       /* bit 0 a message for it, bit 2 ours taken (wakes it) */

#define SLOT_RECV       0x5c0
#define SLOT_SEND       0x6e0
#define SLOT_PAYLOAD    272

static const struct {
	uint32_t pa, value;
} scp_regs[] = {
	{ CLK_CFG_0_CLR, SCP_SEL }, { CLK_CFG_UPDATE, 1u << 1 },
	{ INFRA_IRQ_SET, 0xa1 }, { INFRA_IRQ_SET, 0xa2 },
	{ SCP_RUN, 1 }, { SCP_TO_HOST, 1 }, { SCP_TO_SPM, 1 }, { SCP_GIPC_IN, 1 }, { SCP_GIPC_IN, 4 },
};

static vm_offset_t topckgen, infracfg, sram, cfg;

static vm_offset_t
va(uint32_t pa)
{
	if (pa >= SCP_CFG && pa < SCP_CFG + 0x1000) {
		return cfg + (pa - SCP_CFG);
	}
	if (pa >= INFRACFG_AO && pa < INFRACFG_AO + 0x1000) {
		return infracfg + (pa - INFRACFG_AO);
	}
	return topckgen + (pa - TOPCKGEN);
}

static uint32_t
rd(uint32_t pa)
{
	return *(volatile uint32_t *)va(pa);
}

/* The one way to write a register: listed ones and values only (INFRA_IRQ_CLR takes back what INFRA_IRQ_SET reads). */
static bool
scp_write(uint32_t pa, uint32_t value)
{
	unsigned i;

	for (i = 0; i < sizeof(scp_regs) / sizeof(scp_regs[0]); i++) {
		if (scp_regs[i].pa == pa && scp_regs[i].value == value) {
			break;
		}
	}
	if (i == sizeof(scp_regs) / sizeof(scp_regs[0]) && !(pa == INFRA_IRQ_CLR && (value & ~0xffu) == 0)) {
		printf("scp: refused write %#x <- %#x\n", pa, value);
		return false;
	}
	*(volatile uint32_t *)va(pa) = value;
	__builtin_arm_dsb(0xf);
	return true;
}

bool
mt6781_scp_map(void)
{
	if (cfg == 0) {
		topckgen = ml_io_map(TOPCKGEN, 0x1000);
		infracfg = ml_io_map(INFRACFG_AO, 0x1000);
		sram = ml_io_map(SCP_SRAM, SCP_SRAM_SIZE);
		cfg = ml_io_map(SCP_CFG, 0x1000);
	}
	return topckgen && infracfg && sram && cfg;
}

uint32_t
mt6781_scp_cfg(uint32_t off)
{
	return off < 0x1000 && (off & 3) == 0 ? rd(SCP_CFG + off) : 0;
}

uint32_t
mt6781_scp_sram(uint32_t off)
{
	return off < SCP_SRAM_SIZE && (off & 3) == 0 ? *(volatile uint32_t *)(sram + off) : 0;
}

/* CLK_CFG_0: scp_sel's parent in [10:8], its power-down at bit 15 */
uint32_t
mt6781_scp_clock(void)
{
	return rd(CLK_CFG_0);
}

/* scp_sel on clk26m, as Android's kernel sets it before the start (clk-mux.c set_parent: CLR the field, UPDATE bit 1) */
bool
mt6781_scp_clock_26m(void)
{
	if ((rd(CLK_CFG_0) & SCP_SEL) == 0) {
		return true;
	}
	if (!scp_write(CLK_CFG_0_CLR, SCP_SEL) || !scp_write(CLK_CFG_UPDATE, 1u << 1)) {
		return false;
	}
	delay(100);
	return (rd(CLK_CFG_0) & SCP_SEL) == 0;
}

/* Let it run, once: only from held (cfg+0 reads 0). */
bool
mt6781_scp_run(void)
{
	return rd(SCP_RUN) == 0 && scp_write(SCP_RUN, 1);
}

/* Keep it awake while its SRAM is written: 0xA0 | (1 << which), wait for it to take it, then clear what was set. */
static bool
awake(unsigned which)
{
	uint32_t v = 0;

	if (!scp_write(INFRA_IRQ_SET, 0xa0u | (1u << which))) {
		return false;
	}
	for (int i = 0; i < 10000; i++) {
		v = rd(INFRA_IRQ_SET);
		if ((v & (1u << which)) == 0 && (v & 0xf0u) == 0xa0u) {
			return scp_write(INFRA_IRQ_CLR, v & 0xffu);
		}
		delay(10);
	}
	printf("scp: awake %s not taken (%#x)\n", which ? "unlock" : "lock", v);
	return false;
}

/* A message into its slot and the doorbell; false if it has not taken the last one yet. */
bool
mt6781_scp_send(uint32_t id, const void *data, uint32_t len)
{
	bool ok = false;

	if (len > SLOT_PAYLOAD || !awake(0)) {
		return false;
	}
	if ((rd(SCP_GIPC_IN) & 1) == 0) {
		volatile uint32_t *slot = (volatile uint32_t *)(sram + SLOT_SEND);
		const uint8_t *p = data;
		for (uint32_t i = 0; i < (len + 3) / 4; i++) {
			uint32_t w = 0;
			for (uint32_t j = 0; j < 4 && 4 * i + j < len; j++) {
				w |= (uint32_t)p[4 * i + j] << (8 * j);
			}
			slot[4 + i] = w;
		}
		slot[1] = len;
		slot[0] = id;
		__builtin_arm_dsb(0xf);
		ok = slot[0] == id && scp_write(SCP_GIPC_IN, 1);
	}
	awake(1);
	return ok;
}

/* Its message, if one waits: read out of its slot, then taken (GIPC bit 2, SPM, clear). */
bool
mt6781_scp_recv(uint32_t *id, void *data, uint32_t *len, uint32_t max)
{
	static bool wdt_said;
	uint32_t pending = rd(SCP_TO_HOST);

	if ((pending & (1u << 8)) && !wdt_said) {
		wdt_said = true;
		printf("scp: watchdog (to host %#x, its PC then %#x)\n", pending, rd(SCP_CFG + 0xc4));
	}
	if ((pending & 1) == 0) {
		return false;
	}
	volatile uint32_t *slot = (volatile uint32_t *)(sram + SLOT_RECV);
	uint32_t n = slot[1];
	uint8_t *p = data;

	*id = slot[0];
	*len = n = n > SLOT_PAYLOAD ? SLOT_PAYLOAD : n;
	for (uint32_t i = 0; i < n && i < max; i++) {
		p[i] = (uint8_t)(slot[4 + i / 4] >> (8 * (i % 4)));
	}
	scp_write(SCP_GIPC_IN, 4);
	scp_write(SCP_TO_SPM, 1);
	scp_write(SCP_TO_HOST, 1);
	return true;
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
