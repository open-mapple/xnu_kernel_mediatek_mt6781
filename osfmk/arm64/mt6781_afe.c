/*
 * MT6781: the audio front end's playback path, memif DL1 -> ADDA DL -> the MT6366 codec, for bsd/dev/mtk_audio.c.
 * 48 kHz, 16-bit stereo, from a ring in the AFE's own SRAM, its position polled (no interrupt, no DMA mapping).
 *
 * Facts from the realme 8i's kernel (sound/soc/mediatek/mt6781, drivers/clk/mediatek/clk-mt6781.c): there is no
 * audio power domain; the AFE sits behind infracfg_ao's audio gates; audio_sel and aud_intbus_sel run from clk26m
 * (APLLs are for I2S and rates above 48 kHz only). Every write goes through afe_write() and its whitelist: these
 * registers and bits only, read back where the register can be read.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/clock.h>
#include <kern/misc_protos.h>
#include <machine/machine_routines.h>

void mt6781_gpio_pin(uint32_t pin, uint32_t *mode, bool *output, bool *high);         /* for bsd/dev/mtk_audio.c */
void mt6781_afe_status(void);
bool mt6781_afe_up(void);
bool mt6781_afe_dl1_start(volatile uint32_t **ring, uint32_t *bytes);
uint32_t mt6781_afe_dl1_position(void);
void mt6781_afe_dl1_stop(void);
bool mt6781_afe_ul_start(volatile uint32_t **ring, uint32_t *bytes);
uint32_t mt6781_afe_ul_position(void);
void mt6781_afe_ul_stop(void);
void mt6781_afe_down(void);

#define TOPCKGEN                0x10000000u
#define INFRACFG_AO             0x10001000u
#define GPIO                    0x10005000u
#define AFE                     0x11210000u
#define AFE_SRAM                0x11212000u

#define CLK_CFG_UPDATE          (TOPCKGEN + 0x004)      /* bit 15 audio_sel, bit 16 aud_intbus_sel */
#define CLK_CFG_3               (TOPCKGEN + 0x070)      /* [25:24] audio_sel (0 = clk26m) */
#define CLK_CFG_3_CLR           (TOPCKGEN + 0x078)
#define CLK_CFG_4               (TOPCKGEN + 0x080)      /* [1:0] aud_intbus_sel (0 = clk26m) */
#define CLK_CFG_4_CLR           (TOPCKGEN + 0x088)
#define INFRA_CG1_SET           (INFRACFG_AO + 0x088)   /* bit 25 infra_audio */
#define INFRA_CG1_CLR           (INFRACFG_AO + 0x08c)
#define INFRA_CG1_STA           (INFRACFG_AO + 0x094)
#define INFRA_CG3_SET           (INFRACFG_AO + 0x0a4)   /* bit 4 audio_26m_bclk */
#define INFRA_CG3_CLR           (INFRACFG_AO + 0x0a8)
#define INFRA_CG3_STA           (INFRACFG_AO + 0x0ac)
#define GPIO_MODE_180           (GPIO + 0x460)          /* pins 176-183: 183 AUD_CLK_MOSI at [31:28] */
#define GPIO_MODE_184           (GPIO + 0x470)          /* pins 184-191: 184 SYNC [3:0], 185/186 DAT_MOSI0/1 [7:4]/[11:8],
                                                         * 189/190 DAT_MISO0/1 [23:20]/[27:24] */
#define GPIO_MODE_184_MWR       (GPIO + 0x47c)          /* 4 bits a pin: bit 3 = write this pin's mode */

#define AUDIO_TOP_CON0          (AFE + 0x000)           /* 1 = off: bit 2 AFE, 24 ADC, 25 DAC, 26 DAC_PREDIS */
#define AFE_DAC_CON0            (AFE + 0x010)           /* bit 0 AFE_ON, bit 4 DL1_ON, bit 31 VUL12_ON */
#define AFE_CONN3               (AFE + 0x02c)           /* bit 5: I05 (DL1 left) -> O03 */
#define AFE_CONN4               (AFE + 0x030)           /* bit 6: I06 (DL1 right) -> O04 */
#define AFE_DL1_CON0            (AFE + 0x04c)           /* [27:24] rate 10 = 48k, bit 8 mono, bit 5 normal mode, [1:0] 16-bit */
#define AFE_DL1_BASE_MSB        (AFE + 0x050)
#define AFE_DL1_BASE            (AFE + 0x054)
#define AFE_DL1_CUR             (AFE + 0x05c)
#define AFE_DL1_END_MSB         (AFE + 0x060)
#define AFE_DL1_END             (AFE + 0x064)
#define AFE_ADDA_DL_SRC2_CON0   (AFE + 0x108)
#define AFE_ADDA_UL_SRC_CON0    (AFE + 0x114)           /* [19:17] 3 = 48k, bit 10 IIR on; bit 0 = on */
#define AFE_ADDA_TOP_CON0       (AFE + 0x120)           /* bit 0: 0 = the codec's ADC (not the loopback) */
#define AFE_ADDA_DL_SRC2_CON1   (AFE + 0x10c)           /* [31:16] gain: 0xf74f, Android's "normal" (-0.3 dB) */
#define AFE_ADDA_UL_DL_CON0     (AFE + 0x124)           /* bit 0 ADDA_AFE_ON */
#define AFE_ADDA_PREDIS_CON0    (AFE + 0x260)
#define AFE_ADDA_PREDIS_CON1    (AFE + 0x264)
#define AFE_ADDA_IIR_COEF_02_01 (AFE + 0x290)           /* .. 0x2a0: the UL's 35 Hz high-pass, Android's coefficients */
#define AFE_DAC_MON             (AFE + 0x2ec)           /* bit 0: AFE_ON as the hardware has it */
#define AFE_VUL12_CON0          (AFE + 0x374)           /* [27:24] rate 10 = 48k, bit 5 normal mode, bit 8 mono, [1:0] 16-bit */
#define AFE_VUL12_BASE_MSB      (AFE + 0x378)
#define AFE_VUL12_BASE          (AFE + 0x37c)
#define AFE_VUL12_CUR           (AFE + 0x384)
#define AFE_VUL12_END_MSB       (AFE + 0x388)
#define AFE_VUL12_END           (AFE + 0x38c)
#define AFE_CONN21              (AFE + 0x470)           /* bit 3: I03 (ADDA UL left) -> O21 (VUL12 left) */
#define AFE_CONN22              (AFE + 0x474)           /* bit 4: I04 (ADDA UL right) -> O22 */
#define AFE_MEMIF_CON0          (AFE + 0x8cc)           /* bit 1 CPU_HD_ALIGN, bit 2 CPU_COMPACT_MODE */
#define AFE_ADDA_DL_SDM_DCCOMP  (AFE + 0xc50)           /* [5:0] attenuation 0x1d, bit 28 3rd-order SDM */
#define AFE_ADDA_DL_SDM_DITHER  (AFE + 0xc70)           /* bit 20 64-tap dither */
#define AFE_ADDA_DL_SDM_AUTO    (AFE + 0xc74)           /* Android leaves 0x80190000 */
#define AFE_ADDA_MTKAIF_CFG0    (AFE + 0xe00)           /* 0: protocol 1, as the machine driver sets it */
#define AFE_ADDA_MTKAIF_RX_CFG0 (AFE + 0xe20)           /* bit 0: 0 = AMIC */
#define AFE_AUD_PAD_TOP         (AFE + 0xe40)           /* 0x30, bit 0 RX_FIFO_ON */

#define DL_SRC2_CON0_48K        0x83001802u             /* in 48k, x8 out, both mutes off, gain on; bit 0 = on */
#define DL_SRC2_GAIN            0xf74f0000u
#define SRAM_BYTES              0x4000u                 /* DL1's ring: 4096 frames, 85 ms */
#define UL_SRAM                 (AFE_SRAM + SRAM_BYTES) /* VUL12's ring, the same size, in the SRAM Android leaves free */
#define UL_SRC_CON0_48K         0x00060400u

/* What may be written: register, bits; wo = a set/clear/update register (written as given, not read back). */
static const struct {
	uint32_t pa, mask;
	bool wo;
} afe_regs[] = {
	{ INFRA_CG1_SET, 1u << 25, true }, { INFRA_CG1_CLR, 1u << 25, true },
	{ INFRA_CG3_SET, 1u << 4, true }, { INFRA_CG3_CLR, 1u << 4, true },
	{ CLK_CFG_3_CLR, 3u << 24, true }, { CLK_CFG_4_CLR, 3u, true }, { CLK_CFG_UPDATE, 3u << 15, true },
	{ GPIO_MODE_184_MWR, 0xff00ff0, true },
	{ AUDIO_TOP_CON0, (1u << 2) | (1u << 24) | (1u << 25) | (1u << 26), false },
	{ AFE_DAC_CON0, 0x80000011u, false },
	{ AFE_CONN3, 1u << 5, false }, { AFE_CONN4, 1u << 6, false },
	{ AFE_DL1_CON0, 0x0f00313fu, false },
	{ AFE_DL1_BASE_MSB, ~0u, false }, { AFE_DL1_BASE, ~0u, false }, { AFE_DL1_END_MSB, ~0u, false }, { AFE_DL1_END, ~0u, false },
	{ AFE_ADDA_DL_SRC2_CON0, ~0u, false }, { AFE_ADDA_DL_SRC2_CON1, 0xffff0000u, false },
	{ AFE_ADDA_UL_DL_CON0, 1u, false },
	{ AFE_ADDA_PREDIS_CON0, ~0u, false }, { AFE_ADDA_PREDIS_CON1, ~0u, false },
	{ AFE_MEMIF_CON0, 0x6, false },
	{ AFE_ADDA_DL_SDM_DCCOMP, (1u << 28) | 0x3f, false }, { AFE_ADDA_DL_SDM_DITHER, 1u << 20, false },
	{ AFE_ADDA_DL_SDM_AUTO, ~0u, false },
	{ AFE_ADDA_UL_SRC_CON0, ~0u, false }, { AFE_ADDA_TOP_CON0, 1u, false },
	{ AFE_ADDA_IIR_COEF_02_01, ~0u, false }, { AFE_ADDA_IIR_COEF_02_01 + 4, ~0u, false },
	{ AFE_ADDA_IIR_COEF_02_01 + 8, ~0u, false }, { AFE_ADDA_IIR_COEF_02_01 + 12, ~0u, false },
	{ AFE_ADDA_IIR_COEF_02_01 + 16, ~0u, false },
	{ AFE_VUL12_CON0, 0x0f000b23u, false },
	{ AFE_VUL12_BASE_MSB, ~0u, false }, { AFE_VUL12_BASE, ~0u, false }, { AFE_VUL12_END_MSB, ~0u, false }, { AFE_VUL12_END, ~0u, false },
	{ AFE_CONN21, 1u << 3, false }, { AFE_CONN22, 1u << 4, false },
	{ AFE_ADDA_MTKAIF_CFG0, ~0u, false }, { AFE_ADDA_MTKAIF_RX_CFG0, 1u, false }, { AFE_AUD_PAD_TOP, 0xff, false },
};

static const uint32_t ul_iir[5] = { 0, 0x3fb8, 0x3fb80000u, 0x3fb80000u, 0xc048 };

static vm_offset_t topckgen, infracfg, gpio, afe, sram;
static bool afe_open;           /* the infra gates are open: AFE registers can be touched */

static vm_offset_t
va(uint32_t pa)
{
	if (pa >= AFE_SRAM && pa < AFE_SRAM + 2 * SRAM_BYTES) {
		return sram + (pa - AFE_SRAM);
	}
	if (pa >= AFE && pa < AFE + 0x2000) {
		return afe + (pa - AFE);
	}
	if (pa >= GPIO && pa < GPIO + 0x1000) {
		return gpio + (pa - GPIO);
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

/* The one way to write: whitelisted bits only; read-modify-write and read back, or a plain write to a wo register. */
static bool
afe_write(uint32_t pa, uint32_t mask, uint32_t value)
{
	unsigned i;
	uint32_t now;

	for (i = 0; i < sizeof(afe_regs) / sizeof(afe_regs[0]) && afe_regs[i].pa != pa; i++) {
	}
	if (i == sizeof(afe_regs) / sizeof(afe_regs[0]) || mask == 0 || (mask & ~afe_regs[i].mask) || (value & ~mask) ||
	    (pa >= AFE && pa < AFE + 0x2000 && !afe_open)) {
		printf("audio: afe: refused write %#x mask %#x <- %#x\n", pa, mask, value);
		return false;
	}
	if (afe_regs[i].wo) {
		*(volatile uint32_t *)va(pa) = value;
		__builtin_arm_dsb(0xf);
		return true;
	}
	*(volatile uint32_t *)va(pa) = (rd(pa) & ~mask) | value;
	__builtin_arm_dsb(0xf);
	if (((now = rd(pa)) & mask) != value) {
		printf("audio: afe: %#x reads %#x after writing %#x (mask %#x)\n", pa, now, value, mask);
		return false;
	}
	return true;
}

static void
afe_map(void)
{
	if (afe == 0) {
		topckgen = ml_io_map(TOPCKGEN, 0x1000);
		infracfg = ml_io_map(INFRACFG_AO, 0x1000);
		gpio = ml_io_map(GPIO, 0x1000);
		afe = ml_io_map(AFE, 0x2000);
		sram = ml_io_map(AFE_SRAM, 2 * SRAM_BYTES);
	}
}

static bool
gates_open(void)
{
	return !(rd(INFRA_CG1_STA) & (1u << 25)) && !(rd(INFRA_CG3_STA) & (1u << 4));
}

/* Read only: a pin's mode, direction and level (bsd/dev/mtk_audio.c checks the amplifier's before driving it). */
void
mt6781_gpio_pin(uint32_t pin, uint32_t *mode, bool *output, bool *high)
{
	afe_map();
	*mode = (rd(GPIO + 0x300 + 0x10 * (pin / 8)) >> (4 * (pin % 8))) & 0xf;
	*output = (rd(GPIO + 0x000 + 0x10 * (pin / 32)) >> (pin % 32)) & 1;
	*high = (rd(GPIO + 0x100 + 0x10 * (pin / 32)) >> (pin % 32)) & 1;
}

/* Read only: the clocks, the pins, and (only if its gates are already open) the AFE. */
void
mt6781_afe_status(void)
{
	afe_map();
	printf("audio: afe: infra audio gate %s, 26m gate %s; audio_sel %u, aud_intbus_sel %u (0 = clk26m); pins 183-186 modes "
	    "%u %u %u %u (1 = AUD)\n", rd(INFRA_CG1_STA) & (1u << 25) ? "closed" : "open", rd(INFRA_CG3_STA) & (1u << 4) ? "closed" : "open",
	    (rd(CLK_CFG_3) >> 24) & 3, rd(CLK_CFG_4) & 3, rd(GPIO_MODE_180) >> 28, rd(GPIO_MODE_184) & 0xf, (rd(GPIO_MODE_184) >> 4) & 0xf,
	    (rd(GPIO_MODE_184) >> 8) & 0xf);
	if (gates_open()) {
		printf("audio: afe: top %08x dac %08x dl1 %08x conn3 %08x conn4 %08x src2 %08x %08x ul_dl %08x\n", rd(AUDIO_TOP_CON0),
		    rd(AFE_DAC_CON0), rd(AFE_DL1_CON0), rd(AFE_CONN3), rd(AFE_CONN4), rd(AFE_ADDA_DL_SRC2_CON0), rd(AFE_ADDA_DL_SRC2_CON1),
		    rd(AFE_ADDA_UL_DL_CON0));
	}
}

/* Clocks, AFE on, ADDA DL set for 48 kHz and on, the data pins to the codec. Android's order (mt6781-afe-clk.c, -dai-adda.c). */
bool
mt6781_afe_up(void)
{
	bool ok;

	afe_map();
	if ((rd(GPIO_MODE_180) >> 28) != 1 || (rd(GPIO_MODE_184) & 0xf) != 1) {
		printf("audio: afe: AUD_CLK_MOSI/SYNC (pins 183/184) are not in their audio mode: not started\n");
		return false;
	}
	ok = afe_write(INFRA_CG1_CLR, 1u << 25, 1u << 25) && afe_write(INFRA_CG3_CLR, 1u << 4, 1u << 4);
	if (ok && ((rd(CLK_CFG_3) >> 24) & 3) != 0) {
		ok = afe_write(CLK_CFG_3_CLR, 3u << 24, 3u << 24) && afe_write(CLK_CFG_UPDATE, 1u << 15, 1u << 15);
	}
	if (ok && (rd(CLK_CFG_4) & 3) != 0) {
		ok = afe_write(CLK_CFG_4_CLR, 3u, 3u) && afe_write(CLK_CFG_UPDATE, 1u << 16, 1u << 16);
	}
	delay(10);
	if (!ok || !gates_open()) {
		printf("audio: afe: the infra gates did not open\n");
		return false;
	}
	afe_open = true;
	ok = afe_write(AUDIO_TOP_CON0, 1u << 2, 0) &&
	    afe_write(AFE_MEMIF_CON0, 0x6, 0) &&
	    afe_write(AFE_DAC_CON0, 1u, 1u) &&
	    afe_write(AFE_ADDA_PREDIS_CON0, ~0u, 0) && afe_write(AFE_ADDA_PREDIS_CON1, ~0u, 0) &&
	    afe_write(AFE_ADDA_DL_SRC2_CON0, ~0u, DL_SRC2_CON0_48K) &&
	    afe_write(AFE_ADDA_DL_SRC2_CON1, 0xffff0000u, DL_SRC2_GAIN) &&
	    afe_write(AFE_ADDA_DL_SDM_DCCOMP, 0x3f, 0x1d) &&
	    afe_write(AFE_ADDA_DL_SDM_DITHER, 1u << 20, 1u << 20) &&
	    afe_write(AFE_ADDA_DL_SDM_AUTO, ~0u, 0x80190000u) &&
	    afe_write(AFE_ADDA_DL_SDM_DCCOMP, 1u << 28, 0) &&
	    afe_write(AUDIO_TOP_CON0, (1u << 25) | (1u << 26), 0) &&
	    afe_write(AFE_ADDA_UL_DL_CON0, 1u, 1u) &&
	    afe_write(GPIO_MODE_184_MWR, 0xff0, 0x990) &&
	    afe_write(AFE_ADDA_DL_SRC2_CON0, 1u, 1u);
	printf("audio: afe: %s\n", ok ? "up: clocks from clk26m, AFE on, ADDA DL at 48 kHz" : "FAILED to come up");
	mt6781_afe_status();
	return ok;
}

/* DL1 from the SRAM ring, zeroed, into the ADDA DL; the CPU writes frames ahead of mt6781_afe_dl1_position(). */
bool
mt6781_afe_dl1_start(volatile uint32_t **ring, uint32_t *bytes)
{
	bool ok;

	if (!afe_open) {
		return false;
	}
	for (uint32_t i = 0; i < SRAM_BYTES; i += 4) {
		*(volatile uint32_t *)(sram + i) = 0;
	}
	ok = afe_write(AFE_DAC_CON0, 1u << 4, 0) &&
	    afe_write(AFE_DL1_BASE_MSB, ~0u, 0) && afe_write(AFE_DL1_BASE, ~0u, AFE_SRAM) &&
	    afe_write(AFE_DL1_END_MSB, ~0u, 0) && afe_write(AFE_DL1_END, ~0u, AFE_SRAM + SRAM_BYTES - 1) &&
	    afe_write(AFE_DL1_CON0, 0x0f00313fu, (10u << 24) | (3u << 12) | (1u << 5)) &&
	    afe_write(AFE_CONN3, 1u << 5, 1u << 5) && afe_write(AFE_CONN4, 1u << 6, 1u << 6) &&
	    afe_write(AFE_DAC_CON0, 1u << 4, 1u << 4);
	*ring = (volatile uint32_t *)sram;
	*bytes = SRAM_BYTES;
	printf("audio: afe: DL1 %s (%u bytes of SRAM at %#x)\n", ok ? "runs" : "did NOT start", SRAM_BYTES, AFE_SRAM);
	return ok;
}

/* Where the hardware reads, in bytes from the ring's start (16-byte steps). */
uint32_t
mt6781_afe_dl1_position(void)
{
	uint32_t cur = rd(AFE_DL1_CUR);

	return cur >= AFE_SRAM && cur < AFE_SRAM + SRAM_BYTES ? (cur - AFE_SRAM) & ~15u : 0;
}

void
mt6781_afe_dl1_stop(void)
{
	if (afe_open) {
		(void)(afe_write(AFE_DAC_CON0, 1u << 4, 0) && afe_write(AFE_CONN3, 1u << 5, 0) && afe_write(AFE_CONN4, 1u << 6, 0));
	}
}

/*
 * The mic: ADDA UL at 48 kHz with Android's high-pass, the codec's data pins, VUL12 into its own SRAM ring (stereo,
 * zeroed); the CPU reads behind mt6781_afe_ul_position(). Android's order (mt6781-dai-adda.c, -afe-pcm.c).
 */
bool
mt6781_afe_ul_start(volatile uint32_t **ring, uint32_t *bytes)
{
	bool ok = afe_open;

	if (!ok) {
		return false;
	}
	for (uint32_t i = 0; i < SRAM_BYTES; i += 4) {
		*(volatile uint32_t *)va(UL_SRAM + i) = 0;
	}
	for (int i = 0; ok && i < 5; i++) {
		ok = afe_write(AFE_ADDA_IIR_COEF_02_01 + 4 * (uint32_t)i, ~0u, ul_iir[i]);
	}
	ok = ok && afe_write(AFE_ADDA_UL_SRC_CON0, ~0u, UL_SRC_CON0_48K) &&
	    afe_write(AFE_ADDA_TOP_CON0, 1u, 0) && afe_write(AFE_ADDA_MTKAIF_RX_CFG0, 1u, 0) &&
	    afe_write(AUDIO_TOP_CON0, 1u << 24, 0) &&
	    afe_write(AFE_AUD_PAD_TOP, 0xff, 0x30) && afe_write(AFE_AUD_PAD_TOP, 0xff, 0x31) &&
	    afe_write(AFE_ADDA_MTKAIF_CFG0, ~0u, 0) &&
	    afe_write(GPIO_MODE_184_MWR, 0xff00000, 0x9900000) &&
	    afe_write(AFE_ADDA_UL_SRC_CON0, 1u, 1u) &&
	    afe_write(AFE_CONN21, 1u << 3, 1u << 3) && afe_write(AFE_CONN22, 1u << 4, 1u << 4) &&
	    afe_write(AFE_DAC_CON0, 1u << 31, 0) &&
	    afe_write(AFE_VUL12_BASE_MSB, ~0u, 0) && afe_write(AFE_VUL12_BASE, ~0u, UL_SRAM) &&
	    afe_write(AFE_VUL12_END_MSB, ~0u, 0) && afe_write(AFE_VUL12_END, ~0u, UL_SRAM + SRAM_BYTES - 1) &&
	    afe_write(AFE_VUL12_CON0, 0x0f000b23u, (10u << 24) | (1u << 5)) &&
	    afe_write(AFE_DAC_CON0, 1u << 31, 1u << 31);
	*ring = (volatile uint32_t *)va(UL_SRAM);
	*bytes = SRAM_BYTES;
	printf("audio: afe: VUL12 %s (%u bytes of SRAM at %#x)\n", ok ? "runs" : "did NOT start", SRAM_BYTES, UL_SRAM);
	return ok;
}

/* Where the hardware writes, in bytes from the ring's start (16-byte steps). */
uint32_t
mt6781_afe_ul_position(void)
{
	uint32_t cur = rd(AFE_VUL12_CUR);

	return cur >= UL_SRAM && cur < UL_SRAM + SRAM_BYTES ? (cur - UL_SRAM) & ~15u : 0;
}

/* Android's UL power-down: the source off, its pins back to GPIO, the pad FIFO and the ADC clock off, VUL12 off. */
void
mt6781_afe_ul_stop(void)
{
	if (!afe_open || !(rd(AFE_DAC_CON0) & (1u << 31) || rd(AFE_ADDA_UL_SRC_CON0) & 1u)) {
		return;
	}
	(void)afe_write(AFE_ADDA_UL_SRC_CON0, 1u, 0);
	delay(125);
	(void)afe_write(GPIO_MODE_184_MWR, 0xff00000, 0x8800000);
	(void)afe_write(AFE_AUD_PAD_TOP, 1u, 0);
	(void)afe_write(AUDIO_TOP_CON0, 1u << 24, 1u << 24);
	(void)afe_write(AFE_DAC_CON0, 1u << 31, 0);
	(void)afe_write(AFE_CONN21, 1u << 3, 0);
	(void)afe_write(AFE_CONN22, 1u << 4, 0);
}

/* Android's power-down, in reverse: UL and DL off, pins back to GPIO, AFE off (waited for), the gates closed. */
void
mt6781_afe_down(void)
{
	if (!afe_open) {
		return;
	}
	mt6781_afe_ul_stop();
	mt6781_afe_dl1_stop();
	(void)afe_write(AFE_ADDA_DL_SRC2_CON0, 1u, 0);
	delay(125);
	(void)afe_write(GPIO_MODE_184_MWR, 0xff0, 0x880);
	(void)afe_write(AFE_ADDA_UL_DL_CON0, 1u, 0);
	(void)afe_write(AUDIO_TOP_CON0, (1u << 25) | (1u << 26), (1u << 25) | (1u << 26));
	(void)afe_write(AFE_DAC_CON0, 1u, 0);
	for (int i = 0; i < 50000 && (rd(AFE_DAC_MON) & 1); i++) {
		delay(20);
	}
	(void)afe_write(AUDIO_TOP_CON0, 1u << 2, 1u << 2);
	afe_open = false;
	(void)afe_write(INFRA_CG3_SET, 1u << 4, 1u << 4);
	(void)afe_write(INFRA_CG1_SET, 1u << 25, 1u << 25);
	printf("audio: afe: down\n");
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
