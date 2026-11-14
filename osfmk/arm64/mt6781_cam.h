/*
 * MT6781 camera blocks: the always-on registers that say whether CAMSYS, SENINF and CAMSV may be read, the camtg
 * (sensor MCLK) muxes' rates, and the main camera's RST and MCLK pins. For osfmk/arm64/mt6781_cam.c and
 * tools/mt6781_cam_test.c. Facts from realme's clk-mt6781.c, clk-mt6781-pg.c, mt6781.dts, seninf_clk.h,
 * pinctrl-mt6781.c, pinctrl-mtk-common-v2.c and cust_mt6781_camera_space.dtsi.
 */
#ifndef MT6781_CAM_H
#define MT6781_CAM_H
#include <stdbool.h>
#include <stdint.h>

#define MT6781_TOPCKGEN         0x10000000u
#define MT6781_INFRACFG_AO      0x10001000u
#define MT6781_SPM              0x10006000u
#define MT6781_CAMSYS           0x1a000000u
#define MT6781_SENINF           0x1a004000u     /* seninf_top */
#define MT6781_CAMSV            0x1a092000u     /* the device tree's camsv3, the first CAMSV */
#define MT6781_GPIO             0x10005000u
#define MT6781_IOCFG_RM         0x10002a00u
#define MT6781_APMIXED          0x1000c000u

#define CAM_CLK_CFG(n)          (0x040u + 0x10u * (n))
#define CAM_PROT1_STA1          0x258u          /* infracfg_ao: bus protection acks */
#define CAM_PWR_STATUS          0x16cu
#define CAM_PWR_STATUS_2ND      0x170u
#define CAM_MFG4_PWR_CON        0x318u
#define CAM_CAM_PWR_CON         0x35cu
#define CAM_RAWA_PWR_CON        0x360u
#define CAM_RAWB_PWR_CON        0x364u
#define CAM_PWR_CSI             (1u << 6)       /* SCP_SYS_CSI is MFG4's domain */
#define CAM_PWR_CAM             (1u << 23)
#define CAM_PWR_RAWA            (1u << 24)
#define CAM_PWR_RAWB            (1u << 25)
#define CAM_PROT_CAM            ((1u << 13) | (1u << 14) | (1u << 21) | (1u << 22))
#define CAMSYS_CG_CON           0x000u          /* 1 = gated */
#define CAM_CG_SENINF           (1u << 8)
#define CAM_CG_CAMSV            (7u << 10)      /* camsv1-3 */
#define SENINF_TOP_CTRL         0x000u
#define CAMSV_MODULE_EN         0x010u
#define CAMSV_TG_SEN_MODE       0x130u
#define CAM_UNIVPLL_CON0        0x240u          /* bit 0: univ2pll, the camtg muxes' 24 MHz source, on */

#define CAM_PIN_RST             20u             /* the S5K JN1's RST */
#define CAM_PIN_MCLK            141u            /* mode 1: CMMCLK1, the JN1's MCLK */
#define CAM_GPIO_DIR(p)         (0x000u + 0x10u * ((p) / 32))
#define CAM_GPIO_DOUT(p)        (0x100u + 0x10u * ((p) / 32))
#define CAM_GPIO_DIN(p)         (0x200u + 0x10u * ((p) / 32))
#define CAM_GPIO_MODE(p)        (0x300u + 0x10u * ((p) / 8))
#define CAM_SET                 0x4u
#define CAM_CLR                 0x8u
#define CAM_MWR                 0xcu            /* mode: a nibble's bit 3 lets its other three bits be written */
#define CAM_MCLK_DRV            0x000u          /* iocfg_rm: GPIO141's driving [5:3], 2 mA + 2 mA a step */
#define CAM_MCLK_DRV_SHIFT      3
#define CAM_MCLK_DRV_ON         2u              /* 6 mA: the sensor's ISP_DRIVING_6MA, as Android set it */
#define CAM_MCLK_DRV_OFF        1u              /* 4 mA: camera0_mclk_off */

enum { MT6781_CAM_RST_LOW, MT6781_CAM_RST_HIGH, MT6781_CAM_MCLK_OFF, MT6781_CAM_MCLK_ON };

struct mt6781_cam_aon {
	uint32_t pwr, pwr2, prot;
	uint32_t cfg[10];               /* CLK_CFG_0..9 */
};

enum { MT6781_CAM_CAMSYS, MT6781_CAM_SENINF, MT6781_CAM_CAMSV };

static inline uint32_t
mt6781_cam_mux(uint32_t cfg, uint32_t shift, uint32_t width)
{
	return (cfg >> shift) & ((1u << width) - 1);
}

/* MediaTek's power-down bit, bit 7 of the mux's byte (Linux never sets it on this SoC) */
static inline bool
mt6781_cam_mux_pdn(uint32_t cfg, uint32_t shift)
{
	return (cfg >> (shift / 8 * 8 + 7)) & 1;
}

/* camtg_parents: clk26m, univpll_192m_d8, univpll_d3_d8, univpll_192m_d4, univpll_d3_d32, univpll_192m_d16, _d32 */
static inline uint32_t
mt6781_camtg_mhz(uint32_t sel)
{
	static const uint8_t mhz[8] = { 26, 24, 52, 48, 13, 12, 6, 0 };

	return mhz[sel & 7];
}

/* camtg in CLK_CFG_0 [26:24]; camtg1-4 in CLK_CFG_1, camtg5-6 in CLK_CFG_2, a byte each */
static inline void
mt6781_camtg_field(uint32_t i, uint32_t *cfg, uint32_t *shift)
{
	*cfg = i == 0 ? 0 : 1 + (i - 1) / 4;
	*shift = i == 0 ? 24 : 8 * ((i - 1) % 4);
}

/* All seven camtg muxes at 24 MHz and not powered down (which one feeds CMMCLK1 the sources don't say). */
static inline bool
mt6781_camtg_24(const uint32_t cfg[3])
{
	uint32_t c, shift;

	for (uint32_t i = 0; i < 7; i++) {
		mt6781_camtg_field(i, &c, &shift);
		if (mt6781_camtg_mhz(mt6781_cam_mux(cfg[c], shift, 3)) != 24 || mt6781_cam_mux_pdn(cfg[c], shift)) {
			return false;
		}
	}
	return true;
}

static inline uint32_t
mt6781_cam_mwr(uint32_t pin, uint32_t mode)
{
	return (0x8u | mode) << (4 * (pin % 8));
}

static inline uint32_t
mt6781_cam_pin_mode(uint32_t mode_reg, uint32_t pin)
{
	return (mode_reg >> (4 * (pin % 8))) & 0xf;
}

static inline bool
mt6781_cam_on(const struct mt6781_cam_aon *a, uint32_t bit)
{
	return (a->pwr & bit) && (a->pwr2 & bit);
}

/* NULL when the block may be read; else why not. cg is CAMSYS_CG_CON (not looked at for CAMSYS itself). */
static inline const char *
mt6781_cam_skip(const struct mt6781_cam_aon *a, uint32_t cg, int block)
{
	if (!mt6781_cam_on(a, CAM_PWR_CAM)) {
		return "the CAM domain is off";
	}
	if (a->prot & CAM_PROT_CAM) {
		return "CAM's bus protection is on";
	}
	if (mt6781_cam_mux_pdn(a->cfg[9], 8)) {
		return "cam_sel is powered down";
	}
	if (block == MT6781_CAM_SENINF) {
		if (!mt6781_cam_on(a, CAM_PWR_CSI)) {
			return "the CSI domain (MFG4) is off";
		}
		if (cg & CAM_CG_SENINF) {
			return "its CAMSYS gate is closed";
		}
		if (mt6781_cam_mux_pdn(a->cfg[7], 8)) {
			return "seninf_sel is powered down";
		}
	}
	if (block == MT6781_CAM_CAMSV && (cg & CAM_CG_CAMSV)) {
		return "its CAMSYS gates are closed";
	}
	return 0;
}
#endif
