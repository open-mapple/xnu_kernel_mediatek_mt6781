/*
 * MT6781 camera blocks: the always-on registers that say whether CAMSYS, SENINF and CAMSV may be read, and the camtg
 * (sensor MCLK) muxes' rates. For osfmk/arm64/mt6781_cam.c and tools/mt6781_cam_test.c.
 * Facts from realme's clk-mt6781.c, clk-mt6781-pg.c, mt6781.dts and seninf_clk.h.
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
