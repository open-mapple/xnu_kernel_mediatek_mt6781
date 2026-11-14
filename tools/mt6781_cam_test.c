/*
 * osfmk/arm64/mt6781_cam.h: the camtg rates, the muxes' fields, when a camera block may be read, and the main
 * camera's pin registers.
 *   cc -Wall tools/mt6781_cam_test.c -o mt6781_cam_test && ./mt6781_cam_test
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../osfmk/arm64/mt6781_cam.h"

int
main(void)
{
	struct mt6781_cam_aon a;

	assert(mt6781_camtg_mhz(1) == 24 && mt6781_camtg_mhz(0) == 26 && mt6781_camtg_mhz(6) == 6 && mt6781_camtg_mhz(7) == 0);
	/* camtg1 at CLK_CFG_1 [2:0] with its pdn at bit 7; camtg4 at [26:24], pdn bit 31; cam_sel [11:8], pdn bit 15 */
	assert(mt6781_cam_mux(0x00000081, 0, 3) == 1 && mt6781_cam_mux_pdn(0x00000081, 0));
	assert(mt6781_cam_mux(0x81000000, 24, 3) == 1 && mt6781_cam_mux_pdn(0x81000000, 24) && !mt6781_cam_mux_pdn(0x81000000, 16));
	assert(mt6781_cam_mux(0x00000f00, 8, 4) == 0xf && !mt6781_cam_mux_pdn(0x00000f00, 8) && mt6781_cam_mux_pdn(0x8000, 8));

	/* as after LK: CAM off, protected */
	memset(&a, 0, sizeof(a));
	a.prot = CAM_PROT_CAM;
	assert(strcmp(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS), "the CAM domain is off") == 0);
	/* one status register only: still off */
	a.pwr = CAM_PWR_CAM;
	assert(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS) != NULL);
	a.pwr2 = CAM_PWR_CAM;
	assert(strcmp(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS), "CAM's bus protection is on") == 0);
	a.prot = 1u << 21;
	assert(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS) != NULL);
	a.prot = ~CAM_PROT_CAM;
	a.cfg[9] = 1u << 15;
	assert(strcmp(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS), "cam_sel is powered down") == 0);
	a.cfg[9] = 0x0700;
	assert(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS) == NULL);

	/* SENINF: CSI's domain, its gate, seninf_sel */
	assert(strcmp(mt6781_cam_skip(&a, 0, MT6781_CAM_SENINF), "the CSI domain (MFG4) is off") == 0);
	a.pwr |= CAM_PWR_CSI;
	a.pwr2 |= CAM_PWR_CSI;
	assert(strcmp(mt6781_cam_skip(&a, 0x1fff, MT6781_CAM_SENINF), "its CAMSYS gate is closed") == 0);
	a.cfg[7] = 1u << 15;
	assert(strcmp(mt6781_cam_skip(&a, ~CAM_CG_SENINF, MT6781_CAM_SENINF), "seninf_sel is powered down") == 0);
	a.cfg[7] = 0;
	assert(mt6781_cam_skip(&a, ~CAM_CG_SENINF, MT6781_CAM_SENINF) == NULL);

	/* CAMSV: all three camsv gates open, nothing else of SENINF's */
	assert(mt6781_cam_skip(&a, 1u << 11, MT6781_CAM_CAMSV) != NULL);
	assert(mt6781_cam_skip(&a, CAM_CG_SENINF, MT6781_CAM_CAMSV) == NULL);
	a.pwr &= ~CAM_PWR_CSI;
	assert(mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSV) == NULL && mt6781_cam_skip(&a, 0, MT6781_CAM_SENINF) != NULL);
	/* camtg0-6: run 38's CLK_CFG_0-2, all 24 MHz; one at 26 MHz or powered down is not */
	{
		uint32_t cfg[3] = { 0x1010001, 0x1010101, 0x3000101 }, c, shift;

		assert(mt6781_camtg_24(cfg));
		mt6781_camtg_field(0, &c, &shift);
		assert(c == 0 && shift == 24);
		mt6781_camtg_field(4, &c, &shift);
		assert(c == 1 && shift == 24);
		mt6781_camtg_field(6, &c, &shift);
		assert(c == 2 && shift == 8);
		cfg[2] = 0x3000001;
		assert(!mt6781_camtg_24(cfg));
		cfg[2] = 0x3008101;
		assert(!mt6781_camtg_24(cfg));
		cfg[2] = 0x3000101;
		cfg[0] = 0x81010001;
		assert(!mt6781_camtg_24(cfg));
	}

	/* pinctrl-mt6781.c: GPIO20's mode at 0x320 [19:16], DO 0x100 / DIR 0x000 bit 20; GPIO141's mode 0x410 [23:20] */
	assert(CAM_GPIO_MODE(CAM_PIN_RST) == 0x320 && mt6781_cam_mwr(CAM_PIN_RST, 0) == 0x80000);
	assert(CAM_GPIO_DOUT(CAM_PIN_RST) == 0x100 && CAM_GPIO_DIR(CAM_PIN_RST) == 0x000 && CAM_PIN_RST % 32 == 20);
	assert(CAM_GPIO_MODE(CAM_PIN_MCLK) == 0x410 && mt6781_cam_mwr(CAM_PIN_MCLK, 1) == 0x900000);
	assert(mt6781_cam_mwr(CAM_PIN_MCLK, 0) == 0x800000 && CAM_GPIO_DIN(CAM_PIN_MCLK) == 0x240);
	assert(mt6781_cam_pin_mode(0x00100000, CAM_PIN_MCLK) == 1 && mt6781_cam_pin_mode(0x0f0fffff, CAM_PIN_MCLK) == 0);
	/* drive-strength 2 and 1 in [5:3]: 6 mA and 4 mA */
	assert((CAM_MCLK_DRV_ON << CAM_MCLK_DRV_SHIFT) == 0x10 && ((~CAM_MCLK_DRV_ON & 7) << CAM_MCLK_DRV_SHIFT) == 0x28);
	assert((CAM_MCLK_DRV_OFF << CAM_MCLK_DRV_SHIFT) == 0x08 && ((~CAM_MCLK_DRV_OFF & 7) << CAM_MCLK_DRV_SHIFT) == 0x30);
	printf("mt6781_cam: ok\n");
	return 0;
}
