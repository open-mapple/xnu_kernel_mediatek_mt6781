/*
 * osfmk/arm64/mt6781_cam.h: the camtg rates, the muxes' fields and when a camera block may be read.
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
	printf("mt6781_cam: ok\n");
	return 0;
}
