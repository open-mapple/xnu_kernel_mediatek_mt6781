/*
 * MT6781: the camera's power domains, clocks and blocks, read only, for bsd/dev/mtk_cam.c's step 0. SPM, infracfg_ao
 * and topckgen are always on; CAMSYS, SENINF and CAMSV are read only when mt6781_cam_skip() allows it, since a read
 * of a powered-off or gated block can hang the bus. Called under mtk_cam.c's lock.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/misc_protos.h>
#include <machine/machine_routines.h>
#include "mt6781_cam.h"

size_t mt6781_cam_status(char *out, size_t len);

static vm_offset_t top, infra, spm, camsys, seninf, camsv;

static uint32_t
rd(vm_offset_t base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

#define say(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)

size_t
mt6781_cam_status(char *out, size_t len)
{
	struct mt6781_cam_aon a;
	const char *why;
	uint32_t cg = 0;
	size_t o = 0;

	if (top == 0) {
		top = ml_io_map(MT6781_TOPCKGEN, 0x1000);
		infra = ml_io_map(MT6781_INFRACFG_AO, 0x1000);
		spm = ml_io_map(MT6781_SPM, 0x1000);
		camsys = ml_io_map(MT6781_CAMSYS, 0x1000);
		seninf = ml_io_map(MT6781_SENINF, 0x1000);
		camsv = ml_io_map(MT6781_CAMSV, 0x1000);
	}
	a.pwr = rd(spm, CAM_PWR_STATUS);
	a.pwr2 = rd(spm, CAM_PWR_STATUS_2ND);
	a.prot = rd(infra, CAM_PROT1_STA1);
	for (uint32_t i = 0; i < 10; i++) {
		a.cfg[i] = rd(top, CAM_CLK_CFG(i));
	}
	say("cam: step 0: domains CAM %s, RAWA %s, RAWB %s, CSI %s (pwr_status %#x %#x; pwr_con CAM %#x RAWA %#x RAWB %#x "
	    "MFG4 %#x); bus protection %#x (CAM's %s)\n", mt6781_cam_on(&a, CAM_PWR_CAM) ? "on" : "off",
	    mt6781_cam_on(&a, CAM_PWR_RAWA) ? "on" : "off", mt6781_cam_on(&a, CAM_PWR_RAWB) ? "on" : "off",
	    mt6781_cam_on(&a, CAM_PWR_CSI) ? "on" : "off", a.pwr, a.pwr2, rd(spm, CAM_CAM_PWR_CON), rd(spm, CAM_RAWA_PWR_CON),
	    rd(spm, CAM_RAWB_PWR_CON), rd(spm, CAM_MFG4_PWR_CON), a.prot, (a.prot & CAM_PROT_CAM) ? "on" : "off");
	say("cam: step 0: camtg0-6 (MHz, * = pdn bit set)");
	for (uint32_t i = 0; i < 7; i++) {
		/* camtg in CLK_CFG_0 [26:24]; camtg1-4 in CLK_CFG_1, camtg5-6 in CLK_CFG_2, a byte each */
		uint32_t cfg = i == 0 ? a.cfg[0] : a.cfg[1 + (i - 1) / 4], shift = i == 0 ? 24 : 8 * ((i - 1) % 4);

		say(" %u%s", mt6781_camtg_mhz(mt6781_cam_mux(cfg, shift, 3)), mt6781_cam_mux_pdn(cfg, shift) ? "*" : "");
	}
	say("; seninf0-3 sel %u %u %u %u, camtm %u, cam_sel %u%s (clk_cfg0 %#x 1 %#x 2 %#x 7 %#x 8 %#x 9 %#x)\n",
	    mt6781_cam_mux(a.cfg[7], 8, 2), mt6781_cam_mux(a.cfg[7], 16, 2), mt6781_cam_mux(a.cfg[7], 24, 2),
	    mt6781_cam_mux(a.cfg[8], 0, 2), mt6781_cam_mux(a.cfg[8], 24, 1), mt6781_cam_mux(a.cfg[9], 8, 4),
	    mt6781_cam_mux_pdn(a.cfg[9], 8) ? "*" : "", a.cfg[0], a.cfg[1], a.cfg[2], a.cfg[7], a.cfg[8], a.cfg[9]);
	if ((why = mt6781_cam_skip(&a, 0, MT6781_CAM_CAMSYS)) != 0) {
		say("cam: step 0: CAMSYS, SENINF, CAMSV not read: %s\n", why);
		return o;
	}
	cg = rd(camsys, CAMSYS_CG_CON);
	say("cam: step 0: camsys cg %#x (1 = gated)\n", cg);
	if ((why = mt6781_cam_skip(&a, cg, MT6781_CAM_SENINF)) != 0) {
		say("cam: step 0: SENINF not read: %s\n", why);
	} else {
		say("cam: step 0: seninf_top ctrl %#x\n", rd(seninf, SENINF_TOP_CTRL));
	}
	if ((why = mt6781_cam_skip(&a, cg, MT6781_CAM_CAMSV)) != 0) {
		say("cam: step 0: CAMSV not read: %s\n", why);
	} else {
		say("cam: step 0: camsv module_en %#x tg_sen_mode %#x\n", rd(camsv, CAMSV_MODULE_EN), rd(camsv, CAMSV_TG_SEN_MODE));
	}
	return o;
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
