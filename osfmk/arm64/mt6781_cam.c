/*
 * MT6781: the camera's power domains, clocks and blocks, read only, for bsd/dev/mtk_cam.c's step 0; for its step 1,
 * the main camera's MCLK source (read only) and its RST and MCLK pins. SPM, infracfg_ao, topckgen, apmixedsys, GPIO
 * and IOCFG are always on; CAMSYS, SENINF and CAMSV are read only when mt6781_cam_skip() allows it, since a read of a
 * powered-off or gated block can hang the bus. Called under mtk_cam.c's lock.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/misc_protos.h>
#include <machine/machine_routines.h>
#include "mt6781_cam.h"

size_t mt6781_cam_status(char *out, size_t len);
size_t mt6781_cam_mclk_src(char *out, size_t len, bool *ok);
size_t mt6781_cam_pin(int op, char *out, size_t len, bool *ok);

static vm_offset_t top, infra, spm, camsys, seninf, camsv, gpio, iocfg, apmixed;

static uint32_t
rd(vm_offset_t base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static void
wr(vm_offset_t base, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(base + off) = v;
	__builtin_arm_dsb(0xf);
}

static void
map(void)
{
	if (top == 0) {
		top = ml_io_map(MT6781_TOPCKGEN, 0x1000);
		infra = ml_io_map(MT6781_INFRACFG_AO, 0x1000);
		spm = ml_io_map(MT6781_SPM, 0x1000);
		camsys = ml_io_map(MT6781_CAMSYS, 0x1000);
		seninf = ml_io_map(MT6781_SENINF, 0x1000);
		camsv = ml_io_map(MT6781_CAMSV, 0x1000);
		gpio = ml_io_map(MT6781_GPIO, 0x1000);
		iocfg = ml_io_map(MT6781_IOCFG_RM, 0x200);
		apmixed = ml_io_map(MT6781_APMIXED, 0x1000);
	}
}

#define say(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)

size_t
mt6781_cam_status(char *out, size_t len)
{
	struct mt6781_cam_aon a;
	const char *why;
	uint32_t cg = 0;
	size_t o = 0;

	map();
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
		uint32_t c, shift;

		mt6781_camtg_field(i, &c, &shift);
		say(" %u%s", mt6781_camtg_mhz(mt6781_cam_mux(a.cfg[c], shift, 3)), mt6781_cam_mux_pdn(a.cfg[c], shift) ? "*" : "");
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

/*
 * Step 1, read only: MCLK is a camtg mux of topckgen's on the pin's CMMCLK function; in realme's clock tree nothing
 * of CAMSYS is its parent, so step 1 turns on no power domain. ok when every camtg mux is at 24 MHz and its PLL runs.
 */
size_t
mt6781_cam_mclk_src(char *out, size_t len, bool *ok)
{
	uint32_t cfg[3], pll;
	size_t o = 0;

	map();
	for (uint32_t i = 0; i < 3; i++) {
		cfg[i] = rd(top, CAM_CLK_CFG(i));
	}
	pll = rd(apmixed, CAM_UNIVPLL_CON0);
	*ok = mt6781_camtg_24(cfg) && (pll & 1);
	say("cam: step 1: MCLK source: camtg0-6 %s (clk_cfg0 %#x 1 %#x 2 %#x), univ2pll %s (con0 %#x)\n",
	    mt6781_camtg_24(cfg) ? "24 MHz" : "NOT all 24 MHz", cfg[0], cfg[1], cfg[2], (pll & 1) ? "on" : "OFF", pll);
	return o;
}

/*
 * Step 1's pins as realme's pinctrl states for camera 0 set them: RST (GPIO20) a GPIO driven low or high; MCLK
 * (GPIO141) CMMCLK1 at 6 mA, or a GPIO at 4 mA (camera0_mclk_off, also its state before power-up). Written the
 * race-free way pinctrl-mtk-common-v2.c writes, mode first, then read back; ok when it reads as written.
 */
size_t
mt6781_cam_pin(int op, char *out, size_t len, bool *ok)
{
	static const char *const what[] = { "RST low", "RST high", "MCLK off", "MCLK on" };
	bool rst = op == MT6781_CAM_RST_LOW || op == MT6781_CAM_RST_HIGH, high = op == MT6781_CAM_RST_HIGH;
	uint32_t pin = rst ? CAM_PIN_RST : CAM_PIN_MCLK, bit = 1u << (pin % 32), mode = op == MT6781_CAM_MCLK_ON;
	uint32_t drv = op == MT6781_CAM_MCLK_ON ? CAM_MCLK_DRV_ON : CAM_MCLK_DRV_OFF, m, dir, dout, d, highs = 0;
	size_t o = 0;

	if (op < MT6781_CAM_RST_LOW || op > MT6781_CAM_MCLK_ON) {
		*ok = false;
		return o;
	}
	map();
	wr(gpio, CAM_GPIO_MODE(pin) + CAM_MWR, mt6781_cam_mwr(pin, mode));
	if (rst) {
		wr(gpio, CAM_GPIO_DOUT(pin) + (high ? CAM_SET : CAM_CLR), bit);
		wr(gpio, CAM_GPIO_DIR(pin) + CAM_SET, bit);
	} else {
		wr(iocfg, CAM_MCLK_DRV + CAM_SET, drv << CAM_MCLK_DRV_SHIFT);
		wr(iocfg, CAM_MCLK_DRV + CAM_CLR, (~drv & 7) << CAM_MCLK_DRV_SHIFT);
	}
	m = mt6781_cam_pin_mode(rd(gpio, CAM_GPIO_MODE(pin)), pin);
	dir = (rd(gpio, CAM_GPIO_DIR(pin)) & bit) != 0;
	dout = (rd(gpio, CAM_GPIO_DOUT(pin)) & bit) != 0;
	d = (rd(iocfg, CAM_MCLK_DRV) >> CAM_MCLK_DRV_SHIFT) & 7;
	for (uint32_t i = 0; i < 64; i++) {
		highs += (rd(gpio, CAM_GPIO_DIN(pin)) & bit) != 0;
	}
	*ok = m == mode && (rst ? dir && dout == high : d == drv);
	say("cam: step 1: %s (GPIO%u mode %u, %s, out %u, in high %u/64", what[op], pin, m, dir ? "out" : "in", dout, highs);
	if (!rst) {
		say(", drive %u mA", 2 + 2 * d);
	}
	say(")%s\n", *ok ? "" : ": DOES NOT READ AS WRITTEN");
	return o;
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
