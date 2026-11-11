/*
 * MT6781: CPU DVFS for the two clusters (6x Cortex-A55 "LL", cpu 0-5; 2x Cortex-A76 "big", cpu 6-7) and the
 * CCI, plus the scheduler hint that sends foreground threads to the A76s once they run fast.
 *
 * Nothing in this port touched the CPU clocks before: they stayed where LK left them (LL 1866 MHz, big 1085 MHz,
 * CCI 612 MHz), so the A76s ran slower than the A55s. Stock Android does DVFS through the MCUPM firmware (hybrid
 * cpufreq, CSRAM tables at 0x11bc00); that firmware protocol is not used here. This is the AP-side sequence of
 * Linux's cpufreq_v1 (mtk_cpufreq_main.c set_cur_freq, mach/mt6781/mtk_cpufreq_platform.c, Xiaomi viva-r-oss):
 *
 *   rails  big = Vproc11 (MT6358 BUCK_VPROC11_ELR0 0x13a6, readback DBG0 0x139e), LL + CCI = Vproc12 (ELR0 0x1426,
 *          DBG0 0x141e); 500 mV + 6.25 mV * vsel. Vsram_proc11/12 follow in hardware (TRACKING_CON0 0x1b64/0x1b6c =
 *          on, +100 mV, clamped to 850..1118.75 mV), so only Vproc is written.
 *   PLLs   ARMPLL_LL_CON1 0x1000c204, ARMPLL_BL_CON1 0x1000c214, CCIPLL_CON1 0x1000c2a4: PCW bits 21:0
 *          (= MHz * posdiv * 2^14 / 26), posdiv bits 26:24 (0 = /1, 1 = /2, 2 = /4), bit 31 PCW change. FHCTL
 *          hopping is off (0x1000ce00 = 0), so CON1 is the control. Clock dividers MCUCFG 0x0c53a2a0/a4/e0 must read
 *          ckdiv 8 (/1) and mux 1 (ARMPLL): checked, never written.
 *   order  up: Vproc first (wait for the readback), then the PLL; down: the PLL first, then Vproc. posdiv grows
 *          before the PCW write and shrinks after it, as in Linux.
 *
 * The OPP tables are this phone's own (live DTB dvfsp@11bc00 L-table / B-table / CCI-table, which LK fills from the
 * chip's efuse bin): {MHz, vsel, posdiv}. Index 0 is the fastest.
 *
 * Governor (every 16 ms, 100 ms when everything idles at the floor): busy = 1 - idle/wall per CPU from recount, the
 * busiest CPU of a cluster counts. >= 80 % busy: the ceiling; else the slowest OPP that would be <= 50 % busy (0183;
 * 70 % before); lower OPPs only after 3 samples in a row. The CCI follows the faster cluster's OPP index. Ceilings:
 * boot-args, Low Power Mode (mt6781_cpufreq_set_lowpower, BQ27Z561Battery.cpp), thermal (mt6781_cpufreq_set_thermal_cap).
 *
 * UI boost (0183, gpu/docs/ANALISI-PRESTAZIONI-2026-11-06.md P1): a UI thread that works 8 ms of every 33 is 25 %
 * busy and never moved the clusters off the floor (Settings scrolled at 23 fps, 31 with the clocks at max). The
 * display (every SwapEnd, MappleFramebuffer.cpp) and the touch screen (every report, GT9896.cpp) call
 * mt6781_cpufreq_ui_activity(); for mp_cpufreq_boost_ms (250) after the last call LL and big run at least at
 * mp_cpufreq_boost=MHz (default: their fastest OPP), still under every ceiling (thermal 0147, Low Power Mode, link).
 * The first signal after a quiet period samples at once instead of waiting up to 100 ms. mp_cpufreq_boost_ms=0 = off.
 * While boosted with the A76 preference on (sched_mapple_fast_cpus), big never runs slower than 1.4 x an A55 at LL's
 * clock / 1.46 (the A76 does ~1.46x the work per clock): a UI thread sent to an A76 must not be slower than on an A55.
 * If the big ceiling does not allow that, the preference is suspended until it does.
 *
 * Boot-args: mp_cpufreq=0 (never touch clocks/rails), 1 (governor, default), 2 (manual: link only);
 *   mp_cpufreq_big=MHz / mp_cpufreq_ll=MHz / mp_cpufreq_cci=MHz ceilings (default 2050/2000/1400); mp_lpm_cpu=MHz big ceiling
 *   in Low Power Mode (1530; LL 1500, CCI 1050); mp_cpufreq_fast=0 no scheduler preference for the A76s.
 * Thermal (0147): the SoC's LVTS sensors (thermal controller 0x1100b000, 3 controllers at +0/+0x100/+0x200:
 *   LL x3 + CAM, big x2, MD + GPU x2), brought up as Linux's mtk_ts_cpu_noBank probe (mt6781 mtk_lvts_tc.c,
 *   AUTO-RCK): thermal clock on (INFRA_AO CG bit 10), controller reset, device id 0x81+n, device init, efuse
 *   calibration (devinfo of this phone: golden 60, count_r per sensor below), 20 ms periodic sampling.
 *   T = -204.65 * raw / 16384 + golden / 2 + 204.65 (degrees). Read every 256 ms by the governor. Policy on the
 *   hottest sensor: >= mp_therm_trip (85 C) one OPP lower for both clusters every sample, below trip - 5 C one OPP
 *   up every second, >= 105 C straight to the floor. Without valid readings the clusters are capped at opp 6
 *   (LL 1618, big 1670). The controller's hardware reset protection (TOPRGU thermal mode) is NOT armed.
 *   mp_therm=0 = no LVTS (cap at opp 6).
 * USB link: cpu [status] | cpu gov on|off | cpu set ll|big|cci MHz (governor off first) | cpu max ll|big|cci MHz |
 *   cpu fast on|off | cpu boost on|off | cpu reset (counters).
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdint.h>
#include <string.h>
#include <kern/misc_protos.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread_call.h>
#include <kern/processor.h>
#include <kern/recount.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern int mtk_pwrap_read(uint32_t addr, uint32_t *value);                          /* MTKPwrap.cpp */
extern int mtk_pwrap_write_regulator(uint32_t addr, uint32_t mask, uint32_t value);
extern uint64_t sched_mapple_fast_cpus;                                                /* kern/sched_prim.c */

#define NOPP 16
struct mapple_opp {
	uint16_t mhz;
	uint8_t vsel;
	uint8_t pd;
};

static const struct mapple_opp kOppLL[NOPP] = {
	{ 2000, 85, 1 }, { 1933, 81, 1 }, { 1866, 77, 1 }, { 1800, 72, 1 }, { 1733, 68, 1 }, { 1666, 64, 2 },
	{ 1618, 60, 2 }, { 1500, 57, 2 }, { 1375, 52, 2 }, { 1275, 48, 2 }, { 1175, 44, 2 }, { 1075, 40, 2 },
	{ 975, 36, 2 }, { 875, 32, 2 }, { 774, 28, 4 }, { 500, 16, 4 },
};
static const struct mapple_opp kOppBig[NOPP] = {
	{ 2050, 99, 1 }, { 1986, 95, 1 }, { 1923, 90, 1 }, { 1860, 86, 1 }, { 1796, 82, 1 }, { 1733, 77, 1 },
	{ 1670, 73, 1 }, { 1530, 66, 2 }, { 1419, 60, 2 }, { 1308, 55, 2 }, { 1169, 48, 2 }, { 1085, 44, 2 },
	{ 1002, 39, 2 }, { 919, 35, 2 }, { 835, 31, 2 }, { 774, 28, 4 },
};
static const struct mapple_opp kOppCCI[NOPP] = {
	{ 1400, 85, 2 }, { 1330, 79, 2 }, { 1260, 72, 2 }, { 1190, 65, 2 }, { 1155, 62, 2 }, { 1120, 58, 2 },
	{ 1050, 55, 2 }, { 980, 51, 2 }, { 927, 48, 2 }, { 875, 45, 2 }, { 822, 43, 2 }, { 752, 39, 4 },
	{ 682, 35, 4 }, { 612, 31, 4 }, { 560, 28, 4 }, { 500, 16, 4 },
};

enum { DOM_LL, DOM_BIG, DOM_CCI, NDOM };
static const char *const kDomName[NDOM] = { "ll", "big", "cci" };
static const struct mapple_opp *const kOpp[NDOM] = { kOppLL, kOppBig, kOppCCI };
static const uint64_t kPllCon1[NDOM] = { 0x1000c204, 0x1000c214, 0x1000c2a4 };
static const uint64_t kCkDiv[NDOM] = { 0x0c53a2a0, 0x0c53a2a4, 0x0c53a2e0 };
#define FHCTL_HP_EN     0x1000ce00
#define VPROC11_ELR0    0x13a6
#define VPROC11_DBG0    0x139e
#define VPROC12_ELR0    0x1426
#define VPROC12_DBG0    0x141e
#define VSRAM11_TRACK0  0x1b64
#define VSRAM12_TRACK0  0x1b6c
#define FLOOR_IDX       14      /* LL 774, big 835, CCI 560: the lowest OPPs are not used */
#define LL_FIRST        0
#define LL_LAST         5
#define BIG_FIRST       6
#define BIG_LAST        7

static LCK_GRP_DECLARE(mt6781_cpufreq_grp, "mt6781_cpufreq");
static LCK_MTX_DECLARE(mt6781_cpufreq_lock, &mt6781_cpufreq_grp);

static int gMode = 1;           /* mp_cpufreq */
static bool gReady;             /* init passed its checks */
static bool gGov;               /* governor running */
static bool gFast = true;       /* mp_cpufreq_fast */
static bool gLowPower;
static int gIdx[NDOM];          /* current OPP index */
static int gBootIdx[NDOM];
static int gMaxIdx[NDOM];       /* boot-arg / link ceiling (index: smaller = faster) */
static int gLpmIdx[NDOM];       /* Low Power Mode ceiling */
static int gThermIdx = 0;       /* thermal ceiling for big and LL (0 = none) */
static int gDown[NDOM];
static unsigned gBusy[NDOM];    /* last window, per mille, busiest CPU */
static uint64_t gIdle[MAX_SCHED_CPUS], gWall[MAX_SCHED_CPUS];
static uint64_t gSteps, gVoltWrites, gErrors, gSamples;
static uint64_t gTimeAt[NDOM][NOPP];   /* mach ticks spent at each OPP */
static uint64_t gLastAt;
static const char *gWhy = "not started";
static thread_call_t gCall;
/* 0183 UI boost */
static bool gBoost = true;              /* mp_cpufreq_boost_ms != 0, "cpu boost on|off" */
static int gBoostIdx[NDOM];             /* mp_cpufreq_boost: LL and big at least this fast while boosted */
static uint64_t gBoostAbs;              /* mp_cpufreq_boost_ms in mach time */
static volatile uint64_t gBoostUntil;   /* written without the lock by mt6781_cpufreq_ui_activity */
static volatile bool gBoosting;         /* the last sample was boosted */
static volatile bool gSlowPoll;         /* the governor sleeps 100 ms */
static uint64_t gBoostSignals, gBoostStarts, gBoostTime, gBoostLastTick;
static bool gFastSuspended;             /* A76 preference off: big cannot be fast enough (see the top) */
static uint64_t gFastSuspends;

static inline uint32_t rd(uint64_t pa) { return ml_phys_read_word_64(pa); }
static void therm_tick(void);
static inline void wr(uint64_t pa, uint32_t v) { ml_phys_write_word_64(pa, v); }

static unsigned
pll_mhz(int d)
{
	uint32_t con1 = rd(kPllCon1[d]);
	uint32_t sel = (con1 >> 24) & 7;
	return (unsigned)((((uint64_t)(con1 & 0x3fffff) * 26) >> 14) >> (sel > 4 ? 4 : sel));
}

static bool
ckdiv_ok(int d)
{
	uint32_t v = rd(kCkDiv[d]);
	return ((v >> 17) & 0x1f) == 8 && ((v >> 9) & 3) == 1;
}

static int
vproc_read(int rail11, uint32_t *vsel)
{
	uint32_t v = 0;
	int err = mtk_pwrap_read(rail11 ? VPROC11_DBG0 : VPROC12_DBG0, &v);
	*vsel = v & 0x7f;
	return err;
}

/* Writes Vproc and waits until the buck reports it (DA_VSEL), plus the slew time at ~5 mV/us. */
static bool
vproc_set(int rail11, uint32_t vsel)
{
	uint32_t old = 0, now = 0;
	if (vsel < 16 || vsel > (rail11 ? 99u : 85u) || vproc_read(rail11, &old) != 0) {
		return false;
	}
	if (old == vsel) {
		return true;
	}
	gVoltWrites++;
	if (mtk_pwrap_write_regulator(rail11 ? VPROC11_ELR0 : VPROC12_ELR0, 0x7f, vsel) != 0) {
		return false;
	}
	for (int i = 0; i < 200; i++) {
		if (vproc_read(rail11, &now) == 0 && now == vsel) {
			uint32_t dmv = (vsel > old ? vsel - old : old - vsel) * 625 / 100;
			delay(10 + dmv / 5);
			return true;
		}
		delay(10);
	}
	return false;
}

/* Linux adjust_posdiv / adjust_armpll_dds order: posdiv up before the PCW write, down after it. */
static void
pll_set(int d, const struct mapple_opp *o)
{
	uint64_t pa = kPllCon1[d];
	uint32_t con1 = rd(pa);
	uint32_t cur = (con1 >> 24) & 7, sel = o->pd == 1 ? 0 : o->pd == 2 ? 1 : 2;
	if (cur < sel) {
		wr(pa, (con1 & ~(7u << 24)) | (sel << 24));
		delay(1);
	}
	uint32_t dds = (((uint32_t)o->mhz * o->pd) << 14) / 26;
	con1 = rd(pa);
	wr(pa, (con1 & ~0x3fffffu) | (dds & 0x3fffff) | (1u << 31));
	delay(20);
	if (cur > sel) {
		con1 = rd(pa);
		wr(pa, (con1 & ~(7u << 24)) | (sel << 24));
		delay(1);
	}
}

static uint32_t
vsel12_for(const int idx[NDOM])
{
	uint32_t a = kOppLL[idx[DOM_LL]].vsel, b = kOppCCI[idx[DOM_CCI]].vsel;
	return a > b ? a : b;
}

static void
account_time(void)
{
	uint64_t now = mach_absolute_time();
	if (gLastAt != 0) {
		for (int d = 0; d < NDOM; d++) {
			gTimeAt[d][gIdx[d]] += now - gLastAt;
		}
	}
	gLastAt = now;
}

/* Moves all three domains to idx[] with the rail order above. Caller holds the lock. */
static bool
apply(const int want[NDOM])
{
	int idx[NDOM];
	bool change = false;
	for (int d = 0; d < NDOM; d++) {
		idx[d] = want[d] < 0 ? 0 : want[d] >= NOPP ? NOPP - 1 : want[d];
		change |= idx[d] != gIdx[d];
	}
	if (!change) {
		return true;
	}
	account_time();
	uint32_t v11 = kOppBig[idx[DOM_BIG]].vsel, v12 = vsel12_for(idx);
	uint32_t c11 = kOppBig[gIdx[DOM_BIG]].vsel, c12 = vsel12_for(gIdx);
	if ((v11 > c11 && !vproc_set(1, v11)) || (v12 > c12 && !vproc_set(0, v12))) {
		gErrors++;
		gWhy = "Vproc raise failed: frequencies unchanged";
		return false;
	}
	for (int d = 0; d < NDOM; d++) {
		if (idx[d] != gIdx[d]) {
			pll_set(d, &kOpp[d][idx[d]]);
			gIdx[d] = idx[d];
			gSteps++;
		}
	}
	if ((v11 < c11 && !vproc_set(1, v11)) || (v12 < c12 && !vproc_set(0, v12))) {
		gErrors++;      /* the rail stays higher than needed: safe */
		gWhy = "Vproc lower failed (rail left high)";
	}
	return true;
}

static int
ceiling(int d)
{
	int c = gMaxIdx[d];
	if (gLowPower && gLpmIdx[d] > c) {
		c = gLpmIdx[d];
	}
	if (d != DOM_CCI && gThermIdx > c) {
		c = gThermIdx;
	}
	return c > FLOOR_IDX ? FLOOR_IDX : c;
}

static int
idx_for_mhz(int d, unsigned mhz)
{
	for (int i = NOPP - 1; i > 0; i--) {
		if (kOpp[d][i].mhz >= mhz) {
			return i;
		}
	}
	return 0;
}

static void
update_fast_map(void)
{
	uint64_t m = 0;
	if (gFast && gGov && !gFastSuspended && kOppBig[ceiling(DOM_BIG)].mhz >= 1530) {
		for (int c = BIG_FIRST; c <= BIG_LAST; c++) {
			if (processor_array[c] != PROCESSOR_NULL) {
				m |= 1ull << c;
			}
		}
	}
	sched_mapple_fast_cpus = m;
}

/* Busy per mille of the busiest CPU in [first, last] since the last sample. */
static unsigned
cluster_busy(int first, int last)
{
	unsigned best = 0;
	uint64_t now = mach_absolute_time();
	for (int c = first; c <= last; c++) {
		processor_t p = processor_array[c];
		if (p == PROCESSOR_NULL) {
			continue;
		}
		struct recount_usage usage = { 0 };
		uint64_t idle = 0;
		recount_processor_usage(&p->pr_recount, &usage, &idle);
		uint64_t dw = now - gWall[c], di = idle - gIdle[c];
		gWall[c] = now;
		gIdle[c] = idle;
		if (dw == 0 || di > dw) {
			continue;
		}
		unsigned b = (unsigned)(1000 - di * 1000 / dw);
		best = b > best ? b : best;
	}
	return best;
}

static int
gov_target(int d, unsigned busy)
{
	int cap = ceiling(d);
	if (busy >= 800) {
		return cap;
	}
	unsigned need = kOpp[d][gIdx[d]].mhz * busy / 500;
	int t = idx_for_mhz(d, need);
	if (t < cap) {
		t = cap;
	}
	if (t > FLOOR_IDX) {
		t = FLOOR_IDX;
	}
	return t;
}

static void
gov_tick(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	bool slow = false;
	lck_mtx_lock(&mt6781_cpufreq_lock);
	if (gGov && gReady) {
		int want[NDOM];
		gSamples++;
		therm_tick();
		gBusy[DOM_LL] = cluster_busy(LL_FIRST, LL_LAST);
		gBusy[DOM_BIG] = cluster_busy(BIG_FIRST, BIG_LAST);
		uint64_t now = mach_absolute_time();
		bool boost = gBoost && now < gBoostUntil;
		if (boost) {
			gBoostStarts += !gBoosting;
			gBoostTime += gBoosting && gBoostLastTick != 0 ? now - gBoostLastTick : 0;
		}
		gBoosting = boost;
		gBoostLastTick = now;
		for (int d = DOM_LL; d <= DOM_BIG; d++) {
			int t = gov_target(d, gBusy[d]);
			if (boost && t > gBoostIdx[d]) {
				t = gBoostIdx[d];       /* UI boost: at least this fast (the ceiling below still applies) */
			}
			if (t > gIdx[d]) {              /* slower */
				if (++gDown[d] < 3) {
					t = gIdx[d];
				}
			} else {
				gDown[d] = 0;
			}
			if (t < ceiling(d)) {
				t = ceiling(d);         /* a ceiling that moved down applies at once */
			}
			want[d] = t;
		}
		/*
		 * A76 preference: big at least 1.4 x LL's clock / 1.46, or the preference is suspended. Only while boosted
		 * (0187): outside a boost it kept big at ~1920 MHz whenever LL ran fast, idle A76s included (84 C on the phone).
		 */
		if (gFast && boost) {
			int need = idx_for_mhz(DOM_BIG, kOppLL[want[DOM_LL]].mhz * 140 / 146);
			bool suspend = need < ceiling(DOM_BIG);
			if (!suspend && want[DOM_BIG] > need) {
				want[DOM_BIG] = need;
			}
			if (suspend != gFastSuspended) {
				gFastSuspended = suspend;
				gFastSuspends += suspend;
				update_fast_map();
			}
		}
		int fastest = want[DOM_LL] < want[DOM_BIG] ? want[DOM_LL] : want[DOM_BIG];
		want[DOM_CCI] = fastest < ceiling(DOM_CCI) ? ceiling(DOM_CCI) : fastest;
		gBusy[DOM_CCI] = gBusy[DOM_LL] > gBusy[DOM_BIG] ? gBusy[DOM_LL] : gBusy[DOM_BIG];
		if (!apply(want)) {
			gGov = false;
			update_fast_map();
			printf("CPUFREQ: %s, governor stopped\n", gWhy);
		}
		slow = !boost && gIdx[DOM_LL] == FLOOR_IDX && gIdx[DOM_BIG] == FLOOR_IDX && gBusy[DOM_CCI] < 100;
	}
	gSlowPoll = slow;
	bool again = gGov && gReady;
	lck_mtx_unlock(&mt6781_cpufreq_lock);
	if (again) {
		uint64_t deadline;
		clock_interval_to_deadline(slow ? 100 : 16, NSEC_PER_MSEC, &deadline);
		thread_call_enter_delayed(gCall, deadline);
	}
}

static void
gov_start(void)
{
	if (gGov || !gReady) {
		return;
	}
	gGov = true;
	memset(gDown, 0, sizeof(gDown));
	(void)cluster_busy(LL_FIRST, BIG_LAST);  /* baseline */
	update_fast_map();
	uint64_t deadline;
	clock_interval_to_deadline(16, NSEC_PER_MSEC, &deadline);
	thread_call_enter_delayed(gCall, deadline);
}

static void
gov_stop(void)
{
	gGov = false;
	update_fast_map();
	thread_call_cancel(gCall);
}

static bool
init_checks(void)
{
	uint32_t t11 = 0, t12 = 0, v11 = 0, v12 = 0;
	if (rd(FHCTL_HP_EN) != 0) {
		gWhy = "FHCTL hopping enabled (PLL not under CON1 control)";
		return false;
	}
	for (int d = 0; d < NDOM; d++) {
		if (!ckdiv_ok(d)) {
			gWhy = "unexpected CPU clock divider / mux";
			return false;
		}
		unsigned mhz = pll_mhz(d);
		int found = -1;
		for (int i = 0; i < NOPP; i++) {
			if (mhz + 3 >= kOpp[d][i].mhz && mhz <= kOpp[d][i].mhz + 3) {
				found = i;
			}
		}
		if (found < 0) {
			gWhy = "boot frequency not in the OPP table";
			return false;
		}
		gIdx[d] = gBootIdx[d] = found;
	}
	if (mtk_pwrap_read(VSRAM11_TRACK0, &t11) != 0 || mtk_pwrap_read(VSRAM12_TRACK0, &t12) != 0 ||
	    (t11 & 6) != 6 || (t12 & 6) != 6) {
		gWhy = "Vsram tracking off";
		return false;
	}
	if (vproc_read(1, &v11) != 0 || vproc_read(0, &v12) != 0 ||
	    v11 < kOppBig[gIdx[DOM_BIG]].vsel || v12 < vsel12_for(gIdx)) {
		gWhy = "Vproc below the table for the boot frequencies";
		return false;
	}
	return true;
}


/* ---- LVTS thermal sensors (0147) ---- */
#define THERM_BASE      0x1100b000ull
#define INFRA_CG_CLR    0x10001084
#define INFRA_RST0_SET  0x10001120
#define INFRA_RST0_CLR  0x10001124
#define LVTSMONCTL0     0x000
#define LVTSMONCTL1     0x004
#define LVTSMONCTL2     0x008
#define LVTSMSRCTL0     0x038
#define LVTSTSSEL       0x040
#define LVTSCALSCALE    0x048
#define LVTS_ID         0x04c
#define LVTS_CONFIG     0x050
#define LVTSEDATA00     0x054
#define LVTSMSR0        0x090
#define LVTSCLKEN       0x0e4
#define NTC             3
#define NSENS           9
#define THERM_NOVALID_CAP 6

static const int kTcSensors[NTC] = { 4, 2, 3 };
static const int kTcFirst[NTC] = { 0, 4, 6 };
static const char *const kSensName[NSENS] = { "ll0", "ll1", "ll2", "cam", "big0", "big1", "md", "gpu0", "gpu1" };
/* devinfo (atag,devinfo) of this phone: index 112 golden temp, 118.. count_r */
static const uint32_t kGolden = 60;
static const uint32_t kCountR[NSENS] = { 19553, 19526, 19549, 19554, 19520, 19497, 19489, 19494, 19500 };

static bool gTherm;             /* sensors initialised and valid */
static int gTempMC[NSENS];
static int gTempMax = -1000000, gTempPeak = -1000000;
static int gTrip = 85;
static unsigned gThermTicks;
static uint64_t gThermStepsDown, gThermStepsUp;
static const char *gThermWhy = "not started";

static void
lvts_dev_write(int tc, uint32_t reg, uint32_t data)
{
	wr(THERM_BASE + 0x100 * tc + LVTS_CONFIG, 0xC1030000u | ((reg & 0xff) << 8) | (data & 0xff));
	delay(3);
}

static int
lvts_raw_to_mc(uint32_t raw)
{
	return (int)((-204650ll * (int64_t)raw) / 16384) + (int)kGolden * 500 + 204650;
}

static bool
lvts_init(void)
{
	wr(INFRA_CG_CLR, 1u << 10);
	wr(INFRA_RST0_SET, rd(INFRA_RST0_SET) | 1);
	delay(10);
	wr(INFRA_RST0_CLR, rd(INFRA_RST0_CLR) | 1);
	delay(10);
	for (int tc = 0; tc < NTC; tc++) {
		uint64_t b = THERM_BASE + 0x100 * tc;
		wr(b + LVTSCLKEN, 1);
		lvts_dev_write(tc, 0xff, 0xff);                         /* reset all devices */
		wr(b + LVTS_CONFIG, 0xC5020000u | (0xfc << 8) | 0x55);  /* read back the id */
		delay(3);
		if ((rd(b + LVTS_ID) & 0xff) != 0x81u + (uint32_t)tc) {
			gThermWhy = "LVTS device id mismatch";
			return false;
		}
	}
	for (int tc = 0; tc < NTC; tc++) {
		static const uint8_t init[][2] = {
			{ 0x03, 0x00 }, { 0x0e, 0x01 }, { 0x06, 0xb8 }, { 0x07, 0xa6 }, { 0x05, 0x00 }, { 0x04, 0x20 },
			{ 0x0a, 0xac }, { 0x0c, 0xfc }, { 0x09, 0x8d }, { 0x08, 0xf7 }, { 0x0b, 0x04 },
		};
		for (unsigned i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
			lvts_dev_write(tc, init[i][0], init[i][1]);
		}
		lvts_dev_write(tc, 0x0e, 0x01);                         /* auto RCK */
		uint64_t b = THERM_BASE + 0x100 * tc;
		for (int j = 0; j < kTcSensors[tc]; j++) {
			wr(b + LVTSEDATA00 + 4 * j, kCountR[kTcFirst[tc] + j]);
		}
		wr(b + LVTSMONCTL0, 0);                                 /* sensing off while configuring */
		wr(b + LVTSTSSEL, 0x13121110);
		wr(b + LVTSCALSCALE, 0x300);
		lvts_dev_write(tc, 0x06, 0xb8);
		wr(b + LVTSMONCTL1, (1u << 20) | 0x00c);
		wr(b + LVTSMONCTL2, (1u << 16) | 0x001);
		wr(b + LVTSMSRCTL0, 0x492);
		delay(1);
		wr(b + LVTSMONCTL0, 0x200 | ((1u << kTcSensors[tc]) - 1));
	}
	gThermWhy = "waiting for the first samples";
	return true;
}

/* Reads every sensor; false while none is valid yet or a value is out of range. */
static bool
lvts_read(void)
{
	int max = -1000000, valid = 0;
	for (int tc = 0; tc < NTC; tc++) {
		for (int j = 0; j < kTcSensors[tc]; j++) {
			uint32_t v = rd(THERM_BASE + 0x100 * tc + LVTSMSR0 + 4 * j);
			int s = kTcFirst[tc] + j;
			if ((v & 0xffff) == 0) {
				continue;
			}
			gTempMC[s] = lvts_raw_to_mc(v & 0xffff);
			if (gTempMC[s] < -20000 || gTempMC[s] > 150000) {
				gThermWhy = "LVTS reading out of range";
				return false;
			}
			valid++;
			max = gTempMC[s] > max ? gTempMC[s] : max;
		}
	}
	if (valid < NSENS) {
		return false;
	}
	gTempMax = max;
	gTempPeak = max > gTempPeak ? max : gTempPeak;
	gThermWhy = "ok";
	return true;
}

/* Every 16 governor samples (~256 ms). Caller holds the lock. */
static void
therm_tick(void)
{
	if (++gThermTicks % 16 != 0) {
		return;
	}
	bool ok = gTherm && lvts_read();
	if (!ok) {
		gThermIdx = gThermIdx > THERM_NOVALID_CAP ? gThermIdx : THERM_NOVALID_CAP;
		return;
	}
	int t = gTempMax / 1000;
	if (t >= 105) {
		gThermIdx = FLOOR_IDX;
		gThermStepsDown++;
	} else if (t >= gTrip) {
		if (gThermIdx < FLOOR_IDX) {
			gThermIdx = gThermIdx == 0 ? (gIdx[DOM_BIG] < gIdx[DOM_LL] ? gIdx[DOM_BIG] : gIdx[DOM_LL]) + 1 : gThermIdx + 1;
			gThermIdx = gThermIdx > FLOOR_IDX ? FLOOR_IDX : gThermIdx;
			gThermStepsDown++;
		}
	} else if (t < gTrip - 5 && gThermIdx > 0 && gThermTicks % 64 == 0) {
		gThermIdx--;
		gThermStepsUp++;
	}
}

/*
 * 0183: the display presented a frame or the touch screen reported a contact (any thread, never blocks; not from
 * interrupt context). Boosts LL and big for mp_cpufreq_boost_ms from now (see the top).
 */
void
mt6781_cpufreq_ui_activity(void)
{
	if (!gBoost || !gGov) {
		return;
	}
	gBoostUntil = mach_absolute_time() + gBoostAbs;
	gBoostSignals++;
	if (!gBoosting && gSlowPoll && gCall != NULL) {
		gSlowPoll = false;
		thread_call_enter(gCall);       /* idle polling (100 ms): sample now */
	}
}

/* 0183: Viva Manager / "gpu kv": boost on, rising edges, time boosted (ms), activity signals. */
void
mt6781_cpufreq_boost_stats(int *on, uint64_t *starts, uint64_t *ms, uint64_t *signals)
{
	uint64_t ns = 0;
	absolutetime_to_nanoseconds(gBoostTime, &ns);
	*on = gBoost && gGov;
	*starts = gBoostStarts;
	*ms = ns / NSEC_PER_MSEC;
	*signals = gBoostSignals;
}

/* Called from MTKPwrap::start (the PMIC is reachable from then on). */
void
mt6781_cpufreq_start(void)
{
	unsigned v = 0;
	if (PE_parse_boot_argn_alias("mp_cpufreq", "vivacpufreq", &v, sizeof(v))) {
		gMode = (int)v;
	}
	if (PE_parse_boot_argn_alias("mp_cpufreq_fast", "vivacpufast", &v, sizeof(v))) {
		gFast = v != 0;
	}
	unsigned def[NDOM] = { 2000, 2050, 1400 }, lpm[NDOM] = { 1500, 1530, 1050 };
	const char *const arg[NDOM] = { "mp_cpufreq_ll", "mp_cpufreq_big", "mp_cpufreq_cci" };
	const char *const old[NDOM] = { "vivacpull", "vivacpubig", "vivacpucci" };
	for (int d = 0; d < NDOM; d++) {
		unsigned mhz = def[d];
		(void)PE_parse_boot_argn_alias(arg[d], old[d], &mhz, sizeof(mhz));
		gMaxIdx[d] = idx_for_mhz(d, mhz);
		while (gMaxIdx[d] < NOPP - 1 && kOpp[d][gMaxIdx[d]].mhz > mhz) {
			gMaxIdx[d]++;
		}
		gLpmIdx[d] = idx_for_mhz(d, lpm[d]);
		while (gLpmIdx[d] < NOPP - 1 && kOpp[d][gLpmIdx[d]].mhz > lpm[d]) {
			gLpmIdx[d]++;
		}
	}
	unsigned boostMhz = 0, boostMs = 250;
	(void)PE_parse_boot_argn_alias("mp_cpufreq_boost", "vivacpuboost", &boostMhz, sizeof(boostMhz));
	(void)PE_parse_boot_argn_alias("mp_cpufreq_boost_ms", "vivacpuboostms", &boostMs, sizeof(boostMs));
	gBoost = boostMs != 0;
	nanoseconds_to_absolutetime((uint64_t)(boostMs > 5000 ? 5000 : boostMs) * NSEC_PER_MSEC, &gBoostAbs);
	for (int d = DOM_LL; d <= DOM_BIG; d++) {
		gBoostIdx[d] = 0;
		if (boostMhz != 0) {            /* the fastest OPP not above boostMhz */
			gBoostIdx[d] = idx_for_mhz(d, boostMhz);
			while (gBoostIdx[d] < FLOOR_IDX && kOpp[d][gBoostIdx[d]].mhz > boostMhz) {
				gBoostIdx[d]++;
			}
		}
	}
	if (PE_parse_boot_argn_alias("mp_lpm_cpu", "vivalpmcpu", &v, sizeof(v))) {
		gLpmIdx[DOM_BIG] = idx_for_mhz(DOM_BIG, v);
		while (gLpmIdx[DOM_BIG] < NOPP - 1 && kOppBig[gLpmIdx[DOM_BIG]].mhz > v) {
			gLpmIdx[DOM_BIG]++;
		}
	}
	if (gMode == 0) {
		gWhy = "off (mp_cpufreq=0)";
		printf("CPUFREQ: off (mp_cpufreq=0), clocks left as LK set them\n");
		return;
	}
	unsigned th = 1;
	(void)PE_parse_boot_argn_alias("mp_therm", "vivatherm", &th, sizeof(th));
	(void)PE_parse_boot_argn_alias("mp_therm_trip", "vivathermtrip", &gTrip, sizeof(gTrip));
	gTrip = gTrip < 60 ? 60 : gTrip > 100 ? 100 : gTrip;
	gTherm = th != 0 && lvts_init();
	if (th == 0) {
		gThermWhy = "off (mp_therm=0)";
	}
	gThermIdx = THERM_NOVALID_CAP;          /* until the first valid reading */
	gCall = thread_call_allocate_with_options(gov_tick, NULL, THREAD_CALL_PRIORITY_KERNEL, THREAD_CALL_OPTIONS_ONCE);
	lck_mtx_lock(&mt6781_cpufreq_lock);
	gReady = init_checks();
	if (gReady) {
		gWhy = "ok";
		account_time();
		if (gMode == 1) {
			gov_start();
		}
	}
	lck_mtx_unlock(&mt6781_cpufreq_lock);
	printf("THERMAL: LVTS %s\n", gThermWhy);
	printf("CPUFREQ: %s; boot ll %u MHz big %u MHz cci %u MHz; %s\n", gWhy, pll_mhz(DOM_LL), pll_mhz(DOM_BIG),
	    pll_mhz(DOM_CCI), !gReady ? "disabled" : gMode == 1 ? "governor on" : "manual (link)");
}

void
mt6781_cpufreq_set_lowpower(int on)
{
	lck_mtx_lock(&mt6781_cpufreq_lock);
	gLowPower = on != 0;
	update_fast_map();
	lck_mtx_unlock(&mt6781_cpufreq_lock);
}

/* Thermal ceiling for both clusters as an OPP index (0 = none); applied at the next governor sample. */
void
mt6781_cpufreq_set_thermal_cap(int idx)
{
	gThermIdx = idx < 0 ? 0 : idx > FLOOR_IDX ? FLOOR_IDX : idx;
}

int
mt6781_cpufreq_thermal_cap(void)
{
	return gThermIdx;
}

#define SAY(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)

static int
dom_of(const char *s)
{
	for (int d = 0; d < NDOM; d++) {
		if (strncmp(s, kDomName[d], strlen(kDomName[d])) == 0) {
			return d;
		}
	}
	return -1;
}

static const char *
skip(const char *s)
{
	while (*s == ' ') {
		s++;
	}
	return s;
}

static const char *
word_end(const char *s)
{
	while (*s && *s != ' ') {
		s++;
	}
	return s;
}

void
mt6781_cpufreq_link(const char *args, char *out, size_t len)
{
	size_t o = 0;
	const char *a = skip(args ? args : "");
	out[0] = 0;
	lck_mtx_lock(&mt6781_cpufreq_lock);
	if (strncmp(a, "gov", 3) == 0) {
		const char *b = skip(word_end(a));
		if (strncmp(b, "on", 2) == 0) {
			gov_start();
		} else if (strncmp(b, "off", 3) == 0) {
			gov_stop();
		}
		SAY("governor %s\n", gGov ? "on" : "off");
	} else if (strncmp(a, "fast", 4) == 0) {
		const char *b = skip(word_end(a));
		gFast = strncmp(b, "off", 3) != 0;
		update_fast_map();
		SAY("A76 preference %s (fast cpus 0x%llx)\n", gFast ? "on" : "off", sched_mapple_fast_cpus);
	} else if (strncmp(a, "boost", 5) == 0) {
		const char *b = skip(word_end(a));
		if (strncmp(b, "on", 2) == 0) {
			gBoost = true;
			if (gBoostAbs == 0) {
				nanoseconds_to_absolutetime(250 * NSEC_PER_MSEC, &gBoostAbs);
			}
		} else if (strncmp(b, "off", 3) == 0) {
			gBoost = false;
		}
		uint64_t ns = 0;
		absolutetime_to_nanoseconds(gBoostAbs, &ns);
		SAY("UI boost %s (%llu ms, ll >= %u MHz, big >= %u MHz)\n", gBoost ? "on" : "off", ns / NSEC_PER_MSEC,
		    kOppLL[gBoostIdx[DOM_LL]].mhz, kOppBig[gBoostIdx[DOM_BIG]].mhz);
	} else if (strncmp(a, "reset", 5) == 0) {
		gBoostSignals = gBoostStarts = gBoostTime = gFastSuspends = 0;
		gSteps = gVoltWrites = gErrors = gSamples = 0;
		memset(gTimeAt, 0, sizeof(gTimeAt));
		gLastAt = 0;
		account_time();
		SAY("counters reset\n");
	} else if (strncmp(a, "set", 3) == 0 || strncmp(a, "max", 3) == 0) {
		bool set = a[0] == 's';
		const char *b = skip(word_end(a));
		int d = dom_of(b);
		const char *c = skip(word_end(b));
		unsigned mhz = 0;
		while (*c >= '0' && *c <= '9') {
			mhz = mhz * 10 + (unsigned)(*c++ - '0');
		}
		if (d < 0 || mhz == 0) {
			SAY("usage: cpu %s ll|big|cci MHz\n", set ? "set" : "max");
		} else if (!gReady) {
			SAY("not available: %s\n", gWhy);
		} else if (set && gGov) {
			SAY("governor is on: cpu gov off first\n");
		} else {
			int i = idx_for_mhz(d, mhz);
			while (i < NOPP - 1 && kOpp[d][i].mhz > mhz) {
				i++;
			}
			if (set) {
				int want[NDOM] = { gIdx[0], gIdx[1], gIdx[2] };
				want[d] = i < ceiling(d) ? ceiling(d) : i;
				bool ok = apply(want);
				SAY("%s -> %u MHz (opp %d): %s\n", kDomName[d], kOpp[d][gIdx[d]].mhz, gIdx[d], ok ? "ok" : gWhy);
			} else {
				gMaxIdx[d] = i;
				update_fast_map();
				SAY("%s ceiling %u MHz (opp %d)\n", kDomName[d], kOpp[d][i].mhz, i);
			}
		}
	} else {
		uint32_t v11 = 0, v12 = 0;
		(void)vproc_read(1, &v11);
		(void)vproc_read(0, &v12);
		account_time();
		SAY("cpufreq: %s, governor %s, mode %d, low power %s, thermal cap opp %d, A76 preference %s (0x%llx)\n",
		    gWhy, gGov ? "on" : "off", gMode, gLowPower ? "on" : "off", gThermIdx, gFast ? "on" : "off",
		    sched_mapple_fast_cpus);
		SAY("  thermal: %s, trip %d C, hottest %d.%d C (peak %d.%d C), cap opp %d, %llu steps down, %llu up\n  ",
		    gThermWhy, gTrip, gTempMax / 1000, (gTempMax % 1000 + 1000) % 1000 / 100, gTempPeak / 1000,
		    (gTempPeak % 1000 + 1000) % 1000 / 100, gThermIdx, gThermStepsDown, gThermStepsUp);
		for (int i = 0; i < NSENS; i++) {
			SAY(" %s %d.%d", kSensName[i], gTempMC[i] / 1000, (gTempMC[i] % 1000 + 1000) % 1000 / 100);
		}
		SAY("\n");
		for (int d = 0; d < NDOM; d++) {
			SAY("  %-3s %4u MHz (opp %2d, table %4u, boot %4u), ceiling %4u, busy %3u.%u %%\n", kDomName[d],
			    pll_mhz(d), gIdx[d], kOpp[d][gIdx[d]].mhz, kOpp[d][gBootIdx[d]].mhz, kOpp[d][ceiling(d)].mhz,
			    gBusy[d] / 10, gBusy[d] % 10);
		}
		{
			uint64_t bns = 0, bms = 0;
			absolutetime_to_nanoseconds(gBoostTime, &bns);
			absolutetime_to_nanoseconds(gBoostAbs, &bms);
			SAY("  UI boost %s%s (%llu ms, ll >= %u big >= %u MHz): %llu boosts, %llu ms boosted, %llu signals; "
			    "A76 preference suspended %s (%llu times)\n", gBoost ? "on" : "off", gBoosting ? ", active" : "",
			    bms / NSEC_PER_MSEC, kOppLL[gBoostIdx[DOM_LL]].mhz, kOppBig[gBoostIdx[DOM_BIG]].mhz, gBoostStarts,
			    bns / NSEC_PER_MSEC, gBoostSignals, gFastSuspended ? "now" : "no", gFastSuspends);
		}
		SAY("  Vproc11 (big) %u mV, Vproc12 (ll+cci) %u mV; %llu steps, %llu rail writes, %llu errors, %llu samples\n",
		    500 + v11 * 625 / 100, 500 + v12 * 625 / 100, gSteps, gVoltWrites, gErrors, gSamples);
		for (int d = 0; d < NDOM; d++) {
			uint64_t tot = 0;
			for (int i = 0; i < NOPP; i++) {
				tot += gTimeAt[d][i];
			}
			SAY("  time %-3s:", kDomName[d]);
			for (int i = 0; i < NOPP && tot; i++) {
				if (gTimeAt[d][i] * 100 / tot) {
					SAY(" %u:%llu%%", kOpp[d][i].mhz, gTimeAt[d][i] * 100 / tot);
				}
			}
			SAY("\n");
		}
		SAY("usage: cpu [status] | gov on|off | set ll|big|cci MHz | max ll|big|cci MHz | fast on|off | boost on|off | reset\n");
	}
	lck_mtx_unlock(&mt6781_cpufreq_lock);
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
