/*
 * viva (MT6781): MDP backend of AppleM2ScalerCSCDriver (VivaScaler.h): NV12 (420v/420f) -> BGRA at the same size on
 * the MDP engines RDMA0 -> WROT0, without the GCE command queue and without the SCP (MediaTek's userspace
 * libdpframework normally programs these engines; nothing public describes MT6781's values).
 *
 * Everything below was found by hand on the phone over the USB link (2026-11-06, gpu/docs/MDP-MT6781.md §10):
 * - MDP sits in the display's power domain (DIS): only usable while the panel is on (SPM status bit 21); when the
 *   screen goes off the domain loses the clocks and every register, so each transfer sets everything up again.
 * - clocks: MDPSYS_CG_CLR0 (0x1B000108) = 0xBC1 (RDMA0, SMI0, APB, WROT0, RSZ0, MUTEX0).
 * - larb2: CMD_THRT 0x370256, SW_FLAG 1, OSTD 6/6/6/6/1, ports 0 (RDMA0), 1 and 2 (WROT0) = 0x8 (out of the M4U:
 *   physical addresses below 4 GiB).
 * - engine reset: the system reset MMSYS_SW0_RST_B (0x1B000700) = ~0x301 then all ones. WROT's own soft reset
 *   (0x1B00A010) sticks forever when the engine never finished a frame.
 * - routing RDMA0 -> WROT0 directly: RDMA0_MOUT_EN (0x1B000F20) = 0x2, WROT0_SEL_IN (0x1B000F70) = 0.
 * - RDMA source format codes on this chip differ from the MML/mdp3 sources: SRC_CON = 0x60E for BGRA,
 *   0x201 for NV12 4:2:0 (code 1, chroma interpolated horizontally), code 5 = NV16.
 * - WROT writes BGRA with VIDO_CTRL 0xA0005002; YUV -> RGB with VIDO_MAT_CTRL (mat_sel << 4) | 1: 4 = BT.601 full
 *   range, 6 = BT.601 video, 7 = BT.709 video.
 * - start: MUTEX0 MOD0 = RDMA0 | WROT0 (0x81), SOF = 0 (single), EN = 1; end: VIDO_INT_EN = 0x300 selects the
 *   debug word 0x1B00A0D0 whose bit 0 = frame done.
 * Frames are done in vertical strips of at most 512 pixels (WROT's single-tile width, MT6893 value; MT6781's is
 * not known). Surfaces that are not physically contiguous below 4 GiB go through bounce buffers.
 *
 * 0181: BGRA -> BGRA scaling (any ratio) through the resizer, found the same way (2026-11-06 night):
 * - path RDMA0 -> PQ0 chain (HDR0 -> AAL0 -> RSZ0 -> TDSHP0) -> WROT0: RDMA0_MOUT_EN = 1, PQ0_SEL_IN (0xF30) = 0,
 *   AAL0_MOUT_EN (0xF28) = 1, PQ0_SOUT_SEL (0xF80) = 0, WROT0_SEL_IN = 1; HDR0 register 0 = 1 (its reset value 0x38382060
 *   holds the data), HDR0/AAL0/TDSHP0 in relay with their sizes; mutex MOD0 = 0x3FF; all PQ clocks on (CG 0xAFC3).
 * - RGB is converted to YUV full range in RDMA (TRANSFORM_0 = 0x10000) and back in WROT (MAT_CTRL 0x41); RSZ0 with
 *   use121filter, coefficient steps and algorithms of MediaTek's rsz_fw (6 taps up, cubic accumulation down).
 *   WROT writes a fixed alpha 0xFF: only for opaque content. Boot-arg vivamdprgb: 0 = no RGB on the MDP, 1 (default)
 *   = sources whose sampled alpha is all 0xFF, 2 = every BGRA source (alpha lost).
 * - the vertical scaler's line buffer holds ~272 output pixels: frames go in vertical strips of 256 output columns,
 *   each a frame of its own with the RSZ horizontal phase of its first column (accumulation: offset in the SUB field
 *   only; 6 taps: INT/SUB with the chroma INT at 0). Measured: 1125x48 -> 787x34 in 4 strips, max error 3 levels
 *   against a bilinear reference.
 *
 * Boot-arg vivascaler=2 enables it (VivaScaler.cpp); vivamdpmax = largest frame in pixels for the bounce buffers
 * (default 1125x2448, the screen).
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/clock.h>
#include <mach/mach_time.h>

#include "VivaScaler.h"
#include "SweetSurface.h"

#define FOURCC(a, b, c, d)      (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))
#define PF_BGRA                 FOURCC('B', 'G', 'R', 'A')
#define PF_420V                 FOURCC('4', '2', '0', 'v')
#define PF_420F                 FOURCC('4', '2', '0', 'f')

#define PA_SPM                  0x10006000ull
#define PA_MDPSYS               0x1B000000ull   /* config, mutex, larb2, RDMA0 ... WROT0: one 64 KiB window */
#define SPM_PWR_STATUS          0x16c
#define DIS_STATUS              (1u << 21)

/* mdpsys_config */
#define CG_CON0                 0x0100
#define CG_CLR0                 0x0108
#define CG_MDP                  0xbc1
#define CG_MDP_PQ               0xafc3  /* + TDSHP0, HDR0, RSZ1, AAL0 */
#define AAL0_MOUT_EN            0x0f28
#define PQ0_SEL_IN              0x0f30
#define PQ0_SOUT_SEL            0x0f80
/* PQ engines (relay) */
#define HDR0_TOP                0x7000
#define HDR0_RELAY              0x7004
#define HDR0_SIZE               0x7014
#define AAL0_EN                 0x5000
#define AAL0_CFG                0x5020
#define AAL0_SIZE               0x5030
#define AAL0_OUT_SIZE           0x5034
#define AAL0_OUT_OFFSET         0x5038
#define TDSHP0_CTRL             0xc100
#define TDSHP0_CFG              0xc110
#define TDSHP0_IN_SIZE          0xc120
#define TDSHP0_OUT_OFFSET       0xc124
#define TDSHP0_OUT_SIZE         0xc128
/* RSZ0 */
#define RSZ_CON_1               0x8004
#define RSZ_CON_2               0x8008
#define RSZ_IN_IMAGE            0x8010
#define RSZ_OUT_IMAGE           0x8014
#define RSZ_HOR_STEP            0x8018
#define RSZ_VER_STEP            0x801c
#define RSZ_LUMA_HOR_INT        0x8020
#define RSZ_LUMA_HOR_SUB        0x8024
#define RSZ_LUMA_VER_INT        0x8028
#define RSZ_LUMA_VER_SUB        0x802c
#define RSZ_CHROMA_HOR_INT      0x8030
#define RSZ_CHROMA_HOR_SUB      0x8034
#define RSZ_ETC                 0x822c
#define RST_PQ_ENGINES          0x10703u        /* reset bits: RDMA0 TDSHP0 WROT0 RSZ0 HDR0 AAL0 */
#define SW0_RST_B               0x0700
#define MOUT_RST                0x0f00
#define RDMA0_MOUT_EN           0x0f20
#define WROT0_SEL_IN            0x0f70
/* mutex 0 */
#define MUTEX0_EN               0x1020
#define MUTEX0_SOF              0x102c
#define MUTEX0_MOD0             0x1030
#define MUTEX0_MOD1             0x1034
/* larb2 */
#define LARB_CMD_THRT           0x2024
#define LARB_SW_FLAG            0x2040
#define LARB_OSTD(p)            (0x2200 + 4 * (p))
#define LARB_PORT(p)            (0x2380 + 4 * (p))
/* RDMA0 */
#define RDMA_EN                 0x3000
#define RDMA_CON                0x3020
#define RDMA_GMCIF_CON          0x3028
#define RDMA_SRC_CON            0x3030
#define RDMA_COMP_CON           0x3038
#define RDMA_MF_BKGD            0x3060
#define RDMA_MF_SRC_SIZE        0x3070
#define RDMA_MF_CLIP_SIZE       0x3078
#define RDMA_MF_OFFSET_1        0x3080
#define RDMA_SF_BKGD            0x3090
#define RDMA_SRC_END_0          0x3100
#define RDMA_SRC_END_1          0x3108
#define RDMA_SRC_OFFSET_0       0x3118
#define RDMA_SRC_OFFSET_1       0x3120
#define RDMA_SRC_OFFSET_2       0x3128
#define RDMA_TRANSFORM_0        0x3200
#define RDMA_SRC_BASE_0         0x3f00
#define RDMA_SRC_BASE_1         0x3f08
/* RSZ0 (out of the path, kept disabled) */
#define RSZ_ENABLE              0x8000
/* WROT0 */
#define WROT_CTRL               0xa000
#define WROT_MAIN_BUF           0xa008
#define WROT_INT_EN             0xa018
#define WROT_CROP_OFST          0xa020
#define WROT_TAR_SIZE           0xa024
#define WROT_OFST_ADDR          0xa02c
#define WROT_STRIDE             0xa030
#define WROT_OFST_ADDR_C        0xa038
#define WROT_STRIDE_C           0xa03c
#define WROT_CTRL_2             0xa048
#define WROT_DITHER             0xa054
#define WROT_OFST_ADDR_V        0xa068
#define WROT_STRIDE_V           0xa06c
#define WROT_EOL_SEL            0xa070
#define WROT_PREULTRA           0xa074
#define WROT_IN_SIZE            0xa078
#define WROT_ROT_EN             0xa07c
#define WROT_MAT_CTRL           0xa084
#define WROT_DEBUG              0xa0d0
#define WROT_SCAN_10BIT         0xa0dc
#define WROT_PENDING_ZERO       0xa0e0
#define WROT_BASE               0xaf00
#define WROT_BASE_C             0xaf04
#define WROT_BASE_V             0xaf08

#define MDP_TILE_W              512
#define MDP_RSZ_TILE_W          256     /* output columns per strip through RSZ0 (vertical line buffer ~272) */
#define MDP_RDMA_MAX_W          1024    /* input columns per strip (keeps the RDMA tile in what was measured) */
#define MDP_TIMEOUT_MS          100

struct viva_mdp {
	IOMemoryMap *spm, *mdp;
	IOBufferMemoryDescriptor *srcBuf, *dstBuf;
	uint64_t srcPA, dstPA;
	size_t srcSize, dstSize;
	uint64_t frames, strips, timeouts, rgbFrames, rgbAlpha;
	uint32_t rgbMode;       /* vivamdprgb: 0 off, 1 opaque sources only (sampled alpha), 2 always (alpha 0xFF) */
};

static inline uint32_t rd(IOMemoryMap *m, uint32_t off) { return *(volatile uint32_t *)(m->getVirtualAddress() + off); }
static inline void wr(IOMemoryMap *m, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(m->getVirtualAddress() + off) = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static IOMemoryMap *
mapRegs(uint64_t pa, uint32_t size)
{
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionInOut);
	IOMemoryMap *m = md != NULL ? md->map(kIOMapInhibitCache) : NULL;
	OSSafeReleaseNULL(md);
	return m;
}

static IOBufferMemoryDescriptor *
contiguous(size_t size, uint64_t *pa)
{
	IOBufferMemoryDescriptor *b = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, size, 0x00000000fffff000ull);
	if (b == NULL) {
		return NULL;
	}
	IOByteCount len = 0;
	*pa = b->getPhysicalSegment(0, &len, kIOMemoryMapperNone);
	if (*pa == 0 || len < size || *pa + size > (1ull << 32)) {
		b->release();
		return NULL;
	}
	return b;
}

static void
cacheClean(const void *p, size_t len)
{
	uintptr_t a = (uintptr_t)p & ~(uintptr_t)63, end = (uintptr_t)p + len;
	for (; a < end; a += 64) {
		__asm__ volatile ("dc civac, %0" : : "r"(a) : "memory");
	}
	__asm__ volatile ("dsb sy" ::: "memory");
}

struct viva_mdp *
viva_mdp_create(void)
{
	uint32_t maxPixels = 1125 * 2448;
	PE_parse_boot_argn("vivamdpmax", &maxPixels, sizeof(maxPixels));
	uint32_t rgbMode = 1;
	PE_parse_boot_argn("vivamdprgb", &rgbMode, sizeof(rgbMode));
	struct viva_mdp *m = (struct viva_mdp *)IOMallocZero(sizeof(*m));
	if (m == NULL) {
		return NULL;
	}
	m->spm = mapRegs(PA_SPM, 0x1000);
	m->mdp = mapRegs(PA_MDPSYS, 0x10000);
	m->rgbMode = rgbMode;
	m->srcSize = round_page((size_t)maxPixels * 4 + 4096);
	m->dstSize = round_page((size_t)maxPixels * 4 + 4096);
	m->srcBuf = contiguous(m->srcSize, &m->srcPA);
	m->dstBuf = contiguous(m->dstSize, &m->dstPA);
	if (m->spm == NULL || m->mdp == NULL || m->srcBuf == NULL || m->dstBuf == NULL) {
		OSSafeReleaseNULL(m->spm);
		OSSafeReleaseNULL(m->mdp);
		OSSafeReleaseNULL(m->srcBuf);
		OSSafeReleaseNULL(m->dstBuf);
		IOFreeType(m, struct viva_mdp);
		return NULL;
	}
	IOLog("VIVA MDP: NV12 -> BGRA and BGRA scaling on RDMA0 -> (RSZ0) -> WROT0 (bounce buffers for %u pixels)\n", maxPixels);
	return m;
}

/* Clocks, larb2, engine reset, routing: every time (the DIS domain may have been off since the last frame). */
static bool
setup(struct viva_mdp *m)
{
	if (!(rd(m->spm, SPM_PWR_STATUS) & DIS_STATUS)) {
		return false;           /* panel off: the MDP is unpowered, the CPU does it */
	}
	wr(m->mdp, CG_CLR0, CG_MDP_PQ);
	if (rd(m->mdp, CG_CON0) & CG_MDP_PQ) {
		return false;
	}
	wr(m->mdp, LARB_CMD_THRT, 0x00370256);
	wr(m->mdp, LARB_SW_FLAG, 1);
	for (uint32_t p = 0; p < 4; p++) {
		wr(m->mdp, LARB_OSTD(p), 6);
	}
	wr(m->mdp, LARB_OSTD(4), 1);
	for (uint32_t p = 0; p < 3; p++) {
		wr(m->mdp, LARB_PORT(p), 0x8);
	}
	wr(m->mdp, SW0_RST_B, ~0x301u);
	wr(m->mdp, SW0_RST_B, 0xffffffffu);
	wr(m->mdp, RSZ_ENABLE, 0);
	wr(m->mdp, MOUT_RST, 4);
	wr(m->mdp, MOUT_RST, 0);
	wr(m->mdp, RDMA0_MOUT_EN, 0x2);
	wr(m->mdp, WROT0_SEL_IN, 0);
	wr(m->mdp, MUTEX0_MOD0, 0x81);
	wr(m->mdp, MUTEX0_MOD1, 0);
	wr(m->mdp, MUTEX0_SOF, 0);
	return true;
}

/* One strip: NV12 (y, uv at physical addresses, stride ys) -> BGRA (dst, stride ds), w x h pixels. */
static bool
strip(struct viva_mdp *m, uint64_t y, uint64_t uv, uint32_t ys, uint64_t dst, uint32_t ds, uint32_t w, uint32_t h,
    uint32_t mat)
{
	wr(m->mdp, SW0_RST_B, ~0x301u);
	wr(m->mdp, SW0_RST_B, 0xffffffffu);
	wr(m->mdp, RDMA_GMCIF_CON, 0x00010071);
	wr(m->mdp, RDMA_SRC_CON, 0x201);
	wr(m->mdp, RDMA_COMP_CON, 0);
	wr(m->mdp, RDMA_CON, 0x1010);
	wr(m->mdp, RDMA_SRC_BASE_0, (uint32_t)y);
	wr(m->mdp, RDMA_SRC_BASE_1, (uint32_t)uv);
	wr(m->mdp, RDMA_SRC_END_0, (uint32_t)(y + (uint64_t)ys * (h - 1) + w - 1));
	wr(m->mdp, RDMA_SRC_END_1, (uint32_t)(uv + (uint64_t)ys * (h / 2 - 1) + w - 1));
	wr(m->mdp, RDMA_MF_BKGD, ys);
	wr(m->mdp, RDMA_SF_BKGD, ys);
	wr(m->mdp, RDMA_TRANSFORM_0, 0);
	wr(m->mdp, RDMA_SRC_OFFSET_0, 0);
	wr(m->mdp, RDMA_SRC_OFFSET_1, 0);
	wr(m->mdp, RDMA_SRC_OFFSET_2, 0);
	wr(m->mdp, RDMA_MF_SRC_SIZE, (h << 16) | w);
	wr(m->mdp, RDMA_MF_CLIP_SIZE, (h << 16) | w);
	wr(m->mdp, RDMA_MF_OFFSET_1, 0);
	wr(m->mdp, RDMA_EN, 1);
	wr(m->mdp, WROT_CTRL, 0xa0005002);
	wr(m->mdp, WROT_BASE, (uint32_t)dst);
	wr(m->mdp, WROT_BASE_C, 0);
	wr(m->mdp, WROT_BASE_V, 0);
	wr(m->mdp, WROT_STRIDE, ds);
	wr(m->mdp, WROT_STRIDE_C, 0);
	wr(m->mdp, WROT_STRIDE_V, 0);
	wr(m->mdp, WROT_CTRL_2, 0);
	wr(m->mdp, WROT_SCAN_10BIT, 0);
	wr(m->mdp, WROT_PENDING_ZERO, 0);
	wr(m->mdp, WROT_MAT_CTRL, mat);
	wr(m->mdp, WROT_DITHER, 0xff000000);
	wr(m->mdp, WROT_EOL_SEL, 0x80000000);
	wr(m->mdp, WROT_PREULTRA, 0x00060010);
	wr(m->mdp, WROT_OFST_ADDR, 0);
	wr(m->mdp, WROT_OFST_ADDR_C, 0);
	wr(m->mdp, WROT_OFST_ADDR_V, 0);
	wr(m->mdp, WROT_IN_SIZE, (h << 16) | w);
	wr(m->mdp, WROT_TAR_SIZE, (h << 16) | w);
	wr(m->mdp, WROT_CROP_OFST, 0);
	wr(m->mdp, WROT_MAIN_BUF, (w << 16) | (32 << 8));
	wr(m->mdp, WROT_ROT_EN, 1);
	wr(m->mdp, WROT_INT_EN, 0x300);
	wr(m->mdp, MUTEX0_EN, 1);
	uint64_t deadline;
	clock_interval_to_deadline(MDP_TIMEOUT_MS, kMillisecondScale, &deadline);
	bool done = false;
	while (!(done = rd(m->mdp, WROT_DEBUG) & 1) && mach_absolute_time() < deadline) {
		IODelay(20);
	}
	wr(m->mdp, WROT_ROT_EN, 0);
	wr(m->mdp, RDMA_EN, 0);
	wr(m->mdp, MUTEX0_EN, 0);
	return done;
}

/* Physical address of a whole surface when it is one contiguous range below 4 GiB, else 0. */
static uint64_t
contiguousPA(SweetSurface *s)
{
	IOByteCount len = 0;
	uint64_t pa = s->memory->getPhysicalSegment(0, &len, kIOMemoryMapperNone);
	return pa != 0 && len >= s->allocSize && pa + s->allocSize <= (1ull << 32) ? pa : 0;
}

static IOReturn
nv12Transfer(struct viva_mdp *m, SweetSurface *src, SweetSurface *dst, uint32_t sx, uint32_t sy, uint32_t sw,
    uint32_t sh, uint32_t dx, uint32_t dy, uint32_t dw, uint32_t dh, uint64_t flags)
{
	/* NV12 -> BGRA, whole surfaces, same size, no transform */
	if ((src->pixelFormat != PF_420V && src->pixelFormat != PF_420F) || src->planeCount < 2 ||
	    dst->pixelFormat != PF_BGRA || dst->bytesPerElement != 4 || (flags & kVivaScalerFlagTransformMask)) {
		return kIOReturnUnsupported;
	}
	uint32_t w = src->width & ~1u, h = src->height & ~1u;
	if (sx || sy || dx || dy || ((sw || sh) && (sw != (w << 16) || sh != (h << 16))) || ((dw || dh) && (dw != w || dh != h)) ||
	    dst->width < w || dst->height < h || w < 16 || h < 16 || w > 8190 || h > 8190) {
		return kIOReturnUnsupported;
	}
	uint32_t ys = src->planes[0].bytesPerRow, cs = src->planes[1].bytesPerRow, ds = dst->bytesPerRow;
	if (ys != cs || (ys & 15) || (ds & 15)) {
		return kIOReturnUnsupported;
	}
	size_t ySize = (size_t)ys * h, cSize = (size_t)ys * (h / 2), dSize = (size_t)ds * h;
	uint64_t spa = contiguousPA(src), dpa = contiguousPA(dst);
	uint64_t yPA, cPA, oPA;
	bool bounceIn = spa == 0, bounceOut = dpa == 0;
	if ((bounceIn && ySize + cSize > m->srcSize) || (bounceOut && dSize > m->dstSize)) {
		return kIOReturnUnsupported;
	}
	if (!setup(m)) {
		return kIOReturnNotReady;
	}
	uintptr_t sk = src->kernelAddress(), dk = dst->kernelAddress();
	if ((bounceIn && sk == 0) || (bounceOut && dk == 0)) {
		return kIOReturnNoMemory;
	}
	if (bounceIn) {
		uint8_t *b = (uint8_t *)m->srcBuf->getBytesNoCopy();
		memcpy(b, (const uint8_t *)sk + src->planes[0].offset, ySize);
		memcpy(b + ySize, (const uint8_t *)sk + src->planes[1].offset, cSize);
		cacheClean(b, ySize + cSize);
		yPA = m->srcPA;
		cPA = m->srcPA + ySize;
	} else {
		yPA = spa + src->planes[0].offset;
		cPA = spa + src->planes[1].offset;
	}
	oPA = bounceOut ? m->dstPA : dpa + dst->offset;
	if (bounceOut) {
		cacheClean(m->dstBuf->getBytesNoCopy(), dSize);
	}
	/* 420f = BT.601 full range (MDP has no BT.709 full range); 420v = BT.601 / BT.709 video by height */
	uint32_t mat = src->pixelFormat == PF_420F ? 0x41 : h >= 720 ? 0x71 : 0x61;
	bool ok = true;
	for (uint32_t x = 0; x < w && ok; x += MDP_TILE_W) {
		uint32_t tw = w - x < MDP_TILE_W ? w - x : MDP_TILE_W;
		ok = strip(m, yPA + x, cPA + x, ys, oPA + (uint64_t)x * 4, ds, tw, h, mat);
		m->strips++;
	}
	if (!ok) {
		m->timeouts++;
		IOLog("VIVA MDP: %ux%u NV12 -> BGRA timed out (WROT debug 0x%08x, RDMA status 0x%08x)\n", w, h,
		    rd(m->mdp, WROT_DEBUG), rd(m->mdp, 0x3408));
		return kIOReturnTimeout;
	}
	if (bounceOut) {
		cacheClean(m->dstBuf->getBytesNoCopy(), dSize);        /* drop stale lines */
		memcpy((uint8_t *)dk + dst->offset, m->dstBuf->getBytesNoCopy(), dSize);
		sweet_surface_cpu_wrote(dst);
	}
	m->frames++;
	return kIOReturnSuccess;
}

/* ---- BGRA scaling through RSZ0 (0181) ---------------------------------------------------------------------------- */

struct rsz_axis {
	uint32_t algo, step, sub, tbl;
};

/* MediaTek's rsz_fw: 6-tap FIR up (precision 2^15), cubic accumulation down to 1/24 (2^20), source accumulation below. */
static struct rsz_axis
rszAxis(uint32_t in, uint32_t out)
{
	struct rsz_axis a = {};
	if (out >= in) {
		if (in == out) {
			a.algo = 0; a.step = 32768; a.sub = 0; a.tbl = 7;
			return a;
		}
		a.algo = 0;
		a.step = (uint32_t)((32768ull * (in - 1)) / (out - 1));
		a.sub = (uint32_t)((32768ull * (in - 1) - (uint64_t)a.step * (out - 1)) / 2);
		a.tbl = 9;
		return a;
	}
	a.algo = in > 24 * out ? 1 : 2;
	uint64_t one = 1ull << 20;
	uint64_t step = (one * (out - 1)) / (in - 1);
	if (step * (in - 1) != one * (out - 1)) {
		step++;
	}
	a.step = (uint32_t)step;
	a.sub = (uint32_t)(((uint64_t)(in - 1) * step - (uint64_t)(out - 1) * one) / 2);
	a.tbl = 17;
	return a;
}

/* One vertical strip: input columns [ix0, ix1) of the source -> output columns [k0, k1). */
static bool
rszStrip(struct viva_mdp *m, uint64_t src, uint32_t sstride, uint32_t iw, uint32_t ih, uint64_t dst, uint32_t dstride,
    uint32_t ow, uint32_t oh, uint32_t k0, uint32_t k1)
{
	struct rsz_axis h = rszAxis(iw, ow), v = rszAxis(ih, oh);
	uint32_t ix0, ix1, hint, hsub;
	if (h.algo == 0) {
		uint64_t pos = (uint64_t)k0 * h.step + h.sub;           /* input position of output k0, 1/32768 */
		ix0 = (uint32_t)(pos >> 15) >= 2 ? (((uint32_t)(pos >> 15) - 2) & ~1u) : 0;
		uint64_t last = (uint64_t)(k1 - 1) * h.step + h.sub;
		ix1 = (uint32_t)(last >> 15) + 4;
		uint64_t off = pos - ((uint64_t)ix0 << 15);
		hint = (uint32_t)(off >> 15);
		hsub = (uint32_t)(off & 0x7fff);
	} else {
		uint64_t one = 1ull << 20;
		uint64_t first = ((uint64_t)k0 * one + h.sub) / h.step;  /* input pixel at output k0 */
		ix0 = first >= 2 ? (uint32_t)first - 2 : 0;
		uint64_t num = (uint64_t)(k1 - 1) * one + h.sub;
		ix1 = (uint32_t)((num + h.step - 1) / h.step) + 3;
		/* output-space phase of the strip's first input pixel, all in the SUB field (INT 0) */
		hint = 0;
		hsub = (uint32_t)((uint64_t)h.sub + (uint64_t)k0 * one - (uint64_t)ix0 * h.step);
	}
	if (ix1 > iw) {
		ix1 = iw;
	}
	uint32_t tw = ix1 - ix0, ow_ = k1 - k0;
	if (tw > MDP_RDMA_MAX_W) {
		return false;
	}
	wr(m->mdp, SW0_RST_B, ~RST_PQ_ENGINES);
	wr(m->mdp, SW0_RST_B, 0xffffffffu);
	wr(m->mdp, MOUT_RST, 4);
	wr(m->mdp, MOUT_RST, 0);
	/* PQ engines in relay; HDR0 and AAL0 sit before RSZ0 (input size), TDSHP0 after it (output size) */
	wr(m->mdp, TDSHP0_CTRL, 1);
	wr(m->mdp, TDSHP0_CFG, 1);
	wr(m->mdp, TDSHP0_IN_SIZE, (ow_ << 16) | oh);
	wr(m->mdp, TDSHP0_OUT_SIZE, (ow_ << 16) | oh);
	wr(m->mdp, TDSHP0_OUT_OFFSET, 0);
	wr(m->mdp, AAL0_EN, 1);
	wr(m->mdp, AAL0_CFG, 1);
	wr(m->mdp, AAL0_SIZE, (tw << 16) | ih);
	wr(m->mdp, AAL0_OUT_SIZE, (tw << 16) | ih);
	wr(m->mdp, AAL0_OUT_OFFSET, 0);
	wr(m->mdp, HDR0_RELAY, 1);
	wr(m->mdp, HDR0_SIZE, (ih << 16) | tw);
	wr(m->mdp, HDR0_TOP, 1);
	/* routing RDMA0 -> PQ0 -> WROT0 */
	wr(m->mdp, RDMA0_MOUT_EN, 1);
	wr(m->mdp, PQ0_SEL_IN, 0);
	wr(m->mdp, AAL0_MOUT_EN, 1);
	wr(m->mdp, PQ0_SOUT_SEL, 0);
	wr(m->mdp, WROT0_SEL_IN, 1);
	/* RSZ0 */
	uint32_t con1 = 0x3 | ((ow >= iw ? 1u : 0u) << 4) | (h.algo << 5) | (v.algo << 7) | (h.tbl << 16) | (v.tbl << 21) |
	    (1u << 26) | (1u << 31);
	uint32_t con2 = ((tw % 2) == 0 ? (1u << 12) : 0) | (ix0 > 0 ? (1u << 11) : 0) | (k1 == ow ? 0 : (1u << 13));
	wr(m->mdp, RSZ_ENABLE, 0x10000);
	wr(m->mdp, RSZ_ENABLE, 0);
	wr(m->mdp, RSZ_ETC, 0);
	wr(m->mdp, RSZ_CON_1, con1);
	wr(m->mdp, RSZ_CON_2, con2);
	wr(m->mdp, RSZ_HOR_STEP, h.step);
	wr(m->mdp, RSZ_VER_STEP, v.step);
	wr(m->mdp, RSZ_IN_IMAGE, (ih << 16) | tw);
	wr(m->mdp, RSZ_OUT_IMAGE, (oh << 16) | ow_);
	wr(m->mdp, RSZ_LUMA_HOR_INT, hint);
	wr(m->mdp, RSZ_LUMA_HOR_SUB, hsub);
	wr(m->mdp, RSZ_LUMA_VER_INT, 0);
	wr(m->mdp, RSZ_LUMA_VER_SUB, v.sub);
	wr(m->mdp, RSZ_CHROMA_HOR_INT, h.algo == 0 ? 0 : hint);
	wr(m->mdp, RSZ_CHROMA_HOR_SUB, hsub);
	wr(m->mdp, RSZ_ENABLE, 1);
	/* RDMA0: BGRA (code 14 on this chip) -> YUV full range */
	uint64_t s0 = src + (uint64_t)ix0 * 4;
	wr(m->mdp, RDMA_GMCIF_CON, 0x00010071);
	wr(m->mdp, RDMA_SRC_CON, 0x60e);
	wr(m->mdp, RDMA_COMP_CON, 0);
	wr(m->mdp, RDMA_CON, 0x1010);
	wr(m->mdp, RDMA_SRC_BASE_0, (uint32_t)s0);
	wr(m->mdp, RDMA_SRC_END_0, (uint32_t)(s0 + (uint64_t)sstride * (ih - 1) + (uint64_t)tw * 4 - 1));
	wr(m->mdp, RDMA_MF_BKGD, sstride);
	wr(m->mdp, RDMA_SF_BKGD, 0);
	wr(m->mdp, RDMA_TRANSFORM_0, 0x10000);
	wr(m->mdp, RDMA_SRC_OFFSET_0, 0);
	wr(m->mdp, RDMA_SRC_OFFSET_1, 0);
	wr(m->mdp, RDMA_SRC_OFFSET_2, 0);
	wr(m->mdp, RDMA_MF_SRC_SIZE, (ih << 16) | tw);
	wr(m->mdp, RDMA_MF_CLIP_SIZE, (ih << 16) | tw);
	wr(m->mdp, RDMA_MF_OFFSET_1, 0);
	wr(m->mdp, RDMA_EN, 1);
	/* WROT0: YUV full range -> BGRA, alpha 0xFF */
	wr(m->mdp, WROT_CTRL, 0xa0005002);
	wr(m->mdp, WROT_BASE, (uint32_t)(dst + (uint64_t)k0 * 4));
	wr(m->mdp, WROT_BASE_C, 0);
	wr(m->mdp, WROT_BASE_V, 0);
	wr(m->mdp, WROT_STRIDE, dstride);
	wr(m->mdp, WROT_STRIDE_C, 0);
	wr(m->mdp, WROT_STRIDE_V, 0);
	wr(m->mdp, WROT_CTRL_2, 0);
	wr(m->mdp, WROT_SCAN_10BIT, 0);
	wr(m->mdp, WROT_PENDING_ZERO, 0);
	wr(m->mdp, WROT_MAT_CTRL, 0x41);
	wr(m->mdp, WROT_DITHER, 0xff000000);
	wr(m->mdp, WROT_EOL_SEL, 0x80000000);
	wr(m->mdp, WROT_PREULTRA, 0x00060010);
	wr(m->mdp, WROT_OFST_ADDR, 0);
	wr(m->mdp, WROT_OFST_ADDR_C, 0);
	wr(m->mdp, WROT_OFST_ADDR_V, 0);
	wr(m->mdp, WROT_IN_SIZE, (oh << 16) | ow_);
	wr(m->mdp, WROT_TAR_SIZE, (oh << 16) | ow_);
	wr(m->mdp, WROT_CROP_OFST, 0);
	wr(m->mdp, WROT_MAIN_BUF, (ow_ << 16) | (32 << 8));
	wr(m->mdp, WROT_ROT_EN, 1);
	/* every engine of the chain gets the start */
	wr(m->mdp, MUTEX0_MOD0, 0x3ff);
	wr(m->mdp, MUTEX0_MOD1, 0);
	wr(m->mdp, MUTEX0_SOF, 0);
	wr(m->mdp, WROT_INT_EN, 0x300);
	wr(m->mdp, MUTEX0_EN, 1);
	uint64_t deadline;
	clock_interval_to_deadline(MDP_TIMEOUT_MS, kMillisecondScale, &deadline);
	bool done = false;
	while (!(done = rd(m->mdp, WROT_DEBUG) & 1) && mach_absolute_time() < deadline) {
		IODelay(20);
	}
	wr(m->mdp, WROT_ROT_EN, 0);
	wr(m->mdp, RDMA_EN, 0);
	wr(m->mdp, RSZ_ENABLE, 0);
	wr(m->mdp, MUTEX0_EN, 0);
	return done;
}

static IOReturn
rgbTransfer(struct viva_mdp *m, SweetSurface *src, SweetSurface *dst, uint32_t sx, uint32_t sy, uint32_t sw,
    uint32_t sh, uint32_t dx, uint32_t dy, uint32_t dw, uint32_t dh, uint64_t flags)
{
	if (src->pixelFormat != PF_BGRA || dst->pixelFormat != PF_BGRA || src->bytesPerElement != 4 || dst->bytesPerElement != 4 ||
	    (flags & kVivaScalerFlagTransformMask)) {
		return kIOReturnUnsupported;
	}
	/* source rectangle in 16.16 (integer pixels only), destination in pixels; all zero = whole surface */
	if ((sx | sy | sw | sh) & 0xffff) {
		return kIOReturnUnsupported;
	}
	uint32_t x0 = sx >> 16, y0 = sy >> 16, iw = sw >> 16, ih = sh >> 16;
	if (iw == 0 && ih == 0 && x0 == 0 && y0 == 0) {
		iw = src->width;
		ih = src->height;
	}
	if (dw == 0 && dh == 0 && dx == 0 && dy == 0) {
		dw = dst->width;
		dh = dst->height;
	}
	if (iw < 8 || ih < 8 || dw < 8 || dh < 8 || iw > 8190 || ih > 8190 || dw > 8190 || dh > 8190 ||
	    (uint64_t)x0 + iw > src->width || (uint64_t)y0 + ih > src->height ||
	    (uint64_t)dx + dw > dst->width || (uint64_t)dy + dh > dst->height || (src->bytesPerRow & 15) || (dst->bytesPerRow & 15)) {
		return kIOReturnUnsupported;
	}
	/* a strip's input must fit RDMA: output strip width * ratio */
	uint32_t tile = MDP_RSZ_TILE_W;
	while (tile > 16 && (uint64_t)tile * iw / dw + 8 > MDP_RDMA_MAX_W) {
		tile /= 2;
	}
	uint64_t spa = contiguousPA(src), dpa = contiguousPA(dst);
	size_t inBytes = (size_t)iw * 4 * ih, outBytes = (size_t)dw * 4 * dh;
	uintptr_t sk = src->kernelAddress(), dk = dst->kernelAddress();
	if (m->rgbMode == 1) {
		/* WROT writes alpha 0xFF: translucent sources stay on the GPU. A grid of ~4096 samples (every alpha of a
		 * snapshot of the screen or of an app window is 0xFF; a layer with transparent parts has whole areas below). */
		if (sk == 0) {
			return kIOReturnUnsupported;
		}
		sweet_surface_before_cpu_access(src);
		uint32_t ys = ih / 64 ? ih / 64 : 1, xs = iw / 64 ? iw / 64 : 1;
		for (uint32_t y = 0; y < ih; y += ys) {
			const uint8_t *row = (const uint8_t *)sk + src->offset + (size_t)(y0 + y) * src->bytesPerRow + (size_t)x0 * 4;
			for (uint32_t x = 0; x < iw; x += xs) {
				if (row[x * 4 + 3] != 0xff) {
					m->rgbAlpha++;
					return kIOReturnUnsupported;
				}
			}
		}
	}
	if ((spa == 0 && inBytes > m->srcSize) || (dpa == 0 && outBytes > m->dstSize)) {
		return kIOReturnUnsupported;
	}
	if (!setup(m)) {
		return kIOReturnNotReady;
	}
	if ((spa == 0 && sk == 0) || (dpa == 0 && dk == 0)) {
		return kIOReturnNoMemory;
	}
	uint64_t inPA, outPA;
	uint32_t inStride, outStride;
	if (spa != 0) {
		inPA = spa + src->offset + (uint64_t)y0 * src->bytesPerRow + (uint64_t)x0 * 4;
		inStride = src->bytesPerRow;
	} else {
		uint8_t *b = (uint8_t *)m->srcBuf->getBytesNoCopy();
		for (uint32_t r = 0; r < ih; r++) {
			memcpy(b + (size_t)r * iw * 4, (const uint8_t *)sk + src->offset + (size_t)(y0 + r) * src->bytesPerRow + (size_t)x0 * 4,
			    (size_t)iw * 4);
		}
		cacheClean(b, inBytes);
		inPA = m->srcPA;
		inStride = iw * 4;
	}
	if (dpa != 0) {
		outPA = dpa + dst->offset + (uint64_t)dy * dst->bytesPerRow + (uint64_t)dx * 4;
		outStride = dst->bytesPerRow;
	} else {
		cacheClean(m->dstBuf->getBytesNoCopy(), outBytes);
		outPA = m->dstPA;
		outStride = dw * 4;
	}
	bool ok = true;
	for (uint32_t k0 = 0; k0 < dw && ok; k0 += tile) {
		uint32_t k1 = dw - k0 < tile ? dw : k0 + tile;
		ok = rszStrip(m, inPA, inStride, iw, ih, outPA, outStride, dw, dh, k0, k1);
		m->strips++;
	}
	if (!ok) {
		m->timeouts++;
		IOLog("VIVA MDP: %ux%u -> %ux%u BGRA scale timed out (WROT debug 0x%08x, RDMA status 0x%08x)\n", iw, ih, dw, dh,
		    rd(m->mdp, WROT_DEBUG), rd(m->mdp, 0x3408));
		return kIOReturnTimeout;
	}
	if (dpa == 0) {
		cacheClean(m->dstBuf->getBytesNoCopy(), outBytes);
		const uint8_t *b = (const uint8_t *)m->dstBuf->getBytesNoCopy();
		for (uint32_t r = 0; r < dh; r++) {
			memcpy((uint8_t *)dk + dst->offset + (size_t)(dy + r) * dst->bytesPerRow + (size_t)dx * 4, b + (size_t)r * dw * 4,
			    (size_t)dw * 4);
		}
		sweet_surface_cpu_wrote(dst);
	}
	m->frames++;
	m->rgbFrames++;
	return kIOReturnSuccess;
}

IOReturn
viva_mdp_transfer(struct viva_mdp *m, SweetSurface *src, SweetSurface *dst, uint32_t sx, uint32_t sy, uint32_t sw,
    uint32_t sh, uint32_t dx, uint32_t dy, uint32_t dw, uint32_t dh, uint64_t flags)
{
	if (src->pixelFormat == PF_BGRA) {
		if (m->rgbMode == 0) {
			return kIOReturnUnsupported;
		}
		return rgbTransfer(m, src, dst, sx, sy, sw, sh, dx, dy, dw, dh, flags);
	}
	return nv12Transfer(m, src, dst, sx, sy, sw, sh, dx, dy, dw, dh, flags);
}

#endif /* SWEET */
