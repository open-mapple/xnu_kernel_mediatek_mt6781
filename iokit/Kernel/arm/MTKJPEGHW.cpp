/*
 * MT6781: the hardware JPEG encoder (jpgenc@17030000) driven with physical addresses (MTKJPEG.h).
 *
 * Sequence validated by hand on the phone over the USB link (2026-11-05, gpu/docs/JPGENC-MT6781.md): venc_sel
 * mux = mainpll_d3, VEN MTCMOS on (SPM VEN_PWR_CON 0x10006348: PWR_ON, PWR_ON_2ND, status bit 18 in
 * 0x1000616C/0x10006170, CLK_DIS/ISO off, RST_B, SRAM_PDN off, SRAM ack bit 12 -> 0x0000000D), bus protection
 * released (infracfg 0x100012AC bits 19, 31), VENC_GCON 0x17000000 |= larb, venc, jpgenc, gals (1 = on), larb7
 * outstanding limits 5/3/1/5 and ports 9-12 (Y, CbCr, Q table, bitstream) out of the M4U (physical addresses),
 * reset, then one 256x256 NV12 at Q90: INT_STS = DONE after ~37k cycles, a complete JFIF file (FF D8 FF E0 ...)
 * of DMA_ADDR0 - DST_ADDR0 bytes that decodes on the Mac.
 *
 * Every encode goes through two physically contiguous bounce buffers below 4 GiB (source NV12/NV21 with its
 * stride and height padded to 16, destination bitstream): IOSurfaces and user buffers are not contiguous and the
 * block has no IOMMU mapping. The block does not snoop the CPU caches: the source is cleaned to memory before the
 * start, the destination dropped from the caches after DONE.
 */
#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/clock.h>
#include <mach/mach_time.h>

#include "MTKJPEG.h"

#define PA_TOPCKGEN     0x10000000ull
#define PA_INFRACFG     0x10001000ull
#define PA_SPM          0x10006000ull
#define PA_VENC_GCON    0x17000000ull
#define PA_LARB7        0x17010000ull
#define PA_JPGENC       0x17030000ull

/* topckgen */
#define CLK_CFG_UPDATE          0x008
#define CLK_CFG_9               0x0d0   /* venc_sel [2:0], set +4 / clear +8 */
#define CLK_CFG_9_SET           0x0d4
#define CLK_CFG_9_CLR           0x0d8
#define VENC_SEL_MAINPLL_D3     6
#define VENC_SEL_UPDATE         (1u << 5)
/* SPM */
#define SPM_POWERON_CONFIG      0x000
#define SPM_PROJECT_KEY         0x0b160001
#define SPM_PWR_STATUS          0x16c
#define SPM_PWR_STATUS_2ND      0x170
#define SPM_VEN_PWR_CON         0x348
#define PWR_RST_B               (1u << 0)
#define PWR_ISO                 (1u << 1)
#define PWR_ON                  (1u << 2)
#define PWR_ON_2ND              (1u << 3)
#define PWR_CLK_DIS             (1u << 4)
#define SRAM_PDN                (1u << 8)
#define SRAM_PDN_ACK            (1u << 12)
#define VEN_STATUS_BIT          (1u << 18)
/* infracfg_ao bus protection */
#define INFRA_TOPAXI_PROTECTEN_CLR      0x2ac
#define INFRA_TOPAXI_PROTECTEN_STA1     0x250
#define VEN_PROT_STEP2          (1u << 19)
#define VEN_PROT_STEP1          (1u << 31)
/* VENC_GCON: 1 = clock on */
#define VENC_CG                 0x000
#define VENC_CG_ON              ((1u << 0) | (1u << 4) | (1u << 8) | (1u << 28))
/* larb7 */
#define LARB_OSTD(port)         (0x200 + 4 * (port))
#define LARB_PORT_CTRL(port)    (0x380 + 4 * (port))
#define LARB_PORT_MMU_BITS      0xff01u
/* jpgenc */
#define JE_RSTB                 0x100
#define JE_CTRL                 0x104
#define JE_QUALITY              0x108
#define JE_BLK_NUM              0x10c
#define JE_INT_STS              0x11c
#define JE_DST_ADDR0            0x120
#define JE_DMA_ADDR0            0x124
#define JE_STALL_ADDR0          0x128
#define JE_OFFSET_ADDR          0x138
#define JE_RST_MCU_NUM          0x150
#define JE_IMG_SIZE             0x154
#define JE_TOTAL_CYCLE          0x168
#define JE_BYTE_OFFSET_MASK     0x16c
#define JE_SRC_LUMA_ADDR        0x170
#define JE_SRC_CHROMA_ADDR      0x174
#define JE_STRIDE               0x178
#define JE_IMG_STRIDE           0x17c
#define JE_CODEC_SEL            0x314
#define JE_ULTRA_THRES          0x318
#define JE_CTRL_ENABLE          (1u << 0)
#define JE_CTRL_INT_EN          (1u << 2)
#define JE_CTRL_FMT_SHIFT       3
#define JE_CTRL_START_BITS      (JE_CTRL_ENABLE | JE_CTRL_INT_EN | (1u << 20) | (1u << 29))
#define JE_INT_DONE             0x01
#define JE_INT_STALL            0x02
#define JE_INT_VCODEC           0x10

#define JE_TIMEOUT_MS           500

struct mapple_jpgenc {
	IOMemoryMap *topck, *infra, *spm, *gcon, *larb, *jpg;
	IOBufferMemoryDescriptor *src, *dst;
	uint64_t srcPA, dstPA;
	size_t srcSize, dstSize;
	bool powered;
	uint64_t encodes, failures, bytesOut, cycles;
};

static inline volatile uint32_t *
reg(IOMemoryMap *m, uint32_t off)
{
	return (volatile uint32_t *)(m->getVirtualAddress() + off);
}
static inline uint32_t rd(IOMemoryMap *m, uint32_t off) { return *reg(m, off); }
static inline void wr(IOMemoryMap *m, uint32_t off, uint32_t v) { *reg(m, off) = v; __asm__ volatile ("dsb sy" ::: "memory"); }

static IOMemoryMap *
mapRegs(uint64_t pa, uint32_t size)
{
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionInOut);
	IOMemoryMap *m = md != NULL ? md->map(kIOMapInhibitCache) : NULL;
	OSSafeReleaseNULL(md);
	return m;
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

/* VEN power domain, clocks, larb7 ports: once (the block stays on; it draws nothing while idle and clock-gated). */
static bool
powerOn(struct mapple_jpgenc *j)
{
	if (j->powered) {
		return true;
	}
	/* the parent domain (DIS, bit 21) is on whenever the display is */
	if (!(rd(j->spm, SPM_PWR_STATUS) & (1u << 21))) {
		IOLog("JPEG: display power domain off\n");
		return false;
	}
	wr(j->topck, CLK_CFG_9_CLR, 0x7);
	wr(j->topck, CLK_CFG_9_SET, VENC_SEL_MAINPLL_D3);
	wr(j->topck, CLK_CFG_UPDATE, VENC_SEL_UPDATE);
	if (!(rd(j->spm, SPM_PWR_STATUS) & VEN_STATUS_BIT) || rd(j->spm, SPM_VEN_PWR_CON) != 0xd) {
		wr(j->spm, SPM_POWERON_CONFIG, SPM_PROJECT_KEY);
		uint32_t v = rd(j->spm, SPM_VEN_PWR_CON);
		v |= PWR_ON;
		wr(j->spm, SPM_VEN_PWR_CON, v);
		v |= PWR_ON_2ND;
		wr(j->spm, SPM_VEN_PWR_CON, v);
		for (int i = 0; i < 1000 && (!(rd(j->spm, SPM_PWR_STATUS) & VEN_STATUS_BIT) ||
		    !(rd(j->spm, SPM_PWR_STATUS_2ND) & VEN_STATUS_BIT)); i++) {
			IODelay(10);
		}
		if (!(rd(j->spm, SPM_PWR_STATUS) & VEN_STATUS_BIT)) {
			IOLog("JPEG: VEN power domain did not come up (0x%08x)\n", rd(j->spm, SPM_VEN_PWR_CON));
			return false;
		}
		v &= ~PWR_CLK_DIS;
		wr(j->spm, SPM_VEN_PWR_CON, v);
		v &= ~PWR_ISO;
		wr(j->spm, SPM_VEN_PWR_CON, v);
		v |= PWR_RST_B;
		wr(j->spm, SPM_VEN_PWR_CON, v);
		v &= ~SRAM_PDN;
		wr(j->spm, SPM_VEN_PWR_CON, v);
		for (int i = 0; i < 1000 && (rd(j->spm, SPM_VEN_PWR_CON) & SRAM_PDN_ACK); i++) {
			IODelay(10);
		}
		if (rd(j->spm, SPM_VEN_PWR_CON) & SRAM_PDN_ACK) {
			IOLog("JPEG: VEN SRAM did not power up (0x%08x)\n", rd(j->spm, SPM_VEN_PWR_CON));
			return false;
		}
		wr(j->infra, INFRA_TOPAXI_PROTECTEN_CLR, VEN_PROT_STEP2);
		wr(j->infra, INFRA_TOPAXI_PROTECTEN_CLR, VEN_PROT_STEP1);
	}
	/* only now is 0x170xxxxx on the bus */
	wr(j->gcon, VENC_CG, rd(j->gcon, VENC_CG) | VENC_CG_ON);
	static const uint32_t ostd[4] = { 5, 3, 1, 5 };
	for (uint32_t p = 9; p <= 12; p++) {
		wr(j->larb, LARB_OSTD(p), ostd[p - 9]);
		wr(j->larb, LARB_PORT_CTRL(p), rd(j->larb, LARB_PORT_CTRL(p)) & ~LARB_PORT_MMU_BITS);
	}
	wr(j->jpg, JE_RSTB, 0);
	wr(j->jpg, JE_RSTB, 1);
	(void)rd(j->jpg, JE_ULTRA_THRES);
	if (rd(j->jpg, JE_DMA_ADDR0) == 0xffffffff) {
		IOLog("JPEG: jpgenc not answering on the APB bus\n");
		return false;
	}
	wr(j->jpg, JE_CODEC_SEL, 0);
	j->powered = true;
	IOLog("JPEG: jpgenc powered (VEN 0x%08x, VENC_CG 0x%08x)\n", rd(j->spm, SPM_VEN_PWR_CON), rd(j->gcon, VENC_CG));
	return true;
}

struct mapple_jpgenc *
mapple_jpgenc_create(size_t maxPixels)
{
	struct mapple_jpgenc *j = (struct mapple_jpgenc *)IOMallocZero(sizeof(*j));
	if (j == NULL) {
		return NULL;
	}
	j->topck = mapRegs(PA_TOPCKGEN, 0x1000);
	j->infra = mapRegs(PA_INFRACFG, 0x1000);
	j->spm = mapRegs(PA_SPM, 0x1000);
	j->gcon = mapRegs(PA_VENC_GCON, 0x1000);
	j->larb = mapRegs(PA_LARB7, 0x1000);
	j->jpg = mapRegs(PA_JPGENC, 0x1000);
	j->srcSize = round_page(maxPixels * 2 + 64 * 1024);    /* YUYV: 2 bytes per pixel */
	j->dstSize = round_page(maxPixels + 64 * 1024);     /* ~8 bits per pixel worst case at Q97 is far more than enough */
	j->src = contiguous(j->srcSize, &j->srcPA);
	j->dst = contiguous(j->dstSize, &j->dstPA);
	if (j->topck == NULL || j->infra == NULL || j->spm == NULL || j->gcon == NULL || j->larb == NULL || j->jpg == NULL ||
	    j->src == NULL || j->dst == NULL || !powerOn(j)) {
		IOLog("JPEG: hardware encoder unavailable\n");
		mapple_jpgenc_destroy(j);
		return NULL;
	}
	return j;
}

void
mapple_jpgenc_destroy(struct mapple_jpgenc *j)
{
	if (j == NULL) {
		return;
	}
	OSSafeReleaseNULL(j->topck);
	OSSafeReleaseNULL(j->infra);
	OSSafeReleaseNULL(j->spm);
	OSSafeReleaseNULL(j->gcon);
	OSSafeReleaseNULL(j->larb);
	OSSafeReleaseNULL(j->jpg);
	OSSafeReleaseNULL(j->src);
	OSSafeReleaseNULL(j->dst);
	IOFreeType(j, struct mapple_jpgenc);
}

static uint32_t
qualityIndex(uint32_t q)
{
	/* mtk_jpeg_set_param (MT6781): only these tables are ever chosen */
	return q >= 97 ? 0x10 : q >= 95 ? 0x3 : q >= 92 ? 0x7 : q >= 90 ? 0x2 : q >= 87 ? 0xb : q >= 84 ? 0x6 :
	       q >= 80 ? 0x1 : q >= 74 ? 0xa : q >= 64 ? 0xe : q >= 60 ? 0x0 : 0x8;
}

IOReturn
mapple_jpgenc_encode(struct mapple_jpgenc *j, const struct mapple_jpgenc_image *img, uint32_t quality, void *out,
    size_t outCap, size_t *outLen, uint32_t *cyclesOut)
{
	*outLen = 0;
	bool yuyv = img->format == kMappleJpegYUYV;
	if (img->format != kMappleJpegNV12 && img->format != kMappleJpegNV21 && !yuyv) {
		return kIOReturnUnsupported;
	}
	uint32_t w = img->width, h = img->height;
	if (w < 16 || h < 16 || w > 8192 || h > 8192) {
		return kIOReturnUnsupported;
	}
	/* MCU grid: 16x16 for 4:2:0, 16x8 for 4:2:2 (mtk_jpeg_set_param) */
	uint32_t padW = (w + 15) & ~15u, padH = yuyv ? (h + 7) & ~7u : (h + 15) & ~15u;
	uint32_t stride = yuyv ? (2 * padW + 31) & ~31u : padW;
	size_t ySize = (size_t)stride * padH, cSize = yuyv ? 0 : (size_t)stride * padH / 2;
	if (ySize + cSize > j->srcSize) {
		return kIOReturnNoSpace;
	}
	if (!powerOn(j)) {
		return kIOReturnNotReady;
	}
	/* source into the bounce buffer, edges replicated up to the MCU grid */
	uint8_t *sy = (uint8_t *)j->src->getBytesNoCopy(), *sc = sy + ySize;
	uint32_t rowBytes = yuyv ? 2 * (w & ~1u) : w;
	for (uint32_t y = 0; y < padH; y++) {
		const uint8_t *row = img->y + (size_t)(y < h ? y : h - 1) * img->yBytesPerRow;
		uint8_t *d = sy + (size_t)y * stride;
		memcpy(d, row, rowBytes);
		for (uint32_t x = rowBytes; x < stride; x += yuyv ? 4 : 1) {
			if (yuyv) {
				memcpy(d + x, row + rowBytes - 4, 4);   /* last Y0 U Y1 V group */
			} else {
				d[x] = row[w - 1];
			}
		}
	}
	uint32_t ch = h / 2;
	for (uint32_t y = 0; !yuyv && y < padH / 2; y++) {
		const uint8_t *row = img->uv + (size_t)(y < ch ? y : ch - 1) * img->uvBytesPerRow;
		uint32_t cw = w & ~1u;
		memcpy(sc + (size_t)y * stride, row, cw);
		for (uint32_t x = cw; x < padW; x += 2) {
			sc[(size_t)y * stride + x] = row[cw - 2];
			sc[(size_t)y * stride + x + 1] = row[cw - 1];
		}
	}
	cacheClean(sy, ySize + cSize);
	cacheClean(j->dst->getBytesNoCopy(), j->dstSize);
	/* reset, configure, start (mtk_jpeg_device_run) */
	wr(j->jpg, JE_RSTB, 0);
	wr(j->jpg, JE_RSTB, 1);
	wr(j->jpg, JE_CODEC_SEL, 0);
	wr(j->jpg, JE_INT_STS, 0);
	wr(j->jpg, JE_IMG_SIZE, (w << 16) | h);
	uint32_t fmt = yuyv ? 0 : img->format == kMappleJpegNV12 ? 2 : 3;
	wr(j->jpg, JE_CTRL, fmt << JE_CTRL_FMT_SHIFT);
	wr(j->jpg, JE_BLK_NUM, yuyv ? (padW / 16) * (padH / 8) * 4 - 1 : (padW / 16) * (padH / 16) * 6 - 1);
	wr(j->jpg, JE_IMG_STRIDE, stride);
	wr(j->jpg, JE_STRIDE, stride);
	wr(j->jpg, JE_SRC_LUMA_ADDR, (uint32_t)j->srcPA);
	wr(j->jpg, JE_SRC_CHROMA_ADDR, (uint32_t)(j->srcPA + (yuyv ? 0 : ySize)));
	wr(j->jpg, JE_OFFSET_ADDR, 0);
	wr(j->jpg, JE_BYTE_OFFSET_MASK, 0);
	wr(j->jpg, JE_DST_ADDR0, (uint32_t)j->dstPA);
	wr(j->jpg, JE_STALL_ADDR0, (uint32_t)(j->dstPA + j->dstSize));
	wr(j->jpg, JE_QUALITY, (rd(j->jpg, JE_QUALITY) & 0xffff0000u) | qualityIndex(quality));
	wr(j->jpg, JE_RST_MCU_NUM, 0);
	wr(j->jpg, JE_CTRL, ((fmt << JE_CTRL_FMT_SHIFT) | JE_CTRL_START_BITS) & ~(1u << 30));
	uint32_t sts = 0;
	uint64_t deadline;
	clock_interval_to_deadline(JE_TIMEOUT_MS, kMillisecondScale, &deadline);
	while (((sts = rd(j->jpg, JE_INT_STS)) & (JE_INT_DONE | JE_INT_STALL | JE_INT_VCODEC)) == 0 &&
	    mach_absolute_time() < deadline) {
		IODelay(50);
	}
	wr(j->jpg, JE_INT_STS, 0);
	uint32_t produced = rd(j->jpg, JE_DMA_ADDR0) - (uint32_t)j->dstPA;
	uint32_t cycles = rd(j->jpg, JE_TOTAL_CYCLE);
	wr(j->jpg, JE_CTRL, fmt << JE_CTRL_FMT_SHIFT);         /* stop */
	if (!(sts & JE_INT_DONE)) {
		j->failures++;
		wr(j->jpg, JE_RSTB, 0);
		wr(j->jpg, JE_RSTB, 1);
		IOLog("JPEG: encode %ux%u failed, status 0x%x (%s)\n", w, h, sts, sts == 0 ? "timeout" :
		    (sts & JE_INT_STALL) ? "output full" : "codec error");
		return sts == 0 ? kIOReturnTimeout : kIOReturnError;
	}
	cacheClean(j->dst->getBytesNoCopy(), produced);        /* drop stale lines: the block wrote memory */
	if (produced > outCap) {
		return kIOReturnNoSpace;
	}
	memcpy(out, j->dst->getBytesNoCopy(), produced);
	*outLen = produced;
	if (cyclesOut != NULL) {
		*cyclesOut = cycles;
	}
	j->encodes++;
	j->bytesOut += produced;
	j->cycles += cycles;
	return kIOReturnSuccess;
}

void
mapple_jpgenc_stats(struct mapple_jpgenc *j, uint64_t *encodes, uint64_t *failures, uint64_t *bytes, uint64_t *cycles)
{
	*encodes = j->encodes;
	*failures = j->failures;
	*bytes = j->bytesOut;
	*cycles = j->cycles;
}

#endif /* MT6781 */
