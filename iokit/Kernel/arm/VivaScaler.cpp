/*
 * viva (MT6781): AppleM2ScalerCSCDriver (see VivaScaler.h), the IOSurfaceAccelerator service.
 *
 * Selectors (ABI from iOS 16.7.16's IOSurfaceAccelerator.framework):
 *   1  Transfer/Transform/Blit: struct in 0x170 bytes (viva_scaler_request), nothing out. Done synchronously;
 *      with a callback and no SkipAsyncCallback the completion goes to the port registered with
 *      IOConnectSetNotificationPort(type 0) as kIOAsyncCompletionNotificationType with one argument (refcon2):
 *      QuartzCore releases the surfaces of a copy only in that callback. On an error the call fails and no
 *      notification is sent (the framework's callers call back themselves).
 *   2, 3  abort transfers / captures: nothing pending, success
 *   4  custom filter coefficients (struct 0x20): accepted and ignored (bilinear / nearest only)
 *   5  scaler id: 0
 *   7  performance stats configuration: success
 *   0, 8, 9  capture, kernel tests, histogram: kIOReturnUnsupported
 *
 * Backends: the MDP engines (VivaMDP.cpp, vivascaler=2) and the CPU. The CPU path reads the source through the
 * kernel mapping after the GPU jobs on both surfaces are done and the CPU's stale lines are dropped, writes the
 * destination and marks it CPU-dirty with a new seed, like an IOSurfaceLock/Unlock pair would (the Metal driver
 * copies surfaces in again only when the seed moved).
 *
 * Formats: source and destination BGRA/RGBA/ARGB 8888 (any of them, channel order converted), scaled with
 * bilinear or nearest filtering; source 420v/420f (NV12, video/full range, BT.601 below 720 lines, BT.709 from
 * 720) to a 32-bit destination. Flip/rotate (Transform != 0), YUV destinations and 10-bit formats are
 * refused with kIOReturnUnsupported.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>
#include <mach/mach_time.h>
#include <kern/clock.h>

#include "VivaScaler.h"
#include "SweetSurface.h"

extern "C" void iokit_release_port_send(ipc_port_t port);

static_assert(sizeof(struct viva_scaler_request) == 0x170, "selector 1 request is 0x170 bytes");

#define FOURCC(a, b, c, d)      (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))
#define PF_BGRA                 FOURCC('B', 'G', 'R', 'A')
#define PF_RGBA                 FOURCC('R', 'G', 'B', 'A')
#define PF_ARGB                 0x00000020u     /* kCVPixelFormatType_32ARGB */
#define PF_420V                 FOURCC('4', '2', '0', 'v')
#define PF_420F                 FOURCC('4', '2', '0', 'f')

#define SCALER_GPU_WAIT_MS      1000

OSDefineMetaClassAndStructors(AppleM2ScalerCSCDriver, IOService);
OSDefineMetaClassAndStructors(AppleM2ScalerCSCDriverUserClient, IOUserClient);

/* ---- Capabilities (registry) --------------------------------------------------------------------------------- */

static void
setNumber(OSDictionary *d, const char *key, uint64_t value)
{
	OSNumber *n = OSNumber::withNumber(value, 32);
	if (n != NULL) {
		d->setObject(key, n);
		n->release();
	}
}

/*
 * IOSurfaceAcceleratorCapabilitiesDict: limits QuartzCore and AppleJPEG read. Deliberately absent: ColorManager
 * (ImageIO's colour matching), HDR, Accessibility (colour remap), DirectionalScaling, HistogramValues and the
 * Format*2Planes* keys (AppleJPEG's recode plugin), which this scaler does not do.
 */
static OSDictionary *
capabilities(void)
{
	OSDictionary *d = OSDictionary::withCapacity(16);
	if (d == NULL) {
		return NULL;
	}
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMinSourceWidth", 4);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMinSourceHeight", 4);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxSourceWidth", 8192);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxSourceHeight", 8192);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMinDestWidth", 4);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMinDestHeight", 4);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxDestWidth", 8192);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxDestHeight", 8192);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxHUpscale", 8);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxVUpscale", 8);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxHDownscale", 8);
	setNumber(d, "IOSurfaceAcceleratorCapabilitiesMaxVDownscale", 8);
	setNumber(d, "IOSurfaceAcceleratorFilterVerticalTapsCount", 2);
	setNumber(d, "IOSurfaceAcceleratorFilterVerticalPhasesCount", 16);
	setNumber(d, "IOSurfaceAcceleratorFilterHorizontalTapsCount", 2);
	setNumber(d, "IOSurfaceAcceleratorFilterHorizontalPhasesCount", 16);
	return d;
}

/* ---- Service ------------------------------------------------------------------------------------------------ */

bool
AppleM2ScalerCSCDriver::start(IOService *provider)
{
	int mode = 0;
	PE_parse_boot_argn("vivascaler", &mode, sizeof(mode));
	if (mode <= 0) {
		IOLog("VIVA SCALER: off (boot-arg vivascaler=1 CPU, 2 MDP)\n");
		return false;
	}
	fFormats = 3;
	PE_parse_boot_argn("vivascalerformats", &fFormats, sizeof(fFormats));
	fCpuMax = 65536;
	PE_parse_boot_argn("vivascalercpumax", &fCpuMax, sizeof(fCpuMax));
	fLock = IOLockAlloc();
	if (fLock == NULL || !IOService::start(provider)) {
		return false;
	}
	if (mode >= 2) {
		fMdp = viva_mdp_create();
		if (fMdp == NULL) {
			IOLog("VIVA SCALER: MDP unavailable, CPU backend\n");
			mode = 1;
		}
	}
	fMode = mode;
	OSDictionary *caps = capabilities();
	if (caps != NULL) {
		setProperty("IOSurfaceAcceleratorCapabilitiesDict", caps);
		caps->release();
	}
	setProperty("VivaScalerBackend", fMode >= 2 ? "mdp" : "cpu");
	setProperty("VivaScalerFormats", fFormats, 32);
	setProperty("VivaScalerCpuMax", fCpuMax, 32);
	setProperty("IOUserClientClass", "AppleM2ScalerCSCDriverUserClient");
	publishStats();
	registerService();
	IOLog("VIVA SCALER: AppleM2ScalerCSCDriver published (backend %s, formats 0x%x)\n",
	    fMode >= 2 ? "mdp" : "cpu", fFormats);
	return true;
}

IOReturn
AppleM2ScalerCSCDriver::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler)
{
	if (type != 0) {
		return kIOReturnUnsupported;    /* 0xB0000x = AppleSRSDriver's sharpener: not this service */
	}
	AppleM2ScalerCSCDriverUserClient *client = OSTypeAlloc(AppleM2ScalerCSCDriverUserClient);
	if (client == NULL) {
		return kIOReturnNoMemory;
	}
	if (!client->initWithTask(owningTask, securityID, type) || !client->attach(this)) {
		client->release();
		return kIOReturnError;
	}
	if (!client->start(this)) {
		client->detach(this);
		client->release();
		return kIOReturnError;
	}
	OSIncrementAtomic64((SInt64 *)&fClients);
	*handler = client;
	return kIOReturnSuccess;
}

void
AppleM2ScalerCSCDriver::publishStats(void)
{
	OSDictionary *d = OSDictionary::withCapacity(10);
	if (d == NULL) {
		return;
	}
	setNumber(d, "transfers", fTransfers);
	setNumber(d, "cpu", fCpu);
	setNumber(d, "mdp", fMdpDone);
	setNumber(d, "unsupported", fUnsupported);
	setNumber(d, "errors", fErrors);
	setNumber(d, "bytes", fBytes);
	setNumber(d, "us", fNs / 1000);
	setNumber(d, "customFilters", fCustomFilters);
	setNumber(d, "clients", fClients);
	setProperty("VivaScalerStats", d);
	d->release();
}

/* ---- CPU backend --------------------------------------------------------------------------------------------- */

/* Byte positions of R, G, B, A in a 32-bit pixel of each format (memory order). */
static bool
rgbLayout(uint32_t pf, uint8_t pos[4])
{
	switch (pf) {
	case PF_BGRA: pos[0] = 2; pos[1] = 1; pos[2] = 0; pos[3] = 3; return true;
	case PF_RGBA: pos[0] = 0; pos[1] = 1; pos[2] = 2; pos[3] = 3; return true;
	case PF_ARGB: pos[0] = 1; pos[1] = 2; pos[2] = 3; pos[3] = 0; return true;
	default: return false;
	}
}

static inline uint8_t
clamp8(int32_t v)
{
	return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

typedef struct {
	const uint8_t *base;    /* plane 0 */
	uint32_t       bpr;
	const uint8_t *uv;      /* plane 1 (NV12) */
	uint32_t       uvBpr;
	uint32_t       w, h;
	uint8_t        pos[4];
	bool           yuv, full, bt709;
} src_view;

/* One source pixel as R, G, B, A (0-255). x, y inside the surface. */
static inline void
fetch(const src_view *s, uint32_t x, uint32_t y, int32_t out[4])
{
	if (!s->yuv) {
		const uint8_t *p = s->base + (size_t)y * s->bpr + (size_t)x * 4;
		for (int c = 0; c < 4; c++) {
			out[c] = p[s->pos[c]];
		}
		return;
	}
	int32_t Y = s->base[(size_t)y * s->bpr + x];
	const uint8_t *c = s->uv + (size_t)(y >> 1) * s->uvBpr + (size_t)(x & ~1u);
	int32_t U = (int32_t)c[0] - 128, V = (int32_t)c[1] - 128;
	/* 10-bit fixed point coefficients (x1024) */
	int32_t yy, rv, gu, gv, bu;
	if (s->full) {
		yy = Y * 1024;
		if (s->bt709) { rv = 1613; gu = 192; gv = 479; bu = 1900; } else { rv = 1436; gu = 352; gv = 731; bu = 1815; }
	} else {
		yy = (Y - 16) * 1192;
		if (s->bt709) { rv = 1836; gu = 218; gv = 546; bu = 2163; } else { rv = 1634; gu = 401; gv = 832; bu = 2066; }
	}
	out[0] = clamp8((yy + rv * V + 512) >> 10);
	out[1] = clamp8((yy - gu * U - gv * V + 512) >> 10);
	out[2] = clamp8((yy + bu * U + 512) >> 10);
	out[3] = 255;
}

/* Bilinear (or nearest) sample at source position (fx, fy) in 16.16, clamped to [x0, x0+w) x [y0, y0+h). */
static inline void
sample(const src_view *s, int64_t fx, int64_t fy, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, bool nearest,
    int32_t out[4])
{
	if (nearest) {
		int64_t x = fx >> 16, y = fy >> 16;
		x = x < (int64_t)x0 ? x0 : x >= (int64_t)x1 ? x1 - 1 : x;
		y = y < (int64_t)y0 ? y0 : y >= (int64_t)y1 ? y1 - 1 : y;
		fetch(s, (uint32_t)x, (uint32_t)y, out);
		return;
	}
	fx -= 0x8000;           /* pixel centres */
	fy -= 0x8000;
	int64_t xa = fx >> 16, ya = fy >> 16;
	uint32_t ax = (uint32_t)(fx & 0xffff) >> 8, ay = (uint32_t)(fy & 0xffff) >> 8;     /* 8-bit weights */
	int64_t xb = xa + 1, yb = ya + 1;
#define CL(v, lo, hi) ((v) < (int64_t)(lo) ? (int64_t)(lo) : (v) >= (int64_t)(hi) ? (int64_t)(hi) - 1 : (v))
	xa = CL(xa, x0, x1); xb = CL(xb, x0, x1); ya = CL(ya, y0, y1); yb = CL(yb, y0, y1);
#undef CL
	int32_t p00[4], p01[4], p10[4], p11[4];
	fetch(s, (uint32_t)xa, (uint32_t)ya, p00);
	fetch(s, (uint32_t)xb, (uint32_t)ya, p01);
	fetch(s, (uint32_t)xa, (uint32_t)yb, p10);
	fetch(s, (uint32_t)xb, (uint32_t)yb, p11);
	for (int c = 0; c < 4; c++) {
		int32_t top = p00[c] * (256 - (int32_t)ax) + p01[c] * (int32_t)ax;
		int32_t bot = p10[c] * (256 - (int32_t)ax) + p11[c] * (int32_t)ax;
		out[c] = (top * (256 - (int32_t)ay) + bot * (int32_t)ay + 32768) >> 16;
	}
}

static IOReturn
cpuTransfer(SweetSurface *src, SweetSurface *dst, uint64_t sx, uint64_t sy, uint64_t sw, uint64_t sh,
    uint32_t dx, uint32_t dy, uint32_t dw, uint32_t dh, uint64_t flags, uint32_t alphaOverride, uint64_t *bytes)
{
	src_view s = {};
	uint8_t dpos[4];
	if (!rgbLayout(dst->pixelFormat, dpos) || dst->bytesPerElement != 4) {
		return kIOReturnUnsupported;
	}
	uintptr_t sk = src->kernelAddress(), dk = dst->kernelAddress();
	if (sk == 0 || dk == 0) {
		return kIOReturnNoMemory;
	}
	s.w = src->width;
	s.h = src->height;
	if (rgbLayout(src->pixelFormat, s.pos) && src->bytesPerElement == 4) {
		s.base = (const uint8_t *)sk + src->offset;
		s.bpr = src->bytesPerRow;
	} else if ((src->pixelFormat == PF_420V || src->pixelFormat == PF_420F) && src->planeCount >= 2) {
		s.yuv = true;
		s.full = src->pixelFormat == PF_420F;
		s.bt709 = src->height >= 720;
		s.base = (const uint8_t *)sk + src->planes[0].offset;
		s.bpr = src->planes[0].bytesPerRow;
		s.uv = (const uint8_t *)sk + src->planes[1].offset;
		s.uvBpr = src->planes[1].bytesPerRow;
		s.w &= ~1u;
		s.h &= ~1u;
	} else {
		return kIOReturnUnsupported;
	}
	/* source rectangle in 16.16; whole surface when all zero */
	if (sx == 0 && sy == 0 && sw == 0 && sh == 0) {
		sw = (uint64_t)s.w << 16;
		sh = (uint64_t)s.h << 16;
	}
	if (dx == 0 && dy == 0 && dw == 0 && dh == 0) {
		dw = dst->width;
		dh = dst->height;
	}
	if (sw == 0 || sh == 0 || dw == 0 || dh == 0 || ((sx + sw) >> 16) > s.w || ((sy + sh) >> 16) > s.h ||
	    (uint64_t)dx + dw > dst->width || (uint64_t)dy + dh > dst->height) {
		return kIOReturnBadArgument;
	}
	uint32_t x0 = (uint32_t)(sx >> 16), y0 = (uint32_t)(sy >> 16);
	uint32_t x1 = (uint32_t)((sx + sw + 0xffff) >> 16), y1 = (uint32_t)((sy + sh + 0xffff) >> 16);
	x1 = x1 > s.w ? s.w : x1;
	y1 = y1 > s.h ? s.h : y1;
	bool nearest = (flags & kVivaScalerFlagNearest) != 0;
	int64_t stepX = (int64_t)((sw << 16) / dw), stepY = (int64_t)((sh << 16) / dh);     /* 16.16 */
	bool copy = !s.yuv && stepX == 0x10000 && stepY == 0x10000 && (sx & 0xffff) == 0 && (sy & 0xffff) == 0 &&
	    src->pixelFormat == dst->pixelFormat && !(flags & kVivaScalerFlagAlphaOverride);
	uint8_t *dbase = (uint8_t *)dk + dst->offset;
	for (uint32_t j = 0; j < dh; j++) {
		uint8_t *row = dbase + (size_t)(dy + j) * dst->bytesPerRow + (size_t)dx * 4;
		if (copy) {
			memcpy(row, s.base + (size_t)(y0 + j) * s.bpr + (size_t)x0 * 4, (size_t)dw * 4);
			continue;
		}
		int64_t fy = (int64_t)sy + stepY * j + stepY / 2;
		for (uint32_t i = 0; i < dw; i++) {
			int64_t fx = (int64_t)sx + stepX * i + stepX / 2;
			int32_t px[4];
			sample(&s, fx, fy, x0, y0, x1, y1, nearest, px);
			if (flags & kVivaScalerFlagAlphaOverride) {
				px[3] = (int32_t)(alphaOverride & 0xff);
			}
			uint8_t *p = row + (size_t)i * 4;
			for (int c = 0; c < 4; c++) {
				p[dpos[c]] = (uint8_t)px[c];
			}
		}
	}
	*bytes = (uint64_t)dw * dh * 4;
	return kIOReturnSuccess;
}

/* ---- Transfer ----------------------------------------------------------------------------------------------- */

IOReturn
AppleM2ScalerCSCDriver::transferLocked(const struct viva_scaler_request *req, SweetSurface *src, SweetSurface *dst,
    const char **backend)
{
	uint64_t flags = req->flags;
	if ((flags & kVivaScalerFlagTransformMask) != 0) {
		return kIOReturnUnsupported;    /* flip / rotate */
	}
	bool srcYuv = src->pixelFormat == PF_420V || src->pixelFormat == PF_420F;
	if ((srcYuv && !(fFormats & 2)) || (!srcYuv && !(fFormats & 1))) {
		return kIOReturnUnsupported;
	}
	/* the GPU jobs on both surfaces first (they may still be drawing the source or reading the destination) */
	if (!viva_gpu_surface_idle(src, NULL)) {
		viva_gpu_surface_wait(src, SCALER_GPU_WAIT_MS);
	}
	if (!viva_gpu_surface_idle(dst, NULL)) {
		viva_gpu_surface_wait(dst, SCALER_GPU_WAIT_MS);
	}
	IOReturn ret = kIOReturnUnsupported;
	uint64_t bytes = 0;
	if (fMdp != NULL) {
		/* the engines read memory: the CPU's dirty lines of the source go out first */
		sweet_surface_before_device_read(src);
		sweet_surface_before_device_read(dst);
		ret = viva_mdp_transfer(fMdp, src, dst, (uint32_t)req->srcX, (uint32_t)req->srcY, (uint32_t)req->srcW,
		    (uint32_t)req->srcH, req->dstX, req->dstY, req->dstW, req->dstH, flags);
		if (ret == kIOReturnSuccess) {
			sweet_surface_device_wrote(dst);
			sweet_surface_seed_bump(dst);
			*backend = "mdp";
			fMdpDone++;
			return ret;
		}
	}
	/* the CPU only for small work or plain copies: everything else back to the client (QuartzCore uses the GPU) */
	{
		uint64_t dw = req->dstW ? req->dstW : dst->width, dh = req->dstH ? req->dstH : dst->height;
		bool same = src->pixelFormat == dst->pixelFormat && !srcYuv &&
		    (req->srcW == 0 || req->srcW == dw << 16) && (req->srcH == 0 || req->srcH == dh << 16) &&
		    (req->srcW != 0 || (src->width == dw && src->height == dh)) && !(flags & kVivaScalerFlagAlphaOverride);
		if (!same && dw * dh > fCpuMax) {
			return kIOReturnUnsupported;
		}
	}
	sweet_surface_before_cpu_access(src);
	sweet_surface_before_cpu_access(dst);
	ret = cpuTransfer(src, dst, req->srcX, req->srcY, req->srcW, req->srcH, req->dstX, req->dstY, req->dstW,
	    req->dstH, flags, req->alphaOverride, &bytes);
	if (ret == kIOReturnSuccess) {
		sweet_surface_cpu_wrote(dst);
		*backend = "cpu";
		fCpu++;
		fBytes += bytes;
	}
	return ret;
}

IOReturn
AppleM2ScalerCSCDriver::transfer(const struct viva_scaler_request *req, __unused task_t task)
{
	SweetSurface *src = sweet_surface_copy(req->srcID), *dst = sweet_surface_copy(req->dstID);
	IOReturn ret;
	const char *backend = "";
	uint64_t t0 = mach_absolute_time();
	if (src == NULL || dst == NULL) {
		ret = kIOReturnBadArgument;
	} else {
		IOLockLock(fLock);
		ret = transferLocked(req, src, dst, &backend);
		fTransfers++;
		if (ret == kIOReturnUnsupported) {
			fUnsupported++;
		} else if (ret != kIOReturnSuccess) {
			fErrors++;
		}
		uint64_t ns;
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		fNs += ns;
		uint64_t now = mach_absolute_time(), interval;
		nanoseconds_to_absolutetime(NSEC_PER_SEC / 4, &interval);
		bool publish = now - fLastPublish > interval || fTransfers < 8;
		if (publish) {
			fLastPublish = now;
		}
		IOLockUnlock(fLock);
		if (publish) {
			publishStats();
		}
		if (fTransfers <= 16 || (ret != kIOReturnSuccess && fErrors + fUnsupported <= 32)) {
			IOLog("VIVA SCALER: %u (%ux%u '%c%c%c%c') -> %u (%ux%u '%c%c%c%c') flags 0x%llx: 0x%x %s\n",
			    src->id, src->width, src->height, (char)(src->pixelFormat >> 24), (char)(src->pixelFormat >> 16),
			    (char)(src->pixelFormat >> 8), (char)src->pixelFormat, dst->id, dst->width, dst->height,
			    (char)(dst->pixelFormat >> 24), (char)(dst->pixelFormat >> 16), (char)(dst->pixelFormat >> 8),
			    (char)dst->pixelFormat, req->flags, ret, backend);
		}
	}
	if (src != NULL) {
		src->release();
	}
	if (dst != NULL) {
		dst->release();
	}
	return ret;
}

/* ---- User client ------------------------------------------------------------------------------------------- */

bool
AppleM2ScalerCSCDriverUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	fPort = MACH_PORT_NULL;
	return true;
}

bool
AppleM2ScalerCSCDriverUserClient::start(IOService *provider)
{
	fOwner = OSDynamicCast(AppleM2ScalerCSCDriver, provider);
	return fOwner != NULL && IOUserClient::start(provider);
}

IOReturn
AppleM2ScalerCSCDriverUserClient::clientClose(void)
{
	terminate();
	return kIOReturnSuccess;
}

void
AppleM2ScalerCSCDriverUserClient::free(void)
{
	if (fPort != MACH_PORT_NULL) {
		iokit_release_port_send((ipc_port_t)fPort);
		fPort = MACH_PORT_NULL;
	}
	IOUserClient::free();
}

IOReturn
AppleM2ScalerCSCDriverUserClient::registerNotificationPort(mach_port_t port, UInt32 type, __unused io_user_reference_t refCon)
{
	if (type != 0) {
		return kIOReturnUnsupported;
	}
	if (fPort != MACH_PORT_NULL) {
		iokit_release_port_send((ipc_port_t)fPort);
	}
	fPort = port;           /* the send right is ours now (released on replacement and in free) */
	return kIOReturnSuccess;
}

IOReturn
AppleM2ScalerCSCDriverUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target, __unused void *reference)
{
	switch (selector) {
	case 1: {
		if (args->structureInput == NULL || args->structureInputSize != sizeof(struct viva_scaler_request)) {
			return kIOReturnBadArgument;
		}
		struct viva_scaler_request req;
		memcpy(&req, args->structureInput, sizeof(req));
		IOReturn ret = fOwner->transfer(&req, fTask);
		if (ret == kIOReturnSuccess && req.callback != 0 && !(req.flags & kVivaScalerFlagSkipAsync) &&
		    fPort != MACH_PORT_NULL) {
			OSAsyncReference64 ref = {};
			setAsyncReference64(ref, fPort, (mach_vm_address_t)req.callback, (io_user_reference_t)req.refcon, fTask);
			io_user_reference_t arg = (io_user_reference_t)req.refcon2;
			sendAsyncResult64(ref, kIOReturnSuccess, &arg, 1);
		}
		return ret;
	}
	case 2:
	case 3:
	case 7:
		return kIOReturnSuccess;
	case 4:
		if (args->structureInputSize != 0x20) {
			return kIOReturnBadArgument;
		}
		fOwner->noteCustomFilter();
		return kIOReturnSuccess;
	case 5:
		if (args->scalarOutputCount < 1) {
			return kIOReturnBadArgument;
		}
		args->scalarOutput[0] = 0;
		args->scalarOutputCount = 1;
		return kIOReturnSuccess;
	case 0:
	case 8:
	case 9:
		return kIOReturnUnsupported;
	default:
		return kIOReturnBadArgument;
	}
}

#endif /* SWEET */
