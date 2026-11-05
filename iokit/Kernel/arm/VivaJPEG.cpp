/*
 * viva (MT6781): the AppleJPEGDriver that VideoToolbox's JPEG encoder (JPEGH1.videoencoder) opens, on the hardware
 * encoder of VivaJPEGHW.cpp. ABI from iOS 16.7.16: gpu/docs/ABI-APPLEJPEGDRIVER.md in the viva-gpu repository.
 *
 * Phase 1 of that document: the class is VivaJPEGEncoder and only the registry *name* is "AppleJPEGDriver". The VT
 * encoder looks the service up with IOServiceNameMatching and finds it; VT's decoder, CMPhoto and MobileGestalt match
 * the class AppleJPEGDriver and do not, so every decode (MT6781 has no JPEG decoder) and CMPhoto's encoding stay in
 * software exactly as without the service.
 *
 * Selector 3 (encode, struct 0x58 in, 0x58 out): source 'yuvf'/'yuvs' (YUYV 4:2:2) or '420f'/'420v' (NV12) IOSurface,
 * destination IOSurface ('JPEG'); a complete JFIF file at offset 0 of the destination, out+0x10 = its size,
 * out+0x14/+0x18 = width/height. Quality level +0x1C (0..7, VideoToolbox: higher = better) maps to the jpgenc tables.
 * Bigger than the destination: kIOReturnOverrun with out+0x10 = the destination size (VT retries one level lower).
 * Selectors 1, 4, 5 (decode, encode with custom tables): kIOReturnUnsupported.
 * Registry: AppleJPEGRequiresMCUAlignment = 0 (32-bit number) lets VT hand over 420f as well as yuvf.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>

#include "VivaJPEG.h"
#include "SweetSurface.h"

#define FOURCC(a, b, c, d)      (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))
#define PF_YUVF                 FOURCC('y', 'u', 'v', 'f')
#define PF_YUVS                 FOURCC('y', 'u', 'v', 's')
#define PF_420F                 FOURCC('4', '2', '0', 'f')
#define PF_420V                 FOURCC('4', '2', '0', 'v')
#define JPEG_MAX_PIXELS         (4096u * 4096u)
#define JPEG_GPU_WAIT_MS        1000

struct viva_jpeg_request {              /* selectors 1/3, 0x58 bytes */
	uint32_t srcID, srcAllocSize, dstID, dstAllocSize;
	uint32_t written, width, height, quality, flags, cropX, cropY, chroma;
	uint64_t callback, refcon, arg;
	uint32_t traceID, decode[2];         /* 0x48, 0x4c-0x53 (decode only) */
	uint32_t rstLogging;                 /* 0x54 */
};
static_assert(sizeof(struct viva_jpeg_request) == 0x58, "AppleJPEGDriver requests are 0x58 bytes");

class VivaJPEGEncoder : public IOService {
	OSDeclareDefaultStructors(VivaJPEGEncoder);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) APPLE_KEXT_OVERRIDE;
	IOReturn encode(struct viva_jpeg_request *req);
	void refused(void) { OSIncrementAtomic64((SInt64 *)&fRefused); }
private:
	void publishStats(void);
	IOLock *fLock;
	struct viva_jpgenc *fHW;
	uint64_t fRefused, fErrors, fOverruns;
};

class VivaJPEGEncoderUserClient : public IOUserClient {
	OSDeclareDefaultStructors(VivaJPEGEncoderUserClient);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
private:
	VivaJPEGEncoder *fOwner;
};

OSDefineMetaClassAndStructors(VivaJPEGEncoder, IOService);
OSDefineMetaClassAndStructors(VivaJPEGEncoderUserClient, IOUserClient);

static void
setNumber(IORegistryEntry *e, OSDictionary *d, const char *key, uint64_t value)
{
	OSNumber *n = OSNumber::withNumber(value, 32);
	if (n == NULL) {
		return;
	}
	if (d != NULL) {
		d->setObject(key, n);
	} else {
		e->setProperty(key, n);
	}
	n->release();
}

bool
VivaJPEGEncoder::start(IOService *provider)
{
	int mode = 0;
	PE_parse_boot_argn("vivajpeg", &mode, sizeof(mode));
	if (mode <= 0) {
		IOLog("VIVA JPEG: off (boot-arg vivajpeg=1 publishes the hardware encoder)\n");
		return false;
	}
	fLock = IOLockAlloc();
	if (fLock == NULL || !IOService::start(provider)) {
		return false;
	}
	fHW = viva_jpgenc_create(JPEG_MAX_PIXELS);
	if (fHW == NULL) {
		return false;
	}
	setName("AppleJPEGDriver");
	setNumber(this, NULL, "AppleJPEGRequiresMCUAlignment", 0);
	setNumber(this, NULL, "AppleJPEGNumCores", 1);
	setProperty("IOUserClientClass", "VivaJPEGEncoderUserClient");
	publishStats();
	registerService();
	IOLog("VIVA JPEG: AppleJPEGDriver (encode only) published\n");
	return true;
}

IOReturn
VivaJPEGEncoder::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler)
{
	if (type != 0) {
		return kIOReturnUnsupported;
	}
	VivaJPEGEncoderUserClient *c = OSTypeAlloc(VivaJPEGEncoderUserClient);
	if (c == NULL) {
		return kIOReturnNoMemory;
	}
	if (!c->initWithTask(owningTask, securityID, type) || !c->attach(this) || !c->start(this)) {
		c->detach(this);
		c->release();
		return kIOReturnError;
	}
	*handler = c;
	return kIOReturnSuccess;
}

void
VivaJPEGEncoder::publishStats(void)
{
	OSDictionary *d = OSDictionary::withCapacity(6);
	if (d == NULL) {
		return;
	}
	uint64_t enc = 0, fail = 0, bytes = 0, cycles = 0;
	if (fHW != NULL) {
		viva_jpgenc_stats(fHW, &enc, &fail, &bytes, &cycles);
	}
	setNumber(this, d, "encodes", enc);
	setNumber(this, d, "failures", fail + fErrors);
	setNumber(this, d, "overruns", fOverruns);
	setNumber(this, d, "refused", fRefused);
	setNumber(this, d, "bytes", bytes);
	setNumber(this, d, "cycles", cycles);
	setProperty("VivaJPEGStats", d);
	d->release();
}

/* VideoToolbox's level 0..7 (7 = 'sjpg' without rate control) to a 0..100 quality for the jpgenc tables. */
static uint32_t
levelQuality(uint32_t level)
{
	static const uint8_t q[8] = { 40, 55, 65, 75, 85, 92, 97, 97 };
	return q[level < 8 ? level : 4];
}

IOReturn
VivaJPEGEncoder::encode(struct viva_jpeg_request *req)
{
	SweetSurface *src = sweet_surface_copy(req->srcID), *dst = sweet_surface_copy(req->dstID);
	IOReturn ret = kIOReturnBadArgument;
	if (src == NULL || dst == NULL) {
		goto out;
	}
	{
		struct viva_jpgenc_image img = {};
		uint32_t w = req->width ? req->width : src->width, h = req->height ? req->height : src->height;
		uintptr_t sk = src->kernelAddress(), dk = dst->kernelAddress();
		if (sk == 0 || dk == 0 || w > src->width || h > src->height || req->cropX || req->cropY) {
			ret = sk == 0 || dk == 0 ? kIOReturnNoMemory : kIOReturnUnsupported;
			goto out;
		}
		img.width = w;
		img.height = h;
		if ((src->pixelFormat == PF_YUVF || src->pixelFormat == PF_YUVS) && src->bytesPerElement >= 2) {
			img.format = kVivaJpegYUYV;
			img.y = (const uint8_t *)sk + src->offset;
			img.yBytesPerRow = src->bytesPerRow;
		} else if ((src->pixelFormat == PF_420F || src->pixelFormat == PF_420V) && src->planeCount >= 2) {
			img.format = kVivaJpegNV12;
			img.y = (const uint8_t *)sk + src->planes[0].offset;
			img.yBytesPerRow = src->planes[0].bytesPerRow;
			img.uv = (const uint8_t *)sk + src->planes[1].offset;
			img.uvBytesPerRow = src->planes[1].bytesPerRow;
		} else {
			ret = kIOReturnUnsupported;
			goto out;
		}
		/* the encoder writes the source through the GPU possibly (pool buffers); wait, then read fresh lines */
		if (!viva_gpu_surface_idle(src, NULL)) {
			viva_gpu_surface_wait(src, JPEG_GPU_WAIT_MS);
		}
		if (!viva_gpu_surface_idle(dst, NULL)) {
			viva_gpu_surface_wait(dst, JPEG_GPU_WAIT_MS);
		}
		sweet_surface_before_cpu_access(src);
		sweet_surface_before_cpu_access(dst);
		size_t cap = dst->allocSize, len = 0;
		IOLockLock(fLock);
		ret = viva_jpgenc_encode(fHW, &img, levelQuality(req->quality), (uint8_t *)dk, cap, &len, NULL);
		if (ret == kIOReturnNoSpace) {
			fOverruns++;
		} else if (ret != kIOReturnSuccess) {
			fErrors++;
		}
		IOLockUnlock(fLock);
		if (ret == kIOReturnSuccess) {
			sweet_surface_cpu_wrote(dst);
			req->written = (uint32_t)len;
			req->width = w;
			req->height = h;
		} else if (ret == kIOReturnNoSpace) {
			req->written = (uint32_t)cap;
			ret = kIOReturnOverrun;     /* VideoToolbox retries one quality level lower */
		}
	}
out:
	OSSafeReleaseNULL(src);
	OSSafeReleaseNULL(dst);
	publishStats();
	return ret;
}

bool
VivaJPEGEncoderUserClient::start(IOService *provider)
{
	fOwner = OSDynamicCast(VivaJPEGEncoder, provider);
	return fOwner != NULL && IOUserClient::start(provider);
}

IOReturn
VivaJPEGEncoderUserClient::clientClose(void)
{
	terminate();
	return kIOReturnSuccess;
}

IOReturn
VivaJPEGEncoderUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target, __unused void *reference)
{
	switch (selector) {
	case 3: {
		if (args->structureInput == NULL || args->structureInputSize != sizeof(struct viva_jpeg_request) ||
		    args->structureOutput == NULL || args->structureOutputSize < sizeof(struct viva_jpeg_request)) {
			return kIOReturnBadArgument;
		}
		struct viva_jpeg_request req;
		memcpy(&req, args->structureInput, sizeof(req));
		IOReturn ret = fOwner->encode(&req);
		memcpy(args->structureOutput, &req, sizeof(req));
		args->structureOutputSize = sizeof(req);
		return ret;
	}
	case 1:
	case 4:
	case 5:
		fOwner->refused();      /* decode / custom tables: the clients keep (or fall back to) software */
		return kIOReturnUnsupported;
	default:
		return kIOReturnBadArgument;
	}
}

#endif /* SWEET */
