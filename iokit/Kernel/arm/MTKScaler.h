/*
 * MT6781: AppleM2ScalerCSCDriver, the service behind IOSurfaceAccelerator.framework (scaling, colour
 * conversion and copies between IOSurfaces). User-client ABI reverse-engineered from iOS 16.7.16 (20H392):
 * gpu/docs/ABI-IOSURFACEACCELERATOR.md in the viva-gpu repository.
 *
 * Boot-arg mp_scaler (the Viva Manager app sets it, MaliMetal > Acceleratori):
 *   0 (default)  the service is not published: every client falls back as before (QuartzCore to the GPU,
 *                VideoToolbox to software conversions)
 *   1            published, transfers done by the CPU in the kernel (backend "cpu")
 *   2            published, transfers on the MDP engines (RDMA -> RSZ -> WROT) when the format and size allow it,
 *                the CPU otherwise (backend "mdp"; needs the MDP power domain: falls back to 1 if it is off)
 * mp_scaler_formats (bitmask, default 3): 1 = 32-bit RGB (BGRA/RGBA/ARGB), 2 = 4:2:0 biplanar (420v/420f) as
 *   source. Formats outside the mask are refused with kIOReturnUnsupported, which every client handles by
 *   falling back (VideoToolbox propagates errors of a published service, hence the mask).
 *
 * mp_scaler_cpumax (pixels, default 65536): largest destination the CPU scales or converts. Bigger requests that the MDP
 *   cannot take are refused with kIOReturnUnsupported: QuartzCore then does them on the GPU as without the service
 *   (its switcher/snapshot reductions of the whole screen, 1125x2436 -> 787x1705, cost tens of milliseconds on the
 *   CPU against ~2 ms on the GPU). Same-size copies of one RGB format are always done (one memcpy per row).
 *
 * Counters: property "MTKScalerStats" of the service (ioreg / Viva Manager).
 */
#ifndef _IOKIT_ARM_MTKSCALER_H
#define _IOKIT_ARM_MTKSCALER_H

#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOLocks.h>

/* Selector 1 request (0x170 bytes), see the ABI document section 2.2 */
struct mapple_scaler_request {
	uint32_t srcID, dstID;
	uint64_t callback, refcon, refcon2;
	uint64_t flags;
	uint64_t srcX, srcY, srcW, srcH;        /* 16.16 fixed point; all zero = the whole surface */
	uint64_t reserved0, reserved1;
	uint32_t dstX, dstY, dstW, dstH;        /* pixels; all zero = the whole surface */
	uint32_t alphaOverride;
	uint32_t borderX, borderY, borderW, borderH, borderA, borderR, borderG, borderB;
	uint16_t woAlpha, woRed, woGreen, woBlue;
	uint8_t  reserved2[8];
	uint32_t histBinMode, histX, histY, histW, histH;
	uint32_t histBins[8];
	uint8_t  comm[4][0x28];
};

enum {
	kMappleScalerFlagTransformMask    = 0x7,
	kMappleScalerFlagNearest          = 1ull << 7,
	kMappleScalerFlagSkipAsync        = 1ull << 10,
	kMappleScalerFlagAlphaOverride    = 1ull << 18,
	kMappleScalerFlagBorderFill       = 1ull << 28,
	kMappleScalerFlagHistogram        = 1ull << 29,
	kMappleScalerFlagZeroFill         = 1ull << 30,
};

class MappleSurface;
struct mapple_mdp;

class AppleM2ScalerCSCDriver : public IOService {
	OSDeclareDefaultStructors(AppleM2ScalerCSCDriver);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;

	/* Selector 1: does the transfer (synchronously) and returns its status. */
	IOReturn transfer(const struct mapple_scaler_request *req, task_t task);
	void noteCustomFilter(void) { fCustomFilters++; }

private:
	IOReturn transferLocked(const struct mapple_scaler_request *req, MappleSurface *src, MappleSurface *dst,
	    const char **backend);
	void publishStats(void);

	IOLock   *fLock;            /* one transfer at a time (CPU work is short; the MDP has one pipe) */
	int       fMode;
	uint32_t  fFormats;
	uint32_t  fCpuMax;          /* largest destination (pixels) the CPU scales / converts */
	struct mapple_mdp *fMdp;      /* NULL = CPU only */
	uint64_t  fTransfers, fCpu, fMdpDone, fUnsupported, fErrors, fBytes, fNs, fCustomFilters, fClients;
	uint64_t  fLastPublish;
};

class AppleM2ScalerCSCDriverUserClient : public IOUserClient {
	OSDeclareDefaultStructors(AppleM2ScalerCSCDriverUserClient);
public:
	virtual bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;

private:
	AppleM2ScalerCSCDriver *fOwner;
	task_t                  fTask;
	mach_port_t             fPort;
};

/*
 * MDP backend (MTKMDP.cpp). mapple_mdp_create returns NULL when the engines cannot be used (power domain off,
 * boot-arg). mapple_mdp_transfer returns kIOReturnUnsupported for anything it does not handle, so the caller
 * falls back to the CPU.
 */
struct mapple_mdp *mapple_mdp_create(void);
IOReturn mapple_mdp_transfer(struct mapple_mdp *mdp, MappleSurface *src, MappleSurface *dst,
    uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, uint32_t dx, uint32_t dy, uint32_t dw, uint32_t dh,
    uint64_t flags);

#endif /* _IOKIT_ARM_MTKSCALER_H */
