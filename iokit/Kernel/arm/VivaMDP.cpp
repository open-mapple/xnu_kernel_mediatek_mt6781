/*
 * viva (MT6781): MDP backend of AppleM2ScalerCSCDriver (VivaScaler.h). Placeholder until the engine sequence is
 * validated on the phone (gpu/docs/MDP-MT6781.md): no engine is touched, every transfer is left to the CPU.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include "VivaScaler.h"

struct viva_mdp *
viva_mdp_create(void)
{
	return NULL;
}

IOReturn
viva_mdp_transfer(__unused struct viva_mdp *mdp, __unused SweetSurface *src, __unused SweetSurface *dst,
    __unused uint32_t sx, __unused uint32_t sy, __unused uint32_t sw, __unused uint32_t sh, __unused uint32_t dx,
    __unused uint32_t dy, __unused uint32_t dw, __unused uint32_t dh, __unused uint64_t flags)
{
	return kIOReturnUnsupported;
}

#endif /* SWEET */
