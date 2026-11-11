/*
 * MT6781: hardware JPEG encoder (MTKJPEGHW.cpp) and the AppleJPEGDriver service on top of it
 * (MTKJPEG.cpp). Encode only: MT6781 has no JPEG decoder, decode requests are refused so clients keep decoding in
 * software.
 *
 * Boot-arg mp_jpeg (Mapple Manager, MaliMetal > Acceleratori): 0 (default) = not published, 1 = published.
 */
#ifndef _IOKIT_ARM_MTKJPEG_H
#define _IOKIT_ARM_MTKJPEG_H

#include <IOKit/IOReturn.h>
#include <stddef.h>
#include <stdint.h>

enum { kMappleJpegNV12 = 1, kMappleJpegNV21 = 2, kMappleJpegYUYV = 3 };   /* YUYV: one plane, y = the packed pixels */

struct mapple_jpgenc_image {
	uint32_t format;                /* kMappleJpegNV12 / NV21 */
	uint32_t width, height;
	const uint8_t *y, *uv;          /* kernel addresses of the planes */
	uint32_t yBytesPerRow, uvBytesPerRow;
};

struct mapple_jpgenc;

/* Powers the block and allocates bounce buffers for images up to maxPixels; NULL if the hardware is unusable. */
struct mapple_jpgenc *mapple_jpgenc_create(size_t maxPixels);
void mapple_jpgenc_destroy(struct mapple_jpgenc *j);
/* One encode (synchronous, polls the block): a complete JFIF file into out. quality 0..100. Not reentrant. */
IOReturn mapple_jpgenc_encode(struct mapple_jpgenc *j, const struct mapple_jpgenc_image *img, uint32_t quality, void *out,
    size_t outCap, size_t *outLen, uint32_t *cycles);
void mapple_jpgenc_stats(struct mapple_jpgenc *j, uint64_t *encodes, uint64_t *failures, uint64_t *bytes, uint64_t *cycles);

#endif /* _IOKIT_ARM_MTKJPEG_H */
