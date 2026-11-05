/*
 * viva (MT6781): hardware JPEG encoder (VivaJPEGHW.cpp) and the AppleJPEGDriver service on top of it
 * (VivaJPEG.cpp). Encode only: MT6781 has no JPEG decoder, decode requests are refused so clients keep decoding in
 * software.
 *
 * Boot-arg vivajpeg (Viva Manager, MaliMetal > Acceleratori): 0 (default) = not published, 1 = published.
 */
#ifndef _IOKIT_ARM_VIVAJPEG_H
#define _IOKIT_ARM_VIVAJPEG_H

#include <IOKit/IOReturn.h>
#include <stddef.h>
#include <stdint.h>

enum { kVivaJpegNV12 = 1, kVivaJpegNV21 = 2, kVivaJpegYUYV = 3 };   /* YUYV: one plane, y = the packed pixels */

struct viva_jpgenc_image {
	uint32_t format;                /* kVivaJpegNV12 / NV21 */
	uint32_t width, height;
	const uint8_t *y, *uv;          /* kernel addresses of the planes */
	uint32_t yBytesPerRow, uvBytesPerRow;
};

struct viva_jpgenc;

/* Powers the block and allocates bounce buffers for images up to maxPixels; NULL if the hardware is unusable. */
struct viva_jpgenc *viva_jpgenc_create(size_t maxPixels);
void viva_jpgenc_destroy(struct viva_jpgenc *j);
/* One encode (synchronous, polls the block): a complete JFIF file into out. quality 0..100. Not reentrant. */
IOReturn viva_jpgenc_encode(struct viva_jpgenc *j, const struct viva_jpgenc_image *img, uint32_t quality, void *out,
    size_t outCap, size_t *outLen, uint32_t *cycles);
void viva_jpgenc_stats(struct viva_jpgenc *j, uint64_t *encodes, uint64_t *failures, uint64_t *bytes, uint64_t *cycles);

#endif /* _IOKIT_ARM_VIVAJPEG_H */
