/*
 * Private Hi3798CV200 capture layout helpers.
 * This file is part of FFmpeg, licensed under the GNU LGPL v2.1 or later.
 */
#ifndef AVUTIL_HWCONTEXT_V4L2REQUEST_INTERNAL_H
#define AVUTIL_HWCONTEXT_V4L2REQUEST_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include <drm/histb_drm.h>

#include "error.h"

typedef struct V4L2RequestHT10Layout {
    size_t stride;
    size_t alloc_height;
    size_t high8_uv;
    size_t low2_y;
    size_t low2_uv;
    size_t size;
} V4L2RequestHT10Layout;

/* Intersection of the CV200 decoder and native V0 scanout bounds. The
 * canonical modifier defines the four regions; object tail guards do not
 * participate in their offsets. These bounds also make all products safe
 * on 32-bit hosts. Never substitute visible frame geometry here. */
static inline int ff_v4l2request_ht10_layout(uint32_t coded_width,
                                            uint32_t coded_height,
                                            ptrdiff_t pitch, size_t size,
                                            V4L2RequestHT10Layout *layout)
{
    if (coded_width < 64 || coded_width > 4096 || (coded_width & 3) ||
        coded_height < 64 || coded_height > 2160 || (coded_height & 1))
        return AVERROR(EINVAL);

    layout->stride       = (coded_width + 255U) & ~255U;
    layout->alloc_height = (coded_height + 63U) & ~63U;
    layout->high8_uv     = layout->stride * layout->alloc_height;
    layout->low2_y       = layout->high8_uv * 3 / 2;
    layout->low2_uv      = layout->low2_y + layout->high8_uv / 4;
    layout->size         = layout->high8_uv * 15 / 8;
    if (pitch < 0 || (size_t)pitch != layout->stride || size < layout->size)
        return AVERROR(EINVAL);
    return 0;
}

#endif /* AVUTIL_HWCONTEXT_V4L2REQUEST_INTERNAL_H */
