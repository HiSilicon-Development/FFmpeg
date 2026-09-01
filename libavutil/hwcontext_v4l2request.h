/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVUTIL_HWCONTEXT_V4L2REQUEST_H
#define AVUTIL_HWCONTEXT_V4L2REQUEST_H

#include "hwcontext_drm.h"
#include "rational.h"

/**
 * @file
 * An API-specific header for AV_HWDEVICE_TYPE_V4L2REQUEST.
 */

struct AVFrame;

/**
 * V4L2 Request API device details.
 *
 * Allocated as AVHWDeviceContext.hwctx
 */
typedef struct AVV4L2RequestDeviceContext {
    /**
     * File descriptor of media device.
     *
     * Defaults to -1 for auto-detect.
     */
    int media_fd;
} AVV4L2RequestDeviceContext;

/**
 * V4L2 Request capture layout, allocated as AVHWFramesContext.hwctx.
 * The negotiated row pitch is independent of the visible frame width.
 */
typedef struct AVV4L2RequestFramesContext {
    int bytesperline;
    /** Coded picture cadence from codec syntax, not a demux packet guess. */
    AVRational frame_rate;
} AVV4L2RequestFramesContext;

/**
 * V4L2 Request frame descriptor.
 *
 * The DRM descriptor is first so AVFrame.data[0] remains compatible with
 * AV_PIX_FMT_DRM_PRIME consumers.  The optional wait callback allows a
 * producer to defer completion until a consumer actually maps the frame.
 */
typedef struct AVV4L2RequestFrameDescriptor {
    AVDRMFrameDescriptor drm;
    int (*wait)(void *opaque);
    void *wait_opaque;
    /**
     * Optional synchronous hardware DEI after codec display reordering.
     * Inputs are retained woven NV12 frames. Output owns a separate pair
     * of progressive NV12 surfaces; no CPU pixel copy.
     */
    int (*deinterlace)(const struct AVFrame *previous,
                       const struct AVFrame *current,
                       const struct AVFrame *next, struct AVFrame *output[2]);
} AVV4L2RequestFrameDescriptor;

#endif /* AVUTIL_HWCONTEXT_V4L2REQUEST_H */
