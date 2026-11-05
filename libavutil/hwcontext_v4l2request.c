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

#include "config.h"

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/media.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "avassert.h"
#include "hwcontext_drm.h"
#include "hwcontext_internal.h"
#include "hwcontext_v4l2request.h"
#include "hwcontext_v4l2request_internal.h"
#include "intreadwrite.h"
#include "mem.h"

static void v4l2request_device_free(AVHWDeviceContext *hwdev)
{
    AVV4L2RequestDeviceContext *hwctx = hwdev->hwctx;

    if (hwctx->media_fd >= 0) {
        close(hwctx->media_fd);
        hwctx->media_fd = -1;
    }
}

static int v4l2request_device_create(AVHWDeviceContext *hwdev, const char *device,
                                     AVDictionary *opts, int flags)
{
    AVV4L2RequestDeviceContext *hwctx = hwdev->hwctx;

    hwctx->media_fd = -1;
    hwdev->free = v4l2request_device_free;

    // Use auto-detect
    if (!device || !device[0])
        return 0;

    hwctx->media_fd = open(device, O_RDWR);
    if (hwctx->media_fd < 0)
        return AVERROR(errno);

    return 0;
}

static int v4l2request_device_init(AVHWDeviceContext *hwdev)
{
    AVV4L2RequestDeviceContext *hwctx = hwdev->hwctx;
    struct media_device_info device_info;

    // Use auto-detect
    if (hwctx->media_fd < 0)
        return 0;

    if (ioctl(hwctx->media_fd, MEDIA_IOC_DEVICE_INFO, &device_info) < 0)
        return AVERROR(errno);

    av_log(hwdev, AV_LOG_VERBOSE, "Using V4L2 media driver %s (%u.%u.%u)\n",
           device_info.driver,
           device_info.driver_version >> 16,
           (device_info.driver_version >> 8) & 0xff,
           device_info.driver_version & 0xff);

    return 0;
}

static int v4l2request_get_buffer(AVHWFramesContext *hwfc, AVFrame *frame)
{
    frame->buf[0] = av_buffer_pool_get(hwfc->pool);
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    frame->data[0] = (uint8_t *)frame->buf[0]->data;

    frame->format = AV_PIX_FMT_DRM_PRIME;
    frame->width  = hwfc->width;
    frame->height = hwfc->height;

    return 0;
}

typedef struct DRMMapping {
    // Address and length of each mmap()ed region.
    int nb_regions;
    int object[AV_DRM_MAX_PLANES];
    void *address[AV_DRM_MAX_PLANES];
    size_t length[AV_DRM_MAX_PLANES];
} DRMMapping;

static void v4l2request_unmap_frame(AVHWFramesContext *hwfc,
                                    HWMapDescriptor *hwmap)
{
    DRMMapping *map = hwmap->priv;

    for (int i = 0; i < map->nb_regions; i++) {
        struct dma_buf_sync sync = {
            .flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ,
        };
        ioctl(map->object[i], DMA_BUF_IOCTL_SYNC, &sync);
        munmap(map->address[i], map->length[i]);
    }

    av_free(map);
}

static int v4l2request_is_ht10(const AVFrame *frame)
{
    const AVV4L2RequestFrameDescriptor *request = (const void *)frame->data[0];

    return request->drm.nb_objects > 0 &&
           request->drm.objects[0].format_modifier == DRM_FORMAT_MOD_HISTB_VDH_10BIT;
}

/* CV200, not MV310: histb-vdec-core.c histb_vdec_detile_p010(),
 * histb_vdec_row_map_{y,uv} and *_2bit(). The high-eight permutations
 * also match SPC070 vpss_v4_0/vpss_osal.c row_map_table_{y,uv}.
 * Native chroma is Cr,Cb; P010 must publish Cb,Cr, including the low bits. */
static const uint8_t v4l2request_cv200_row_y[128] = {
    0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15,
    12, 0, 13, 1, 14, 2, 15, 3, 8, 4, 9, 5, 10, 6, 11, 7,
    4, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11,
    8, 4, 9, 5, 10, 6, 11, 7, 12, 0, 13, 1, 14, 2, 15, 3,
    8, 0, 9, 1, 10, 2, 11, 3, 12, 4, 13, 5, 14, 6, 15, 7,
    0, 12, 1, 13, 2, 14, 3, 15, 4, 8, 5, 9, 6, 10, 7, 11,
    12, 4, 13, 5, 14, 6, 15, 7, 8, 0, 9, 1, 10, 2, 11, 3,
    4, 8, 5, 9, 6, 10, 7, 11, 0, 12, 1, 13, 2, 14, 3, 15,
};

static const uint8_t v4l2request_cv200_row_uv[64] = {
    0, 4, 1, 5, 2, 6, 3, 7, 4, 0, 5, 1, 6, 2, 7, 3,
    4, 0, 5, 1, 6, 2, 7, 3, 0, 4, 1, 5, 2, 6, 3, 7,
    4, 0, 5, 1, 6, 2, 7, 3, 0, 4, 1, 5, 2, 6, 3, 7,
    0, 4, 1, 5, 2, 6, 3, 7, 4, 0, 5, 1, 6, 2, 7, 3,
};

static unsigned int v4l2request_cv200_row_y_2bit(unsigned int index)
{
    unsigned int group = index / 32;
    unsigned int row = index % 32;

    return (row >> 1) + ((group == 0 || group == 3 ? row & 1 : !(row & 1)) << 4);
}

static void v4l2request_cv200_unpack(uint8_t *dst, const uint8_t *high,
                                    const uint8_t *low, unsigned int samples,
                                    int swap_uv)
{
    unsigned int offset = 0;

    for (; offset + 4 <= samples; offset += 4) {
        uint32_t h = AV_RL32(high + offset);
        uint8_t l = low[offset / 4];
        uint64_t packed;

        if (swap_uv) {
            h = ((h & 0x00ff00ffU) << 8) | ((h & 0xff00ff00U) >> 8);
            l = ((l & 0x33) << 2) | ((l & 0xcc) >> 2);
        }
        packed = ((uint64_t)(h & 0xff) << 8) |
                 ((uint64_t)(l & 3) << 6) |
                 ((uint64_t)((h >> 8) & 0xff) << 24) |
                 ((uint64_t)((l >> 2) & 3) << 22) |
                 ((uint64_t)((h >> 16) & 0xff) << 40) |
                 ((uint64_t)((l >> 4) & 3) << 38) |
                 ((uint64_t)((h >> 24) & 0xff) << 56) |
                 ((uint64_t)((l >> 6) & 3) << 54);
        AV_WL64(dst + 2 * offset, packed);
    }
    for (; offset < samples; offset++) {
        unsigned int source = swap_uv ? offset ^ 1U : offset;
        unsigned int sample = (high[source] << 8) |
                              (((low[source / 4] >> (2 * (source & 3))) & 3) << 6);
        AV_WL16(dst + 2 * offset, sample);
    }
}

static void v4l2request_cv200_detile(AVFrame *dst, const uint8_t *src,
                                    const V4L2RequestHT10Layout *layout)
{
    for (unsigned int plane = 0; plane < 2; plane++) {
        unsigned int rows = plane ? (dst->height + 1) / 2 : dst->height;
        unsigned int width = plane ? FFALIGN(dst->width, 2) : dst->width;

        for (unsigned int row = 0; row < rows; row++) {
            for (unsigned int column = 0; column < width; column += 64) {
                unsigned int tile_x = column / 64;
                unsigned int high_index, low_row;
                size_t high_source, low_source;

                if (!plane) {
                    high_index = (row & 15) + 16 * (((row / 16) & 1) * 4 + (tile_x & 3));
                    high_source = layout->stride * 16 * (row / 16) +
                                  1024 * tile_x + 64 * v4l2request_cv200_row_y[high_index];
                    low_row = v4l2request_cv200_row_y_2bit((row & 31) + ((tile_x & 3) << 5));
                    low_source = layout->low2_y + layout->stride * 8 * (row / 32) +
                                 512 * tile_x + 16 * low_row;
                } else {
                    high_index = (row & 7) + 8 * (((row / 8) & 1) * 4 + (tile_x & 3));
                    high_source = layout->high8_uv + layout->stride * 8 * (row / 8) +
                                  512 * tile_x + 64 * v4l2request_cv200_row_uv[high_index];
                    low_row = ((row & 15) >> 1) + ((row & 1) << 3);
                    low_source = layout->low2_uv + layout->stride * 4 * (row / 16) +
                                 256 * tile_x + 16 * low_row;
                }
                v4l2request_cv200_unpack(dst->data[plane] + (size_t)row * dst->linesize[plane] +
                                        2 * column, src + high_source, src + low_source,
                                        FFMIN(width - column, 64), plane);
            }
        }
    }
}

static int v4l2request_transfer_ht10(AVHWFramesContext *hwfc,
                                     AVFrame *dst, const AVFrame *src)
{
    const AVV4L2RequestFrameDescriptor *request = (const void *)src->data[0];
    const AVV4L2RequestFramesContext *frames = hwfc->hwctx;
    const AVDRMFrameDescriptor *desc = &request->drm;
    const AVDRMLayerDescriptor *layer = &desc->layers[0];
    const AVDRMObjectDescriptor *object = &desc->objects[0];
    V4L2RequestHT10Layout layout;
    struct dma_buf_sync sync = { .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
    void *address;
    int ret;

    if (desc->nb_objects != 1 || desc->nb_layers != 1 ||
        layer->format != DRM_FORMAT_YUV420_10BIT || layer->nb_planes != 1 ||
        layer->planes[0].object_index || layer->planes[0].offset ||
        object->format_modifier != DRM_FORMAT_MOD_HISTB_VDH_10BIT || object->fd < 0 ||
        frames->coded_width != request->coded_width ||
        frames->coded_height != request->coded_height ||
        ff_v4l2request_ht10_layout(request->coded_width, request->coded_height,
                                  layer->planes[0].pitch, object->size, &layout) < 0 ||
        src->width <= 0 || src->height <= 0 ||
        src->width > request->coded_width || src->height > request->coded_height ||
        dst->format != AV_PIX_FMT_P010 || dst->width <= 0 || dst->height <= 0 ||
        dst->width > src->width || dst->height > src->height ||
        !dst->data[0] || !dst->data[1] ||
        dst->linesize[0] < 2 * dst->width ||
        dst->linesize[1] < 2 * FFALIGN(dst->width, 2))
        return AVERROR(EINVAL);

    if (request->wait) {
        ret = request->wait(request->wait_opaque);
        if (ret < 0)
            return ret;
    }

    address = mmap(NULL, object->size, PROT_READ, MAP_SHARED, object->fd, 0);
    if (address == MAP_FAILED)
        return AVERROR(errno);
    if (ioctl(object->fd, DMA_BUF_IOCTL_SYNC, &sync) < 0) {
        ret = AVERROR(errno);
        goto unmap;
    }

    /* Validated full allocation geometry bounds every tile in all four
     * regions, including partial visible rows and final 64-sample tiles. */
    v4l2request_cv200_detile(dst, address, &layout);
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ret = ioctl(object->fd, DMA_BUF_IOCTL_SYNC, &sync) < 0 ? AVERROR(errno) : 0;
unmap:
    munmap(address, object->size);
    return ret;
}

static void v4l2request_unmap_ht10(AVHWFramesContext *hwfc, HWMapDescriptor *hwmap)
{
    AVFrame *linear = hwmap->priv;

    av_frame_free(&linear);
}

static int v4l2request_map_ht10(AVHWFramesContext *hwfc, AVFrame *dst,
                                const AVFrame *src, int flags)
{
    AVFrame *linear;
    int ret;

    /* This is a read-only detiled software map, never a direct pixel view. */
    if (flags & (AV_HWFRAME_MAP_DIRECT | AV_HWFRAME_MAP_WRITE))
        return AVERROR(ENOSYS);
    linear = av_frame_alloc();
    if (!linear)
        return AVERROR(ENOMEM);
    linear->format = AV_PIX_FMT_P010;
    linear->width = src->width;
    linear->height = src->height;
    ret = av_frame_get_buffer(linear, 0);
    if (ret < 0)
        goto fail;
    ret = v4l2request_transfer_ht10(hwfc, linear, src);
    if (ret < 0)
        goto fail;
    ret = ff_hwframe_map_create(src->hw_frames_ctx, dst, src,
                                v4l2request_unmap_ht10, linear);
    if (ret < 0)
        goto fail;
    for (int i = 0; i < 2; i++) {
        dst->data[i] = linear->data[i];
        dst->linesize[i] = linear->linesize[i];
    }
    dst->width = linear->width;
    dst->height = linear->height;
    return 0;
fail:
    av_frame_free(&linear);
    return ret;
}

static int v4l2request_map_frame(AVHWFramesContext *hwfc,
                                 AVFrame *dst, const AVFrame *src)
{
    const AVV4L2RequestFrameDescriptor *request_desc =
        (AVV4L2RequestFrameDescriptor *)src->data[0];
    const AVDRMFrameDescriptor *desc = &request_desc->drm;
    struct dma_buf_sync sync = {
        .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ,
    };
    DRMMapping *map;
    int ret, i, p, plane;
    void *addr;


    /* A tiled surface must never reach generic linear plane mapping. */
    if (v4l2request_is_ht10(src))
        return AVERROR(ENOSYS);
    if (request_desc->wait) {
        ret = request_desc->wait(request_desc->wait_opaque);
        if (ret < 0)
            return ret;
    }

    map = av_mallocz(sizeof(*map));
    if (!map)
        return AVERROR(ENOMEM);

    av_assert0(desc->nb_objects <= AV_DRM_MAX_PLANES);
    for (i = 0; i < desc->nb_objects; i++) {
        addr = mmap(NULL, desc->objects[i].size, AV_HWFRAME_MAP_READ, MAP_SHARED,
                    desc->objects[i].fd, 0);
        if (addr == MAP_FAILED) {
            av_log(hwfc, AV_LOG_ERROR, "Failed to map DRM object %d to memory: %s (%d)\n",
                   desc->objects[i].fd, strerror(errno), errno);
            ret = AVERROR(errno);
            goto fail;
        }

        map->address[i] = addr;
        map->length[i]  = desc->objects[i].size;
        map->object[i]  = desc->objects[i].fd;

        /*
         * We're not checking for errors here because the kernel may not
         * support the ioctl, in which case its okay to carry on
         */
        ioctl(desc->objects[i].fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    map->nb_regions = i;

    plane = 0;
    for (i = 0; i < desc->nb_layers; i++) {
        const AVDRMLayerDescriptor *layer = &desc->layers[i];
        for (p = 0; p < layer->nb_planes; p++) {
            dst->data[plane] =
                (uint8_t *)map->address[layer->planes[p].object_index] +
                                        layer->planes[p].offset;
            dst->linesize[plane] =      layer->planes[p].pitch;
            ++plane;
        }
    }
    av_assert0(plane <= AV_DRM_MAX_PLANES);

    dst->width  = src->width;
    dst->height = src->height;

    ret = ff_hwframe_map_create(src->hw_frames_ctx, dst, src,
                                v4l2request_unmap_frame, map);
    if (ret < 0)
        goto fail;

    return 0;

fail:
    for (i = 0; i < desc->nb_objects; i++) {
        if (map->address[i])
            munmap(map->address[i], map->length[i]);
    }
    av_free(map);
    return ret;
}

static int v4l2request_map_from(AVHWFramesContext *hwfc, AVFrame *dst,
                                const AVFrame *src, int flags)
{
    const AVV4L2RequestFrameDescriptor *desc =
        (const AVV4L2RequestFrameDescriptor *)src->data[0];
    int ret;

    if (dst->format == AV_PIX_FMT_NONE)
        dst->format = hwfc->sw_format;
    if (dst->format == hwfc->sw_format && !(flags & AV_HWFRAME_MAP_WRITE)) {
        ret = v4l2request_is_ht10(src) ?
              v4l2request_map_ht10(hwfc, dst, src, flags) :
              v4l2request_map_frame(hwfc, dst, src);
        if (ret < 0)
            return ret;
        return av_frame_copy_props(dst, src);
    }

    /* Foreign hardware importers must not bypass capture-error checks. */
    if (desc->wait) {
        ret = desc->wait(desc->wait_opaque);
        if (ret < 0)
            return ret;
    }
    return AVERROR(ENOSYS);
}

static int v4l2request_transfer_get_formats(AVHWFramesContext *hwfc,
                                            enum AVHWFrameTransferDirection dir,
                                            enum AVPixelFormat **formats)
{
    enum AVPixelFormat *pix_fmts;

    if (dir == AV_HWFRAME_TRANSFER_DIRECTION_TO)
        return AVERROR(ENOSYS);

    pix_fmts = av_malloc_array(2, sizeof(*pix_fmts));
    if (!pix_fmts)
        return AVERROR(ENOMEM);

    pix_fmts[0] = hwfc->sw_format;
    pix_fmts[1] = AV_PIX_FMT_NONE;

    *formats = pix_fmts;
    return 0;
}

static int v4l2request_transfer_data_from(AVHWFramesContext *hwfc,
                                          AVFrame *dst, const AVFrame *src)
{
    AVFrame *map;
    int ret;

    if (dst->width > hwfc->width || dst->height > hwfc->height)
        return AVERROR(EINVAL);

    if (v4l2request_is_ht10(src))
        return v4l2request_transfer_ht10(hwfc, dst, src);

    map = av_frame_alloc();
    if (!map)
        return AVERROR(ENOMEM);
    map->format = dst->format;

    ret = v4l2request_map_frame(hwfc, map, src);
    if (ret)
        goto fail;

    map->width  = dst->width;
    map->height = dst->height;

    ret = av_frame_copy(dst, map);
    if (ret)
        goto fail;

    ret = 0;
fail:
    av_frame_free(&map);
    return ret;
}

const HWContextType ff_hwcontext_type_v4l2request = {
    .type                   = AV_HWDEVICE_TYPE_V4L2REQUEST,
    .name                   = "V4L2 Request API",

    .device_hwctx_size      = sizeof(AVV4L2RequestDeviceContext),
    .frames_hwctx_size      = sizeof(AVV4L2RequestFramesContext),
    .device_create          = v4l2request_device_create,
    .device_init            = v4l2request_device_init,

    .frames_get_buffer      = v4l2request_get_buffer,

    .transfer_get_formats   = v4l2request_transfer_get_formats,
    .transfer_data_from     = v4l2request_transfer_data_from,
    .map_from               = v4l2request_map_from,

    .pix_fmts = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_DRM_PRIME,
        AV_PIX_FMT_NONE
    },
};
