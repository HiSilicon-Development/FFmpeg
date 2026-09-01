/*
 * V4L2 buffer helper functions.
 *
 * Copyright (C) 2017 Alexis Ballier <aballier@gentoo.org>
 * Copyright (C) 2017 Jorge Ramirez <jorge.ramirez-ortiz@linaro.org>
 *
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

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "libavcodec/avcodec.h"
#include "libavutil/attributes.h"
#include "libavutil/pixdesc.h"
#include "libavutil/refstruct.h"
#include "v4l2_context.h"
#include "v4l2_buffers.h"
#include "v4l2_m2m.h"

#ifndef USEC_PER_SEC
#define USEC_PER_SEC 1000000
#endif

static AVRational v4l2_timebase = { 1, USEC_PER_SEC };

static inline V4L2m2mContext *buf_to_m2mctx(V4L2Buffer *buf)
{
    return V4L2_TYPE_IS_OUTPUT(buf->context->type) ?
        container_of(buf->context, V4L2m2mContext, output) :
        container_of(buf->context, V4L2m2mContext, capture);
}

static inline AVCodecContext *logger(V4L2Buffer *buf)
{
    return buf_to_m2mctx(buf)->avctx;
}

static inline AVRational v4l2_get_timebase(V4L2Buffer *avbuf)
{
    V4L2m2mContext *s = buf_to_m2mctx(avbuf);

    if (s->avctx->pkt_timebase.num)
        return s->avctx->pkt_timebase;
    return s->avctx->time_base;
}

static inline void v4l2_set_pts(V4L2Buffer *out, int64_t pts)
{
    int64_t v4l2_pts;

    if (pts == AV_NOPTS_VALUE)
        pts = 0;

    /* convert pts to v4l2 timebase */
    v4l2_pts = av_rescale_q(pts, v4l2_get_timebase(out), v4l2_timebase);
    out->buf.timestamp.tv_usec = v4l2_pts % USEC_PER_SEC;
    out->buf.timestamp.tv_sec = v4l2_pts / USEC_PER_SEC;
}

static inline int64_t v4l2_get_pts(V4L2Buffer *avbuf)
{
    int64_t v4l2_pts;

    /* convert pts back to encoder timebase */
    v4l2_pts = (int64_t)avbuf->buf.timestamp.tv_sec * USEC_PER_SEC +
                        avbuf->buf.timestamp.tv_usec;

    return av_rescale_q(v4l2_pts, v4l2_timebase, v4l2_get_timebase(avbuf));
}

static enum AVColorPrimaries v4l2_get_color_primaries(V4L2Buffer *buf)
{
    enum v4l2_ycbcr_encoding ycbcr;
    enum v4l2_colorspace cs;

    cs = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.colorspace :
        buf->context->format.fmt.pix.colorspace;

    ycbcr = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.ycbcr_enc:
        buf->context->format.fmt.pix.ycbcr_enc;

    switch(ycbcr) {
    case V4L2_YCBCR_ENC_XV709:
    case V4L2_YCBCR_ENC_709: return AVCOL_PRI_BT709;
    case V4L2_YCBCR_ENC_XV601:
    case V4L2_YCBCR_ENC_601:return AVCOL_PRI_BT470M;
    default:
        break;
    }

    switch(cs) {
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_PRI_BT470BG;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_PRI_SMPTE170M;
    case V4L2_COLORSPACE_SMPTE240M: return AVCOL_PRI_SMPTE240M;
    case V4L2_COLORSPACE_BT2020: return AVCOL_PRI_BT2020;
    default:
        break;
    }

    return AVCOL_PRI_UNSPECIFIED;
}

static enum AVColorRange v4l2_get_color_range(V4L2Buffer *buf)
{
    enum v4l2_quantization qt;

    qt = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.quantization :
        buf->context->format.fmt.pix.quantization;

    switch (qt) {
    case V4L2_QUANTIZATION_LIM_RANGE: return AVCOL_RANGE_MPEG;
    case V4L2_QUANTIZATION_FULL_RANGE: return AVCOL_RANGE_JPEG;
    default:
        break;
    }

     return AVCOL_RANGE_UNSPECIFIED;
}

static enum AVColorSpace v4l2_get_color_space(V4L2Buffer *buf)
{
    enum v4l2_ycbcr_encoding ycbcr;
    enum v4l2_colorspace cs;

    cs = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.colorspace :
        buf->context->format.fmt.pix.colorspace;

    ycbcr = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.ycbcr_enc:
        buf->context->format.fmt.pix.ycbcr_enc;

    switch(cs) {
    case V4L2_COLORSPACE_SRGB: return AVCOL_SPC_RGB;
    case V4L2_COLORSPACE_REC709: return AVCOL_SPC_BT709;
    case V4L2_COLORSPACE_470_SYSTEM_M: return AVCOL_SPC_FCC;
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_SPC_BT470BG;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_SPC_SMPTE170M;
    case V4L2_COLORSPACE_SMPTE240M: return AVCOL_SPC_SMPTE240M;
    case V4L2_COLORSPACE_BT2020:
        if (ycbcr == V4L2_YCBCR_ENC_BT2020_CONST_LUM)
            return AVCOL_SPC_BT2020_CL;
        else
             return AVCOL_SPC_BT2020_NCL;
    default:
        break;
    }

    return AVCOL_SPC_UNSPECIFIED;
}

static enum AVColorTransferCharacteristic v4l2_get_color_trc(V4L2Buffer *buf)
{
    enum v4l2_ycbcr_encoding ycbcr;
    enum v4l2_xfer_func xfer;
    enum v4l2_colorspace cs;

    cs = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.colorspace :
        buf->context->format.fmt.pix.colorspace;

    ycbcr = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.ycbcr_enc:
        buf->context->format.fmt.pix.ycbcr_enc;

    xfer = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.xfer_func:
        buf->context->format.fmt.pix.xfer_func;

    switch (xfer) {
    case V4L2_XFER_FUNC_709: return AVCOL_TRC_BT709;
    case V4L2_XFER_FUNC_SRGB: return AVCOL_TRC_IEC61966_2_1;
    case V4L2_XFER_FUNC_SMPTE240M: return AVCOL_TRC_SMPTE240M;
    case V4L2_XFER_FUNC_NONE: return AVCOL_TRC_LINEAR;
    case V4L2_XFER_FUNC_SMPTE2084: return AVCOL_TRC_SMPTE2084;
    default:
        break;
    }

    switch (cs) {
    case V4L2_COLORSPACE_470_SYSTEM_M: return AVCOL_TRC_GAMMA22;
    case V4L2_COLORSPACE_470_SYSTEM_BG: return AVCOL_TRC_GAMMA28;
    case V4L2_COLORSPACE_SMPTE170M: return AVCOL_TRC_SMPTE170M;
    case V4L2_COLORSPACE_SMPTE240M: return AVCOL_TRC_SMPTE240M;
    default:
        break;
    }

    switch (ycbcr) {
    case V4L2_YCBCR_ENC_XV709:
    case V4L2_YCBCR_ENC_XV601: return AVCOL_TRC_BT1361_ECG;
    default:
        break;
    }

    return AVCOL_TRC_UNSPECIFIED;
}

static void v4l2_get_interlacing(AVFrame *frame, V4L2Buffer *buf)
{
    enum v4l2_field field = V4L2_TYPE_IS_MULTIPLANAR(buf->buf.type) ?
        buf->context->format.fmt.pix_mp.field :
        buf->context->format.fmt.pix.field;

    switch (field) {
    case V4L2_FIELD_INTERLACED:
    case V4L2_FIELD_INTERLACED_TB:
        frame->flags |=  AV_FRAME_FLAG_TOP_FIELD_FIRST;
        av_fallthrough;
    case V4L2_FIELD_INTERLACED_BT:
        frame->flags |=  AV_FRAME_FLAG_INTERLACED;
        break;
    }
}

static void v4l2_free_buffer(void *opaque, uint8_t *unused)
{
    V4L2Buffer* avbuf = opaque;
    V4L2m2mContext *s = buf_to_m2mctx(avbuf);

    if (atomic_fetch_sub(&avbuf->context_refcount, 1) == 1) {
        atomic_fetch_sub_explicit(&s->refcount, 1, memory_order_acq_rel);

        if (s->reinit) {
            if (!atomic_load(&s->refcount))
                sem_post(&s->refsync);
        } else {
            if (s->draining && V4L2_TYPE_IS_OUTPUT(avbuf->context->type)) {
                /* no need to queue more buffers to the driver */
                avbuf->status = V4L2BUF_AVAILABLE;
            }
            else if (avbuf->context->streamon)
                ff_v4l2_buffer_enqueue(avbuf);
        }

        av_refstruct_unref(&avbuf->context_ref);
    }
}

static int v4l2_buf_increase_ref(V4L2Buffer *in)
{
    V4L2m2mContext *s = buf_to_m2mctx(in);

    if (in->context_ref)
        atomic_fetch_add(&in->context_refcount, 1);
    else {
        in->context_ref = av_refstruct_ref(s->self_ref);

        in->context_refcount = 1;
    }

    in->status = V4L2BUF_RET_USER;
    atomic_fetch_add_explicit(&s->refcount, 1, memory_order_relaxed);

    return 0;
}

static int v4l2_buf_to_bufref(V4L2Buffer *in, int plane, AVBufferRef **buf)
{
    int ret;

    if (plane >= in->num_planes)
        return AVERROR(EINVAL);

    /* even though most encoders return 0 in data_offset encoding vp8 does require this value */
    *buf = av_buffer_create((char *)in->plane_info[plane].mm_addr + in->planes[plane].data_offset,
                            in->plane_info[plane].length, v4l2_free_buffer, in, 0);
    if (!*buf)
        return AVERROR(ENOMEM);

    ret = v4l2_buf_increase_ref(in);
    if (ret)
        av_buffer_unref(buf);

    return ret;
}

static int v4l2_bufref_to_buf(V4L2Buffer *out, int plane, const uint8_t *data,
                              size_t size, size_t offset)
{
    size_t length;

    if (plane < 0 || plane >= out->num_planes || (!data && size))
        return AVERROR(EINVAL);

    length = out->plane_info[plane].length;
    if (length > UINT32_MAX)
        return AVERROR(EINVAL);
    if (offset > length || size > length - offset)
        return AVERROR(ENOSPC);
    if (size)
        memcpy((uint8_t *)out->plane_info[plane].mm_addr + offset, data, size);

    if (V4L2_TYPE_IS_MULTIPLANAR(out->buf.type)) {
        out->planes[plane].bytesused = offset + size;
        out->planes[plane].length = length;
    } else {
        out->buf.bytesused = offset + size;
        out->buf.length = length;
    }

    return 0;
}

static int v4l2_frame_plane_bounds(const AVFrame *frame, int plane,
                                   size_t row_bytes, unsigned int rows)
{
    const AVBufferRef *buf;
    uintptr_t buf_addr, data_addr, first_addr;
    size_t abs_stride, back, offset, span;
    int stride;

    if (plane < 0 || plane >= AV_NUM_DATA_POINTERS || !rows || !row_bytes ||
        !frame->data[plane])
        return AVERROR(EINVAL);

    stride = frame->linesize[plane];
    abs_stride = stride < 0 ? -(int64_t)stride : stride;
    if (abs_stride < row_bytes || rows - 1 > (SIZE_MAX - row_bytes) / abs_stride)
        return AVERROR(EINVAL);

    back = (size_t)(rows - 1) * abs_stride;
    span = back + row_bytes;
    if (back > PTRDIFF_MAX || span > PTRDIFF_MAX)
        return AVERROR(EINVAL);

    buf = av_frame_get_plane_buffer(frame, plane);
    if (!buf)
        return AVERROR(EINVAL);

    data_addr = (uintptr_t)frame->data[plane];
    if (stride < 0 && back > data_addr)
        return AVERROR(EINVAL);
    first_addr = stride < 0 ? data_addr - back : data_addr;
    buf_addr = (uintptr_t)buf->data;
    if (first_addr < buf_addr)
        return AVERROR(EINVAL);

    offset = first_addr - buf_addr;
    if (offset > buf->size || span > buf->size - offset)
        return AVERROR(EINVAL);

    return 0;
}

static int v4l2_buffer_swframe_nv12_to_buf(const AVFrame *frame,
                                            V4L2Buffer *out,
                                            uint32_t pixel_format)
{
    const struct v4l2_pix_format *pix = &out->context->format.fmt.pix;
    uint8_t *dst = out->plane_info[0].mm_addr;
    unsigned int src_width, src_height, src_chroma_rows;
    unsigned int dst_width = pix->width;
    unsigned int dst_height = pix->height;
    unsigned int dst_chroma_rows;
    size_t src_chroma_width, dst_chroma_width;
    size_t dst_stride, luma_size, image_size;
    unsigned int y;
    int ret;

    if (out->num_planes != 1 ||
        frame->format != (pixel_format == V4L2_PIX_FMT_NV12 ?
                         AV_PIX_FMT_NV12 : AV_PIX_FMT_NV21) ||
        frame->width <= 0 || frame->height <= 0 ||
        frame->width != out->context->width ||
        frame->height != out->context->height)
        return AVERROR(EINVAL);

    src_width = frame->width;
    src_height = frame->height;
    if (!dst_width || !dst_height || src_width > dst_width ||
        src_height > dst_height)
        return AVERROR(EINVAL);

    src_chroma_width = (size_t)src_width + (src_width & 1);
    dst_chroma_width = (size_t)dst_width + (dst_width & 1);
    src_chroma_rows = (src_height >> 1) + (src_height & 1);
    dst_chroma_rows = (dst_height >> 1) + (dst_height & 1);
    if (!dst || out->plane_info[0].bytesperline <= 0)
        return AVERROR(EINVAL);
    dst_stride = out->plane_info[0].bytesperline;
    if (dst_stride < dst_width || dst_stride < dst_chroma_width ||
        dst_height > SIZE_MAX / dst_stride)
        return AVERROR(EINVAL);

    luma_size = dst_stride * dst_height;
    if (dst_chroma_rows > (SIZE_MAX - luma_size) / dst_stride)
        return AVERROR(EINVAL);
    image_size = luma_size + dst_stride * dst_chroma_rows;
    if (out->plane_info[0].length > UINT32_MAX)
        return AVERROR(EINVAL);
    if (pix->sizeimage < image_size ||
        pix->sizeimage > out->plane_info[0].length)
        return AVERROR(ENOSPC);

    ret = v4l2_frame_plane_bounds(frame, 0, src_width, src_height);
    if (ret < 0)
        return ret;
    ret = v4l2_frame_plane_bounds(frame, 1, src_chroma_width,
                                  src_chroma_rows);
    if (ret < 0)
        return ret;

    memset(dst, 0, pix->sizeimage);
    for (y = 0; y < src_height; y++)
        memcpy(dst + y * dst_stride,
               frame->data[0] + (ptrdiff_t)y * frame->linesize[0],
               src_width);
    dst += luma_size;
    for (y = 0; y < src_chroma_rows; y++)
        memcpy(dst + y * dst_stride,
               frame->data[1] + (ptrdiff_t)y * frame->linesize[1],
               src_chroma_width);

    out->buf.bytesused = pix->sizeimage;
    out->buf.length = out->plane_info[0].length;
    return 0;
}

static int v4l2_buffer_buf_to_swframe(AVFrame *frame, V4L2Buffer *avbuf)
{
    int i, ret;

    frame->format = avbuf->context->av_pix_fmt;

    for (i = 0; i < avbuf->num_planes; i++) {
        ret = v4l2_buf_to_bufref(avbuf, i, &frame->buf[i]);
        if (ret)
            return ret;

        frame->linesize[i] = avbuf->plane_info[i].bytesperline;
        frame->data[i] = frame->buf[i]->data;
    }

    /* fixup special cases */
    switch (avbuf->context->av_pix_fmt) {
    case AV_PIX_FMT_NV12:
    case AV_PIX_FMT_NV21:
        if (avbuf->num_planes > 1)
            break;
        frame->linesize[1] = avbuf->plane_info[0].bytesperline;
        frame->data[1] = frame->buf[0]->data + avbuf->plane_info[0].bytesperline * avbuf->context->format.fmt.pix_mp.height;
        break;

    case AV_PIX_FMT_YUV420P:
        if (avbuf->num_planes > 1)
            break;
        frame->linesize[1] = avbuf->plane_info[0].bytesperline >> 1;
        frame->linesize[2] = avbuf->plane_info[0].bytesperline >> 1;
        frame->data[1] = frame->buf[0]->data + avbuf->plane_info[0].bytesperline * avbuf->context->format.fmt.pix_mp.height;
        frame->data[2] = frame->data[1] + ((avbuf->plane_info[0].bytesperline * avbuf->context->format.fmt.pix_mp.height) >> 2);
        break;

    default:
        break;
    }

    return 0;
}

static int v4l2_buffer_swframe_to_buf(const AVFrame *frame, V4L2Buffer *out)
{
    int i, ret;
    struct v4l2_format fmt = out->context->format;
    uint32_t pixel_format = V4L2_TYPE_IS_MULTIPLANAR(fmt.type) ?
                            fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;
    int height       = V4L2_TYPE_IS_MULTIPLANAR(fmt.type) ?
                       fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    int is_planar_format = 0;

    if (!V4L2_TYPE_IS_MULTIPLANAR(fmt.type) &&
        (pixel_format == V4L2_PIX_FMT_NV12 ||
         pixel_format == V4L2_PIX_FMT_NV21))
        return v4l2_buffer_swframe_nv12_to_buf(frame, out, pixel_format);

    switch (pixel_format) {
    case V4L2_PIX_FMT_YUV420M:
    case V4L2_PIX_FMT_YVU420M:
#ifdef V4L2_PIX_FMT_YUV422M
    case V4L2_PIX_FMT_YUV422M:
#endif
#ifdef V4L2_PIX_FMT_YVU422M
    case V4L2_PIX_FMT_YVU422M:
#endif
#ifdef V4L2_PIX_FMT_YUV444M
    case V4L2_PIX_FMT_YUV444M:
#endif
#ifdef V4L2_PIX_FMT_YVU444M
    case V4L2_PIX_FMT_YVU444M:
#endif
    case V4L2_PIX_FMT_NV12M:
    case V4L2_PIX_FMT_NV21M:
    case V4L2_PIX_FMT_NV12MT_16X16:
    case V4L2_PIX_FMT_NV12MT:
    case V4L2_PIX_FMT_NV16M:
    case V4L2_PIX_FMT_NV61M:
        is_planar_format = 1;
    }

    if (!is_planar_format) {
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frame->format);
        int planes_nb = 0;
        size_t offset = 0;

        if (!desc || height <= 0)
            return AVERROR(EINVAL);

        for (i = 0; i < desc->nb_components; i++)
            planes_nb = FFMAX(planes_nb, desc->comp[i].plane + 1);

        for (i = 0; i < planes_nb; i++) {
            size_t size;
            int h = height;
            if (i == 1 || i == 2) {
                h = AV_CEIL_RSHIFT(h, desc->log2_chroma_h);
            }
            if (!frame->data[i] || frame->linesize[i] <= 0 ||
                h > SIZE_MAX / frame->linesize[i])
                return AVERROR(EINVAL);
            size = (size_t)frame->linesize[i] * h;
            if (size > SIZE_MAX - offset)
                return AVERROR(EINVAL);
            ret = v4l2_bufref_to_buf(out, 0, frame->data[i], size, offset);
            if (ret)
                return ret;
            offset += size;
        }
        return 0;
    }

    for (i = 0; i < out->num_planes; i++) {
        ret = v4l2_bufref_to_buf(out, i, frame->buf[i]->data, frame->buf[i]->size, 0);
        if (ret)
            return ret;
    }

    return 0;
}

/******************************************************************************
 *
 *              V4L2Buffer interface
 *
 ******************************************************************************/

int ff_v4l2_buffer_avframe_to_buf(const AVFrame *frame, V4L2Buffer *out)
{
    v4l2_set_pts(out, frame->pts);

    return v4l2_buffer_swframe_to_buf(frame, out);
}

int ff_v4l2_buffer_buf_to_avframe(AVFrame *frame, V4L2Buffer *avbuf)
{
    int ret;

    av_frame_unref(frame);

    /* 1. get references to the actual data */
    ret = v4l2_buffer_buf_to_swframe(frame, avbuf);
    if (ret)
        return ret;

    /* 2. get frame information */
    if (avbuf->buf.flags & V4L2_BUF_FLAG_KEYFRAME)
        frame->flags |= AV_FRAME_FLAG_KEY;
    frame->color_primaries = v4l2_get_color_primaries(avbuf);
    frame->colorspace = v4l2_get_color_space(avbuf);
    frame->color_range = v4l2_get_color_range(avbuf);
    frame->color_trc = v4l2_get_color_trc(avbuf);
    frame->pts = v4l2_get_pts(avbuf);
    frame->pkt_dts = AV_NOPTS_VALUE;
    v4l2_get_interlacing(frame, avbuf);

    /* these values are updated also during re-init in v4l2_process_driver_event */
    frame->height = avbuf->context->height;
    frame->width = avbuf->context->width;
    frame->sample_aspect_ratio = avbuf->context->sample_aspect_ratio;

    /* 3. report errors upstream */
    if (avbuf->buf.flags & V4L2_BUF_FLAG_ERROR) {
        av_log(logger(avbuf), AV_LOG_ERROR, "%s: driver decode error\n", avbuf->context->name);
        frame->decode_error_flags |= FF_DECODE_ERROR_INVALID_BITSTREAM;
    }

    return 0;
}

int ff_v4l2_buffer_buf_to_avpkt(AVPacket *pkt, V4L2Buffer *avbuf)
{
    int ret;

    av_packet_unref(pkt);
    ret = v4l2_buf_to_bufref(avbuf, 0, &pkt->buf);
    if (ret)
        return ret;

    pkt->size = V4L2_TYPE_IS_MULTIPLANAR(avbuf->buf.type) ? avbuf->buf.m.planes[0].bytesused : avbuf->buf.bytesused;
    pkt->data = pkt->buf->data;

    if (avbuf->buf.flags & V4L2_BUF_FLAG_KEYFRAME)
        pkt->flags |= AV_PKT_FLAG_KEY;

    if (avbuf->buf.flags & V4L2_BUF_FLAG_ERROR) {
        av_log(logger(avbuf), AV_LOG_ERROR, "%s driver encode error\n", avbuf->context->name);
        pkt->flags |= AV_PKT_FLAG_CORRUPT;
    }

    pkt->dts = pkt->pts = v4l2_get_pts(avbuf);

    return 0;
}

int ff_v4l2_buffer_avpkt_to_buf(const AVPacket *pkt, V4L2Buffer *out)
{
    int ret;

    if (pkt->size < 0 || pkt->size > out->plane_info[0].length) {
        av_log(logger(out), AV_LOG_ERROR,
               "compressed packet size %d exceeds output buffer size %zu\n",
               pkt->size, out->plane_info[0].length);
        return AVERROR(ENOSPC);
    }

    ret = v4l2_bufref_to_buf(out, 0, pkt->data, pkt->size, 0);
    if (ret)
        return ret;

    v4l2_set_pts(out, pkt->pts);

    if (pkt->flags & AV_PKT_FLAG_KEY)
        out->flags = V4L2_BUF_FLAG_KEYFRAME;

    return 0;
}

int ff_v4l2_buffer_initialize(V4L2Buffer* avbuf, int index)
{
    V4L2Context *ctx = avbuf->context;
    int ret, i;

    avbuf->buf.memory = V4L2_MEMORY_MMAP;
    avbuf->buf.type = ctx->type;
    avbuf->buf.index = index;

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->type)) {
        avbuf->buf.length = VIDEO_MAX_PLANES;
        avbuf->buf.m.planes = avbuf->planes;
    }

    ret = ioctl(buf_to_m2mctx(avbuf)->fd, VIDIOC_QUERYBUF, &avbuf->buf);
    if (ret < 0)
        return AVERROR(errno);

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->type)) {
        avbuf->num_planes = 0;
        /* in MP, the V4L2 API states that buf.length means num_planes */
        for (i = 0; i < avbuf->buf.length; i++) {
            if (avbuf->buf.m.planes[i].length)
                avbuf->num_planes++;
        }
    } else
        avbuf->num_planes = 1;

    for (i = 0; i < avbuf->num_planes; i++) {

        avbuf->plane_info[i].bytesperline = V4L2_TYPE_IS_MULTIPLANAR(ctx->type) ?
            ctx->format.fmt.pix_mp.plane_fmt[i].bytesperline :
            ctx->format.fmt.pix.bytesperline;

        if (V4L2_TYPE_IS_MULTIPLANAR(ctx->type)) {
            avbuf->plane_info[i].length = avbuf->buf.m.planes[i].length;
            avbuf->plane_info[i].mm_addr = mmap(NULL, avbuf->buf.m.planes[i].length,
                                           PROT_READ | PROT_WRITE, MAP_SHARED,
                                           buf_to_m2mctx(avbuf)->fd, avbuf->buf.m.planes[i].m.mem_offset);
        } else {
            avbuf->plane_info[i].length = avbuf->buf.length;
            avbuf->plane_info[i].mm_addr = mmap(NULL, avbuf->buf.length,
                                          PROT_READ | PROT_WRITE, MAP_SHARED,
                                          buf_to_m2mctx(avbuf)->fd, avbuf->buf.m.offset);
        }

        if (avbuf->plane_info[i].mm_addr == MAP_FAILED)
            return AVERROR(ENOMEM);
    }

    avbuf->status = V4L2BUF_AVAILABLE;

    if (V4L2_TYPE_IS_OUTPUT(ctx->type))
        return 0;

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->type)) {
        avbuf->buf.m.planes = avbuf->planes;
        avbuf->buf.length   = avbuf->num_planes;

    } else {
        avbuf->buf.bytesused = avbuf->planes[0].bytesused;
        avbuf->buf.length    = avbuf->planes[0].length;
    }

    return ff_v4l2_buffer_enqueue(avbuf);
}

int ff_v4l2_buffer_enqueue(V4L2Buffer* avbuf)
{
    int ret;

    avbuf->buf.flags = avbuf->flags;

    ret = ioctl(buf_to_m2mctx(avbuf)->fd, VIDIOC_QBUF, &avbuf->buf);
    if (ret < 0)
        return AVERROR(errno);

    avbuf->status = V4L2BUF_IN_DRIVER;

    return 0;
}
