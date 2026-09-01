/*
 * V4L2 mem2mem JPEG codec
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
#include <string.h>
#include <sys/ioctl.h>
#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "codec_internal.h"
#include "libavcodec/decode.h"
#include "encode.h"

#include "v4l2_context.h"
#include "v4l2_m2m.h"
#include "v4l2_fmt.h"

/*
 * The HiSilicon JPEG block is a mem2mem codec with NV12 on the raw side and
 * JPEG on the coded side (drivers/media/platform/hisilicon/histb-jpge.c, which
 * enumerates V4L2_PIX_FMT_JPEG and V4L2_PIX_FMT_MJPEG on its capture queue and
 * V4L2_PIX_FMT_NV12 on its output queue).  Because ff_v4l2_format_avcodec_to_v4l2()
 * maps AV_CODEC_ID_MJPEG to V4L2_PIX_FMT_MJPEG, the coded side is driven as
 * MJPEG; both wrappers below therefore keep V4L2_PIX_FMT_MJPEG as the coded
 * format and AV_PIX_FMT_NV12 as the raw one.
 */

/******************************************************************************
 *
 *                  Encoder
 *
 ******************************************************************************/

static inline void v4l2_subscribe_eos_event(V4L2m2mContext *s)
{
    struct v4l2_event_subscription sub;

    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_EOS;
    if (ioctl(s->fd, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0)
        av_log(s->avctx, AV_LOG_WARNING,
               "the v4l2 driver does not support end of stream VIDIOC_SUBSCRIBE_EVENT\n");
}

/*
 * The block carries a single control: the libjpeg compression quality, 1-100,
 * default 75.  global_quality holds it exactly as it does for the VAAPI JPEG
 * encoder, with FF_QP2LAMBDA applied when the caller used a quantizer scale.
 * Nothing is written when the caller left it unset, so the block's own
 * default (75) stands.
 */
static inline void v4l2_set_jpeg_quality(V4L2m2mContext *s)
{
    AVCodecContext *avctx = s->avctx;
    struct v4l2_ext_controls ctrls = { { 0 } };
    struct v4l2_ext_control ctrl = { 0 };
    int quality = avctx->global_quality;

    if (quality <= 0)
        return;

    if (avctx->flags & AV_CODEC_FLAG_QSCALE)
        quality /= FF_QP2LAMBDA;

    if (quality < 1 || quality > 100) {
        av_log(avctx, AV_LOG_WARNING,
               "Invalid JPEG quality %d (must be 1-100), using the driver default\n",
               quality);
        return;
    }

    ctrls.ctrl_class = V4L2_CTRL_CLASS_JPEG;
    ctrls.controls = &ctrl;
    ctrls.count = 1;

    ctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
    ctrl.value = quality;

    if (ioctl(s->fd, VIDIOC_S_EXT_CTRLS, &ctrls) < 0)
        av_log(avctx, AV_LOG_WARNING, "Failed to set JPEG compression quality: %s\n",
               strerror(errno));
    else
        av_log(avctx, AV_LOG_DEBUG, "Encoder: JPEG compression quality = %d\n", quality);
}

static int v4l2_prepare_encoder(V4L2m2mContext *s)
{
    /**
     * requirements
     */
    v4l2_subscribe_eos_event(s);

    /**
     * settings
     */
    v4l2_set_jpeg_quality(s);

    return 0;
}

static int v4l2_send_frame(AVCodecContext *avctx, const AVFrame *frame)
{
    V4L2m2mContext *s = ((V4L2m2mPriv*)avctx->priv_data)->context;
    V4L2Context *const output = &s->output;

    return ff_v4l2_context_enqueue_frame(output, frame);
}

static int v4l2_receive_packet(AVCodecContext *avctx, AVPacket *avpkt)
{
    V4L2m2mContext *s = ((V4L2m2mPriv*)avctx->priv_data)->context;
    V4L2Context *const capture = &s->capture;
    V4L2Context *const output = &s->output;
    AVFrame *frame = s->frame;
    int ret;

    if (s->draining)
        goto dequeue;

    if (!frame->buf[0]) {
        ret = ff_encode_get_frame(avctx, frame);
        if (ret < 0 && ret != AVERROR_EOF)
            return ret;

        if (ret == AVERROR_EOF)
            frame = NULL;
    }

    ret = v4l2_send_frame(avctx, frame);
    if (ret != AVERROR(EAGAIN))
        av_frame_unref(frame);

    if (ret < 0 && ret != AVERROR(EAGAIN))
        return ret;

    if (!output->streamon) {
        ret = ff_v4l2_context_set_status(output, VIDIOC_STREAMON);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "VIDIOC_STREAMON failed on output context\n");
            return ret;
        }
    }

    if (!capture->streamon) {
        ret = ff_v4l2_context_set_status(capture, VIDIOC_STREAMON);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "VIDIOC_STREAMON failed on capture context\n");
            return ret;
        }
    }

dequeue:
    return ff_v4l2_context_dequeue_packet(capture, avpkt);
}

static av_cold int v4l2_encode_init(AVCodecContext *avctx)
{
    V4L2Context *capture, *output;
    V4L2m2mContext *s;
    V4L2m2mPriv *priv = avctx->priv_data;
    enum AVPixelFormat pix_fmt_output;
    uint32_t v4l2_fmt_output;
    int ret;

    ret = ff_v4l2_m2m_create_context(priv, &s);
    if (ret < 0)
        return ret;

    capture = &s->capture;
    output  = &s->output;

    /* common settings output/capture */
    output->height = capture->height = avctx->height;
    output->width = capture->width = avctx->width;

    /* output context */
    output->av_codec_id = AV_CODEC_ID_RAWVIDEO;
    output->av_pix_fmt = avctx->pix_fmt;

    /* capture context */
    capture->av_codec_id = avctx->codec_id;
    capture->av_pix_fmt = AV_PIX_FMT_NONE;

    s->avctx = avctx;
    ret = ff_v4l2_m2m_codec_init(priv);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "can't configure encoder\n");
        return ret;
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(output->type))
        v4l2_fmt_output = output->format.fmt.pix_mp.pixelformat;
    else
        v4l2_fmt_output = output->format.fmt.pix.pixelformat;

    pix_fmt_output = ff_v4l2_format_v4l2_to_avfmt(v4l2_fmt_output, AV_CODEC_ID_RAWVIDEO);
    if (pix_fmt_output != avctx->pix_fmt) {
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt_output);
        av_log(avctx, AV_LOG_ERROR, "Encoder requires %s pixel format.\n", desc->name);
        return AVERROR(EINVAL);
    }

    return v4l2_prepare_encoder(s);
}

static av_cold int v4l2_encode_close(AVCodecContext *avctx)
{
    return ff_v4l2_m2m_codec_end(avctx->priv_data);
}

#define OFFSET(x) offsetof(V4L2m2mPriv, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption mjpeg_enc_options[] = {
    V4L_M2M_DEFAULT_OPTS,
    { "num_capture_buffers", "Number of buffers in the capture context",
        OFFSET(num_capture_buffers), AV_OPT_TYPE_INT, {.i64 = 4 }, 4, INT_MAX, FLAGS },
    { NULL },
};

static const FFCodecDefault v4l2_m2m_defaults[] = {
    { "qmin", "-1" },
    { "qmax", "-1" },
    { NULL },
};

#define M2MENC_CLASS(NAME, OPTIONS_NAME) \
    static const AVClass v4l2_m2m_ ## NAME ## _enc_class = { \
        .class_name = #NAME "_v4l2m2m_encoder", \
        .item_name  = av_default_item_name, \
        .option     = OPTIONS_NAME, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define M2MENC(NAME, LONGNAME, OPTIONS_NAME, CODEC) \
    M2MENC_CLASS(NAME, OPTIONS_NAME) \
    const FFCodec ff_ ## NAME ## _v4l2m2m_encoder = { \
        .p.name         = #NAME "_v4l2m2m" , \
        CODEC_LONG_NAME("V4L2 mem2mem " LONGNAME " encoder wrapper"), \
        .p.type         = AVMEDIA_TYPE_VIDEO, \
        .p.id           = CODEC , \
        .priv_data_size = sizeof(V4L2m2mPriv), \
        .p.priv_class   = &v4l2_m2m_ ## NAME ##_enc_class, \
        .init           = v4l2_encode_init, \
        FF_CODEC_RECEIVE_PACKET_CB(v4l2_receive_packet), \
        .close          = v4l2_encode_close, \
        .defaults       = v4l2_m2m_defaults, \
        CODEC_PIXFMTS(AV_PIX_FMT_NV12), \
        .p.capabilities = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_DELAY, \
        .color_ranges   = AVCOL_RANGE_MPEG, \
        .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE | \
                          FF_CODEC_CAP_INIT_CLEANUP, \
        .p.wrapper_name = "v4l2m2m", \
    }

M2MENC(mjpeg, "MJPEG", mjpeg_enc_options, AV_CODEC_ID_MJPEG);

/******************************************************************************
 *
 *                  Decoder
 *
 ******************************************************************************/

static int v4l2_try_start(AVCodecContext *avctx)
{
    V4L2m2mContext *s = ((V4L2m2mPriv*)avctx->priv_data)->context;
    V4L2Context *const capture = &s->capture;
    V4L2Context *const output = &s->output;
    struct v4l2_selection selection = { 0 };
    int ret;

    /* 1. start the output process */
    if (!output->streamon) {
        ret = ff_v4l2_context_set_status(output, VIDIOC_STREAMON);
        if (ret < 0) {
            av_log(avctx, AV_LOG_DEBUG, "VIDIOC_STREAMON on output context\n");
            return ret;
        }
    }

    if (capture->streamon)
        return 0;

    /* 2. get the capture format */
    capture->format.type = capture->type;
    ret = ioctl(s->fd, VIDIOC_G_FMT, &capture->format);
    if (ret) {
        av_log(avctx, AV_LOG_WARNING, "VIDIOC_G_FMT ioctl\n");
        return ret;
    }

    /* 2.1 update the AVCodecContext */
    avctx->pix_fmt = ff_v4l2_format_v4l2_to_avfmt(capture->format.fmt.pix_mp.pixelformat, AV_CODEC_ID_RAWVIDEO);
    capture->av_pix_fmt = avctx->pix_fmt;

    /* 3. set the crop parameters */
    selection.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    selection.r.height = avctx->coded_height;
    selection.r.width = avctx->coded_width;
    ret = ioctl(s->fd, VIDIOC_S_SELECTION, &selection);
    if (!ret) {
        ret = ioctl(s->fd, VIDIOC_G_SELECTION, &selection);
        if (ret) {
            av_log(avctx, AV_LOG_WARNING, "VIDIOC_G_SELECTION ioctl\n");
        } else {
            av_log(avctx, AV_LOG_DEBUG, "crop output %dx%d\n", selection.r.width, selection.r.height);
            /* update the size of the resulting frame */
            capture->height = selection.r.height;
            capture->width  = selection.r.width;
        }
    }

    /* 4. init the capture context now that we have the capture format */
    if (!capture->buffers) {
        ret = ff_v4l2_context_init(capture);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "can't request capture buffers\n");
            return AVERROR(ENOMEM);
        }
    }

    /* 5. start the capture process */
    ret = ff_v4l2_context_set_status(capture, VIDIOC_STREAMON);
    if (ret) {
        av_log(avctx, AV_LOG_DEBUG, "VIDIOC_STREAMON, on capture context\n");
        return ret;
    }

    return 0;
}

static int v4l2_prepare_decoder(V4L2m2mContext *s)
{
    struct v4l2_event_subscription sub;
    V4L2Context *output = &s->output;
    int ret;

    /**
     * requirements
     */
    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_SOURCE_CHANGE;
    ret = ioctl(s->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    if ( ret < 0) {
        if (output->height == 0 || output->width == 0) {
            av_log(s->avctx, AV_LOG_ERROR,
                "the v4l2 driver does not support VIDIOC_SUBSCRIBE_EVENT\n"
                "you must provide codec_height and codec_width on input\n");
            return ret;
        }
    }

    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_EOS;
    ret = ioctl(s->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    if (ret < 0)
        av_log(s->avctx, AV_LOG_WARNING,
               "the v4l2 driver does not support end of stream VIDIOC_SUBSCRIBE_EVENT\n");

    return 0;
}

static int v4l2_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    V4L2m2mContext *s = ((V4L2m2mPriv*)avctx->priv_data)->context;
    V4L2Context *const capture = &s->capture;
    V4L2Context *const output = &s->output;
    int ret;

    if (!s->buf_pkt.size) {
        ret = ff_decode_get_packet(avctx, &s->buf_pkt);
        if (ret < 0) {
            if (ret == AVERROR(EAGAIN))
                return ff_v4l2_context_dequeue_frame(capture, frame, 0);
            else if (ret != AVERROR_EOF)
                return ret;
        }
    }

    if (s->draining)
        goto dequeue;

    ret = ff_v4l2_context_enqueue_packet(output, &s->buf_pkt);
    if (ret < 0 && ret != AVERROR(EAGAIN))
        goto fail;

    /* if EAGAIN don't unref packet and try to enqueue in the next iteration */
    if (ret != AVERROR(EAGAIN))
        av_packet_unref(&s->buf_pkt);

    if (!s->draining) {
        ret = v4l2_try_start(avctx);
        if (ret)
            goto fail;
    }

dequeue:
    return ff_v4l2_context_dequeue_frame(capture, frame, -1);
fail:
    av_packet_unref(&s->buf_pkt);
    return ret;
}

static av_cold int v4l2_decode_init(AVCodecContext *avctx)
{
    V4L2Context *capture, *output;
    V4L2m2mContext *s;
    V4L2m2mPriv *priv = avctx->priv_data;
    int ret;

    ret = ff_v4l2_m2m_create_context(priv, &s);
    if (ret < 0)
        return ret;

    capture = &s->capture;
    output = &s->output;

    /* if these dimensions are invalid (ie, 0 or too small) an event will be raised
     * by the v4l2 driver; this event will trigger a full pipeline reconfig and
     * the proper values will be retrieved from the kernel driver.
     */
    output->height = capture->height = avctx->coded_height;
    output->width = capture->width = avctx->coded_width;

    output->av_codec_id = avctx->codec_id;
    output->av_pix_fmt  = AV_PIX_FMT_NONE;

    capture->av_codec_id = AV_CODEC_ID_RAWVIDEO;
    /* the block decodes straight to NV12, so ask for that unless the caller
     * pinned a raw format of its own
     */
    capture->av_pix_fmt = avctx->pix_fmt == AV_PIX_FMT_NONE ?
                          AV_PIX_FMT_NV12 : avctx->pix_fmt;

    s->avctx = avctx;
    ret = ff_v4l2_m2m_codec_init(priv);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "can't configure decoder\n");
        return ret;
    }

    return v4l2_prepare_decoder(s);
}

static av_cold int v4l2_decode_close(AVCodecContext *avctx)
{
    return ff_v4l2_m2m_codec_end(avctx->priv_data);
}

#undef FLAGS
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM

static const AVOption mjpeg_dec_options[] = {
    V4L_M2M_DEFAULT_OPTS,
    { "num_capture_buffers", "Number of buffers in the capture context",
        OFFSET(num_capture_buffers), AV_OPT_TYPE_INT, {.i64 = 20}, 2, INT_MAX, FLAGS },
    { NULL},
};

#define M2MDEC_CLASS(NAME, OPTIONS_NAME) \
    static const AVClass v4l2_m2m_ ## NAME ## _dec_class = { \
        .class_name = #NAME "_v4l2m2m_decoder", \
        .item_name  = av_default_item_name, \
        .option     = OPTIONS_NAME, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define M2MDEC(NAME, LONGNAME, OPTIONS_NAME, CODEC, bsf_name) \
    M2MDEC_CLASS(NAME, OPTIONS_NAME) \
    const FFCodec ff_ ## NAME ## _v4l2m2m_decoder = { \
        .p.name         = #NAME "_v4l2m2m" , \
        CODEC_LONG_NAME("V4L2 mem2mem " LONGNAME " decoder wrapper"), \
        .p.type         = AVMEDIA_TYPE_VIDEO, \
        .p.id           = CODEC , \
        .priv_data_size = sizeof(V4L2m2mPriv), \
        .p.priv_class   = &v4l2_m2m_ ## NAME ## _dec_class, \
        .init           = v4l2_decode_init, \
        FF_CODEC_RECEIVE_FRAME_CB(v4l2_receive_frame), \
        .close          = v4l2_decode_close, \
        .bsfs           = bsf_name, \
        .p.capabilities = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING, \
        .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE | \
                          FF_CODEC_CAP_INIT_CLEANUP, \
        .p.wrapper_name = "v4l2m2m", \
    }

M2MDEC(mjpeg, "MJPEG", mjpeg_dec_options, AV_CODEC_ID_MJPEG, NULL);

#undef OFFSET
#undef FLAGS
