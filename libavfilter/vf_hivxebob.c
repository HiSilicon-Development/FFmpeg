/*
 * Fast NV12 bob deinterlacer for HiVXE transcoding
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <string.h>

#include "libavutil/mathematics.h"
#include "libavutil/pixfmt.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

static int config_output(AVFilterLink *outlink)
{
    AVFilterLink *inlink = outlink->src->inputs[0];
    FilterLink *inl = ff_filter_link(inlink);
    FilterLink *outl = ff_filter_link(outlink);

    if ((inlink->w | inlink->h) & 1) {
        av_log(outlink->src, AV_LOG_ERROR,
               "NV12 bob requires even width and height\n");
        return AVERROR(EINVAL);
    }

    outlink->w = inlink->w;
    outlink->h = inlink->h;
    outlink->time_base.num = inlink->time_base.num;
    outlink->time_base.den = inlink->time_base.den * 2;
    outl->frame_rate = av_mul_q(inl->frame_rate, (AVRational) { 2, 1 });

    return 0;
}

static void mark_progressive(AVFrame *frame)
{
    frame->flags &= ~(AV_FRAME_FLAG_INTERLACED |
                      AV_FRAME_FLAG_TOP_FIELD_FIRST);
}

typedef struct HivxeBobContext {
    AVFrame *pending_input;
    int pending_field;
    int64_t pending_pts;
    int64_t pending_duration;
} HivxeBobContext;

static AVFrame *make_field(AVFilterLink *outlink, const AVFrame *in,
                           int field, int64_t pts, int64_t duration)
{
    AVFrame *out;
    const int visible_h = outlink->h;
    const int alloc_h = (visible_h + 15) & ~15;
    int y;

    /* CV200 advertises 1080 but its NV12 V4L2 queue copies the 1088-line
     * macroblock surface.  Keep the visible frame at 1080 while allocating
     * the padded backing store expected by the encoder. */
    out = ff_get_video_buffer(outlink, outlink->w, alloc_h);
    if (!out)
        return NULL;

    if (av_frame_copy_props(out, in) < 0) {
        av_frame_free(&out);
        return NULL;
    }

    out->pts = pts;
    out->duration = duration;
    out->width = outlink->w;
    out->height = visible_h;
    out->sample_aspect_ratio = in->sample_aspect_ratio;
    mark_progressive(out);

    for (y = 0; y < visible_h; y++) {
        const int src_y = field + 2 * (y >> 1);

        memcpy(out->data[0] + y * out->linesize[0],
               in->data[0] + src_y * in->linesize[0], outlink->w);
    }

    for (y = 0; y < visible_h / 2; y++) {
        const int src_y = field + 2 * (y >> 1);

        memcpy(out->data[1] + y * out->linesize[1],
               in->data[1] + src_y * in->linesize[1], outlink->w);
    }

    /* Keep the hidden macroblock rows initialized because V4L2 copies the
     * complete coded surface, not just AVFrame::height rows. */
    for (y = visible_h; y < alloc_h; y++)
        memcpy(out->data[0] + y * out->linesize[0],
               out->data[0] + (visible_h - 1) * out->linesize[0],
               outlink->w);
    for (y = visible_h / 2; y < alloc_h / 2; y++)
        memcpy(out->data[1] + y * out->linesize[1],
               out->data[1] + (visible_h / 2 - 1) * out->linesize[1],
               outlink->w);

    return out;
}

static int process_frame(AVFilterLink *inlink, AVFrame *in)
{
    HivxeBobContext *s = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    FilterLink *inl = ff_filter_link(inlink);
    const int interlaced = !!(in->flags & AV_FRAME_FLAG_INTERLACED);
    const int first_field =
        (in->flags & AV_FRAME_FLAG_TOP_FIELD_FIRST) ? 0 : 1;
    int64_t first_pts = AV_NOPTS_VALUE;
    int64_t field_duration = in->duration;
    AVFrame *first;
    int ret;

    if (!interlaced) {
        if (in->pts != AV_NOPTS_VALUE)
            in->pts *= 2;
        if (in->duration > 0)
            in->duration *= 2;
        return ff_filter_frame(outlink, in);
    }

    if (in->pts != AV_NOPTS_VALUE)
        first_pts = in->pts * 2;
    if (field_duration <= 0 && inl->frame_rate.num > 0) {
        field_duration = av_rescale_q(1, av_inv_q(inl->frame_rate),
                                      outlink->time_base) / 2;
    }

    first = make_field(outlink, in, first_field, first_pts, field_duration);
    if (!first) {
        av_frame_free(&first);
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    s->pending_input = in;
    s->pending_field = first_field ^ 1;
    s->pending_pts = first_pts == AV_NOPTS_VALUE ? AV_NOPTS_VALUE
                                                 : first_pts + field_duration;
    s->pending_duration = field_duration;

    ret = ff_filter_frame(outlink, first);
    if (ret < 0) {
        av_frame_free(&s->pending_input);
        return ret;
    }
    return 0;
}

static int activate(AVFilterContext *ctx)
{
    HivxeBobContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *in;
    int64_t pts;
    int ret, status;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    /* Emit a saved second field on a separate scheduler activation.  V4L2
     * encoders can retain the submitted AVFrame until their next dequeue;
     * never push two newly allocated frames from one input callback. */
    if (s->pending_input) {
        AVFrame *source = s->pending_input;
        AVFrame *pending;

        s->pending_input = NULL;
        pending = make_field(outlink, source, s->pending_field,
                             s->pending_pts, s->pending_duration);
        av_frame_free(&source);
        if (!pending)
            return AVERROR(ENOMEM);
        return ff_filter_frame(outlink, pending);
    }

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return process_frame(inlink, in);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (pts != AV_NOPTS_VALUE)
            pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);
        ff_outlink_set_status(outlink, status, pts);
        return 0;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);
    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    HivxeBobContext *s = ctx->priv;

    av_frame_free(&s->pending_input);
}

static const AVFilterPad hivxebob_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
};

static const AVFilterPad hivxebob_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_vf_hivxebob = {
    .p.name        = "hivxebob",
    .p.description = NULL_IF_CONFIG_SMALL(
        "Fast field-aware NV12 bob deinterlacer for HiVXE transcoding."),
    .priv_size   = sizeof(HivxeBobContext),
    .activate    = activate,
    .uninit      = uninit,
    FILTER_INPUTS(hivxebob_inputs),
    FILTER_OUTPUTS(hivxebob_outputs),
    FILTER_PIXFMTS(AV_PIX_FMT_NV12),
};
