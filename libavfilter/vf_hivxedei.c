/*
 * HiSilicon VPSS display-ordered field-rate de-interlacer
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "libavutil/common.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_v4l2request.h"
#include "libavutil/mathematics.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct HiVXEDeiContext {
    AVFrame *previous;
    AVFrame *current;
    AVFrame *pending_output;
    int eof_status;
    int64_t eof_pts;
    int rate_initialized;
    AVRational input_rate;
} HiVXEDeiContext;

static int config_output(AVFilterLink *outlink)
{
    AVFilterLink *inlink = outlink->src->inputs[0];
    FilterLink *inl = ff_filter_link(inlink);
    FilterLink *outl = ff_filter_link(outlink);
    AVHWFramesContext *frames;
    HiVXEDeiContext *s = outlink->src->priv;
    const AVV4L2RequestFramesContext *request;

    if ((inlink->w | inlink->h) & 1 || !inl->hw_frames_ctx)
        return AVERROR(EINVAL);
    frames = (AVHWFramesContext *)inl->hw_frames_ctx->data;
    if (frames->device_ctx->type != AV_HWDEVICE_TYPE_V4L2REQUEST ||
        frames->sw_format != AV_PIX_FMT_NV12)
        return AVERROR(EINVAL);
    request = frames->hwctx;
    s->input_rate = request->frame_rate.num > 0 ?
                    request->frame_rate : inl->frame_rate;

    outlink->w = inlink->w;
    outlink->h = inlink->h;
    outlink->time_base = av_mul_q(inlink->time_base, (AVRational) { 1, 2 });
    /* Field cadence is known only when the first decoded frame arrives. */
    outl->frame_rate = (AVRational) { 0, 1 };
    outl->hw_frames_ctx = av_buffer_ref(inl->hw_frames_ctx);
    return outl->hw_frames_ctx ? 0 : AVERROR(ENOMEM);
}

static int emit_current(AVFilterContext *ctx, const AVFrame *next)
{
    HiVXEDeiContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    const AVFrame *current = s->current;
    AVFrame *output[2] = { NULL, NULL };
    int64_t pts = AV_NOPTS_VALUE;
    int64_t duration = s->input_rate.num > 0 ?
        av_rescale_q(1, av_inv_q(s->input_rate), outlink->time_base) :
        av_rescale_q(current->duration, inlink->time_base, outlink->time_base);
    int ret;

    if (current->pts != AV_NOPTS_VALUE)
        pts = av_rescale_q(current->pts, inlink->time_base, outlink->time_base);
    if (!(current->flags & AV_FRAME_FLAG_INTERLACED)) {
        output[0] = av_frame_clone(current);
        if (!output[0])
            return AVERROR(ENOMEM);
        output[0]->pts = pts;
        output[0]->duration = duration;
        return ff_filter_frame(outlink, output[0]);
    }
    duration /= 2;

    output[0] = av_frame_alloc();
    output[1] = av_frame_alloc();
    if (!output[0] || !output[1]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    {
        const AVV4L2RequestFrameDescriptor *desc = (void *)current->data[0];

        if (!desc->deinterlace)
            ret = AVERROR(ENOSYS);
        else
            ret = desc->deinterlace(s->previous ? s->previous : current,
                                     current, next ? next : current, output);
    }
    if (ret < 0)
        goto fail;
    for (int i = 0; i < 2; i++) {
        ret = av_frame_copy_props(output[i], current);
        if (ret < 0)
            goto fail;
        output[i]->width = outlink->w;
        output[i]->height = outlink->h;
        output[i]->pts = pts == AV_NOPTS_VALUE ? pts : pts + i * duration;
        output[i]->duration = duration;
        output[i]->flags &= ~(AV_FRAME_FLAG_INTERLACED |
                              AV_FRAME_FLAG_TOP_FIELD_FIRST);
    }
    s->pending_output = output[1];
    return ff_filter_frame(outlink, output[0]);
fail:
    av_log(ctx, AV_LOG_ERROR, "Display-ordered VPSS DEI failed: %s\n",
           av_err2str(ret));
    av_frame_free(&output[0]);
    av_frame_free(&output[1]);
    return ret;
}

static int activate(AVFilterContext *ctx)
{
    HiVXEDeiContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *input;
    int64_t pts;
    int ret, status;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);
    if (s->pending_output) {
        AVFrame *second = s->pending_output;

        s->pending_output = NULL;
        return ff_filter_frame(outlink, second);
    }
    if (s->current && !(s->current->flags & AV_FRAME_FLAG_INTERLACED)) {
        ret = emit_current(ctx, NULL);
        av_frame_free(&s->previous);
        av_frame_free(&s->current);
        return ret;
    }
    if (s->eof_status) {
        if (s->current) {
            ret = emit_current(ctx, NULL);
            av_frame_free(&s->previous);
            av_frame_free(&s->current);
            return ret;
        }
        ff_outlink_set_status(outlink, s->eof_status, s->eof_pts);
        return 0;
    }

    ret = ff_inlink_consume_frame(inlink, &input);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        if (!s->rate_initialized) {
            AVRational input_rate = s->input_rate;
            FilterLink *outl = ff_filter_link(outlink);

            outl->frame_rate = av_mul_q(input_rate, (AVRational) {
                (input->flags & AV_FRAME_FLAG_INTERLACED) ? 2 : 1, 1
            });
            s->rate_initialized = 1;
        }
        if (s->current) {
            ret = emit_current(ctx, input);
            if (ret < 0) {
                av_frame_free(&input);
                return ret;
            }
            av_frame_free(&s->previous);
            s->previous = s->current;
        }
        s->current = input;
        ff_filter_set_ready(ctx, 100);
        return 0;
    }
    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        s->eof_status = status;
        s->eof_pts = pts == AV_NOPTS_VALUE ? pts :
                    av_rescale_q(pts, inlink->time_base, outlink->time_base);
        ff_filter_set_ready(ctx, 100);
        return 0;
    }
    FF_FILTER_FORWARD_WANTED(outlink, inlink);
    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    HiVXEDeiContext *s = ctx->priv;

    av_frame_free(&s->previous);
    av_frame_free(&s->current);
    av_frame_free(&s->pending_output);
}

static const AVFilterPad inputs[] = {
    { .name = "default", .type = AVMEDIA_TYPE_VIDEO },
};

static const AVFilterPad outputs[] = {
    { .name = "default", .type = AVMEDIA_TYPE_VIDEO, .config_props = config_output },
};

const FFFilter ff_vf_hivxedei = {
    .p.name = "hivxedei",
    .p.description = NULL_IF_CONFIG_SMALL("HiSilicon display-ordered VPSS field-rate DEI"),
    .priv_size = sizeof(HiVXEDeiContext),
    .uninit = uninit,
    .activate = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_DRM_PRIME),
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
