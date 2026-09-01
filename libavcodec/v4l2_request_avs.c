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

#include "cavs.h"
#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "internal.h"
#include "v4l2_request.h"

#define V4L2_REQUEST_AVS_MAX_SLICES 200

typedef struct V4L2RequestControlsAVS {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_avs_sequence sequence;
    struct v4l2_ctrl_avs_picture picture;
    struct v4l2_ctrl_avs_decode_params decode;
    struct v4l2_ctrl_avs_slice_params slices[V4L2_REQUEST_AVS_MAX_SLICES];
    unsigned int num_slices;
} V4L2RequestControlsAVS;

static void fill_sequence(struct v4l2_ctrl_avs_sequence *ctrl,
                          const AVSSequenceHeader *sequence)
{
    *ctrl = (struct v4l2_ctrl_avs_sequence) {
        .horizontal_size = sequence->width,
        .vertical_size   = sequence->height,
        .profile_id      = sequence->profile_id,
        .level_id        = sequence->level_id,
        .chroma_format   = sequence->chroma_format,
        .sample_precision = sequence->sample_precision,
        .aspect_ratio    = sequence->aspect_ratio,
        .frame_rate_code = sequence->frame_rate_code,
    };

    if (sequence->flags & CAVS_SEQUENCE_FLAG_PROGRESSIVE)
        ctrl->flags |= V4L2_AVS_SEQUENCE_FLAG_PROGRESSIVE;
    if (sequence->flags & CAVS_SEQUENCE_FLAG_LOW_DELAY)
        ctrl->flags |= V4L2_AVS_SEQUENCE_FLAG_LOW_DELAY;
}

static void fill_picture(struct v4l2_ctrl_avs_picture *ctrl,
                         const AVSPictureHeader *picture)
{
    *ctrl = (struct v4l2_ctrl_avs_picture) {
        .bbv_delay          = picture->bbv_delay,
        .picture_distance   = picture->picture_distance,
        .picture_coding_type = picture->picture_coding_type,
        .picture_structure  = picture->picture_structure,
        .picture_qp         = picture->picture_qp,
        .alpha_c_offset     = picture->alpha_c_offset,
        .beta_offset        = picture->beta_offset,
        .chroma_qp_delta_u  = picture->chroma_qp_delta_u,
        .chroma_qp_delta_v  = picture->chroma_qp_delta_v,
    };

#define COPY_FLAG(name) do {                                      \
    if (picture->flags & CAVS_PICTURE_FLAG_ ## name)              \
        ctrl->flags |= V4L2_AVS_PICTURE_FLAG_ ## name;            \
} while (0)
    COPY_FLAG(PROGRESSIVE_FRAME);
    COPY_FLAG(TOP_FIELD_FIRST);
    COPY_FLAG(REPEAT_FIRST_FIELD);
    COPY_FLAG(FIXED_QP);
    COPY_FLAG(SKIP_MODE);
    COPY_FLAG(LOOP_FILTER_DISABLE);
    COPY_FLAG(LOOP_FILTER_PARAMS);
    COPY_FLAG(REFERENCE);
    COPY_FLAG(NO_FORWARD_REFERENCE);
    COPY_FLAG(ADVANCED_PRED_DISABLE);
    COPY_FLAG(WEIGHTING_QUANT);
    COPY_FLAG(CHROMA_QP_DISABLE);
    COPY_FLAG(AEC);
    COPY_FLAG(P_FIELD_ENHANCED);
    COPY_FLAG(B_FIELD_ENHANCED);
#undef COPY_FLAG

    memcpy(ctrl->weighting_quant_matrix, picture->weighting_quant_matrix,
           sizeof(ctrl->weighting_quant_matrix));
}

static int fill_decode_params(AVCodecContext *avctx,
                              struct v4l2_ctrl_avs_decode_params *decode)
{
    const AVSContext *h = avctx->priv_data;

    *decode = (struct v4l2_ctrl_avs_decode_params) { 0 };
    switch (h->picture.picture_coding_type) {
    case V4L2_AVS_PICTURE_TYPE_I:
        return 0;
    case V4L2_AVS_PICTURE_TYPE_P:
        if (!h->DPB[0].f->data[0])
            return AVERROR_INVALIDDATA;
        decode->flags |= V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF0;
        decode->forward_ref_ts[0] =
            ff_v4l2_request_get_capture_timestamp(h->DPB[0].f);
        if (h->DPB[1].f->data[0]) {
            decode->flags |= V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF1;
            decode->forward_ref_ts[1] =
                ff_v4l2_request_get_capture_timestamp(h->DPB[1].f);
        }
        return 0;
    case V4L2_AVS_PICTURE_TYPE_B:
        if (!h->DPB[0].f->data[0] || !h->DPB[1].f->data[0])
            return AVERROR_INVALIDDATA;
        decode->flags = V4L2_AVS_DECODE_PARAM_FLAG_BACKWARD_REF |
                        V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF0;
        decode->backward_ref_ts =
            ff_v4l2_request_get_capture_timestamp(h->DPB[0].f);
        decode->forward_ref_ts[0] =
            ff_v4l2_request_get_capture_timestamp(h->DPB[1].f);
        if (h->DPB[2].f->data[0]) {
            decode->flags |= V4L2_AVS_DECODE_PARAM_FLAG_FORWARD_REF1;
            decode->forward_ref_ts[1] =
                ff_v4l2_request_get_capture_timestamp(h->DPB[2].f);
        }
        return 0;
    default:
        return AVERROR_INVALIDDATA;
    }
}

static int v4l2_request_avs_start_frame(AVCodecContext *avctx,
                                        av_unused const AVBufferRef *buf_ref,
                                        av_unused const uint8_t *buffer,
                                        av_unused uint32_t size)
{
    const AVSContext *h = avctx->priv_data;
    V4L2RequestControlsAVS *controls = h->cur.hwaccel_picture_private;
    int ret;

    fill_sequence(&controls->sequence, &h->sequence);
    fill_picture(&controls->picture, &h->picture);
    ret = fill_decode_params(avctx, &controls->decode);
    if (ret < 0)
        return ret;

    controls->num_slices = 0;
    return ff_v4l2_request_start_frame(avctx, &controls->pic, h->cur.f);
}

static int v4l2_request_avs_decode_slice(AVCodecContext *avctx,
                                         const uint8_t *buffer, uint32_t size)
{
    const AVSContext *h = avctx->priv_data;
    V4L2RequestControlsAVS *controls = h->cur.hwaccel_picture_private;
    struct v4l2_ctrl_avs_slice_params *slice;
    uint32_t height_mbs, start_mb;
    int ret;

    if (size < 4 || size > UINT32_MAX / 8 || buffer[0] || buffer[1] ||
        buffer[2] != 1 || buffer[3] > (SLICE_MAX_START_CODE & 0xff) ||
        controls->num_slices >= V4L2_REQUEST_AVS_MAX_SLICES)
        return AVERROR_INVALIDDATA;

    start_mb = buffer[3] * h->mb_width;
    height_mbs = h->sequence.flags & CAVS_SEQUENCE_FLAG_PROGRESSIVE ?
                 (h->sequence.height + 15) / 16 :
                 2 * ((h->sequence.height + 31) / 32);
    if ((!controls->num_slices && start_mb) ||
        (controls->num_slices &&
         start_mb <= controls->slices[controls->num_slices - 1].slice_start_mb) ||
        start_mb >= h->mb_width * height_mbs)
        return AVERROR_INVALIDDATA;

    slice = &controls->slices[controls->num_slices];
    *slice = (struct v4l2_ctrl_avs_slice_params) {
        .bit_size         = size * 8,
        .data_byte_offset = controls->pic.output->used,
        .slice_start_mb   = start_mb,
    };

    ret = ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
    if (ret < 0)
        return ret;
    controls->num_slices++;
    return 0;
}

static int v4l2_request_avs_end_frame(AVCodecContext *avctx)
{
    const AVSContext *h = avctx->priv_data;
    V4L2RequestControlsAVS *controls = h->cur.hwaccel_picture_private;
    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_AVS_SEQUENCE,
            .ptr = &controls->sequence,
            .size = sizeof(controls->sequence),
        }, {
            .id = V4L2_CID_STATELESS_AVS_PICTURE,
            .ptr = &controls->picture,
            .size = sizeof(controls->picture),
        }, {
            .id = V4L2_CID_STATELESS_AVS_SLICE_PARAMS,
            .ptr = controls->slices,
            .size = controls->num_slices * sizeof(controls->slices[0]),
        }, {
            .id = V4L2_CID_STATELESS_AVS_DECODE_PARAMS,
            .ptr = &controls->decode,
            .size = sizeof(controls->decode),
        },
    };

    if (!controls->num_slices)
        return AVERROR_INVALIDDATA;
    return ff_v4l2_request_decode_frame(avctx, &controls->pic,
                                        control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_avs_post_probe(AVCodecContext *avctx)
{
    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_AVS_DECODE_MODE,
            .value = V4L2_STATELESS_AVS_DECODE_MODE_FRAME_BASED,
        }, {
            .id = V4L2_CID_STATELESS_AVS_START_CODE,
            .value = V4L2_STATELESS_AVS_START_CODE_PREFIX,
        },
    };
    int value;

    value = ff_v4l2_request_query_control_default_value(
                avctx, V4L2_CID_STATELESS_AVS_DECODE_MODE);
    if (value != V4L2_STATELESS_AVS_DECODE_MODE_FRAME_BASED)
        return value < 0 ? value : AVERROR(ENOTSUP);
    value = ff_v4l2_request_query_control_default_value(
                avctx, V4L2_CID_STATELESS_AVS_START_CODE);
    if (value != V4L2_STATELESS_AVS_START_CODE_PREFIX)
        return value < 0 ? value : AVERROR(ENOTSUP);

    return ff_v4l2_request_set_controls(avctx, control,
                                        FF_ARRAY_ELEMS(control));
}

static int v4l2_request_avs_init(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    const AVSContext *h = avctx->priv_data;
    struct v4l2_ctrl_avs_sequence sequence;
    struct v4l2_ext_control control = {
        .id = V4L2_CID_STATELESS_AVS_SEQUENCE,
        .ptr = &sequence,
        .size = sizeof(sequence),
    };

    fill_sequence(&sequence, &h->sequence);
    ctx->post_probe = v4l2_request_avs_post_probe;
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_AVS_SLICE,
                                8 * 1024 * 1024, &control, 1);
}

const FFHWAccel ff_cavs_v4l2request_hwaccel = {
    .p.name               = "cavs_v4l2request",
    .p.type               = AVMEDIA_TYPE_VIDEO,
    .p.id                 = AV_CODEC_ID_CAVS,
    .p.pix_fmt            = AV_PIX_FMT_DRM_PRIME,
    .start_frame          = v4l2_request_avs_start_frame,
    .decode_slice         = v4l2_request_avs_decode_slice,
    .end_frame            = v4l2_request_avs_end_frame,
    .flush                = ff_v4l2_request_flush,
    .frame_priv_data_size = sizeof(V4L2RequestControlsAVS),
    .init                 = v4l2_request_avs_init,
    .uninit               = ff_v4l2_request_uninit,
    .priv_data_size       = sizeof(V4L2RequestContext),
    .frame_params         = ff_v4l2_request_frame_params,
};
