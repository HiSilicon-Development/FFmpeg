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

#include "h264dec.h"
#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "internal.h"
#include "v4l2_request_internal.h"

typedef struct V4L2RequestContextH264 {
    V4L2RequestContext base;
    enum v4l2_stateless_h264_decode_mode decode_mode;
    enum v4l2_stateless_h264_start_code start_code;
    /* A failed queue teardown requires decoder reinitialization, not retry. */
    int error;
} V4L2RequestContextH264;

typedef struct V4L2RequestControlsH264 {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_h264_sps sps;
    struct v4l2_ctrl_h264_pps pps;
    struct v4l2_ctrl_h264_scaling_matrix scaling_matrix;
    struct v4l2_ctrl_h264_decode_params decode_params;
    struct v4l2_ctrl_h264_slice_params slice_params;
    struct v4l2_ctrl_h264_pred_weights pred_weights;
    bool pred_weights_required;
    bool first_slice;
    int num_slices;
    const H264Picture *dpb_pictures[V4L2_H264_NUM_DPB_ENTRIES];
    /* State is per picture; submitted_fields survives the complementary field.
     * An error invalidates the whole surface and is never cleared by start. */
    unsigned int submitted_fields;
    bool started;
    bool held;
    int error;
} V4L2RequestControlsH264;

static uint8_t nalu_slice_start_code[] = { 0x00, 0x00, 0x01 };

static void fill_weight_factors(struct v4l2_h264_weight_factors *weight_factors,
                                int list, const H264SliceContext *sl)
{
    for (int i = 0; i < sl->ref_count[list]; i++) {
        if (sl->pwt.luma_weight_flag[list]) {
            weight_factors->luma_weight[i] = sl->pwt.luma_weight[i][list][0];
            weight_factors->luma_offset[i] = sl->pwt.luma_weight[i][list][1];
        } else {
            weight_factors->luma_weight[i] = 1 << sl->pwt.luma_log2_weight_denom;
            weight_factors->luma_offset[i] = 0;
        }
        for (int j = 0; j < 2; j++) {
            if (sl->pwt.chroma_weight_flag[list]) {
                weight_factors->chroma_weight[i][j] = sl->pwt.chroma_weight[i][list][j][0];
                weight_factors->chroma_offset[i][j] = sl->pwt.chroma_weight[i][list][j][1];
            } else {
                weight_factors->chroma_weight[i][j] = 1 << sl->pwt.chroma_log2_weight_denom;
                weight_factors->chroma_offset[i][j] = 0;
            }
        }
    }
}

static unsigned int reference_fields(const H264Picture *pic)
{
    const V4L2RequestControlsH264 *controls;
    V4L2RequestFrameDescriptor *desc;
    unsigned int fields;

    if (!pic || !pic->f || !pic->hwaccel_picture_private || pic->invalid_gap ||
        (pic->field_poc[0] == INT_MAX && pic->field_poc[1] == INT_MAX))
        return 0;

    controls = pic->hwaccel_picture_private;
    desc = v4l2_request_framedesc(pic->f);
    if (controls->error || !desc || !desc->ctx ||
        (atomic_load(&desc->ctx->capture_errors) &
         (UINT64_C(1) << desc->capture.index)))
        return 0;

    fields = controls->submitted_fields;
    if (pic->field_poc[0] == INT_MAX)
        fields &= ~V4L2_H264_TOP_FIELD_REF;
    if (pic->field_poc[1] == INT_MAX)
        fields &= ~V4L2_H264_BOTTOM_FIELD_REF;
    return fields;
}

static void fill_dpb_entry(struct v4l2_h264_dpb_entry *entry,
                           const H264Picture *pic, int long_idx,
                           unsigned int fields)
{
    entry->reference_ts = ff_v4l2_request_get_capture_timestamp(pic->f);
    entry->pic_num = pic->pic_id;
    entry->frame_num = pic->long_ref ? long_idx : pic->frame_num;
    entry->fields = fields;
    entry->flags = V4L2_H264_DPB_ENTRY_FLAG_VALID |
                   V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
    if (pic->long_ref)
        entry->flags |= V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
    if (pic->field_picture)
        entry->flags |= V4L2_H264_DPB_ENTRY_FLAG_FIELD;
    if (pic->field_poc[0] != INT_MAX)
        entry->top_field_order_cnt = pic->field_poc[0];
    if (pic->field_poc[1] != INT_MAX)
        entry->bottom_field_order_cnt = pic->field_poc[1];
}

static int fill_dpb(V4L2RequestControlsH264 *controls, const H264Context *h)
{
    struct v4l2_ctrl_h264_decode_params *decode_params = &controls->decode_params;
    int entries = 0;

    memset(controls->dpb_pictures, 0, sizeof(controls->dpb_pictures));
    for (int i = 0; i < h->short_ref_count; i++) {
        const H264Picture *pic = h->short_ref[i];
        unsigned int fields = reference_fields(pic);
        if (pic)
            fields &= pic->reference;
        if (fields) {
            if (entries == FF_ARRAY_ELEMS(decode_params->dpb))
                return AVERROR(ENOSPC);
            controls->dpb_pictures[entries] = pic;
            fill_dpb_entry(&decode_params->dpb[entries++], pic, pic->pic_id, fields);
        }
    }

    for (int i = 0; i < FF_ARRAY_ELEMS(h->long_ref); i++) {
        const H264Picture *pic = h->long_ref[i];
        unsigned int fields = reference_fields(pic);
        if (pic)
            fields &= pic->reference;
        if (fields) {
            if (entries == FF_ARRAY_ELEMS(decode_params->dpb))
                return AVERROR(ENOSPC);
            controls->dpb_pictures[entries] = pic;
            fill_dpb_entry(&decode_params->dpb[entries++], pic, i, fields);
        }
    }
    if (h->mvc_sps && h->mvc_active_view && h->mvc_base && h->slice_ctx[0].list_count &&
        (h->mvc_anchor_pic ? h->mvc_sps->mvc_anchor_ref[0] || h->mvc_sps->mvc_anchor_ref[1] :
                             h->mvc_sps->mvc_non_anchor_ref[0] || h->mvc_sps->mvc_non_anchor_ref[1])) {
        unsigned int fields = reference_fields(h->mvc_base);
        if (fields != V4L2_H264_FRAME_REF)
            return AVERROR_INVALIDDATA;
        if (entries == FF_ARRAY_ELEMS(decode_params->dpb))
            return AVERROR(ENOSPC);
        controls->dpb_pictures[entries] = h->mvc_base;
        /* Inter-view eligibility is independent of temporal marking. */
        fill_dpb_entry(&decode_params->dpb[entries], h->mvc_base,
                       h->mvc_base->pic_id, fields);
    }
    return 0;
}

static int fill_ref_list(struct v4l2_h264_reference *reference,
                         const V4L2RequestControlsH264 *controls,
                         const H264Ref *ref)
{
    const unsigned int required = V4L2_H264_DPB_ENTRY_FLAG_VALID |
                                  V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
    unsigned int fields = ref->reference & V4L2_H264_FRAME_REF;
    uint64_t timestamp;

    if (!fields || (ref->reference & ~(V4L2_H264_FRAME_REF | DELAYED_PIC_REF)) ||
        (reference_fields(ref->parent) & fields) != fields)
        return AVERROR_INVALIDDATA;

    timestamp = ff_v4l2_request_get_capture_timestamp(ref->parent->f);
    if (!timestamp)
        return AVERROR_INVALIDDATA;

    for (uint8_t i = 0; i < FF_ARRAY_ELEMS(controls->decode_params.dpb); i++) {
        const struct v4l2_h264_dpb_entry *entry = &controls->decode_params.dpb[i];
        const H264Picture *pic = controls->dpb_pictures[i];
        if (pic && pic->hwaccel_picture_private == ref->parent->hwaccel_picture_private &&
            v4l2_request_framedesc(pic->f) == v4l2_request_framedesc(ref->parent->f) &&
            (entry->flags & required) == required &&
            entry->reference_ts == timestamp &&
            (entry->fields & fields) == fields) {
            *reference = (struct v4l2_h264_reference) { .fields = fields, .index = i };
            return 0;
        }
    }
    return AVERROR_INVALIDDATA;
}

static int fill_slice_references(AVCodecContext *avctx,
                                 const V4L2RequestControlsH264 *controls,
                                 struct v4l2_ctrl_h264_slice_params *slice,
                                 const H264SliceContext *sl)
{
    struct v4l2_h264_reference *lists[] = {
        slice->ref_pic_list0, slice->ref_pic_list1,
    };

    if (sl->list_count > FF_ARRAY_ELEMS(lists))
        return AVERROR_INVALIDDATA;
    for (int list = 0; list < sl->list_count; list++) {
        if (sl->ref_count[list] <= 0 ||
            sl->ref_count[list] > FF_ARRAY_ELEMS(slice->ref_pic_list0))
            return AVERROR_INVALIDDATA;
        for (int i = 0; i < sl->ref_count[list]; i++) {
            int ret = fill_ref_list(&lists[list][i], controls, &sl->ref_list[list][i]);
            if (ret < 0) {
                const H264Picture *pic = sl->ref_list[list][i].parent;
                av_log(avctx, AV_LOG_ERROR,
                       "H264 reference L%d[%d] has no reconstructed compatible DPB field "
                       "(frame_num=%d, selected=%u, available=%u)\n",
                       list, i, pic ? pic->frame_num : -1,
                       sl->ref_list[list][i].reference, reference_fields(pic));
                return ret;
            }
        }
    }
    return 0;
}

static int v4l2_request_h264_fail_frame(AVCodecContext *avctx, int error,
                                        bool queue_failed)
{
    const H264Context *h = avctx->priv_data;
    V4L2RequestContextH264 *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    int ret;

    if (!controls->error)
        controls->error = error;
    controls->started = false;
    controls->submitted_fields = 0;
    ret = ff_v4l2_request_abort_frame(avctx, &controls->pic, h->cur_pic_ptr->f,
                                     controls->held, queue_failed);
    controls->held = false;
    if (ret < 0)
        ctx->error = ret;
    return controls->error;
}

static void fill_sps(struct v4l2_ctrl_h264_sps *ctrl, const H264Context *h)
{
    const SPS *sps = h->ps.sps;

    *ctrl = (struct v4l2_ctrl_h264_sps) {
        .profile_idc = sps->profile_idc,
        .constraint_set_flags = sps->constraint_set_flags,
        .level_idc = sps->level_idc,
        .seq_parameter_set_id = sps->sps_id,
        .chroma_format_idc = sps->chroma_format_idc,
        .bit_depth_luma_minus8 = sps->bit_depth_luma - 8,
        .bit_depth_chroma_minus8 = sps->bit_depth_chroma - 8,
        .log2_max_frame_num_minus4 = sps->log2_max_frame_num - 4,
        .pic_order_cnt_type = sps->poc_type,
        .log2_max_pic_order_cnt_lsb_minus4 = sps->log2_max_poc_lsb - 4,
        .max_num_ref_frames = sps->ref_frame_count,
        .num_ref_frames_in_pic_order_cnt_cycle = sps->poc_cycle_length,
        .offset_for_non_ref_pic = sps->offset_for_non_ref_pic,
        .offset_for_top_to_bottom_field = sps->offset_for_top_to_bottom_field,
        .pic_width_in_mbs_minus1 = h->mb_width - 1,
        .pic_height_in_map_units_minus1 = sps->frame_mbs_only_flag ?
                                          h->mb_height - 1 : h->mb_height / 2 - 1,
    };

    if (sps->poc_cycle_length > 0 && sps->poc_cycle_length <= 255)
        memcpy(ctrl->offset_for_ref_frame, sps->offset_for_ref_frame,
               sps->poc_cycle_length * sizeof(ctrl->offset_for_ref_frame[0]));

    if (sps->residual_color_transform_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE;

    if (sps->transform_bypass)
        ctrl->flags |= V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS;

    if (sps->delta_pic_order_always_zero_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO;

    if (sps->gaps_in_frame_num_allowed_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_GAPS_IN_FRAME_NUM_VALUE_ALLOWED;

    if (sps->frame_mbs_only_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;

    if (sps->mb_aff)
        ctrl->flags |= V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;

    if (sps->direct_8x8_inference_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;
}

static void fill_pps(struct v4l2_ctrl_h264_pps *ctrl, const H264Context *h)
{
    const SPS *sps = h->ps.sps;
    const PPS *pps = h->ps.pps;
    const H264SliceContext *sl = &h->slice_ctx[0];
    int qp_bd_offset = 6 * (sps->bit_depth_luma - 8);

    *ctrl = (struct v4l2_ctrl_h264_pps) {
        .pic_parameter_set_id = sl->pps_id,
        .seq_parameter_set_id = pps->sps_id,
        .num_slice_groups_minus1 = pps->slice_group_count - 1,
        .num_ref_idx_l0_default_active_minus1 = pps->ref_count[0] - 1,
        .num_ref_idx_l1_default_active_minus1 = pps->ref_count[1] - 1,
        .weighted_bipred_idc = pps->weighted_bipred_idc,
        .pic_init_qp_minus26 = pps->init_qp - 26 - qp_bd_offset,
        .pic_init_qs_minus26 = pps->init_qs - 26 - qp_bd_offset,
        .chroma_qp_index_offset = pps->chroma_qp_index_offset[0],
        .second_chroma_qp_index_offset = pps->chroma_qp_index_offset[1],
    };

    if (pps->cabac)
        ctrl->flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;

    if (pps->pic_order_present)
        ctrl->flags |= V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT;

    if (pps->weighted_pred)
        ctrl->flags |= V4L2_H264_PPS_FLAG_WEIGHTED_PRED;

    if (pps->deblocking_filter_parameters_present)
        ctrl->flags |= V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

    if (pps->constrained_intra_pred)
        ctrl->flags |= V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED;

    if (pps->redundant_pic_cnt_present)
        ctrl->flags |= V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT;

    if (pps->transform_8x8_mode)
        ctrl->flags |= V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;

    /* FFmpeg always provide a scaling matrix */
    ctrl->flags |= V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
}

static int v4l2_request_h264_start_frame(AVCodecContext *avctx,
                                         av_unused const AVBufferRef *buffer_ref,
                                         av_unused const uint8_t *buffer,
                                         av_unused uint32_t size)
{
    const H264Context *h = avctx->priv_data;
    const PPS *pps = h->ps.pps;
    const SPS *sps = h->ps.sps;
    const H264SliceContext *sl = &h->slice_ctx[0];
    V4L2RequestContextH264 *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    int ret;

    if (controls->started || controls->held)
        return v4l2_request_h264_fail_frame(avctx, AVERROR_INVALIDDATA, false);

    controls->started = false;
    controls->held = false;
    controls->first_slice = true;
    controls->num_slices = 0;
    if (ctx->error || controls->error)
        return v4l2_request_h264_fail_frame(avctx,
                                           ctx->error ? ctx->error : controls->error,
                                           false);

    fill_sps(&controls->sps, h);
    fill_pps(&controls->pps, h);

    memcpy(controls->scaling_matrix.scaling_list_4x4, pps->scaling_matrix4,
           sizeof(controls->scaling_matrix.scaling_list_4x4));
    memcpy(controls->scaling_matrix.scaling_list_8x8[0], pps->scaling_matrix8[0],
           sizeof(controls->scaling_matrix.scaling_list_8x8[0]));
    memcpy(controls->scaling_matrix.scaling_list_8x8[1], pps->scaling_matrix8[3],
           sizeof(controls->scaling_matrix.scaling_list_8x8[1]));

    if (sps->chroma_format_idc == 3) {
        memcpy(controls->scaling_matrix.scaling_list_8x8[2], pps->scaling_matrix8[1],
               sizeof(controls->scaling_matrix.scaling_list_8x8[2]));
        memcpy(controls->scaling_matrix.scaling_list_8x8[3], pps->scaling_matrix8[4],
               sizeof(controls->scaling_matrix.scaling_list_8x8[3]));
        memcpy(controls->scaling_matrix.scaling_list_8x8[4], pps->scaling_matrix8[2],
               sizeof(controls->scaling_matrix.scaling_list_8x8[4]));
        memcpy(controls->scaling_matrix.scaling_list_8x8[5], pps->scaling_matrix8[5],
               sizeof(controls->scaling_matrix.scaling_list_8x8[5]));
    }

    controls->decode_params = (struct v4l2_ctrl_h264_decode_params) {
        .nal_ref_idc = h->nal_ref_idc,
        .frame_num = h->poc.frame_num,
        .top_field_order_cnt = h->cur_pic_ptr->field_poc[0] != INT_MAX ?
                               h->cur_pic_ptr->field_poc[0] : 0,
        .bottom_field_order_cnt = h->cur_pic_ptr->field_poc[1] != INT_MAX ?
                                  h->cur_pic_ptr->field_poc[1] : 0,
        .idr_pic_id = sl->idr_pic_id,
        .pic_order_cnt_lsb = sl->poc_lsb,
        .delta_pic_order_cnt_bottom = sl->delta_poc_bottom,
        .delta_pic_order_cnt0 = sl->delta_poc[0],
        .delta_pic_order_cnt1 = sl->delta_poc[1],
        /* Size in bits of dec_ref_pic_marking() syntax element. */
        .dec_ref_pic_marking_bit_size = sl->ref_pic_marking_bit_size,
        /* Size in bits of pic order count syntax. */
        .pic_order_cnt_bit_size = sl->pic_order_cnt_bit_size,
        .slice_group_change_cycle = 0, /* slice group not supported by FFmpeg */
    };

    if (h->picture_idr)
        controls->decode_params.flags |= V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;

    if (FIELD_PICTURE(h))
        controls->decode_params.flags |= V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;

    if (h->picture_structure == PICT_BOTTOM_FIELD)
        controls->decode_params.flags |= V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;

#if defined(V4L2_H264_DECODE_PARAM_FLAG_PFRAME)
    if (sl->slice_type_nos == AV_PICTURE_TYPE_P)
        controls->decode_params.flags |= V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
#endif

#if defined(V4L2_H264_DECODE_PARAM_FLAG_BFRAME)
    if (sl->slice_type_nos == AV_PICTURE_TYPE_B)
        controls->decode_params.flags |= V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
#endif

    ret = fill_dpb(controls, h);
    if (ret < 0)
        return v4l2_request_h264_fail_frame(avctx, ret, false);

    /* A leading open-GOP field can select an FFmpeg gap placeholder. Reject
     * it before acquiring OUTPUT or publishing a successful capture callback. */
    controls->slice_params = (struct v4l2_ctrl_h264_slice_params) { 0 };
    ret = fill_slice_references(avctx, controls, &controls->slice_params, sl);
    if (ret < 0)
        return v4l2_request_h264_fail_frame(avctx, ret, false);

    ret = ff_v4l2_request_start_frame(avctx, &controls->pic, h->cur_pic_ptr->f);
    if (ret < 0)
        return v4l2_request_h264_fail_frame(avctx, ret, false);
    controls->started = true;

    return 0;
}

static int v4l2_request_h264_queue_decode(AVCodecContext *avctx, bool last_slice)
{
    const H264Context *h = avctx->priv_data;
    V4L2RequestContextH264 *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    int ret;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_H264_SPS,
            .ptr = &controls->sps,
            .size = sizeof(controls->sps),
        },
        {
            .id = V4L2_CID_STATELESS_H264_PPS,
            .ptr = &controls->pps,
            .size = sizeof(controls->pps),
        },
        {
            .id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
            .ptr = &controls->scaling_matrix,
            .size = sizeof(controls->scaling_matrix),
        },
        {
            .id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
            .ptr = &controls->decode_params,
            .size = sizeof(controls->decode_params),
        },
        {
            .id = V4L2_CID_STATELESS_H264_SLICE_PARAMS,
            .ptr = &controls->slice_params,
            .size = sizeof(controls->slice_params),
        },
        {
            .id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
            .ptr = &controls->pred_weights,
            .size = sizeof(controls->pred_weights),
        },
    };

    if (ctx->decode_mode == V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED) {
        int count = FF_ARRAY_ELEMS(control) - (controls->pred_weights_required ? 0 : 1);
        ret = ff_v4l2_request_decode_slice(avctx, &controls->pic, control, count,
                                          controls->first_slice, last_slice);
    } else {
        ret = ff_v4l2_request_decode_frame(avctx, &controls->pic,
                                          control, FF_ARRAY_ELEMS(control) - 2);
    }
    if (ret < 0)
        return v4l2_request_h264_fail_frame(avctx, ret, true);

    controls->held = !last_slice &&
        (controls->pic.output->capabilities & V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF);
    return 0;
}

static int v4l2_request_h264_decode_slice(AVCodecContext *avctx,
                                          const uint8_t *buffer, uint32_t size)
{
    const H264Context *h = avctx->priv_data;
    const PPS *pps = h->ps.pps;
    const H264SliceContext *sl = &h->slice_ctx[0];
    V4L2RequestContextH264 *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    struct v4l2_ctrl_h264_slice_params slice = {
        /* Offset in bits to slice_data() from the beginning of this slice. */
        .header_bit_size = get_bits_count(&sl->gb),
        .first_mb_in_slice = sl->first_mb_addr,
        .slice_type = ff_h264_get_slice_type(sl),
        .colour_plane_id = 0, /* separate colour plane not supported by FFmpeg */
        .redundant_pic_cnt = sl->redundant_pic_count,
        .cabac_init_idc = sl->cabac_init_idc,
        .slice_qp_delta = sl->qscale - pps->init_qp,
        .slice_qs_delta = 0, /* not implemented by FFmpeg */
        .disable_deblocking_filter_idc = sl->deblocking_filter < 2 ?
                                         !sl->deblocking_filter :
                                         sl->deblocking_filter,
        .slice_alpha_c0_offset_div2 = sl->slice_alpha_c0_offset / 2,
        .slice_beta_offset_div2 = sl->slice_beta_offset / 2,
        .num_ref_idx_l0_active_minus1 = sl->list_count > 0 ? sl->ref_count[0] - 1 : 0,
        .num_ref_idx_l1_active_minus1 = sl->list_count > 1 ? sl->ref_count[1] - 1 : 0,
    };
    int ret;

    if (controls->error)
        return controls->error;
    if (!controls->started)
        return v4l2_request_h264_fail_frame(avctx, AVERROR_INVALIDDATA, false);

    /* Validate the new slice without replacing the pending slice's controls.
     * Only then may the preceding slice be submitted with HOLD. */
    ret = fill_slice_references(avctx, controls, &slice, sl);
    if (ret < 0)
        goto fail;

    if (ctx->decode_mode == V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED &&
        controls->num_slices) {
        ret = v4l2_request_h264_queue_decode(avctx, false);
        if (ret < 0)
            return ret;

        ret = ff_v4l2_request_reset_picture(avctx, &controls->pic);
        if (ret < 0)
            goto fail;
        controls->first_slice = false;
    }

    if (h->mvc_sps && controls->first_slice && h->mvc_subsps) {
        /* Parameter NAL plus the actual type-20/base slice, both Annex B.
         * header_bit_size above remains relative to the slice NAL. */
        ret = ff_v4l2_request_append_output(avctx, &controls->pic,
                                           nalu_slice_start_code, 3);
        if (ret < 0)
            goto fail;
        ret = ff_v4l2_request_append_output(avctx, &controls->pic,
                                           h->mvc_subsps->data, h->mvc_subsps->size);
        if (ret < 0)
            goto fail;
    }
    if (ctx->start_code == V4L2_STATELESS_H264_START_CODE_ANNEX_B) {
        ret = ff_v4l2_request_append_output(avctx, &controls->pic,
                                            nalu_slice_start_code, 3);
        if (ret < 0)
            goto fail;
    }

    ret = ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
    if (ret < 0)
        goto fail;

    controls->num_slices++;
    if (ctx->decode_mode != V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED)
        return 0;

    if (sl->slice_type == AV_PICTURE_TYPE_B && sl->direct_spatial_mv_pred)
        slice.flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;

    /* V4L2_H264_SLICE_FLAG_SP_FOR_SWITCH: not implemented by FFmpeg */
    controls->slice_params = slice;
    controls->pred_weights_required =
        V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(&controls->pps, &slice);
    if (controls->pred_weights_required) {
        controls->pred_weights.chroma_log2_weight_denom = sl->pwt.chroma_log2_weight_denom;
        controls->pred_weights.luma_log2_weight_denom = sl->pwt.luma_log2_weight_denom;
        for (int list = 0; list < sl->list_count; list++)
            fill_weight_factors(&controls->pred_weights.weight_factors[list], list, sl);
    }

    return 0;
fail:
    return v4l2_request_h264_fail_frame(avctx, ret, false);
}

static int v4l2_request_h264_end_frame(AVCodecContext *avctx)
{
    const H264Context *h = avctx->priv_data;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    int ret;

    if (controls->error)
        return controls->error;
    if (!controls->started || !controls->num_slices)
        return v4l2_request_h264_fail_frame(avctx, AVERROR_INVALIDDATA, false);

    ret = v4l2_request_h264_queue_decode(avctx, true);
    if (ret < 0)
        return ret;
    controls->submitted_fields |= h->picture_structure;
    controls->started = false;
    return 0;
}

static int v4l2_request_h264_post_probe(AVCodecContext *avctx)
{
    V4L2RequestContextH264 *ctx = avctx->internal->hwaccel_priv_data;

    struct v4l2_ext_control control[] = {
        { .id = V4L2_CID_STATELESS_H264_DECODE_MODE, },
        { .id = V4L2_CID_STATELESS_H264_START_CODE, },
    };

    ctx->decode_mode = ff_v4l2_request_query_control_default_value(avctx,
                                            V4L2_CID_STATELESS_H264_DECODE_MODE);
    if (ctx->decode_mode != V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED &&
        ctx->decode_mode != V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED) {
        av_log(ctx, AV_LOG_VERBOSE, "Unsupported decode mode: %d\n",
               ctx->decode_mode);
        return AVERROR(EINVAL);
    }

    ctx->start_code = ff_v4l2_request_query_control_default_value(avctx,
                                            V4L2_CID_STATELESS_H264_START_CODE);
    if (ctx->start_code != V4L2_STATELESS_H264_START_CODE_NONE &&
        ctx->start_code != V4L2_STATELESS_H264_START_CODE_ANNEX_B) {
        av_log(ctx, AV_LOG_VERBOSE, "Unsupported start code: %d\n",
               ctx->start_code);
        return AVERROR(EINVAL);
    }

    if (((const H264Context *)avctx->priv_data)->mvc) {
        if (ctx->decode_mode != V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED)
            return AVERROR(ENOSYS);
        ctx->start_code = V4L2_STATELESS_H264_START_CODE_ANNEX_B;
    }
    // TODO: check V4L2_CID_MPEG_VIDEO_H264_PROFILE control
    // TODO: check V4L2_CID_MPEG_VIDEO_H264_LEVEL control

    control[0].value = ctx->decode_mode;
    control[1].value = ctx->start_code;

    return ff_v4l2_request_set_controls(avctx, control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_h264_init(AVCodecContext *avctx)
{
    V4L2RequestContextH264 *ctx = avctx->internal->hwaccel_priv_data;
    const H264Context *h = avctx->priv_data;
    struct v4l2_ctrl_h264_sps sps;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_H264_SPS,
            .ptr = &sps,
            .size = sizeof(sps),
        },
    };

    fill_sps(&sps, h);

    ctx->base.post_probe = v4l2_request_h264_post_probe;
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_H264_SLICE,
                                4 * 1024 * 1024,
                                control, FF_ARRAY_ELEMS(control));
}

const FFHWAccel ff_h264_v4l2request_hwaccel = {
    .p.name             = "h264_v4l2request",
    .p.type             = AVMEDIA_TYPE_VIDEO,
    .p.id               = AV_CODEC_ID_H264,
    .p.pix_fmt          = AV_PIX_FMT_DRM_PRIME,
    .start_frame        = v4l2_request_h264_start_frame,
    .decode_slice       = v4l2_request_h264_decode_slice,
    .end_frame          = v4l2_request_h264_end_frame,
    .flush              = ff_v4l2_request_flush,
    .frame_priv_data_size = sizeof(V4L2RequestControlsH264),
    .init               = v4l2_request_h264_init,
    .uninit             = ff_v4l2_request_uninit,
    .priv_data_size     = sizeof(V4L2RequestContextH264),
    .frame_params       = ff_v4l2_request_frame_params,
};
