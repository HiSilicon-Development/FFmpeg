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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"

#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "mathops.h"
#include "mpegvideo.h"
#include "v4l2_request_internal.h"

typedef struct V4L2RequestControlsMPEG1 {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_mpeg1_sequence sequence;
    struct v4l2_ctrl_mpeg1_picture picture;
    struct v4l2_ctrl_mpeg1_quantisation quantisation;
} V4L2RequestControlsMPEG1;

static int v4l2_request_mpeg1_post_probe(AVCodecContext *avctx)
{
    static const struct {
        uint32_t id;
        uint32_t type;
        uint32_t size;
        const char *name;
    } expected[] = {
        {
            V4L2_CID_STATELESS_MPEG1_SEQUENCE,
            V4L2_CTRL_TYPE_MPEG1_SEQUENCE,
            sizeof(struct v4l2_ctrl_mpeg1_sequence),
            "sequence",
        },
        {
            V4L2_CID_STATELESS_MPEG1_PICTURE,
            V4L2_CTRL_TYPE_MPEG1_PICTURE,
            sizeof(struct v4l2_ctrl_mpeg1_picture),
            "picture",
        },
        {
            V4L2_CID_STATELESS_MPEG1_QUANTISATION,
            V4L2_CTRL_TYPE_MPEG1_QUANTISATION,
            sizeof(struct v4l2_ctrl_mpeg1_quantisation),
            "quantisation",
        },
    };

    for (int i = 0; i < FF_ARRAY_ELEMS(expected); i++) {
        struct v4l2_query_ext_ctrl query = { .id = expected[i].id };
        int ret = ff_v4l2_request_query_control(avctx, &query);

        if (ret < 0)
            return ret;
        if (query.type != expected[i].type ||
            query.elem_size != expected[i].size || query.elems != 1 ||
            !(query.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD)) {
            av_log(avctx, AV_LOG_ERROR,
                   "MPEG-1 V4L2 control %s has incompatible ABI "
                   "(type %#x, elem_size %u, elems %u, flags %#x)\n",
                   expected[i].name, query.type, query.elem_size,
                   query.elems, query.flags);
            return AVERROR(ENOTSUP);
        }
    }

    return 0;
}

static int v4l2_request_mpeg1_start_frame(AVCodecContext *avctx,
                                          av_unused const AVBufferRef *buf_ref,
                                          av_unused const uint8_t *buffer,
                                          av_unused uint32_t size)
{
    const MpegEncContext *s = avctx->priv_data;
    V4L2RequestControlsMPEG1 *controls =
        s->cur_pic.ptr->hwaccel_picture_private;
    struct v4l2_ctrl_mpeg1_picture *picture = &controls->picture;
    int ret;

    ret = ff_v4l2_request_start_frame(avctx, &controls->pic,
                                      s->cur_pic.ptr->f);
    if (ret)
        return ret;

    controls->sequence = (struct v4l2_ctrl_mpeg1_sequence) {
        .horizontal_size = s->width,
        .vertical_size = s->height,
        .vbv_buffer_size = controls->pic.output->size,
    };

    /* MPEG-1 has one f_code per prediction direction.  The kernel ABI
     * requires both bytes to be non-zero even when a direction is unused. */
    *picture = (struct v4l2_ctrl_mpeg1_picture) {
        .f_code = { 1, 1 },
    };

    switch (s->pict_type) {
    case AV_PICTURE_TYPE_I:
        picture->picture_coding_type = V4L2_MPEG1_PIC_CODING_TYPE_I;
        break;
    case AV_PICTURE_TYPE_P:
        picture->picture_coding_type = V4L2_MPEG1_PIC_CODING_TYPE_P;
        picture->f_code[0] = s->mpeg_f_code[0][0];
        if (s->full_pel[0])
            picture->flags |= V4L2_MPEG1_PIC_FLAG_FULL_PEL_FORWARD;
        if (s->last_pic.ptr)
            picture->forward_ref_ts =
                ff_v4l2_request_get_capture_timestamp(s->last_pic.ptr->f);
        break;
    case AV_PICTURE_TYPE_B:
        picture->picture_coding_type = V4L2_MPEG1_PIC_CODING_TYPE_B;
        picture->f_code[0] = s->mpeg_f_code[0][0];
        picture->f_code[1] = s->mpeg_f_code[1][0];
        if (s->full_pel[0])
            picture->flags |= V4L2_MPEG1_PIC_FLAG_FULL_PEL_FORWARD;
        if (s->full_pel[1])
            picture->flags |= V4L2_MPEG1_PIC_FLAG_FULL_PEL_BACKWARD;
        if (s->last_pic.ptr)
            picture->forward_ref_ts =
                ff_v4l2_request_get_capture_timestamp(s->last_pic.ptr->f);
        if (s->next_pic.ptr)
            picture->backward_ref_ts =
                ff_v4l2_request_get_capture_timestamp(s->next_pic.ptr->f);
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    for (int i = 0; i < 64; i++) {
        int n = s->idsp.idct_permutation[ff_zigzag_direct[i]];

        controls->quantisation.intra_quantiser_matrix[i] = s->intra_matrix[n];
        controls->quantisation.non_intra_quantiser_matrix[i] = s->inter_matrix[n];
    }

    return 0;
}

static int v4l2_request_mpeg1_decode_slice(AVCodecContext *avctx,
                                           const uint8_t *buffer, uint32_t size)
{
    const MpegEncContext *s = avctx->priv_data;
    V4L2RequestControlsMPEG1 *controls =
        s->cur_pic.ptr->hwaccel_picture_private;

    return ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
}

static int v4l2_request_mpeg1_end_frame(AVCodecContext *avctx)
{
    const MpegEncContext *s = avctx->priv_data;
    V4L2RequestControlsMPEG1 *controls =
        s->cur_pic.ptr->hwaccel_picture_private;
    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_MPEG1_SEQUENCE,
            .ptr = &controls->sequence,
            .size = sizeof(controls->sequence),
        },
        {
            .id = V4L2_CID_STATELESS_MPEG1_PICTURE,
            .ptr = &controls->picture,
            .size = sizeof(controls->picture),
        },
        {
            .id = V4L2_CID_STATELESS_MPEG1_QUANTISATION,
            .ptr = &controls->quantisation,
            .size = sizeof(controls->quantisation),
        },
    };

    return ff_v4l2_request_decode_frame(avctx, &controls->pic,
                                        control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_mpeg1_init(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    ctx->post_probe = v4l2_request_mpeg1_post_probe;
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_MPEG1_SLICE,
                                1024 * 1024, NULL, 0);
}

const FFHWAccel ff_mpeg1_v4l2request_hwaccel = {
    .p.name             = "mpeg1_v4l2request",
    .p.type             = AVMEDIA_TYPE_VIDEO,
    .p.id               = AV_CODEC_ID_MPEG1VIDEO,
    .p.pix_fmt          = AV_PIX_FMT_DRM_PRIME,
    .start_frame        = v4l2_request_mpeg1_start_frame,
    .decode_slice       = v4l2_request_mpeg1_decode_slice,
    .end_frame          = v4l2_request_mpeg1_end_frame,
    .flush              = ff_v4l2_request_flush,
    .frame_priv_data_size = sizeof(V4L2RequestControlsMPEG1),
    .init               = v4l2_request_mpeg1_init,
    .uninit             = ff_v4l2_request_uninit,
    .priv_data_size     = sizeof(V4L2RequestContext),
    .frame_params       = ff_v4l2_request_frame_params,
};
