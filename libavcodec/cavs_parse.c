/*
 * Chinese AVS video header parsing helpers.
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

#include <limits.h>

#include "libavutil/error.h"

#include "cavs.h"
#include "get_bits.h"

#define CAVS_PROFILE_JIZHUN    0x20
#define CAVS_PROFILE_GUANGDIAN 0x48

typedef struct AVSWeightingQuant {
    int param_index;
    int model;
    int delta[2][6];
} AVSWeightingQuant;

/* Parameter-model maps used to derive the AVS+ raster-order matrix. */
static const uint8_t cavs_weight_quant_model[4][64] = {
    {
        0, 0, 0, 4, 4, 4, 5, 5,
        0, 0, 3, 3, 3, 3, 5, 5,
        0, 3, 2, 2, 1, 1, 5, 5,
        4, 3, 2, 2, 1, 5, 5, 5,
        4, 3, 1, 1, 5, 5, 5, 5,
        4, 3, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
    }, {
        0, 0, 0, 4, 4, 4, 5, 5,
        0, 0, 4, 4, 4, 4, 5, 5,
        0, 3, 2, 2, 2, 1, 5, 5,
        3, 3, 2, 2, 1, 5, 5, 5,
        3, 3, 2, 1, 5, 5, 5, 5,
        3, 3, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
    }, {
        0, 0, 0, 4, 4, 3, 5, 5,
        0, 0, 4, 4, 3, 2, 5, 5,
        0, 4, 4, 3, 2, 1, 5, 5,
        4, 4, 3, 2, 1, 5, 5, 5,
        4, 3, 2, 1, 5, 5, 5, 5,
        3, 2, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
    }, {
        0, 0, 0, 3, 2, 1, 5, 5,
        0, 0, 4, 3, 2, 1, 5, 5,
        0, 4, 4, 3, 2, 1, 5, 5,
        3, 3, 3, 3, 2, 5, 5, 5,
        2, 2, 2, 2, 5, 5, 5, 5,
        1, 1, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5,
    },
};

static const int cavs_weight_quant_default[2][6] = {
    { 135, 143, 143, 160, 160, 213 },
    { 128,  98, 106, 116, 116, 128 },
};

static int cavs_read_bits(GetBitContext *gb, unsigned int count,
                          unsigned int *value)
{
    if (count > 32 || get_bits_left(gb) < count)
        return AVERROR_INVALIDDATA;

    *value = count == 32 ? get_bits_long(gb, count) : get_bits(gb, count);
    return 0;
}

static int cavs_read_marker(GetBitContext *gb)
{
    unsigned int value;

    if (cavs_read_bits(gb, 1, &value) < 0 || value != 1)
        return AVERROR_INVALIDDATA;
    return 0;
}

static int cavs_read_ue(GetBitContext *gb, unsigned int *value)
{
    unsigned int bit, suffix = 0, zeros = 0;

    do {
        if (cavs_read_bits(gb, 1, &bit) < 0)
            return AVERROR_INVALIDDATA;
        if (!bit && ++zeros >= 32)
            return AVERROR_INVALIDDATA;
    } while (!bit);

    if (zeros && cavs_read_bits(gb, zeros, &suffix) < 0)
        return AVERROR_INVALIDDATA;
    *value = ((1U << zeros) - 1) + suffix;
    return 0;
}

static int cavs_read_se(GetBitContext *gb, int *value)
{
    unsigned int code;

    if (cavs_read_ue(gb, &code) < 0 || code == UINT_MAX)
        return AVERROR_INVALIDDATA;
    *value = code & 1 ? (int)(code / 2) + 1 : -(int)(code / 2);
    return 0;
}

int ff_cavs_parse_sequence_header(const uint8_t *buf, size_t size,
                                  AVSSequenceHeader *sequence)
{
    GetBitContext gb;
    unsigned int bit_rate_low, bit_rate_high, bbv_buffer_size;
    unsigned int value, reserved;
    int ret;

    if (size > INT_MAX)
        return AVERROR_INVALIDDATA;
    ret = init_get_bits8(&gb, buf, size);
    if (ret < 0)
        return ret;

    *sequence = (AVSSequenceHeader) { 0 };
    if (cavs_read_bits(&gb, 8, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->profile_id = value;
    if (cavs_read_bits(&gb, 8, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->level_id = value;
    if (cavs_read_bits(&gb, 1, &value) < 0)
        return AVERROR_INVALIDDATA;
    if (value)
        sequence->flags |= CAVS_SEQUENCE_FLAG_PROGRESSIVE;
    if (cavs_read_bits(&gb, 14, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->width = value;
    if (cavs_read_bits(&gb, 14, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->height = value;
    if (cavs_read_bits(&gb, 2, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->chroma_format = value;
    if (cavs_read_bits(&gb, 3, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->sample_precision = value;
    if (cavs_read_bits(&gb, 4, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->aspect_ratio = value;
    if (cavs_read_bits(&gb, 4, &value) < 0)
        return AVERROR_INVALIDDATA;
    sequence->frame_rate_code = value;

    if (cavs_read_bits(&gb, 18, &bit_rate_low) < 0 ||
        cavs_read_marker(&gb) < 0 ||
        cavs_read_bits(&gb, 12, &bit_rate_high) < 0 ||
        cavs_read_bits(&gb, 1, &value) < 0)
        return AVERROR_INVALIDDATA;
    if (value)
        sequence->flags |= CAVS_SEQUENCE_FLAG_LOW_DELAY;
    if (cavs_read_marker(&gb) < 0 ||
        cavs_read_bits(&gb, 18, &bbv_buffer_size) < 0 ||
        cavs_read_bits(&gb, 3, &reserved) < 0 || reserved)
        return AVERROR_INVALIDDATA;

    if ((sequence->profile_id != CAVS_PROFILE_JIZHUN &&
         sequence->profile_id != CAVS_PROFILE_GUANGDIAN) ||
        !sequence->width || !sequence->height ||
        sequence->chroma_format != 1 || sequence->sample_precision != 1 ||
        sequence->frame_rate_code == 0 || sequence->frame_rate_code > 13)
        return AVERROR_INVALIDDATA;

    sequence->valid = 1;
    return 0;
}

static int cavs_parse_weighting_quant(GetBitContext *gb,
                                      AVSPictureHeader *picture,
                                      AVSWeightingQuant *wq)
{
    unsigned int enabled, chroma_disabled, value;
    int delta;

    if (cavs_read_bits(gb, 1, &enabled) < 0)
        return AVERROR_INVALIDDATA;
    if (!enabled)
        return 0;

    picture->flags |= CAVS_PICTURE_FLAG_WEIGHTING_QUANT;
    /* reserved_bits r(1): decoders shall ignore it (clause 5.8.4). */
    if (cavs_read_bits(gb, 1, &value) < 0)
        return AVERROR_INVALIDDATA;
    if (cavs_read_bits(gb, 1, &chroma_disabled) < 0)
        return AVERROR_INVALIDDATA;
    if (chroma_disabled) {
        picture->flags |= CAVS_PICTURE_FLAG_CHROMA_QP_DISABLE;
    } else {
        if (cavs_read_se(gb, &delta) < 0 || delta < -32 || delta > 31)
            return AVERROR_INVALIDDATA;
        picture->chroma_qp_delta_u = delta;
        if (cavs_read_se(gb, &delta) < 0 || delta < -32 || delta > 31)
            return AVERROR_INVALIDDATA;
        picture->chroma_qp_delta_v = delta;
    }

    if (cavs_read_bits(gb, 2, &value) < 0)
        return AVERROR_INVALIDDATA;
    wq->param_index = value;
    if (cavs_read_bits(gb, 2, &value) < 0)
        return AVERROR_INVALIDDATA;
    wq->model = value;
    /* The value 3 is reserved for both syntax elements (clause 7.2.3). */
    if (wq->param_index == 3 || wq->model == 3)
        return AVERROR_INVALIDDATA;

    if (wq->param_index == 1) {
        for (int i = 0; i < 6; i++) {
            if (cavs_read_se(gb, &wq->delta[0][i]) < 0 ||
                wq->delta[0][i] < -128 || wq->delta[0][i] > 127)
                return AVERROR_INVALIDDATA;
        }
    }
    if (wq->param_index == 2) {
        for (int i = 0; i < 6; i++) {
            if (cavs_read_se(gb, &wq->delta[1][i]) < 0 ||
                wq->delta[1][i] < -128 || wq->delta[1][i] > 127)
                return AVERROR_INVALIDDATA;
        }
    }

    return 0;
}

static int cavs_resolve_weighting_quant(AVSPictureHeader *picture,
                                        const AVSWeightingQuant *wq)
{
    int parameters[2][6];
    int selected = -1;

    if (!(picture->flags & CAVS_PICTURE_FLAG_WEIGHTING_QUANT))
        return 0;
    if (wq->param_index == 3 || wq->model == 3)
        return AVERROR_INVALIDDATA;

    for (int set = 0; set < 2; set++)
        for (int i = 0; i < 6; i++)
            parameters[set][i] = 128;

    switch (wq->param_index) {
    case 0:
        for (int i = 0; i < 6; i++)
            parameters[1][i] = cavs_weight_quant_default[1][i];
        selected = 1;
        break;
    case 1:
        for (int i = 0; i < 6; i++)
            parameters[0][i] = cavs_weight_quant_default[0][i] +
                               wq->delta[0][i];
        selected = 0;
        break;
    case 2:
        for (int i = 0; i < 6; i++)
            parameters[1][i] = cavs_weight_quant_default[1][i] +
                               wq->delta[1][i];
        selected = 1;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    for (int i = 0; i < 64; i++) {
        int value = selected < 0 ? 128 :
                    parameters[selected][cavs_weight_quant_model[wq->model][i]];

        if (value < 0 || value > UINT8_MAX)
            return AVERROR_INVALIDDATA;
        picture->weighting_quant_matrix[i] = value;
    }
    return 0;
}

int ff_cavs_parse_picture_header(const uint8_t *buf, size_t size,
                                 const AVSSequenceHeader *sequence,
                                 int intra, AVSPictureHeader *picture)
{
    AVSWeightingQuant wq = { 0 };
    GetBitContext gb;
    unsigned int value, extension = 0, reserved;
    unsigned int progressive_frame, top_field_first, repeat_first_field;
    unsigned int fixed_qp, skip_mode = 0, loop_filter_disable;
    unsigned int loop_filter_params = 0, reference = intra, no_forward = 0;
    unsigned int advanced_pred_disable = 0, field_enhanced = 0;
    int offset, ret;

    if (!sequence->valid || size > INT_MAX)
        return AVERROR_INVALIDDATA;
    ret = init_get_bits8(&gb, buf, size);
    if (ret < 0)
        return ret;

    *picture = (AVSPictureHeader) { 0 };
    if (cavs_read_bits(&gb, 16, &value) < 0)
        return AVERROR_INVALIDDATA;
    if (sequence->profile_id == CAVS_PROFILE_GUANGDIAN &&
        (cavs_read_marker(&gb) < 0 ||
         cavs_read_bits(&gb, 7, &extension) < 0))
        return AVERROR_INVALIDDATA;
    picture->bbv_delay = sequence->profile_id == CAVS_PROFILE_GUANGDIAN ?
                         (value << 7) | extension : value;

    if (intra) {
        picture->picture_coding_type = 0;
        if (cavs_read_bits(&gb, 1, &value) < 0 ||
            (value && cavs_read_bits(&gb, 24, &value) < 0) ||
            cavs_read_marker(&gb) < 0 ||
            cavs_read_bits(&gb, 8, &picture->picture_distance) < 0)
            return AVERROR_INVALIDDATA;
    } else {
        if (cavs_read_bits(&gb, 2, &value) < 0 || value < 1 || value > 2)
            return AVERROR_INVALIDDATA;
        picture->picture_coding_type = value;
        if (cavs_read_bits(&gb, 8, &picture->picture_distance) < 0)
            return AVERROR_INVALIDDATA;
    }

    if ((sequence->flags & CAVS_SEQUENCE_FLAG_LOW_DELAY) &&
        cavs_read_ue(&gb, &value) < 0)
        return AVERROR_INVALIDDATA;
    if (cavs_read_bits(&gb, 1, &progressive_frame) < 0)
        return AVERROR_INVALIDDATA;

    picture->picture_structure = 1;
    if (!(sequence->flags & CAVS_SEQUENCE_FLAG_PROGRESSIVE) &&
        !progressive_frame) {
        if (cavs_read_bits(&gb, 1, &value) < 0)
            return AVERROR_INVALIDDATA;
        picture->picture_structure = value;
    } else if ((sequence->flags & CAVS_SEQUENCE_FLAG_PROGRESSIVE) &&
               !progressive_frame) {
        return AVERROR_INVALIDDATA;
    }

    if (!intra && !progressive_frame && !picture->picture_structure &&
        cavs_read_bits(&gb, 1, &advanced_pred_disable) < 0)
        return AVERROR_INVALIDDATA;
    if (cavs_read_bits(&gb, 1, &top_field_first) < 0 ||
        cavs_read_bits(&gb, 1, &repeat_first_field) < 0 ||
        cavs_read_bits(&gb, 1, &fixed_qp) < 0 ||
        cavs_read_bits(&gb, 6, &value) < 0)
        return AVERROR_INVALIDDATA;
    picture->picture_qp = value;

    if (!intra) {
        if (picture->picture_coding_type == 2 && picture->picture_structure) {
            reference = 1;
        } else if (cavs_read_bits(&gb, 1, &reference) < 0) {
            return AVERROR_INVALIDDATA;
        }
        if (cavs_read_bits(&gb, 1, &no_forward) < 0)
            return AVERROR_INVALIDDATA;
        if (sequence->profile_id == CAVS_PROFILE_GUANGDIAN) {
            if (cavs_read_bits(&gb, 1, &field_enhanced) < 0 ||
                cavs_read_bits(&gb, 2, &reserved) < 0 || reserved ||
                (picture->picture_structure && field_enhanced))
                return AVERROR_INVALIDDATA;
        } else if (cavs_read_bits(&gb, 3, &reserved) < 0 || reserved) {
            return AVERROR_INVALIDDATA;
        }
        if (cavs_read_bits(&gb, 1, &skip_mode) < 0)
            return AVERROR_INVALIDDATA;
    } else {
        if (!progressive_frame && !picture->picture_structure &&
            cavs_read_bits(&gb, 1, &skip_mode) < 0)
            return AVERROR_INVALIDDATA;
        if (cavs_read_bits(&gb, 4, &reserved) < 0 || reserved)
            return AVERROR_INVALIDDATA;
    }

    if (cavs_read_bits(&gb, 1, &loop_filter_disable) < 0)
        return AVERROR_INVALIDDATA;
    if (!loop_filter_disable) {
        if (cavs_read_bits(&gb, 1, &loop_filter_params) < 0)
            return AVERROR_INVALIDDATA;
        if (loop_filter_params) {
            if (cavs_read_se(&gb, &offset) < 0 || offset < -16 || offset > 15)
                return AVERROR_INVALIDDATA;
            picture->alpha_c_offset = offset;
            if (cavs_read_se(&gb, &offset) < 0 || offset < -16 || offset > 15)
                return AVERROR_INVALIDDATA;
            picture->beta_offset = offset;
        }
    }
    if (sequence->profile_id == CAVS_PROFILE_GUANGDIAN) {
        ret = cavs_parse_weighting_quant(&gb, picture, &wq);
        if (ret < 0)
            return ret;
        if (cavs_read_bits(&gb, 1, &value) < 0)
            return AVERROR_INVALIDDATA;
        if (value)
            picture->flags |= CAVS_PICTURE_FLAG_AEC;
    }

    if (progressive_frame)
        picture->flags |= CAVS_PICTURE_FLAG_PROGRESSIVE_FRAME;
    if (top_field_first)
        picture->flags |= CAVS_PICTURE_FLAG_TOP_FIELD_FIRST;
    if (repeat_first_field)
        picture->flags |= CAVS_PICTURE_FLAG_REPEAT_FIRST_FIELD;
    if (fixed_qp)
        picture->flags |= CAVS_PICTURE_FLAG_FIXED_QP;
    if (skip_mode)
        picture->flags |= CAVS_PICTURE_FLAG_SKIP_MODE;
    if (loop_filter_disable)
        picture->flags |= CAVS_PICTURE_FLAG_LOOP_FILTER_DISABLE;
    if (loop_filter_params)
        picture->flags |= CAVS_PICTURE_FLAG_LOOP_FILTER_PARAMS;
    if (reference)
        picture->flags |= CAVS_PICTURE_FLAG_REFERENCE;
    if (no_forward)
        picture->flags |= CAVS_PICTURE_FLAG_NO_FORWARD_REFERENCE;
    if (advanced_pred_disable)
        picture->flags |= CAVS_PICTURE_FLAG_ADVANCED_PRED_DISABLE;
    if (field_enhanced) {
        picture->flags |= picture->picture_coding_type == 2 ?
                          CAVS_PICTURE_FLAG_B_FIELD_ENHANCED :
                          CAVS_PICTURE_FLAG_P_FIELD_ENHANCED;
    }

    return cavs_resolve_weighting_quant(picture, &wq);
}
