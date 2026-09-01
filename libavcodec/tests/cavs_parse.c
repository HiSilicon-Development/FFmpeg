/*
 * Chinese AVS video header parser test
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

#include <stdio.h>
#include <string.h>

#include "libavcodec/cavs.h"
#include "libavcodec/put_bits.h"
#include "libavcodec/put_golomb.h"

static int failures;

#define CHECK_EQ(name, have, want) do {                                    \
    if ((have) != (want)) {                                                \
        printf("FAIL %s: got %u, expected %u\n", name,                     \
               (unsigned int)(have), (unsigned int)(want));                \
        failures++;                                                        \
    }                                                                      \
} while (0)

static void test_guangdian_field_i(void)
{
    static const uint8_t header[] = {
        0xff, 0xff, 0xff, 0x40, 0x08, 0xa4, 0x06,
    };
    static const uint8_t bad_reserved[] = {
        0xff, 0xff, 0xff, 0x40, 0x08, 0xa6, 0x06,
    };
    const AVSSequenceHeader sequence = {
        .width            = 1920,
        .height           = 1080,
        .profile_id       = 0x48,
        .level_id         = 0x41,
        .chroma_format    = 1,
        .sample_precision = 1,
        .valid            = 1,
    };
    AVSPictureHeader picture;
    int ret;

    ret = ff_cavs_parse_picture_header(header, sizeof(header), &sequence, 1,
                                       &picture);
    CHECK_EQ("valid field I return", ret, 0);
    if (ret >= 0) {
        CHECK_EQ("bbv_delay", picture.bbv_delay, 0x7fffff);
        CHECK_EQ("picture_distance", picture.picture_distance, 0);
        CHECK_EQ("picture_coding_type", picture.picture_coding_type, 0);
        CHECK_EQ("picture_structure", picture.picture_structure, 0);
        CHECK_EQ("picture_qp", picture.picture_qp, 20);
        CHECK_EQ("picture flags", picture.flags,
                 CAVS_PICTURE_FLAG_TOP_FIELD_FIRST |
                 CAVS_PICTURE_FLAG_SKIP_MODE |
                 CAVS_PICTURE_FLAG_REFERENCE |
                 CAVS_PICTURE_FLAG_AEC);
    }

    ret = ff_cavs_parse_picture_header(bad_reserved, sizeof(bad_reserved),
                                       &sequence, 1, &picture);
    CHECK_EQ("nonzero reserved return", ret < 0, 1);
}

static size_t make_guangdian_i_wq(uint8_t *header, size_t size,
                                  int reserved, int param_index, int model,
                                  int delta)
{
    PutBitContext pb;
    int bits;

    memset(header, 0, size);
    init_put_bits(&pb, header, size);
    put_bits(&pb, 16, 0xffff); /* bbv_delay */
    put_bits(&pb, 1, 1);       /* marker_bit */
    put_bits(&pb, 7, 0x7f);    /* bbv_delay_extension */
    put_bits(&pb, 1, 0);       /* time_code_flag */
    put_bits(&pb, 1, 1);       /* marker_bit */
    put_bits(&pb, 8, 0);       /* picture_distance */
    put_bits(&pb, 1, 1);       /* progressive_frame */
    put_bits(&pb, 1, 0);       /* top_field_first */
    put_bits(&pb, 1, 0);       /* repeat_first_field */
    put_bits(&pb, 1, 1);       /* fixed_picture_qp */
    put_bits(&pb, 6, 20);      /* picture_qp */
    put_bits(&pb, 4, 0);       /* reserved_bits */
    put_bits(&pb, 1, 1);       /* loop_filter_disable */
    put_bits(&pb, 1, 1);       /* weighting_quant_flag */
    put_bits(&pb, 1, reserved); /* reserved_bits */
    put_bits(&pb, 1, 1);       /* chroma_quant_param_disable */
    put_bits(&pb, 2, param_index);
    put_bits(&pb, 2, model);
    if (param_index == 1 || param_index == 2)
        for (int i = 0; i < 6; i++)
            set_se_golomb(&pb, delta);
    put_bits(&pb, 1, 1);       /* aec_enable */
    bits = put_bits_count(&pb);
    flush_put_bits(&pb);
    return (bits + 7) / 8;
}

static void test_guangdian_weighting_quant(void)
{
    const AVSSequenceHeader sequence = {
        .width            = 1920,
        .height           = 1080,
        .profile_id       = 0x48,
        .level_id         = 0x41,
        .chroma_format    = 1,
        .sample_precision = 1,
        .valid            = 1,
    };
    AVSPictureHeader picture[2] = { 0 };
    uint8_t header[32];
    size_t header_size;
    int ret;

    for (int reserved = 0; reserved <= 1; reserved++) {
        header_size = make_guangdian_i_wq(header, sizeof(header), reserved,
                                          1, 0, 0);
        ret = ff_cavs_parse_picture_header(header, header_size, &sequence, 1,
                                           &picture[reserved]);
        CHECK_EQ(reserved ? "WQ reserved=1 return" : "WQ reserved=0 return",
                 ret, 0);
        if (ret >= 0) {
            CHECK_EQ("WQ matrix[0]", picture[reserved].weighting_quant_matrix[0],
                     135);
            CHECK_EQ("WQ matrix[3]", picture[reserved].weighting_quant_matrix[3],
                     160);
            CHECK_EQ("WQ matrix[6]", picture[reserved].weighting_quant_matrix[6],
                     213);
        }
    }
    CHECK_EQ("WQ reserved matrix equality",
             memcmp(picture[0].weighting_quant_matrix,
                    picture[1].weighting_quant_matrix,
                    sizeof(picture[0].weighting_quant_matrix)), 0);

    header_size = make_guangdian_i_wq(header, sizeof(header), 0, 2, 2, 1);
    ret = ff_cavs_parse_picture_header(header, header_size, &sequence, 1,
                                       &picture[0]);
    CHECK_EQ("WQ param index 2 return", ret, 0);
    if (ret >= 0) {
        CHECK_EQ("WQ index 2 matrix[0]", picture[0].weighting_quant_matrix[0],
                 129);
        CHECK_EQ("WQ index 2 matrix[3]", picture[0].weighting_quant_matrix[3],
                 117);
        CHECK_EQ("WQ index 2 matrix[6]", picture[0].weighting_quant_matrix[6],
                 129);
    }

    header_size = make_guangdian_i_wq(header, sizeof(header), 0, 3, 0, 0);
    ret = ff_cavs_parse_picture_header(header, header_size, &sequence, 1,
                                       &picture[0]);
    CHECK_EQ("WQ reserved param index return", ret < 0, 1);

    header_size = make_guangdian_i_wq(header, sizeof(header), 0, 0, 3, 0);
    ret = ff_cavs_parse_picture_header(header, header_size, &sequence, 1,
                                       &picture[0]);
    CHECK_EQ("WQ reserved model return", ret < 0, 1);

    header_size = make_guangdian_i_wq(header, sizeof(header), 0, 1, 0, 128);
    ret = ff_cavs_parse_picture_header(header, header_size, &sequence, 1,
                                       &picture[0]);
    CHECK_EQ("WQ delta out of range return", ret < 0, 1);
}

int main(void)
{
    test_guangdian_field_i();
    test_guangdian_weighting_quant();

    if (failures)
        printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");

    return !!failures;
}
