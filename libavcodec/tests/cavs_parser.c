/*
 * Chinese AVS video parser tests
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

/* Exercise the parser's static frame-boundary scanner directly. */
#define ff_cavsvideo_parser test_cavsvideo_parser
#include "libavcodec/cavs_parser.c"

static int failures;

#define CHECK_INT(name, have, want) do {                                \
    int have_ = (have);                                                 \
    int want_ = (want);                                                 \
    if (have_ != want_) {                                               \
        printf("FAIL %s: got %d, expected %d\n", name, have_, want_); \
        failures++;                                                     \
    }                                                                   \
} while (0)

static void reset_context(ParseContext *pc)
{
    *pc = (ParseContext) { .state = -1 };
}

static void test_epoch_boundaries(void)
{
    static const uint8_t picture_b1_b0[] = {
        0x00, 0x00, 0x01, 0xb3, 0xaa,
        0x00, 0x00, 0x01, 0xb1,
        0x00, 0x00, 0x01, 0xb0,
    };
    static const uint8_t picture_b7_i[] = {
        0x00, 0x00, 0x01, 0xb6, 0xaa,
        0x00, 0x00, 0x01, 0xb7,
        0x00, 0x00, 0x01, 0xb3,
    };
    ParseContext pc;
    int end;

    reset_context(&pc);
    end = cavs_find_frame_end(&pc, picture_b1_b0,
                              sizeof(picture_b1_b0));
    CHECK_INT("picture ends before B1", end, 5);
    end = cavs_find_frame_end(&pc, picture_b1_b0 + end,
                              sizeof(picture_b1_b0) - end);
    CHECK_INT("B1 ends before B0", end, 4);

    reset_context(&pc);
    end = cavs_find_frame_end(&pc, picture_b7_i, sizeof(picture_b7_i));
    CHECK_INT("picture ends before B7", end, 5);
    end = cavs_find_frame_end(&pc, picture_b7_i + end,
                              sizeof(picture_b7_i) - end);
    CHECK_INT("B7 ends before I", end, 4);
}

static void test_split_prefix(void)
{
    static const uint8_t first[] = {
        0x00, 0x00, 0x01, 0xb3, 0xaa, 0x00, 0x00,
    };
    static const uint8_t second[] = {
        0x01, 0xb1, 0x00, 0x00, 0x01, 0xb0,
    };
    static const uint8_t combined_boundary[] = {
        0x00, 0x00, 0x01, 0xb1,
        0x00, 0x00, 0x01, 0xb0,
    };
    ParseContext pc;
    int end;

    reset_context(&pc);
    end = cavs_find_frame_end(&pc, first, sizeof(first));
    CHECK_INT("split prefix first chunk", end, END_NOT_FOUND);
    end = cavs_find_frame_end(&pc, second, sizeof(second));
    CHECK_INT("split prefix overlap", end, -2);

    reset_context(&pc);
    end = cavs_find_frame_end(&pc, combined_boundary,
                              sizeof(combined_boundary));
    CHECK_INT("split B1 ends before B0", end, 4);
}

int main(void)
{
    test_epoch_boundaries();
    test_split_prefix();

    if (failures)
        printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");

    return !!failures;
}
