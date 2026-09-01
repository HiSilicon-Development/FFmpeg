/*
 * Chinese AVS video weighted prediction tests
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

#include "libavcodec/cavs.h"

static int failures;

#define CHECK_INT(have, want) do {                                      \
    int have_ = (have);                                                 \
    int want_ = (want);                                                 \
    if (have_ != want_) {                                               \
        printf("FAIL %s:%d: got %d, expected %d\n",                    \
               __func__, __LINE__, have_, want_);                       \
        failures++;                                                     \
    }                                                                   \
} while (0)

static void test_weight_sample(void)
{
    /* Neutral weights preserve all input values, including the endpoints. */
    CHECK_INT(ff_cavs_weight_sample(0,   32, 0),   0);
    CHECK_INT(ff_cavs_weight_sample(1,   32, 0),   1);
    CHECK_INT(ff_cavs_weight_sample(127, 32, 0), 127);
    CHECK_INT(ff_cavs_weight_sample(255, 32, 0), 255);

    /* The +16 term rounds before the signed shift is applied. */
    CHECK_INT(ff_cavs_weight_sample(1, 16,  0), 1);
    CHECK_INT(ff_cavs_weight_sample(1, 15,  0), 0);
    CHECK_INT(ff_cavs_weight_sample(7, 24, 16), 21);

    /* Both ends are clipped after scale and shift. */
    CHECK_INT(ff_cavs_weight_sample(0,   32, -1),   0);
    CHECK_INT(ff_cavs_weight_sample(255, 40, 16), 255);
}

static void test_weight_index(void)
{
    /* P and the second field of an I picture index their one list directly. */
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_P, 1, 1, 0), 0);
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_P, 2, 1, 0), 1);
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_I, 3, 3, 0), 0);

    /* B forward slots are 2*i, backward slots are 2*j+1. */
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_B, 0, 0, 0), 0);
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_B, 1, 0, 0), 2);
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_B, 2, 2, 1), 1);
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_B, 3, 2, 1), 3);

    /* The caller rejects results outside the four signalled slots. */
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_P, 0, 1, 0), -1);
    CHECK_INT(ff_cavs_weight_index(AV_PICTURE_TYPE_B, 2, 0, 0),  4);
}

int main(void)
{
    test_weight_sample();
    test_weight_index();

    if (failures)
        printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");

    return !!failures;
}
