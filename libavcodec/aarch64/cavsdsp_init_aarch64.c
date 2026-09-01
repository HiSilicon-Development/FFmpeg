/*
 * Chinese AVS video (AVS1-P2, JiZhun profile) NEON optimisations
 * Copyright (c) 2026 Rainbaby <rainbaby@outlook.jp>
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

#include <stdint.h>
#include <stddef.h>

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/aarch64/cpu.h"
#include "libavcodec/cavsdsp.h"
#include "libavcodec/idctdsp.h"

void ff_cavs_idct8_add_neon(uint8_t *dst, int16_t *block, ptrdiff_t stride);

void ff_cavs_filter_lv_neon(uint8_t *d, ptrdiff_t stride, int alpha, int beta,
                            int tc, int bs1, int bs2);
void ff_cavs_filter_lh_neon(uint8_t *d, ptrdiff_t stride, int alpha, int beta,
                            int tc, int bs1, int bs2);
void ff_cavs_filter_cv_neon(uint8_t *d, ptrdiff_t stride, int alpha, int beta,
                            int tc, int bs1, int bs2);
void ff_cavs_filter_ch_neon(uint8_t *d, ptrdiff_t stride, int alpha, int beta,
                            int tc, int bs1, int bs2);

#define CAVS_QPEL_PROTO(OP, SIZE, MC) \
void ff_##OP##_cavs_qpel##SIZE##_##MC##_neon(uint8_t *dst,   \
                                             const uint8_t *src, \
                                             ptrdiff_t stride)

#define CAVS_QPEL_PROTOS(OP, SIZE)   \
CAVS_QPEL_PROTO(OP, SIZE, mc00);     \
CAVS_QPEL_PROTO(OP, SIZE, mc10);     \
CAVS_QPEL_PROTO(OP, SIZE, mc20);     \
CAVS_QPEL_PROTO(OP, SIZE, mc30);     \
CAVS_QPEL_PROTO(OP, SIZE, mc01);     \
CAVS_QPEL_PROTO(OP, SIZE, mc02);     \
CAVS_QPEL_PROTO(OP, SIZE, mc03)

CAVS_QPEL_PROTOS(put, 8);
CAVS_QPEL_PROTOS(put, 16);
CAVS_QPEL_PROTOS(avg, 8);
CAVS_QPEL_PROTOS(avg, 16);

av_cold void ff_cavsdsp_init_aarch64(CAVSDSPContext *c)
{
    int cpu_flags = av_get_cpu_flags();

    if (have_neon(cpu_flags)) {
        c->cavs_idct8_add = ff_cavs_idct8_add_neon;
        c->idct_perm      = FF_IDCT_PERM_TRANSPOSE;
        c->cavs_filter_lv = ff_cavs_filter_lv_neon;
        c->cavs_filter_lh = ff_cavs_filter_lh_neon;
        c->cavs_filter_cv = ff_cavs_filter_cv_neon;
        c->cavs_filter_ch = ff_cavs_filter_ch_neon;

#define CAVS_QPEL_INIT(OP, IDX, SIZE)                                        \
        c->OP##_cavs_qpel_pixels_tab[IDX][ 0] = ff_##OP##_cavs_qpel##SIZE##_mc00_neon; \
        c->OP##_cavs_qpel_pixels_tab[IDX][ 1] = ff_##OP##_cavs_qpel##SIZE##_mc10_neon; \
        c->OP##_cavs_qpel_pixels_tab[IDX][ 2] = ff_##OP##_cavs_qpel##SIZE##_mc20_neon; \
        c->OP##_cavs_qpel_pixels_tab[IDX][ 3] = ff_##OP##_cavs_qpel##SIZE##_mc30_neon; \
        c->OP##_cavs_qpel_pixels_tab[IDX][ 4] = ff_##OP##_cavs_qpel##SIZE##_mc01_neon; \
        c->OP##_cavs_qpel_pixels_tab[IDX][ 8] = ff_##OP##_cavs_qpel##SIZE##_mc02_neon; \
        c->OP##_cavs_qpel_pixels_tab[IDX][12] = ff_##OP##_cavs_qpel##SIZE##_mc03_neon

        CAVS_QPEL_INIT(put, 0, 16);
        CAVS_QPEL_INIT(put, 1, 8);
        CAVS_QPEL_INIT(avg, 0, 16);
        CAVS_QPEL_INIT(avg, 1, 8);
    }
}
