/*
 * Chinese AVS video (AVS1-P16, GuangDian profile) AEC arithmetic decoder
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

/**
 * @file
 * Advanced entropy coding (AEC) arithmetic decoding engine used by the
 * GuangDian profile of Chinese AVS video (AVS1-P16, "AVS+").
 *
 * Clause numbers without a prefix refer to GY/T 257.1-2012; "GB" prefixed
 * ones to GB/T 20090.16-2016, whose clause 8.4 is word for word the same.
 *
 * This implements the equivalent Annex E formulation of GB/T 20090.16-2016
 * (E.1 requires it whole): unlike the main body's renormalisation loops it
 * bounds the bits read on truncated input, via bFlag and boundS. On a well
 * formed bitstream bFlag stays 0 and the two formulations decode the same
 * bins. The caller must have removed the annex A escapes from the payload.
 */

#include <stdint.h>

#include "libavutil/avassert.h"
#include "libavutil/macros.h"

#include "cavs_aec.h"
#include "get_bits.h"

void ff_cavs_aec_init(CAVSAECContext *aec, GetBitContext *gb)
{
    int i;

    /* 8.4.2.1: every context model starts from the same triple */
    for (i = 0; i < CAVS_AEC_NUM_CTX; i++) {
        aec->ctx[i].mps     = 0;
        aec->ctx[i].cycno   = 0;
        aec->ctx[i].lg_pmps = CAVS_AEC_LGPMPS_INIT;
    }

    /* 8.4.2.2 / GB E.2 */
    aec->gb      = gb;
    aec->rs1     = 0;
    aec->rt1     = CAVS_AEC_RT1_INIT;
    aec->value_s = 0;
    aec->value_t = get_bits(gb, 9);
    while (!((aec->value_t >> 8) & 0x01) && aec->value_s < CAVS_AEC_BOUND_S) {
        aec->value_t = (aec->value_t << 1) | get_bits1(gb);
        aec->value_s++;
    }
    aec->b_flag   = aec->value_t < 0x100;
    aec->value_t &= 0xFF;
}
