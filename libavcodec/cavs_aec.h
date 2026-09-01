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
 * Specified by GY/T 257.1-2012 clause 8.4, implemented in the equivalent
 * Annex E formulation of GB/T 20090.16-2016; see cavs_aec.c. This header
 * exposes the engine only, the binarisations live in the caller.
 */

#ifndef AVCODEC_CAVS_AEC_H
#define AVCODEC_CAVS_AEC_H

#include <stdint.h>

#include "libavutil/avassert.h"
#include "libavutil/error.h"
#include "libavutil/macros.h"

#include "get_bits.h"

/** Number of context models: Table 51's largest start index is 322
    (weighting_prediction), a single context. */
#define CAVS_AEC_NUM_CTX 323

/**
 * Form a context index from a Table 51 start index, the element's model
 * count and a ctxIdxInc, which is ultimately bitstream driven; call this
 * where the increment is derived, the per-bin functions only assert.
 * @return the index, AVERROR_INVALIDDATA if @p inc is outside @p span, or
 *         AVERROR_BUG if the range is not inside the context array
 */
static inline int ff_cavs_aec_ctx_idx(int start, int span, int inc)
{
    if (span <= 0 || start < 0 || start > CAVS_AEC_NUM_CTX - span)
        return AVERROR_BUG;
    if ((unsigned)inc >= (unsigned)span)
        return AVERROR_INVALIDDATA;

    return start + inc;
}

/** One adaptive context model, GY/T 257.1-2012 8.4.2.1; all three members
    are reset at the start of every slice. */
typedef struct CAVSAECModel {
    uint8_t  mps;      ///< mps:    more probable symbol, 1 bit, init 0
    uint8_t  cycno;    ///< cycno:  adaptation counter, 2 bits, init 0
    uint16_t lg_pmps;  ///< lgPmps: log domain P(MPS), 11 bits, init 1023
} CAVSAECModel;

/** AEC engine state: the registers of 8.4.2.2 plus the b_flag of GB E.2;
    range (rs1, rt1) and offset (value_s, value_t) are each an exponent
    and an 8 bit mantissa. */
typedef struct CAVSAECContext {
    GetBitContext *gb; ///< bitstream reader, shared with the slice parser
    int rs1;           ///< range exponent,  init 0    (rS1)
    int rt1;           ///< range mantissa,  init 0xFF (rT1)
    int value_s;       ///< offset exponent            (valueS)
    int value_t;       ///< offset mantissa, 9 bits inside a call (valueT)
    int b_flag;        ///< offset window exhausted, GB/T 20090.16-2016 E.2
    CAVSAECModel ctx[CAVS_AEC_NUM_CTX]; ///< context models, 8.4.2.1
} CAVSAECContext;

/**
 * Reset all context models and start the arithmetic decoder (8.4.1).
 * @param gb reader, byte aligned after the slice header's
 *           aec_byte_alignment_bit padding; at least 9 bits are consumed
 */
void ff_cavs_aec_init(CAVSAECContext *aec, GetBitContext *gb);

/** 8.4.2.1: initial value of lgPmps, i.e. P(LPS) = 1/2. */
#define CAVS_AEC_LGPMPS_INIT 1023
/** 8.4.4.3.5: lgPmps saturation limit tested after the LPS increment. */
#define CAVS_AEC_LGPMPS_MAX 1023
/** 8.4.4.3.5: lgPmps is reflected through this value when the MPS flips. */
#define CAVS_AEC_LGPMPS_REFLECT 2047
/** 8.4.4.3.3: decode_bypass hard codes lgPmps to 1023. */
#define CAVS_AEC_LGPMPS_BYPASS 1023
/** 8.4.4.3.4: decode_aec_stuffing_bit hard codes lgPmps to 4. */
#define CAVS_AEC_LGPMPS_STUFF 4
/** 8.4.2.2: initial value of rT1. */
#define CAVS_AEC_RT1_INIT 0xFF
/** GB E.2: boundS, the cap on valueS that bounds both renormalisation loops. */
#define CAVS_AEC_BOUND_S 0xFE

/** Adapt one context model after a bin has been decoded, 8.4.4.3.5. */
static av_always_inline void cavs_aec_update_ctx(CAVSAECModel *m, int bin)
{
    /* cwr from cycno before the update; entries 3..5 of lps_inc_tab are the
     * standard's switch (cwr), the rest only pad it so cwr indexes it */
    static const uint8_t  cwr_tab[4]     = { 3, 3, 4, 5 };
    static const uint16_t lps_inc_tab[6] = { 0, 0, 0, 197, 95, 46 };
    const int cwr = cwr_tab[m->cycno];

    if (bin != m->mps)
        m->cycno = FFMIN(m->cycno + 1, 3);
    else if (m->cycno == 0)
        m->cycno = 1;

    if (bin == m->mps) {
        /* log domain decay; both shifts reach 0 for small lgPmps, so
         * lgPmps never falls below 7 */
        m->lg_pmps -= (m->lg_pmps >> cwr) + (m->lg_pmps >> (cwr + 2));
    } else {
        m->lg_pmps += lps_inc_tab[cwr];
        if (m->lg_pmps > CAVS_AEC_LGPMPS_MAX) {
            /* the MPS flips; 1023 + 197 reflects back inside [0,1023] */
            m->lg_pmps = CAVS_AEC_LGPMPS_REFLECT - m->lg_pmps;
            m->mps    ^= 1;
        }
    }
}

/** Decode one bin (GB E.3/E.4/E.5); the bodies of 8.4.4.3.2 to 8.4.4.3.4
    are identical, the callers supply predMps and lgPmps. */
static av_always_inline int cavs_aec_decode_bin(CAVSAECContext *aec,
                                                int pred_mps, int lg_pmps)
{
    GetBitContext *gb = aec->gb;
    /* LPS width in rT1 units; lgPmps >= 4, so never 0, bounding the loops */
    const int lps_range = lg_pmps >> 2;
    int rs2, rt2, s_flag, bin;

    av_assert2(lps_range > 0);

    s_flag = aec->rt1 < lps_range;
    rs2    = aec->rs1 + s_flag;
    rt2    = (s_flag << 8) + aec->rt1 - lps_range;

    /* with bFlag set (offset window exhausted, Annex E) the engine emits
     * MPS bins instead of consuming past the end of the buffer. The
     * two term compare of the standard collapses to one: with rt2 in
     * [0, 510] and valueT in [0, 255], rS2 > valueS or (equal and
     * valueT >= rT2) is exactly (rS2 << 9) - rT2 >= (valueS << 9) - valueT */
    if (!aec->b_flag &&
        (rs2 << 9) - rt2 >= (aec->value_s << 9) - aec->value_t) {
        int t_rlps = s_flag ? aec->rt1 + lps_range : lps_range;
        /* renormalisation shift; t_rlps is in [1,510], so n is in [0,8] */
        int n = t_rlps < 0x100 ? 8 - av_log2(t_rlps) : 0;

        bin = !pred_mps;
        if (rs2 == aec->value_s) {
            aec->value_t = aec->value_t - rt2;
            if (n)
                aec->value_t = (aec->value_t << n) | get_bits(gb, n);
        } else {
            /* the offset bit and the n renormalisation bits in one read */
            unsigned in  = get_bits(gb, n + 1);
            aec->value_t = ((256 + ((aec->value_t << 1) | (in >> n)) - rt2)
                            << n) | (in & ((1 << n) - 1));
        }
        aec->rt1 = (t_rlps << n) & 0xFF;
    } else {
        bin      = pred_mps;
        aec->rs1 = rs2;
        aec->rt1 = rt2;
    }

    /* renormalise the offset, skipping leading zeroes into value_s */
    if (bin != pred_mps || (aec->b_flag && rs2 == CAVS_AEC_BOUND_S)) {
        aec->rs1     = 0;
        aec->value_s = 0;
        if (aec->value_t > 0 && aec->value_t < 0x100) {
            /* the top set bit of valueT fixes the shift; the bits read
             * cannot terminate the loop earlier than it does */
            int n = 8 - av_log2(aec->value_t);

            aec->value_s = n;
            aec->value_t = (aec->value_t << n) | get_bits(gb, n);
        } else if (!aec->value_t) {
            /* rare: valueS counts the leading zeroes of the stream itself */
            while (aec->value_t < 0x100 && aec->value_s < CAVS_AEC_BOUND_S) {
                aec->value_s++;
                aec->value_t = (aec->value_t << 1) | get_bits1(gb);
            }
        }
        aec->b_flag   = aec->value_t < 0x100;
        aec->value_t &= 0xFF;
    }

    return bin;
}

/**
 * Decode one bin against a single context model and adapt it (8.4.4.3.2).
 * @param ctx_idx formed with ff_cavs_aec_ctx_idx() when bitstream driven
 */
static av_always_inline int ff_cavs_aec_decode_decision(CAVSAECContext *aec,
                                                        int ctx_idx)
{
    CAVSAECModel *m;
    int bin;

    /* bitstream derived indices have been checked by ff_cavs_aec_ctx_idx() */
    av_assert2(ctx_idx >= 0 && ctx_idx < CAVS_AEC_NUM_CTX);

    m   = &aec->ctx[ctx_idx];
    bin = cavs_aec_decode_bin(aec, m->mps, m->lg_pmps);
    cavs_aec_update_ctx(m, bin);

    return bin;
}

/** Decode one bin against two weighted context models and adapt both
    (8.4.4.3.2 with contextWeighting == 1, used by 8.4.4.2 j) only). */
static av_always_inline int ff_cavs_aec_decode_decision_w(CAVSAECContext *aec,
                                                          int ctx_idx,
                                                          int ctx_idx_w)
{
    CAVSAECModel *m1, *m2;
    int pred_mps, lg_pmps, bin;

    av_assert2(ctx_idx   >= 0 && ctx_idx   < CAVS_AEC_NUM_CTX);
    av_assert2(ctx_idx_w >= 0 && ctx_idx_w < CAVS_AEC_NUM_CTX);

    m1 = &aec->ctx[ctx_idx];
    m2 = &aec->ctx[ctx_idx_w];

    /* 8.4.4.3.2, contextWeighting == 1 */
    if (m1->mps == m2->mps) {
        pred_mps = m1->mps;
        lg_pmps  = (m1->lg_pmps + m2->lg_pmps) / 2;
    } else if (m1->lg_pmps < m2->lg_pmps) {
        pred_mps = m1->mps;
        lg_pmps  = 1023 - ((m2->lg_pmps - m1->lg_pmps) >> 1);
    } else {
        /* also taken when the two lgPmps are equal */
        pred_mps = m2->mps;
        lg_pmps  = 1023 - ((m1->lg_pmps - m2->lg_pmps) >> 1);
    }

    bin = cavs_aec_decode_bin(aec, pred_mps, lg_pmps);
    cavs_aec_update_ctx(m1, bin);
    cavs_aec_update_ctx(m2, bin);

    return bin;
}

/** Decode one bypass bin (8.4.4.3.3); unlike CABAC not a plain bit read,
    the engine range state is carried forward. */
static av_always_inline int ff_cavs_aec_decode_bypass(CAVSAECContext *aec)
{
    return cavs_aec_decode_bin(aec, 0, CAVS_AEC_LGPMPS_BYPASS);
}

/** Decode one aec_mb_stuffing_bit (8.4.4.3.4): 1 on the last macroblock
    of the slice (7.2.4), the counterpart of CABAC's end_of_slice_flag. */
static av_always_inline int ff_cavs_aec_decode_stuffing_bit(CAVSAECContext *aec)
{
    return cavs_aec_decode_bin(aec, 0, CAVS_AEC_LGPMPS_STUFF);
}

#endif /* AVCODEC_CAVS_AEC_H */
