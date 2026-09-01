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
 * Self test for the AVS+ AEC arithmetic decoding engine: hand traced
 * vectors from GY/T 257.1-2012 clause 8.4, agreement with an independent
 * transcription of the clause 8.4 main body (GB/T 20090.16-2016 E.1), and
 * termination on exhausted buffers.
 */

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "libavutil/mem.h"

#include "libavcodec/defs.h"

/* included whole to reach the static helpers */
#include "libavcodec/cavs_aec.c"

static int failures;

#define CHECK(expr, fmt, have, want) do {                                    \
    if (!(expr)) {                                                           \
        printf("FAIL %s:%d: %s: got " fmt ", expected " fmt "\n",            \
               __func__, __LINE__, #have, (have), (want));                   \
        failures++;                                                          \
    }                                                                        \
} while (0)

#define CHECK_INT(have, want) CHECK((have) == (want), "%d", have, want)

static void check_state(const CAVSAECContext *aec, int rs1, int rt1,
                        int value_s, int value_t, int b_flag, int bits)
{
    CHECK_INT(aec->rs1, rs1);
    CHECK_INT(aec->rt1, rt1);
    CHECK_INT(aec->value_s, value_s);
    CHECK_INT(aec->value_t, value_t);
    CHECK_INT(aec->b_flag, b_flag);
    CHECK_INT(get_bits_count(aec->gb), bits);
}

static void check_model(const CAVSAECModel *m, int mps, int cycno, int lg_pmps)
{
    CHECK_INT(m->mps, mps);
    CHECK_INT(m->cycno, cycno);
    CHECK_INT(m->lg_pmps, lg_pmps);
}

/*
 * ff_cavs_aec_ctx_idx(): the bounds check that stands between a ctxIdxInc
 * derived from bitstream data and the context array.
 */
static void test_ctx_idx(void)
{
    /* An increment inside the span of its syntax element is passed through. */
    CHECK_INT(ff_cavs_aec_ctx_idx(0, 4, 0), 0);
    CHECK_INT(ff_cavs_aec_ctx_idx(0, 4, 3), 3);
    CHECK_INT(ff_cavs_aec_ctx_idx(10, 4, 3), 13);

    /* weighting_prediction, the last entry of Table 51: start 322, span 1,
     * so ctxIdxInc 1 would be the first index outside the array. */
    CHECK_INT(ff_cavs_aec_ctx_idx(CAVS_AEC_NUM_CTX - 1, 1, 0),
              CAVS_AEC_NUM_CTX - 1);
    CHECK_INT(ff_cavs_aec_ctx_idx(CAVS_AEC_NUM_CTX - 1, 1, 1),
              AVERROR_INVALIDDATA);

    /* An increment past the span is rejected rather than borrowing the models
     * of the next syntax element, and a negative one is rejected rather than
     * being read as a large unsigned value. */
    CHECK_INT(ff_cavs_aec_ctx_idx(10, 4, 4), AVERROR_INVALIDDATA);
    CHECK_INT(ff_cavs_aec_ctx_idx(10, 4, -1), AVERROR_INVALIDDATA);
    CHECK_INT(ff_cavs_aec_ctx_idx(10, 4, INT_MIN), AVERROR_INVALIDDATA);
    CHECK_INT(ff_cavs_aec_ctx_idx(10, 4, INT_MAX), AVERROR_INVALIDDATA);

    /* A start index and span that do not describe a range inside the array
     * are a mistake in the caller's Table 51 data, not in the bitstream, and
     * are caught even when the increment itself is inside the span. */
    CHECK_INT(ff_cavs_aec_ctx_idx(CAVS_AEC_NUM_CTX - 1, 2, 0), AVERROR_BUG);
    CHECK_INT(ff_cavs_aec_ctx_idx(CAVS_AEC_NUM_CTX, 1, 0), AVERROR_BUG);
    CHECK_INT(ff_cavs_aec_ctx_idx(-1, 2, 0), AVERROR_BUG);
    CHECK_INT(ff_cavs_aec_ctx_idx(0, 0, 0), AVERROR_BUG);
    CHECK_INT(ff_cavs_aec_ctx_idx(0, -1, 0), AVERROR_BUG);
    CHECK_INT(ff_cavs_aec_ctx_idx(0, INT_MAX, 0), AVERROR_BUG);
}

/* 8.4.4.3.5 traced by hand from a freshly initialised context model. */
static void test_update_ctx(void)
{
    CAVSAECModel m = { 0, 0, 1023 };

    /* cwr = 3; cycno 0 -> 1; 1023 - (1023 >> 3) - (1023 >> 5) = 865. */
    cavs_aec_update_ctx(&m, 0);
    check_model(&m, 0, 1, 865);

    /* cwr = 3; cycno 1 -> 2; 865 + 197 = 1062 > 1023, so the MPS flips and
     * lgPmps becomes 2047 - 1062 = 985. */
    cavs_aec_update_ctx(&m, 1);
    check_model(&m, 1, 2, 985);

    /* cwr is now 4 because cycno is 2; 985 - (985 >> 4) - (985 >> 6) = 909. */
    cavs_aec_update_ctx(&m, 1);
    check_model(&m, 1, 2, 909);

    /* cwr = 4; cycno 2 -> 3; 909 + 95 = 1004, no flip. */
    cavs_aec_update_ctx(&m, 0);
    check_model(&m, 1, 3, 1004);

    /* cwr is now 5 and cycno saturates at 3; 1004 + 46 = 1050 > 1023, so the
     * MPS flips again and lgPmps becomes 2047 - 1050 = 997. */
    cavs_aec_update_ctx(&m, 0);
    check_model(&m, 0, 3, 997);
}

/*
 * 8.4.2.2 / GB E.2 traced by hand.
 *
 * The buffer starts 0x0A 0xB3, so the bits are
 *   0 0 0 0 1 0 1 0 | 1 0 1 1 0 0 1 1
 * read_bits(9) yields 000010101 = 21, whose bit 8 is 0, so the loop shifts in
 * b9 = 0, b10 = 1, b11 = 1, b12 = 0 giving 42, 85, 171, 342.  342 has bit 8
 * set, so valueS = 4, bFlag = 0, valueT = 342 & 0xFF = 86, 13 bits consumed.
 */
static void test_init(void)
{
    static const uint8_t buf[8] = { 0x0A, 0xB3, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00 };
    CAVSAECContext aec;
    GetBitContext gb;

    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);
    check_state(&aec, 0, 0xFF, 4, 86, 0, 13);
    check_model(&aec.ctx[0], 0, 0, 1023);
    check_model(&aec.ctx[CAVS_AEC_NUM_CTX - 1], 0, 0, 1023);
}

/*
 * 8.4.4.3.2 and 8.4.4.3.3 traced by hand over 0xB6 0x5D 0x2A, i.e. the bits
 *   1 0 1 1 0 1 1 0 | 0 1 0 1 1 1 0 1 | 0 0 1 0 1 0 1 0
 * Init: read_bits(9) = 101101100 = 364, bit 8 already set, so valueS = 0,
 * valueT = 108, 9 bits consumed.
 */
static void test_decision(void)
{
    static const uint8_t buf[8] = { 0xB6, 0x5D, 0x2A, 0x00,
                                    0x00, 0x00, 0x00, 0x00 };
    CAVSAECContext aec;
    GetBitContext gb;

    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);
    check_state(&aec, 0, 0xFF, 0, 108, 0, 9);

    /* lgPmps 1023 -> lps_range 255; rT1 255 >= 255 so rS2 = 0, rT2 = 0,
     * sFlag = 0.  rS2 == valueS and valueT 108 >= 0, so this is an LPS.
     * tRlps = 255 -> 510 after one shift, valueT = (108 << 1) | b9(1) = 217,
     * rT1 = 510 & 0xFF = 254.  Offset renormalisation then reads b10 = 0,
     * giving valueS = 1 and valueT = 434 & 0xFF = 178, 11 bits consumed.
     * update_ctx: 1023 + 197 = 1220 > 1023, so lgPmps = 827 and mps = 1. */
    CHECK_INT(ff_cavs_aec_decode_decision(&aec, 0), 1);
    check_state(&aec, 0, 254, 1, 178, 0, 11);
    check_model(&aec.ctx[0], 1, 1, 827);

    /* lps_range = 827 >> 2 = 206; rT1 254 >= 206 so rS2 = 0, rT2 = 48.
     * rS2 < valueS, so this is an MPS and no bits are read.
     * update_ctx: 827 - 103 - 25 = 699. */
    CHECK_INT(ff_cavs_aec_decode_decision(&aec, 0), 1);
    check_state(&aec, 0, 48, 1, 178, 0, 11);
    check_model(&aec.ctx[0], 1, 1, 699);

    /* lps_range = 174; rT1 48 < 174 so rS2 = 1, rT2 = 256 + 48 - 174 = 130,
     * sFlag = 1.  rS2 == valueS and valueT 178 >= 130, so LPS with
     * tRlps = 48 + 174 = 222 -> 444, valueT = (48 << 1) | b11(1) = 97,
     * rT1 = 444 & 0xFF = 188.  Offset renormalisation reads b12 = 1 and
     * b13 = 1, giving valueS = 2 and valueT = 391 & 0xFF = 135.
     * update_ctx: 699 + 197 = 896, cycno 1 -> 2, no flip. */
    CHECK_INT(ff_cavs_aec_decode_decision(&aec, 0), 0);
    check_state(&aec, 0, 188, 2, 135, 0, 14);
    check_model(&aec.ctx[0], 1, 2, 896);

    /* Three bypass bins, 8.4.4.3.3.  lps_range is always 255, so rT1 < 255
     * forces sFlag = 1 and rS2 = rS1 + 1 every time.  The first two are MPS
     * (rS2 = 1 then 2, both <= valueS = 2, and 135 < 190).  The third has
     * rS2 = 3 > valueS, the branch that reads a bit before subtracting:
     * valueT = 256 + ((135 << 1) | b14(0)) - 191 = 335, tRlps = 190 + 255 =
     * 445 needs no shift, rT1 = 445 & 0xFF = 189, and the offset
     * renormalisation loop does not run because 335 >= 0x100. */
    CHECK_INT(ff_cavs_aec_decode_bypass(&aec), 0);
    check_state(&aec, 1, 189, 2, 135, 0, 14);
    CHECK_INT(ff_cavs_aec_decode_bypass(&aec), 0);
    check_state(&aec, 2, 190, 2, 135, 0, 14);
    CHECK_INT(ff_cavs_aec_decode_bypass(&aec), 1);
    check_state(&aec, 0, 189, 0, 79, 0, 15);

    /* 8.4.4.3.4 with lgPmps = 4: lps_range = 1, rT1 189 >= 1 so rS2 = 0 and
     * rT2 = 188.  valueT 79 < 188, so the stuffing bit is 0 and no bit is
     * read.  This is the common case, one non-terminating macroblock. */
    CHECK_INT(ff_cavs_aec_decode_stuffing_bit(&aec), 0);
    check_state(&aec, 0, 188, 0, 79, 0, 15);
}

/* weighting_prediction is the sole user of Table 51 context 322. */
static void test_weighting_prediction_context(void)
{
    static const uint8_t buf[8] = { 0xB6, 0x5D, 0x2A, 0x00,
                                    0x00, 0x00, 0x00, 0x00 };
    CAVSAECContext aec;
    GetBitContext gb;

    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);

    CHECK_INT(ff_cavs_aec_decode_decision(&aec,
                                          CAVS_AEC_NUM_CTX - 1), 1);
    check_state(&aec, 0, 254, 1, 178, 0, 11);
    check_model(&aec.ctx[CAVS_AEC_NUM_CTX - 1], 1, 1, 827);
    /* Decoding the weighting bin must not adapt a neighbouring model. */
    check_model(&aec.ctx[CAVS_AEC_NUM_CTX - 2], 0, 0, 1023);
}

/*
 * The rare branch of 8.4.4.3.4, plus the maximum length range
 * renormalisation.  Over 0xFF 0xA5 0x3C the first nine bits are all ones, so
 * valueS = 0 and valueT = 0xFF.  With lgPmps = 4 the MPS subinterval base is
 * rT2 = 254 and valueT 255 >= 254 selects the LPS, i.e. the last macroblock
 * of a slice.  tRlps = 1 then needs the full eight shifts to reach 0x100,
 * reading b9..b16 = 0,1,0,0,1,0,1,0 and giving valueT = 330, rT1 = 0.
 */
static void test_stuffing_bit(void)
{
    static const uint8_t buf[8] = { 0xFF, 0xA5, 0x3C, 0x00,
                                    0x00, 0x00, 0x00, 0x00 };
    CAVSAECContext aec;
    GetBitContext gb;

    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);
    check_state(&aec, 0, 0xFF, 0, 255, 0, 9);

    CHECK_INT(ff_cavs_aec_decode_stuffing_bit(&aec), 1);
    check_state(&aec, 0, 0, 0, 74, 0, 17);
}

/* 8.4.4.3.2 with contextWeighting == 1, both branches, traced by hand from
 * the same initial state as test_stuffing_bit(). */
static void test_decision_weighted(void)
{
    static const uint8_t buf[8] = { 0xFF, 0xA5, 0x3C, 0x00,
                                    0x00, 0x00, 0x00, 0x00 };
    CAVSAECContext aec;
    GetBitContext gb;

    /* Equal MPS: predMps = 0 and lgPmps = (800 + 600) / 2 = 700, so
     * lps_range = 175, rT2 = 255 - 175 = 80 and valueT 255 >= 80 gives an
     * LPS.  tRlps = 175 -> 350 reading b9 = 0, rT1 = 350 & 0xFF = 94, and the
     * offset renormalisation does not loop.  Both models are then adapted
     * with bin 1, which is an LPS for both. */
    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);
    aec.ctx[10].mps = 0; aec.ctx[10].cycno = 0; aec.ctx[10].lg_pmps = 800;
    aec.ctx[20].mps = 0; aec.ctx[20].cycno = 0; aec.ctx[20].lg_pmps = 600;
    CHECK_INT(ff_cavs_aec_decode_decision_w(&aec, 10, 20), 1);
    check_state(&aec, 0, 94, 0, 94, 0, 10);
    check_model(&aec.ctx[10], 0, 1, 997);
    check_model(&aec.ctx[20], 0, 1, 797);

    /* Different MPS with ctx1->lgPmps < ctx2->lgPmps: predMps = ctx1->mps = 0
     * and lgPmps = 1023 - ((800 - 600) >> 1) = 923, so lps_range = 230,
     * rT2 = 25 and valueT 255 >= 25 gives an LPS.  tRlps = 230 -> 460 reading
     * b9 = 0, rT1 = 460 & 0xFF = 204.  Bin 1 is an LPS for ctx1 and an MPS
     * for ctx2. */
    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);
    aec.ctx[10].mps = 0; aec.ctx[10].cycno = 0; aec.ctx[10].lg_pmps = 600;
    aec.ctx[20].mps = 1; aec.ctx[20].cycno = 0; aec.ctx[20].lg_pmps = 800;
    CHECK_INT(ff_cavs_aec_decode_decision_w(&aec, 10, 20), 1);
    check_state(&aec, 0, 204, 0, 204, 0, 10);
    check_model(&aec.ctx[10], 0, 1, 797);
    check_model(&aec.ctx[20], 1, 1, 675);

    /* Different MPS and equal lgPmps: the else branch is taken, so predMps is
     * ctx2->mps = 1 and lgPmps = 1023.  lps_range = 255 and rT1 = 255 gives
     * rT2 = 0, so valueT 255 >= 0 is an LPS and the bin is 0. */
    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);
    aec.ctx[10].mps = 0; aec.ctx[10].cycno = 0; aec.ctx[10].lg_pmps = 700;
    aec.ctx[20].mps = 1; aec.ctx[20].cycno = 0; aec.ctx[20].lg_pmps = 700;
    CHECK_INT(ff_cavs_aec_decode_decision_w(&aec, 10, 20), 0);
}

/*
 * GB E.3 to E.5 gate the LPS branch on bFlag == 0, and the trailing block on
 * "bFlag == 1 && rS2 == boundS".  Both are only reachable once the offset
 * window has been padded out to boundS with a non-zero tail, which takes a
 * specific truncated payload to provoke, so the state is set up directly.
 */
static void test_b_flag_gates_lps(void)
{
    static const uint8_t buf[8] = { 0xFF, 0xA5, 0x3C, 0x00,
                                    0x00, 0x00, 0x00, 0x00 };
    CAVSAECContext aec;
    GetBitContext gb;

    init_get_bits8(&gb, buf, sizeof(buf));
    ff_cavs_aec_init(&aec, &gb);

    /* rS1 = 253 and rT1 = 0 give rS2 = 254 == valueS and rT2 = 1, so the LPS
     * test of E.4 is true on its own and only bFlag suppresses it. */
    aec.rs1     = 253;
    aec.rt1     = 0;
    aec.value_s = CAVS_AEC_BOUND_S;
    aec.value_t = 255;
    aec.b_flag  = 1;

    CHECK_INT(ff_cavs_aec_decode_bypass(&aec), 0);
    /* The MPS path leaves rS1 = 254 = boundS, so the trailing block runs even
     * though the bin matched predMps: valueT 255 -> 510 reading b9 = 0, which
     * clears bFlag again and leaves valueS = 1 and valueT = 510 & 0xFF. */
    check_state(&aec, 0, 1, 1, 254, 0, 10);
}

/*
 * An independent, literal transcription of the clause 8.4 main body, used
 * only to cross check the Annex E implementation above.  Deliberately written
 * out in full rather than factored, so that it can be read against 8.4.2.2,
 * 8.4.4.3.2, 8.4.4.3.3 and 8.4.4.3.4 line by line.  It has no bFlag and no
 * boundS and would not terminate at the end of the buffer, so it must only be
 * driven over data it cannot exhaust.
 */
typedef struct RefAEC {
    GetBitContext *gb;
    int rS1, rT1, valueS, valueT;
    CAVSAECModel ctx[CAVS_AEC_NUM_CTX];
} RefAEC;

static void ref_init(RefAEC *a, GetBitContext *gb)
{
    int i;

    for (i = 0; i < CAVS_AEC_NUM_CTX; i++) {
        a->ctx[i].mps     = 0;
        a->ctx[i].cycno   = 0;
        a->ctx[i].lg_pmps = 1023;
    }
    a->gb     = gb;
    a->rS1    = 0;
    a->rT1    = 0xFF;
    a->valueS = 0;
    a->valueT = get_bits(gb, 9);
    while (!((a->valueT >> 8) & 0x01)) {
        a->valueT = (a->valueT << 1) | get_bits1(gb);
        a->valueS++;
    }
    a->valueT = a->valueT & 0xFF;
}

static int ref_core(RefAEC *a, int predMps, int lgPmps)
{
    int rS2, rT2, sFlag, tRlps, binVal;

    if (a->rT1 >= (lgPmps >> 2)) {
        rS2   = a->rS1;
        rT2   = a->rT1 - (lgPmps >> 2);
        sFlag = 0;
    } else {
        rS2   = a->rS1 + 1;
        rT2   = 256 + a->rT1 - (lgPmps >> 2);
        sFlag = 1;
    }
    if (rS2 > a->valueS || (rS2 == a->valueS && a->valueT >= rT2)) {
        binVal = !predMps;
        if (sFlag == 0)
            tRlps = lgPmps >> 2;
        else
            tRlps = a->rT1 + (lgPmps >> 2);
        if (rS2 == a->valueS)
            a->valueT = a->valueT - rT2;
        else
            a->valueT = 256 + ((a->valueT << 1) | get_bits1(a->gb)) - rT2;
        while (tRlps < 0x100) {
            tRlps     = tRlps << 1;
            a->valueT = (a->valueT << 1) | get_bits1(a->gb);
        }
        a->rS1    = 0;
        a->rT1    = tRlps & 0xFF;
        a->valueS = 0;
        while (a->valueT < 0x100) {
            a->valueS++;
            a->valueT = (a->valueT << 1) | get_bits1(a->gb);
        }
        a->valueT = a->valueT & 0xFF;
    } else {
        binVal = predMps;
        a->rS1 = rS2;
        a->rT1 = rT2;
    }
    return binVal;
}

static int ref_decision(RefAEC *a, int ctx_idx)
{
    int binVal = ref_core(a, a->ctx[ctx_idx].mps, a->ctx[ctx_idx].lg_pmps);
    cavs_aec_update_ctx(&a->ctx[ctx_idx], binVal);
    return binVal;
}

static int ref_decision_w(RefAEC *a, int i1, int i2)
{
    CAVSAECModel *ctx1 = &a->ctx[i1], *ctx2 = &a->ctx[i2];
    int predMps, lgPmps, binVal;

    if (ctx1->mps == ctx2->mps) {
        predMps = ctx1->mps;
        lgPmps  = (ctx1->lg_pmps + ctx2->lg_pmps) / 2;
    } else if (ctx1->lg_pmps < ctx2->lg_pmps) {
        predMps = ctx1->mps;
        lgPmps  = 1023 - ((ctx2->lg_pmps - ctx1->lg_pmps) >> 1);
    } else {
        predMps = ctx2->mps;
        lgPmps  = 1023 - ((ctx1->lg_pmps - ctx2->lg_pmps) >> 1);
    }
    binVal = ref_core(a, predMps, lgPmps);
    cavs_aec_update_ctx(ctx1, binVal);
    cavs_aec_update_ctx(ctx2, binVal);
    return binVal;
}

#define XCHECK_BUF_SIZE 65536
#define XCHECK_OPS      20000

/* Both engines over the same pseudo-random payload must agree bin for bin,
 * bit position for bit position and context model for context model. */
static void test_cross_check(void)
{
    uint8_t *buf = av_mallocz(XCHECK_BUF_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
    GetBitContext gb_a, gb_b;
    CAVSAECContext aec;
    RefAEC ref;
    uint32_t rnd = 0x2C1A57F3;
    int i, mismatch = 0;

    if (!buf) {
        printf("FAIL %s: out of memory\n", __func__);
        failures++;
        return;
    }
    for (i = 0; i < XCHECK_BUF_SIZE; i++) {
        rnd    = rnd * 1103515245 + 12345;
        buf[i] = rnd >> 19;
    }

    init_get_bits8(&gb_a, buf, XCHECK_BUF_SIZE);
    init_get_bits8(&gb_b, buf, XCHECK_BUF_SIZE);
    ff_cavs_aec_init(&aec, &gb_a);
    ref_init(&ref, &gb_b);

    for (i = 0; i < XCHECK_OPS; i++) {
        int bin_a, bin_b;

        rnd = rnd * 1103515245 + 12345;
        switch ((rnd >> 21) & 3) {
        case 0:
            bin_a = ff_cavs_aec_decode_bypass(&aec);
            bin_b = ref_core(&ref, 0, 1023);
            break;
        case 1:
            bin_a = ff_cavs_aec_decode_stuffing_bit(&aec);
            bin_b = ref_core(&ref, 0, 4);
            break;
        case 2:
            bin_a = ff_cavs_aec_decode_decision_w(&aec, (rnd >> 5) % 323,
                                                  (rnd >> 13) % 323);
            bin_b = ref_decision_w(&ref, (rnd >> 5) % 323, (rnd >> 13) % 323);
            break;
        default:
            bin_a = ff_cavs_aec_decode_decision(&aec, (rnd >> 5) % 323);
            bin_b = ref_decision(&ref, (rnd >> 5) % 323);
            break;
        }
        /* The main body transcription has no boundS, so stop well before the
         * buffer runs out rather than letting it spin. */
        if (get_bits_count(&gb_b) > XCHECK_BUF_SIZE * 8 - 4096)
            break;
        if (bin_a != bin_b || aec.rs1 != ref.rS1 || aec.rt1 != ref.rT1 ||
            aec.value_s != ref.valueS || aec.value_t != ref.valueT ||
            get_bits_count(&gb_a) != get_bits_count(&gb_b) || aec.b_flag) {
            printf("FAIL %s: diverged at op %d\n", __func__, i);
            failures++;
            mismatch = 1;
            break;
        }
    }
    if (!mismatch && memcmp(aec.ctx, ref.ctx, sizeof(aec.ctx))) {
        printf("FAIL %s: context models diverged\n", __func__);
        failures++;
    }
    /* Sanity: the run must have gone somewhere without exhausting the buffer,
     * which the literal main body transcription above could not survive. */
    if (get_bits_count(&gb_a) < XCHECK_OPS / 4 ||
        get_bits_count(&gb_a) > XCHECK_BUF_SIZE * 8 - 512) {
        printf("FAIL %s: consumed %d bits of %d\n", __func__,
               get_bits_count(&gb_a), XCHECK_BUF_SIZE * 8);
        failures++;
    }

    av_free(buf);
}

#define TRUNC_BUF_SIZE  (1 << 17)
#define TRUNC_OPS       2000

/*
 * GB E.2 to E.5 on input the engine can exhaust.  With an all zero payload
 * the offset window never finds its leading one, so bFlag is set at
 * initialisation and every renormalisation loop has to be stopped by boundS.
 * The engine must still return, must keep rS1 bounded, and must emit nothing
 * but MPS bins because GB E.3 to E.5 disable the LPS branch while bFlag is 1.
 */
static void test_bounded_on_exhausted_input(void)
{
    uint8_t *buf = av_mallocz(TRUNC_BUF_SIZE + AV_INPUT_BUFFER_PADDING_SIZE);
    CAVSAECContext aec;
    GetBitContext gb;
    int i;

    if (!buf) {
        printf("FAIL %s: out of memory\n", __func__);
        failures++;
        return;
    }

    init_get_bits8(&gb, buf, TRUNC_BUF_SIZE);
    ff_cavs_aec_init(&aec, &gb);
    check_state(&aec, 0, 0xFF, CAVS_AEC_BOUND_S, 0, 1, 9 + CAVS_AEC_BOUND_S);

    for (i = 0; i < TRUNC_OPS; i++) {
        int bin;

        switch (i & 3) {
        case 0:  bin = ff_cavs_aec_decode_bypass(&aec);            break;
        case 1:  bin = ff_cavs_aec_decode_stuffing_bit(&aec);      break;
        case 2:  bin = ff_cavs_aec_decode_decision_w(&aec, 7, 11); break;
        default: bin = ff_cavs_aec_decode_decision(&aec, i % 323); break;
        }
        /* predMps is 0 for bypass and for the stuffing bit, and the two
         * context models below are only ever fed zeroes, so their MPS stays
         * 0 as well: with the LPS branch disabled every bin must be 0. */
        if (bin != 0) {
            printf("FAIL %s: LPS decoded with bFlag set at op %d\n",
                   __func__, i);
            failures++;
            break;
        }
        if (aec.rs1 < 0 || aec.rs1 > CAVS_AEC_BOUND_S ||
            aec.value_s < 0 || aec.value_s > CAVS_AEC_BOUND_S ||
            aec.rt1 < 0 || aec.rt1 > 0xFF ||
            aec.value_t < 0 || aec.value_t > 0xFF) {
            printf("FAIL %s: state escaped its bounds at op %d\n", __func__, i);
            failures++;
            break;
        }
    }
    CHECK_INT(aec.b_flag, 1);

    av_free(buf);
}

int main(void)
{
    test_ctx_idx();
    test_update_ctx();
    test_init();
    test_decision();
    test_weighting_prediction_context();
    test_stuffing_bit();
    test_decision_weighted();
    test_b_flag_gates_lps();
    test_cross_check();
    test_bounded_on_exhausted_input();

    if (failures)
        printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");

    return !!failures;
}
