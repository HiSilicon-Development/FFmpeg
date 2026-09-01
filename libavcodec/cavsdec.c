/*
 * Chinese AVS video (AVS1-P2, JiZhun profile) decoder.
 * Copyright (c) 2006  Stefan Gehrer <stefan.gehrer@gmx.de>
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
 * Chinese AVS video (AVS1-P2, JiZhun profile) decoder
 * @author Stefan Gehrer <stefan.gehrer@gmx.de>
 */

#include "libavutil/attributes.h"
#include "config_components.h"
#include "libavutil/avassert.h"
#include "libavutil/emms.h"
#include "libavutil/intmath.h"
#include "libavutil/mem.h"
#include "avcodec.h"
#include "cavs.h"
#include "codec_internal.h"
#include "profiles.h"
#include "decode.h"
#include "get_bits.h"
#include "golomb.h"
#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "mathops.h"
#include "mpeg12data.h"
#include "put_bits.h"
#include "libavutil/refstruct.h"
#include "startcode.h"

static const uint8_t mv_scan[4] = {
    MV_FWD_X0, MV_FWD_X1,
    MV_FWD_X2, MV_FWD_X3
};

static const uint8_t cbp_tab[64][2] = {
  { 63,  0 }, { 15, 15 }, { 31, 63 }, { 47, 31 }, {  0, 16 }, { 14, 32 }, { 13, 47 }, { 11, 13 },
  {  7, 14 }, {  5, 11 }, { 10, 12 }, {  8,  5 }, { 12, 10 }, { 61,  7 }, {  4, 48 }, { 55,  3 },
  {  1,  2 }, {  2,  8 }, { 59,  4 }, {  3,  1 }, { 62, 61 }, {  9, 55 }, {  6, 59 }, { 29, 62 },
  { 45, 29 }, { 51, 27 }, { 23, 23 }, { 39, 19 }, { 27, 30 }, { 46, 28 }, { 53,  9 }, { 30,  6 },
  { 43, 60 }, { 37, 21 }, { 60, 44 }, { 16, 26 }, { 21, 51 }, { 28, 35 }, { 19, 18 }, { 35, 20 },
  { 42, 24 }, { 26, 53 }, { 44, 17 }, { 32, 37 }, { 58, 39 }, { 24, 45 }, { 20, 58 }, { 17, 43 },
  { 18, 42 }, { 48, 46 }, { 22, 36 }, { 33, 33 }, { 25, 34 }, { 49, 40 }, { 40, 52 }, { 36, 49 },
  { 34, 50 }, { 50, 56 }, { 52, 25 }, { 54, 22 }, { 41, 54 }, { 56, 57 }, { 38, 41 }, { 57, 38 }
};

static const uint8_t scan3x3[4] = { 4, 5, 7, 8 };

static const uint8_t dequant_shift[64] = {
  14, 14, 14, 14, 14, 14, 14, 14,
  13, 13, 13, 13, 13, 13, 13, 13,
  13, 12, 12, 12, 12, 12, 12, 12,
  11, 11, 11, 11, 11, 11, 11, 11,
  11, 10, 10, 10, 10, 10, 10, 10,
  10,  9,  9,  9,  9,  9,  9,  9,
  9,   8,  8,  8,  8,  8,  8,  8,
  7,   7,  7,  7,  7,  7,  7,  7
};

static const uint16_t dequant_mul[64] = {
  32768, 36061, 38968, 42495, 46341, 50535, 55437, 60424,
  32932, 35734, 38968, 42495, 46177, 50535, 55109, 59933,
  65535, 35734, 38968, 42577, 46341, 50617, 55027, 60097,
  32809, 35734, 38968, 42454, 46382, 50576, 55109, 60056,
  65535, 35734, 38968, 42495, 46320, 50515, 55109, 60076,
  65535, 35744, 38968, 42495, 46341, 50535, 55099, 60087,
  65535, 35734, 38973, 42500, 46341, 50535, 55109, 60097,
  32771, 35734, 38965, 42497, 46341, 50535, 55109, 60099
};

static const uint8_t weighting_quant_model[4][64] = {
    {
        0, 0, 0, 4, 4, 4, 5, 5, 0, 0, 3, 3, 3, 3, 5, 5,
        0, 3, 2, 2, 1, 1, 5, 5, 4, 3, 2, 2, 1, 5, 5, 5,
        4, 3, 1, 1, 5, 5, 5, 5, 4, 3, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    }, {
        0, 0, 0, 4, 4, 4, 5, 5, 0, 0, 4, 4, 4, 4, 5, 5,
        0, 3, 2, 2, 2, 1, 5, 5, 3, 3, 2, 2, 1, 5, 5, 5,
        3, 3, 2, 1, 5, 5, 5, 5, 3, 3, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    }, {
        0, 0, 0, 4, 4, 3, 5, 5, 0, 0, 4, 4, 3, 2, 5, 5,
        0, 4, 4, 3, 2, 1, 5, 5, 4, 4, 3, 2, 1, 5, 5, 5,
        4, 3, 2, 1, 5, 5, 5, 5, 3, 2, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    }, {
        0, 0, 0, 3, 2, 1, 5, 5, 0, 0, 4, 3, 2, 1, 5, 5,
        0, 4, 4, 3, 2, 1, 5, 5, 3, 3, 3, 3, 2, 5, 5, 5,
        2, 2, 2, 2, 5, 5, 5, 5, 1, 1, 1, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    },
};

static const int weighting_quant_default[2][6] = {
    { 135, 143, 143, 160, 160, 213 },
    { 128,  98, 106, 116, 116, 128 },
};

#define EOB 0, 0, 0

static const struct dec_2dvlc intra_dec[7] = {
    {
        { //level / run / table_inc
            {  1,  1,  1 }, { -1,  1,  1 }, {  1,  2,  1 }, { -1,  2,  1 }, {  1,  3,  1 }, { -1,  3, 1 },
            {  1,  4,  1 }, { -1,  4,  1 }, {  1,  5,  1 }, { -1,  5,  1 }, {  1,  6,  1 }, { -1,  6, 1 },
            {  1,  7,  1 }, { -1,  7,  1 }, {  1,  8,  1 }, { -1,  8,  1 }, {  1,  9,  1 }, { -1,  9, 1 },
            {  1, 10,  1 }, { -1, 10,  1 }, {  1, 11,  1 }, { -1, 11,  1 }, {  2,  1,  2 }, { -2,  1, 2 },
            {  1, 12,  1 }, { -1, 12,  1 }, {  1, 13,  1 }, { -1, 13,  1 }, {  1, 14,  1 }, { -1, 14, 1 },
            {  1, 15,  1 }, { -1, 15,  1 }, {  2,  2,  2 }, { -2,  2,  2 }, {  1, 16,  1 }, { -1, 16, 1 },
            {  1, 17,  1 }, { -1, 17,  1 }, {  3,  1,  3 }, { -3,  1,  3 }, {  1, 18,  1 }, { -1, 18, 1 },
            {  1, 19,  1 }, { -1, 19,  1 }, {  2,  3,  2 }, { -2,  3,  2 }, {  1, 20,  1 }, { -1, 20, 1 },
            {  1, 21,  1 }, { -1, 21,  1 }, {  2,  4,  2 }, { -2,  4,  2 }, {  1, 22,  1 }, { -1, 22, 1 },
            {  2,  5,  2 }, { -2,  5,  2 }, {  1, 23,  1 }, { -1, 23,  1 }, {   EOB    }
        },
        //level_add
        { 0, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1 },
        2, //golomb_order
        0, //inc_limit
        23, //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, {  2,  1,  1 }, { -2,  1,  1 },
            {  1,  3,  0 }, { -1,  3,  0 }, {     EOB    }, {  1,  4,  0 }, { -1,  4,  0 }, {  1,  5,  0 },
            { -1,  5,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, {  3,  1,  2 }, { -3,  1,  2 }, {  2,  2,  1 },
            { -2,  2,  1 }, {  1,  7,  0 }, { -1,  7,  0 }, {  1,  8,  0 }, { -1,  8,  0 }, {  1,  9,  0 },
            { -1,  9,  0 }, {  2,  3,  1 }, { -2,  3,  1 }, {  4,  1,  2 }, { -4,  1,  2 }, {  1, 10,  0 },
            { -1, 10,  0 }, {  1, 11,  0 }, { -1, 11,  0 }, {  2,  4,  1 }, { -2,  4,  1 }, {  3,  2,  2 },
            { -3,  2,  2 }, {  1, 12,  0 }, { -1, 12,  0 }, {  2,  5,  1 }, { -2,  5,  1 }, {  5,  1,  3 },
            { -5,  1,  3 }, {  1, 13,  0 }, { -1, 13,  0 }, {  2,  6,  1 }, { -2,  6,  1 }, {  1, 14,  0 },
            { -1, 14,  0 }, {  2,  7,  1 }, { -2,  7,  1 }, {  2,  8,  1 }, { -2,  8,  1 }, {  3,  3,  2 },
            { -3,  3,  2 }, {  6,  1,  3 }, { -6,  1,  3 }, {  1, 15,  0 }, { -1, 15,  0 }
        },
        //level_add
        { 0, 7, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        1, //inc_limit
        15, //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {  2,  1,  0 }, { -2,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 },
            {  3,  1,  1 }, { -3,  1,  1 }, {     EOB    }, {  1,  3,  0 }, { -1,  3,  0 }, {  2,  2,  0 },
            { -2,  2,  0 }, {  4,  1,  1 }, { -4,  1,  1 }, {  1,  4,  0 }, { -1,  4,  0 }, {  5,  1,  2 },
            { -5,  1,  2 }, {  1,  5,  0 }, { -1,  5,  0 }, {  3,  2,  1 }, { -3,  2,  1 }, {  2,  3,  0 },
            { -2,  3,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, {  6,  1,  2 }, { -6,  1,  2 }, {  2,  4,  0 },
            { -2,  4,  0 }, {  1,  7,  0 }, { -1,  7,  0 }, {  4,  2,  1 }, { -4,  2,  1 }, {  7,  1,  2 },
            { -7,  1,  2 }, {  3,  3,  1 }, { -3,  3,  1 }, {  2,  5,  0 }, { -2,  5,  0 }, {  1,  8,  0 },
            { -1,  8,  0 }, {  2,  6,  0 }, { -2,  6,  0 }, {  8,  1,  3 }, { -8,  1,  3 }, {  1,  9,  0 },
            { -1,  9,  0 }, {  5,  2,  2 }, { -5,  2,  2 }, {  3,  4,  1 }, { -3,  4,  1 }, {  2,  7,  0 },
            { -2,  7,  0 }, {  9,  1,  3 }, { -9,  1,  3 }, {  1, 10,  0 }, { -1, 10,  0 }
        },
        //level_add
        { 0, 10, 6, 4, 4, 3, 3, 3, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        2, //inc_limit
        10, //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {  2,  1,  0 }, { -2,  1,  0 }, {  3,  1,  0 }, { -3,  1,  0 },
            {  1,  2,  0 }, { -1,  2,  0 }, {     EOB    }, {  4,  1,  0 }, { -4,  1,  0 }, {  5,  1,  1 },
            { -5,  1,  1 }, {  2,  2,  0 }, { -2,  2,  0 }, {  1,  3,  0 }, { -1,  3,  0 }, {  6,  1,  1 },
            { -6,  1,  1 }, {  3,  2,  0 }, { -3,  2,  0 }, {  7,  1,  1 }, { -7,  1,  1 }, {  1,  4,  0 },
            { -1,  4,  0 }, {  8,  1,  2 }, { -8,  1,  2 }, {  2,  3,  0 }, { -2,  3,  0 }, {  4,  2,  0 },
            { -4,  2,  0 }, {  1,  5,  0 }, { -1,  5,  0 }, {  9,  1,  2 }, { -9,  1,  2 }, {  5,  2,  1 },
            { -5,  2,  1 }, {  2,  4,  0 }, { -2,  4,  0 }, { 10,  1,  2 }, {-10,  1,  2 }, {  3,  3,  0 },
            { -3,  3,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, { 11,  1,  3 }, {-11,  1,  3 }, {  6,  2,  1 },
            { -6,  2,  1 }, {  1,  7,  0 }, { -1,  7,  0 }, {  2,  5,  0 }, { -2,  5,  0 }, {  3,  4,  0 },
            { -3,  4,  0 }, { 12,  1,  3 }, {-12,  1,  3 }, {  4,  3,  0 }, { -4,  3,  0 }
         },
        //level_add
        { 0, 13, 7, 5, 4, 3, 2, 2, -1, -1, -1 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        4, //inc_limit
        7, //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {  2,  1,  0 }, { -2,  1,  0 }, {  3,  1,  0 }, { -3,  1,  0 },
            {     EOB    }, {  4,  1,  0 }, { -4,  1,  0 }, {  5,  1,  0 }, { -5,  1,  0 }, {  6,  1,  0 },
            { -6,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, {  7,  1,  0 }, { -7,  1,  0 }, {  8,  1,  1 },
            { -8,  1,  1 }, {  2,  2,  0 }, { -2,  2,  0 }, {  9,  1,  1 }, { -9,  1,  1 }, { 10,  1,  1 },
            {-10,  1,  1 }, {  1,  3,  0 }, { -1,  3,  0 }, {  3,  2,  0 }, { -3,  2,  0 }, { 11,  1,  2 },
            {-11,  1,  2 }, {  4,  2,  0 }, { -4,  2,  0 }, { 12,  1,  2 }, {-12,  1,  2 }, { 13,  1,  2 },
            {-13,  1,  2 }, {  5,  2,  0 }, { -5,  2,  0 }, {  1,  4,  0 }, { -1,  4,  0 }, {  2,  3,  0 },
            { -2,  3,  0 }, { 14,  1,  2 }, {-14,  1,  2 }, {  6,  2,  0 }, { -6,  2,  0 }, { 15,  1,  2 },
            {-15,  1,  2 }, { 16,  1,  2 }, {-16,  1,  2 }, {  3,  3,  0 }, { -3,  3,  0 }, {  1,  5,  0 },
            { -1,  5,  0 }, {  7,  2,  0 }, { -7,  2,  0 }, { 17,  1,  2 }, {-17,  1,  2 }
        },
        //level_add
        { 0,18, 8, 4, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        7, //inc_limit
        5, //max_run
    },
    {
        { //level / run
            {     EOB    }, {  1,  1,  0 }, { -1,  1,  0 }, {  2,  1,  0 }, { -2,  1,  0 }, {  3,  1,  0 },
            { -3,  1,  0 }, {  4,  1,  0 }, { -4,  1,  0 }, {  5,  1,  0 }, { -5,  1,  0 }, {  6,  1,  0 },
            { -6,  1,  0 }, {  7,  1,  0 }, { -7,  1,  0 }, {  8,  1,  0 }, { -8,  1,  0 }, {  9,  1,  0 },
            { -9,  1,  0 }, { 10,  1,  0 }, {-10,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, { 11,  1,  1 },
            {-11,  1,  1 }, { 12,  1,  1 }, {-12,  1,  1 }, { 13,  1,  1 }, {-13,  1,  1 }, {  2,  2,  0 },
            { -2,  2,  0 }, { 14,  1,  1 }, {-14,  1,  1 }, { 15,  1,  1 }, {-15,  1,  1 }, {  3,  2,  0 },
            { -3,  2,  0 }, { 16,  1,  1 }, {-16,  1,  1 }, {  1,  3,  0 }, { -1,  3,  0 }, { 17,  1,  1 },
            {-17,  1,  1 }, {  4,  2,  0 }, { -4,  2,  0 }, { 18,  1,  1 }, {-18,  1,  1 }, {  5,  2,  0 },
            { -5,  2,  0 }, { 19,  1,  1 }, {-19,  1,  1 }, { 20,  1,  1 }, {-20,  1,  1 }, {  6,  2,  0 },
            { -6,  2,  0 }, { 21,  1,  1 }, {-21,  1,  1 }, {  2,  3,  0 }, { -2,  3,  0 }
        },
        //level_add
        { 0, 22, 7, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        10, //inc_limit
        3, //max_run
    },
    {
        { //level / run
            {     EOB    }, {  1,  1,  0 }, { -1,  1,  0 }, {  2,  1,  0 }, { -2,  1,  0 }, {  3,  1,  0 },
            { -3,  1,  0 }, {  4,  1,  0 }, { -4,  1,  0 }, {  5,  1,  0 }, { -5,  1,  0 }, {  6,  1,  0 },
            { -6,  1,  0 }, {  7,  1,  0 }, { -7,  1,  0 }, {  8,  1,  0 }, { -8,  1,  0 }, {  9,  1,  0 },
            { -9,  1,  0 }, { 10,  1,  0 }, {-10,  1,  0 }, { 11,  1,  0 }, {-11,  1,  0 }, { 12,  1,  0 },
            {-12,  1,  0 }, { 13,  1,  0 }, {-13,  1,  0 }, { 14,  1,  0 }, {-14,  1,  0 }, { 15,  1,  0 },
            {-15,  1,  0 }, { 16,  1,  0 }, {-16,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, { 17,  1,  0 },
            {-17,  1,  0 }, { 18,  1,  0 }, {-18,  1,  0 }, { 19,  1,  0 }, {-19,  1,  0 }, { 20,  1,  0 },
            {-20,  1,  0 }, { 21,  1,  0 }, {-21,  1,  0 }, {  2,  2,  0 }, { -2,  2,  0 }, { 22,  1,  0 },
            {-22,  1,  0 }, { 23,  1,  0 }, {-23,  1,  0 }, { 24,  1,  0 }, {-24,  1,  0 }, { 25,  1,  0 },
            {-25,  1,  0 }, {  3,  2,  0 }, { -3,  2,  0 }, { 26,  1,  0 }, {-26,  1,  0 }
        },
        //level_add
        { 0, 27, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        INT_MAX, //inc_limit
        2, //max_run
    }
};

static const struct dec_2dvlc inter_dec[7] = {
    {
        { //level / run
            {  1,  1,  1 }, { -1,  1,  1 }, {  1,  2,  1 }, { -1,  2,  1 }, {  1,  3,  1 }, { -1,  3,  1 },
            {  1,  4,  1 }, { -1,  4,  1 }, {  1,  5,  1 }, { -1,  5,  1 }, {  1,  6,  1 }, { -1,  6,  1 },
            {  1,  7,  1 }, { -1,  7,  1 }, {  1,  8,  1 }, { -1,  8,  1 }, {  1,  9,  1 }, { -1,  9,  1 },
            {  1, 10,  1 }, { -1, 10,  1 }, {  1, 11,  1 }, { -1, 11,  1 }, {  1, 12,  1 }, { -1, 12,  1 },
            {  1, 13,  1 }, { -1, 13,  1 }, {  2,  1,  2 }, { -2,  1,  2 }, {  1, 14,  1 }, { -1, 14,  1 },
            {  1, 15,  1 }, { -1, 15,  1 }, {  1, 16,  1 }, { -1, 16,  1 }, {  1, 17,  1 }, { -1, 17,  1 },
            {  1, 18,  1 }, { -1, 18,  1 }, {  1, 19,  1 }, { -1, 19,  1 }, {  3,  1,  3 }, { -3,  1,  3 },
            {  1, 20,  1 }, { -1, 20,  1 }, {  1, 21,  1 }, { -1, 21,  1 }, {  2,  2,  2 }, { -2,  2,  2 },
            {  1, 22,  1 }, { -1, 22,  1 }, {  1, 23,  1 }, { -1, 23,  1 }, {  1, 24,  1 }, { -1, 24,  1 },
            {  1, 25,  1 }, { -1, 25,  1 }, {  1, 26,  1 }, { -1, 26,  1 }, {   EOB    }
        },
        //level_add
        { 0, 4, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
        3, //golomb_order
        0, //inc_limit
        26 //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {     EOB    }, {  1,  2,  0 }, { -1,  2,  0 }, {  1,  3,  0 },
            { -1,  3,  0 }, {  1,  4,  0 }, { -1,  4,  0 }, {  1,  5,  0 }, { -1,  5,  0 }, {  1,  6,  0 },
            { -1,  6,  0 }, {  2,  1,  1 }, { -2,  1,  1 }, {  1,  7,  0 }, { -1,  7,  0 }, {  1,  8,  0 },
            { -1,  8,  0 }, {  1,  9,  0 }, { -1,  9,  0 }, {  1, 10,  0 }, { -1, 10,  0 }, {  2,  2,  1 },
            { -2,  2,  1 }, {  1, 11,  0 }, { -1, 11,  0 }, {  1, 12,  0 }, { -1, 12,  0 }, {  3,  1,  2 },
            { -3,  1,  2 }, {  1, 13,  0 }, { -1, 13,  0 }, {  1, 14,  0 }, { -1, 14,  0 }, {  2,  3,  1 },
            { -2,  3,  1 }, {  1, 15,  0 }, { -1, 15,  0 }, {  2,  4,  1 }, { -2,  4,  1 }, {  1, 16,  0 },
            { -1, 16,  0 }, {  2,  5,  1 }, { -2,  5,  1 }, {  1, 17,  0 }, { -1, 17,  0 }, {  4,  1,  3 },
            { -4,  1,  3 }, {  2,  6,  1 }, { -2,  6,  1 }, {  1, 18,  0 }, { -1, 18,  0 }, {  1, 19,  0 },
            { -1, 19,  0 }, {  2,  7,  1 }, { -2,  7,  1 }, {  3,  2,  2 }, { -3,  2,  2 }
        },
        //level_add
        { 0, 5, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        1, //inc_limit
        19 //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {     EOB    }, {  1,  2,  0 }, { -1,  2,  0 }, {  2,  1,  0 },
            { -2,  1,  0 }, {  1,  3,  0 }, { -1,  3,  0 }, {  1,  4,  0 }, { -1,  4,  0 }, {  3,  1,  1 },
            { -3,  1,  1 }, {  2,  2,  0 }, { -2,  2,  0 }, {  1,  5,  0 }, { -1,  5,  0 }, {  1,  6,  0 },
            { -1,  6,  0 }, {  1,  7,  0 }, { -1,  7,  0 }, {  2,  3,  0 }, { -2,  3,  0 }, {  4,  1,  2 },
            { -4,  1,  2 }, {  1,  8,  0 }, { -1,  8,  0 }, {  3,  2,  1 }, { -3,  2,  1 }, {  2,  4,  0 },
            { -2,  4,  0 }, {  1,  9,  0 }, { -1,  9,  0 }, {  1, 10,  0 }, { -1, 10,  0 }, {  5,  1,  2 },
            { -5,  1,  2 }, {  2,  5,  0 }, { -2,  5,  0 }, {  1, 11,  0 }, { -1, 11,  0 }, {  2,  6,  0 },
            { -2,  6,  0 }, {  1, 12,  0 }, { -1, 12,  0 }, {  3,  3,  1 }, { -3,  3,  1 }, {  6,  1,  2 },
            { -6,  1,  2 }, {  4,  2,  2 }, { -4,  2,  2 }, {  1, 13,  0 }, { -1, 13,  0 }, {  2,  7,  0 },
            { -2,  7,  0 }, {  3,  4,  1 }, { -3,  4,  1 }, {  1, 14,  0 }, { -1, 14,  0 }
        },
        //level_add
        { 0, 7, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        2, //inc_limit
        14 //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {     EOB    }, {  2,  1,  0 }, { -2,  1,  0 }, {  1,  2,  0 },
            { -1,  2,  0 }, {  3,  1,  0 }, { -3,  1,  0 }, {  1,  3,  0 }, { -1,  3,  0 }, {  2,  2,  0 },
            { -2,  2,  0 }, {  4,  1,  1 }, { -4,  1,  1 }, {  1,  4,  0 }, { -1,  4,  0 }, {  5,  1,  1 },
            { -5,  1,  1 }, {  1,  5,  0 }, { -1,  5,  0 }, {  3,  2,  0 }, { -3,  2,  0 }, {  2,  3,  0 },
            { -2,  3,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, {  6,  1,  1 }, { -6,  1,  1 }, {  2,  4,  0 },
            { -2,  4,  0 }, {  1,  7,  0 }, { -1,  7,  0 }, {  4,  2,  1 }, { -4,  2,  1 }, {  7,  1,  2 },
            { -7,  1,  2 }, {  3,  3,  0 }, { -3,  3,  0 }, {  1,  8,  0 }, { -1,  8,  0 }, {  2,  5,  0 },
            { -2,  5,  0 }, {  8,  1,  2 }, { -8,  1,  2 }, {  1,  9,  0 }, { -1,  9,  0 }, {  3,  4,  0 },
            { -3,  4,  0 }, {  2,  6,  0 }, { -2,  6,  0 }, {  5,  2,  1 }, { -5,  2,  1 }, {  1, 10,  0 },
            { -1, 10,  0 }, {  9,  1,  2 }, { -9,  1,  2 }, {  4,  3,  1 }, { -4,  3,  1 }
        },
        //level_add
        { 0,10, 6, 5, 4, 3, 3, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        3, //inc_limit
        10 //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {     EOB    }, {  2,  1,  0 }, { -2,  1,  0 }, {  3,  1,  0 },
            { -3,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, {  4,  1,  0 }, { -4,  1,  0 }, {  5,  1,  0 },
            { -5,  1,  0 }, {  2,  2,  0 }, { -2,  2,  0 }, {  1,  3,  0 }, { -1,  3,  0 }, {  6,  1,  0 },
            { -6,  1,  0 }, {  3,  2,  0 }, { -3,  2,  0 }, {  7,  1,  1 }, { -7,  1,  1 }, {  1,  4,  0 },
            { -1,  4,  0 }, {  8,  1,  1 }, { -8,  1,  1 }, {  2,  3,  0 }, { -2,  3,  0 }, {  4,  2,  0 },
            { -4,  2,  0 }, {  1,  5,  0 }, { -1,  5,  0 }, {  9,  1,  1 }, { -9,  1,  1 }, {  5,  2,  0 },
            { -5,  2,  0 }, {  2,  4,  0 }, { -2,  4,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, { 10,  1,  2 },
            {-10,  1,  2 }, {  3,  3,  0 }, { -3,  3,  0 }, { 11,  1,  2 }, {-11,  1,  2 }, {  1,  7,  0 },
            { -1,  7,  0 }, {  6,  2,  0 }, { -6,  2,  0 }, {  3,  4,  0 }, { -3,  4,  0 }, {  2,  5,  0 },
            { -2,  5,  0 }, { 12,  1,  2 }, {-12,  1,  2 }, {  4,  3,  0 }, { -4,  3,  0 }
        },
        //level_add
        { 0, 13, 7, 5, 4, 3, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        6, //inc_limit
        7  //max_run
    },
    {
        { //level / run
            {      EOB    }, {  1,  1,  0 }, {  -1,  1,  0 }, {  2,  1,  0 }, {  -2,  1,  0 }, {  3,  1,  0 },
            {  -3,  1,  0 }, {  4,  1,  0 }, {  -4,  1,  0 }, {  5,  1,  0 }, {  -5,  1,  0 }, {  1,  2,  0 },
            {  -1,  2,  0 }, {  6,  1,  0 }, {  -6,  1,  0 }, {  7,  1,  0 }, {  -7,  1,  0 }, {  8,  1,  0 },
            {  -8,  1,  0 }, {  2,  2,  0 }, {  -2,  2,  0 }, {  9,  1,  0 }, {  -9,  1,  0 }, {  1,  3,  0 },
            {  -1,  3,  0 }, { 10,  1,  1 }, { -10,  1,  1 }, {  3,  2,  0 }, {  -3,  2,  0 }, { 11,  1,  1 },
            { -11,  1,  1 }, {  4,  2,  0 }, {  -4,  2,  0 }, { 12,  1,  1 }, { -12,  1,  1 }, {  1,  4,  0 },
            {  -1,  4,  0 }, {  2,  3,  0 }, {  -2,  3,  0 }, { 13,  1,  1 }, { -13,  1,  1 }, {  5,  2,  0 },
            {  -5,  2,  0 }, { 14,  1,  1 }, { -14,  1,  1 }, {  6,  2,  0 }, {  -6,  2,  0 }, {  1,  5,  0 },
            {  -1,  5,  0 }, { 15,  1,  1 }, { -15,  1,  1 }, {  3,  3,  0 }, {  -3,  3,  0 }, { 16,  1,  1 },
            { -16,  1,  1 }, {  2,  4,  0 }, {  -2,  4,  0 }, {  7,  2,  0 }, {  -7,  2,  0 }
        },
        //level_add
        { 0, 17, 8, 4, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        9, //inc_limit
        5  //max_run
    },
    {
        { //level / run
            {      EOB    }, {  1,  1,  0 }, {  -1,  1,  0 }, {  2,  1,  0 }, {  -2,  1,  0 }, {   3,  1,  0 },
            {  -3,  1,  0 }, {  4,  1,  0 }, {  -4,  1,  0 }, {  5,  1,  0 }, {  -5,  1,  0 }, {   6,  1,  0 },
            {  -6,  1,  0 }, {  7,  1,  0 }, {  -7,  1,  0 }, {  1,  2,  0 }, {  -1,  2,  0 }, {   8,  1,  0 },
            {  -8,  1,  0 }, {  9,  1,  0 }, {  -9,  1,  0 }, { 10,  1,  0 }, { -10,  1,  0 }, {  11,  1,  0 },
            { -11,  1,  0 }, { 12,  1,  0 }, { -12,  1,  0 }, {  2,  2,  0 }, {  -2,  2,  0 }, {  13,  1,  0 },
            { -13,  1,  0 }, {  1,  3,  0 }, {  -1,  3,  0 }, { 14,  1,  0 }, { -14,  1,  0 }, {  15,  1,  0 },
            { -15,  1,  0 }, {  3,  2,  0 }, {  -3,  2,  0 }, { 16,  1,  0 }, { -16,  1,  0 }, {  17,  1,  0 },
            { -17,  1,  0 }, { 18,  1,  0 }, { -18,  1,  0 }, {  4,  2,  0 }, {  -4,  2,  0 }, {  19,  1,  0 },
            { -19,  1,  0 }, { 20,  1,  0 }, { -20,  1,  0 }, {  2,  3,  0 }, {  -2,  3,  0 }, {   1,  4,  0 },
            {  -1,  4,  0 }, {  5,  2,  0 }, {  -5,  2,  0 }, { 21,  1,  0 }, { -21,  1,  0 }
        },
        //level_add
        { 0, 22, 6, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        2, //golomb_order
        INT_MAX, //inc_limit
        4 //max_run
    }
};

static const struct dec_2dvlc chroma_dec[5] = {
    {
        { //level / run
            {  1,  1,  1 }, { -1,  1,  1 }, {  1,  2,  1 }, { -1,  2,  1 }, {  1,  3,  1 }, { -1,  3,  1 },
            {  1,  4,  1 }, { -1,  4,  1 }, {  1,  5,  1 }, { -1,  5,  1 }, {  1,  6,  1 }, { -1,  6,  1 },
            {  1,  7,  1 }, { -1,  7,  1 }, {  2,  1,  2 }, { -2,  1,  2 }, {  1,  8,  1 }, { -1,  8,  1 },
            {  1,  9,  1 }, { -1,  9,  1 }, {  1, 10,  1 }, { -1, 10,  1 }, {  1, 11,  1 }, { -1, 11,  1 },
            {  1, 12,  1 }, { -1, 12,  1 }, {  1, 13,  1 }, { -1, 13,  1 }, {  1, 14,  1 }, { -1, 14,  1 },
            {  1, 15,  1 }, { -1, 15,  1 }, {  3,  1,  3 }, { -3,  1,  3 }, {  1, 16,  1 }, { -1, 16,  1 },
            {  1, 17,  1 }, { -1, 17,  1 }, {  1, 18,  1 }, { -1, 18,  1 }, {  1, 19,  1 }, { -1, 19,  1 },
            {  1, 20,  1 }, { -1, 20,  1 }, {  1, 21,  1 }, { -1, 21,  1 }, {  1, 22,  1 }, { -1, 22,  1 },
            {  2,  2,  2 }, { -2,  2,  2 }, {  1, 23,  1 }, { -1, 23,  1 }, {  1, 24,  1 }, { -1, 24,  1 },
            {  1, 25,  1 }, { -1, 25,  1 }, {  4,  1,  3 }, { -4,  1,  3 }, {   EOB    }
        },
        //level_add
        { 0, 5, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1 },
        2, //golomb_order
        0, //inc_limit
        25 //max_run
    },
    {
        { //level / run
            {     EOB    }, {  1,  1,  0 }, { -1,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, {  2,  1,  1 },
            { -2,  1,  1 }, {  1,  3,  0 }, { -1,  3,  0 }, {  1,  4,  0 }, { -1,  4,  0 }, {  1,  5,  0 },
            { -1,  5,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, {  3,  1,  2 }, { -3,  1,  2 }, {  1,  7,  0 },
            { -1,  7,  0 }, {  1,  8,  0 }, { -1,  8,  0 }, {  2,  2,  1 }, { -2,  2,  1 }, {  1,  9,  0 },
            { -1,  9,  0 }, {  1, 10,  0 }, { -1, 10,  0 }, {  1, 11,  0 }, { -1, 11,  0 }, {  4,  1,  2 },
            { -4,  1,  2 }, {  1, 12,  0 }, { -1, 12,  0 }, {  1, 13,  0 }, { -1, 13,  0 }, {  1, 14,  0 },
            { -1, 14,  0 }, {  2,  3,  1 }, { -2,  3,  1 }, {  1, 15,  0 }, { -1, 15,  0 }, {  2,  4,  1 },
            { -2,  4,  1 }, {  5,  1,  3 }, { -5,  1,  3 }, {  3,  2,  2 }, { -3,  2,  2 }, {  1, 16,  0 },
            { -1, 16,  0 }, {  1, 17,  0 }, { -1, 17,  0 }, {  1, 18,  0 }, { -1, 18,  0 }, {  2,  5,  1 },
            { -2,  5,  1 }, {  1, 19,  0 }, { -1, 19,  0 }, {  1, 20,  0 }, { -1, 20,  0 }
        },
        //level_add
        { 0, 6, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1 },
        0, //golomb_order
        1, //inc_limit
        20 //max_run
    },
    {
        { //level / run
            {  1,  1,  0 }, { -1,  1,  0 }, {     EOB    }, {  2,  1,  0 }, { -2,  1,  0 }, {  1,  2,  0 },
            { -1,  2,  0 }, {  3,  1,  1 }, { -3,  1,  1 }, {  1,  3,  0 }, { -1,  3,  0 }, {  4,  1,  1 },
            { -4,  1,  1 }, {  2,  2,  0 }, { -2,  2,  0 }, {  1,  4,  0 }, { -1,  4,  0 }, {  5,  1,  2 },
            { -5,  1,  2 }, {  1,  5,  0 }, { -1,  5,  0 }, {  3,  2,  1 }, { -3,  2,  1 }, {  2,  3,  0 },
            { -2,  3,  0 }, {  1,  6,  0 }, { -1,  6,  0 }, {  6,  1,  2 }, { -6,  1,  2 }, {  1,  7,  0 },
            { -1,  7,  0 }, {  2,  4,  0 }, { -2,  4,  0 }, {  7,  1,  2 }, { -7,  1,  2 }, {  1,  8,  0 },
            { -1,  8,  0 }, {  4,  2,  1 }, { -4,  2,  1 }, {  1,  9,  0 }, { -1,  9,  0 }, {  3,  3,  1 },
            { -3,  3,  1 }, {  2,  5,  0 }, { -2,  5,  0 }, {  2,  6,  0 }, { -2,  6,  0 }, {  8,  1,  2 },
            { -8,  1,  2 }, {  1, 10,  0 }, { -1, 10,  0 }, {  1, 11,  0 }, { -1, 11,  0 }, {  9,  1,  2 },
            { -9,  1,  2 }, {  5,  2,  2 }, { -5,  2,  2 }, {  3,  4,  1 }, { -3,  4,  1 },
        },
        //level_add
        { 0,10, 6, 4, 4, 3, 3, 2, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        1, //golomb_order
        2, //inc_limit
        11 //max_run
    },
    {
        { //level / run
            {     EOB    }, {  1,  1,  0 }, { -1,  1,  0 }, {  2,  1,  0 }, { -2,  1,  0 }, {  3,  1,  0 },
            { -3,  1,  0 }, {  4,  1,  0 }, { -4,  1,  0 }, {  1,  2,  0 }, { -1,  2,  0 }, {  5,  1,  1 },
            { -5,  1,  1 }, {  2,  2,  0 }, { -2,  2,  0 }, {  6,  1,  1 }, { -6,  1,  1 }, {  1,  3,  0 },
            { -1,  3,  0 }, {  7,  1,  1 }, { -7,  1,  1 }, {  3,  2,  0 }, { -3,  2,  0 }, {  8,  1,  1 },
            { -8,  1,  1 }, {  1,  4,  0 }, { -1,  4,  0 }, {  2,  3,  0 }, { -2,  3,  0 }, {  9,  1,  1 },
            { -9,  1,  1 }, {  4,  2,  0 }, { -4,  2,  0 }, {  1,  5,  0 }, { -1,  5,  0 }, { 10,  1,  1 },
            {-10,  1,  1 }, {  3,  3,  0 }, { -3,  3,  0 }, {  5,  2,  1 }, { -5,  2,  1 }, {  2,  4,  0 },
            { -2,  4,  0 }, { 11,  1,  1 }, {-11,  1,  1 }, {  1,  6,  0 }, { -1,  6,  0 }, { 12,  1,  1 },
            {-12,  1,  1 }, {  1,  7,  0 }, { -1,  7,  0 }, {  6,  2,  1 }, { -6,  2,  1 }, { 13,  1,  1 },
            {-13,  1,  1 }, {  2,  5,  0 }, { -2,  5,  0 }, {  1,  8,  0 }, { -1,  8,  0 },
        },
        //level_add
        { 0, 14, 7, 4, 3, 3, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        1, //golomb_order
        4, //inc_limit
        8  //max_run
    },
    {
        { //level / run
            {      EOB    }, {  1,  1,  0 }, {  -1,  1,  0 }, {  2,  1,  0 }, {  -2,  1,  0 }, {  3,  1,  0 },
            {  -3,  1,  0 }, {  4,  1,  0 }, {  -4,  1,  0 }, {  5,  1,  0 }, {  -5,  1,  0 }, {  6,  1,  0 },
            {  -6,  1,  0 }, {  7,  1,  0 }, {  -7,  1,  0 }, {  8,  1,  0 }, {  -8,  1,  0 }, {  1,  2,  0 },
            {  -1,  2,  0 }, {  9,  1,  0 }, {  -9,  1,  0 }, { 10,  1,  0 }, { -10,  1,  0 }, { 11,  1,  0 },
            { -11,  1,  0 }, {  2,  2,  0 }, {  -2,  2,  0 }, { 12,  1,  0 }, { -12,  1,  0 }, { 13,  1,  0 },
            { -13,  1,  0 }, {  3,  2,  0 }, {  -3,  2,  0 }, { 14,  1,  0 }, { -14,  1,  0 }, {  1,  3,  0 },
            {  -1,  3,  0 }, { 15,  1,  0 }, { -15,  1,  0 }, {  4,  2,  0 }, {  -4,  2,  0 }, { 16,  1,  0 },
            { -16,  1,  0 }, { 17,  1,  0 }, { -17,  1,  0 }, {  5,  2,  0 }, {  -5,  2,  0 }, {  1,  4,  0 },
            {  -1,  4,  0 }, {  2,  3,  0 }, {  -2,  3,  0 }, { 18,  1,  0 }, { -18,  1,  0 }, {  6,  2,  0 },
            {  -6,  2,  0 }, { 19,  1,  0 }, { -19,  1,  0 }, {  1,  5,  0 }, {  -1,  5,  0 },
        },
        //level_add
        { 0, 20, 7, 3, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        0, //golomb_order
        INT_MAX, //inc_limit
        5, //max_run
    }
};

#undef EOB

/*****************************************************************************
 *
 * motion vector prediction
 *
 ****************************************************************************/

static inline void store_mvs(AVSContext *h)
{
    h->col_mv[h->mbidx * 4 + 0] = h->mv[MV_FWD_X0];
    h->col_mv[h->mbidx * 4 + 1] = h->mv[MV_FWD_X1];
    h->col_mv[h->mbidx * 4 + 2] = h->mv[MV_FWD_X2];
    h->col_mv[h->mbidx * 4 + 3] = h->mv[MV_FWD_X3];
}

/** Return the macroblock co-located in the future anchor picture.
 *
 * Under BFieldEnhanced both fields use the first field's co-located block;
 * the second field therefore addresses the corresponding MB in the first
 * half of the field-coded anchor picture (clause 9.9.1 b) 3)).
 */
static inline int direct_col_mbidx(const AVSContext *h)
{
    if (!h->pic_structure && h->pb_field_enhanced && h->field)
        return h->mbidx - h->mb_width * h->mb_height / 2;
    return h->mbidx;
}

/** Direct mode: scale the co-located motion vector by its own
    BlockDistance, carried in the mv cache (clause 9.9.1 b)). */
static inline int mv_pred_direct(AVSContext *h, cavs_vector *pmv_fw,
                                 const cavs_vector *col_mv)
{
    cavs_vector *pmv_bw = pmv_fw + MV_BWD_OFFS;
    /* BlockDistanceRef of clause 9.9.1 b) step 2: the distance the
     * co-located block itself was coded with, from the mv cache */
    unsigned den = col_mv->dist ? 16384 / col_mv->dist : 0;
    int delta_ref = 0, delta_fw = 0, delta_bw = 0;
    int col_y = col_mv->y;
    int m = FF_SIGNBIT(col_mv->x);

    pmv_fw->ref = h->ref_base[0];
    pmv_bw->ref = h->ref_base[1];
    if (!h->pic_structure) {
        int di_ref;
        int di_fw0 = (h->DPB[1].poc + 1) & 511;

        if (h->pb_field_enhanced) {
            /* The enhanced co-located vector comes from the future anchor's
             * first field. Its coded reference index identifies one of the
             * two fields in DPB[1] or DPB[2], newest anchor first. */
            const AVSFrame *anchor;

            if ((unsigned)col_mv->ref >= 4) {
                av_log(h->avctx, AV_LOG_ERROR,
                       "enhanced direct reference %d out of range\n",
                       col_mv->ref);
                return AVERROR_INVALIDDATA;
            }
            anchor = &h->DPB[1 + (col_mv->ref >> 1)];
            if (!anchor->f->data[0]) {
                av_log(h->avctx, AV_LOG_ERROR,
                       "enhanced direct reference anchor is unavailable\n");
                return AVERROR_INVALIDDATA;
            }

            di_ref = (anchor->poc + !(col_mv->ref & 1)) & 511;
            /* The second current field always derives Fw0 and Bw0. The first
             * one follows the field selected by the co-located vector. */
            pmv_fw->ref += !h->field && di_ref != di_fw0;
            delta_ref = (!(col_mv->ref & 1)) << 1;
            delta_fw  = ((pmv_fw->ref - h->ref_base[0]) == h->field) << 1;
            delta_bw  = h->field ? -2 : 0;
        } else {
            /* reference field selection with BFieldEnhanced equal to zero */
            int di_col = h->DPB[0].poc + h->field;

            di_ref = (di_col - col_mv->dist) & 511;
            pmv_fw->ref += di_ref != di_fw0;
            pmv_bw->ref += h->field;
        }
    }
    pmv_fw->dist    = h->dist[pmv_fw->ref];
    pmv_bw->dist    = h->dist[pmv_bw->ref];
    pmv_fw->ref_idx = pmv_bw->ref_idx = 0;      /* derived (clause 9.4.5) */
    /* scale the co-located motion vector according to its temporal span */
    pmv_fw->x =     (((den + (den * col_mv->x * pmv_fw->dist ^ m) - m - 1) >> 14) ^ m) - m;
    pmv_bw->x = m - (((den + (den * col_mv->x * pmv_bw->dist ^ m) - m - 1) >> 14) ^ m);
    col_y += delta_ref;
    m = FF_SIGNBIT(col_y);
    pmv_fw->y =     (((den + (den * col_y * pmv_fw->dist ^ m) - m - 1) >> 14) ^ m) - m - delta_fw;
    pmv_bw->y = m - (((den + (den * col_y * pmv_bw->dist ^ m) - m - 1) >> 14) ^ m) - delta_bw;
    return 0;
}

/** Symmetric mode: derive the backward mv from the forward one, scaled by
    the block's own pair of distances (clause 9.9.1 c)). */
static inline void mv_pred_sym(AVSContext *h, cavs_vector *src,
                               enum cavs_block size)
{
    cavs_vector *dst = src + MV_BWD_OFFS;
    int factor;

    if (h->pic_structure)
        dst->ref = h->ref_base[1];
    else
        dst->ref = h->ref_base[1] + 1 - (src->ref - h->ref_base[0]);
    dst->dist    = h->dist[dst->ref];
    dst->ref_idx = 0;               /* derived, not read (clause 9.4.5) */
    factor    = dst->dist * h->scale_den[src->ref];
    /* backward mv is the scaled and negated forward mv */
    dst->x = -((src->x * factor + 256) >> 9);
    dst->y = -((src->y * factor + 256) >> 9);
    set_mvs(dst, size);
}

/*****************************************************************************
 *
 * residual data decoding
 *
 ****************************************************************************/

/** kth-order exponential golomb code */
static inline int get_ue_code(GetBitContext *gb, int order)
{
    unsigned ret = get_ue_golomb(gb);
    if (ret >= ((1U<<31)>>order)) {
        av_log(NULL, AV_LOG_ERROR, "get_ue_code: value too large\n");
        return AVERROR_INVALIDDATA;
    }
    if (order) {
        return (ret<<order) + get_bits(gb, order);
    }
    return ret;
}

static inline int dequant(AVSContext *h, int16_t *level_buf, uint8_t *run_buf,
                          int16_t *dst, int mul, int shift, int coeff_num)
{
    int round = 1 << (shift - 1);
    int pos = -1;
    const uint8_t *scantab = h->scantable;

    /* inverse scan and dequantization */
    while (--coeff_num >= 0) {
        pos += run_buf[coeff_num];
        if (pos > 63) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "position out of block bounds at pic %d MB(%d,%d)\n",
                   h->cur.poc, h->mbx, h->mby);
            return AVERROR_INVALIDDATA;
        }
        if (h->weighting_quant) {
            int weight = h->weighting_quant_matrix[h->weighting_scan[pos]];
            int64_t level = ((int64_t)level_buf[coeff_num] * weight) >> 3;

            dst[scantab[pos]] = (((level * mul) >> 4) + round) >> shift;
        } else {
            dst[scantab[pos]] = (level_buf[coeff_num] * mul + round) >> shift;
        }
    }
    return 0;
}

/**
 * decode coefficients from one 8x8 block, dequantize, inverse transform
 *  and add them to sample block
 * @param r pointer to 2D VLC table
 * @param esc_golomb_order escape codes are k-golomb with this order k
 * @param qp quantizer
 * @param dst location of sample block
 * @param stride line stride in frame buffer
 */
static int decode_residual_block(AVSContext *h, GetBitContext *gb,
                                 const struct dec_2dvlc *r, int esc_golomb_order,
                                 int qp, uint8_t *dst, ptrdiff_t stride)
{
    int i, esc_code, level, mask, ret;
    unsigned int level_code, run;
    int16_t level_buf[65];
    uint8_t run_buf[65];
    int16_t *block = h->block;

    for (i = 0; i < 65; i++) {
        level_code = get_ue_code(gb, r->golomb_order);
        if (level_code >= ESCAPE_CODE) {
            run      = ((level_code - ESCAPE_CODE) >> 1) + 1;
            if(run > 64) {
                av_log(h->avctx, AV_LOG_ERROR, "run %d is too large\n", run);
                return AVERROR_INVALIDDATA;
            }
            esc_code = get_ue_code(gb, esc_golomb_order);
            if (esc_code < 0 || esc_code > 32767) {
                av_log(h->avctx, AV_LOG_ERROR, "esc_code invalid\n");
                return AVERROR_INVALIDDATA;
            }

            level    = esc_code + (run > r->max_run ? 1 : r->level_add[run]);
            while (level > r->inc_limit)
                r++;
            mask  = -(level_code & 1);
            level = (level ^ mask) - mask;
        } else {
            level = r->rltab[level_code][0];
            if (!level) //end of block signal
                break;
            run = r->rltab[level_code][1];
            r  += r->rltab[level_code][2];
        }
        level_buf[i] = level;
        run_buf[i]   = run;
    }
    ret = dequant(h, level_buf, run_buf, block, dequant_mul[qp],
                  dequant_shift[qp], i);
    if (ret >= 0)
        h->cdsp.cavs_idct8_add(dst, block, stride);
    /* dequant() writes coefficients before it can tell that a run is out
     * of range, so the buffer has to be cleared on that path too */
    h->bdsp.clear_block(block);
    return ret;
}


/*****************************************************************************
 *
 * advanced entropy coding (AEC), GY/T 257.1-2012 clause 8.4
 *
 ****************************************************************************/

/* Table 51: start index of every ae(v) coded syntax element; an element
 * owns the models up to the next start index */
#define AEC_MB_SKIP_RUN               0
#define AEC_MB_TYPE                   4
#define AEC_MB_PART_TYPE             19
#define AEC_INTRA_LUMA_PRED_MODE     22
#define AEC_INTRA_CHROMA_PRED_MODE   26
#define AEC_MB_REFERENCE_INDEX       30
#define AEC_MV_DIFF_X                36
#define AEC_MV_DIFF_Y                42
#define AEC_CBP                      48
#define AEC_MB_QP_DELTA              54
#define AEC_COEFF_FRAME_LUMA         58
#define AEC_COEFF_FRAME_CHROMA      124
#define AEC_COEFF_FIELD_LUMA        190
#define AEC_COEFF_FIELD_CHROMA      256
#define AEC_WEIGHTING_PREDICTION    322

/** context models owned by one trans_coefficient set, 14 + 32 + 20 */
#define AEC_COEFF_NUM_CTX            66
/** the coeffRun models of a trans_coefficient set start here */
#define AEC_COEFF_RUN                46
/** and there are priIdx * 4 + secIdx of them */
#define AEC_COEFF_RUN_NUM_CTX        20

/** Table 52, priIdx as a function of lMax, which saturates at 5. */
static const uint8_t cavs_aec_pri_idx[6] = { 0, 1, 2, 3, 3, 4 };

/**
 * Decode mb_skip_run and derive SkipMbCount (table 44 unary, 8.4.4.2 a)).
 *
 * @return SkipMbCount, or a negative error code
 */
static int aec_mb_skip_run(AVSContext *h)
{
    int v = 0;

    while (!ff_cavs_aec_decode_decision(&h->aec,
                                        AEC_MB_SKIP_RUN + FFMIN(v, 3))) {
        /* clause 9.3 skips MbIndex to MbIndex + SkipMbCount - 1, so a run
         * cannot be longer than the picture */
        if (++v > h->mb_width * h->mb_height) {
            av_log(h->avctx, AV_LOG_ERROR, "mb_skip_run out of range\n");
            return AVERROR_INVALIDDATA;
        }
    }
    return v;
}

/**
 * Decode mb_type where table 55 applies (clause 9.4.2 a) 2)): table 44
 * unary, mapped by table 54.
 * @return MbTypeIndex, 0 to 5, or a negative error code
 */
static int aec_mb_type_p(AVSContext *h)
{
    int v = 0;

    while (!ff_cavs_aec_decode_decision(&h->aec, AEC_MB_TYPE + FFMIN(v, 4))) {
        if (++v > 5 - h->skip_mode_flag) {
            av_log(h->avctx, AV_LOG_ERROR, "mb_type out of range\n");
            return AVERROR_INVALIDDATA;
        }
    }
    /* table 54, both columns at once */
    return v ? v - 1 + h->skip_mode_flag : 5;
}

/**
 * Decode mb_type in a B picture (table 45, 8.4.4.2 b)).
 * @return MbTypeIndex, 0 to 24 of table 56, or a negative error code
 */
static int aec_mb_type_b(AVSContext *h)
{
    /* the neighbour is available and neither skipped nor direct coded */
    int a = (h->flags & A_AVAIL) && h->left_mb_type        != B_SKIP &&
                                    h->left_mb_type        != B_DIRECT;
    int b = (h->flags & B_AVAIL) && h->top_mb_type[h->mbx] != B_SKIP &&
                                    h->top_mb_type[h->mbx] != B_DIRECT;
    int v = 1;
    int bin;

    if (!ff_cavs_aec_decode_decision(&h->aec, AEC_MB_TYPE + 5 + a + b))
        return h->skip_mode_flag;
    for (bin = 1;
         !ff_cavs_aec_decode_decision(&h->aec,
                                      AEC_MB_TYPE + 7 + FFMIN(bin, 7));
         bin++) {
        if (++v > 24 - h->skip_mode_flag) {
            av_log(h->avctx, AV_LOG_ERROR, "mb_type out of range\n");
            return AVERROR_INVALIDDATA;
        }
    }
    return v + h->skip_mode_flag;
}

/** Decode mb_part_type (tables 46 and 57, 8.4.4.2 c)).
    @return MbPartType, 0 to 3 */
static int aec_mb_part_type(AVSContext *h)
{
    int b0 = ff_cavs_aec_decode_decision(&h->aec, AEC_MB_PART_TYPE);
    int b1 = ff_cavs_aec_decode_decision(&h->aec,
                                         AEC_MB_PART_TYPE + (b0 ? 2 : 1));

    return (b0 << 1) | b1;
}

/**
 * Decode mb_reference_index: table 44 unary in a P picture, one inverted
 * bin in a B one (8.4.3 j), 8.4.4.2 f)).
 * @return the reference index, or a negative error code
 */
static int aec_ref_index(AVSContext *h, int ctx_inc, int max)
{
    int v;

    if (ff_cavs_aec_decode_decision(&h->aec,
                                    AEC_MB_REFERENCE_INDEX + ctx_inc))
        return 0;
    if (h->cur.f->pict_type == AV_PICTURE_TYPE_B)
        return 1;
    if (ff_cavs_aec_decode_decision(&h->aec, AEC_MB_REFERENCE_INDEX + 4))
        return 1;
    for (v = 2; v <= max; v++)
        if (ff_cavs_aec_decode_decision(&h->aec, AEC_MB_REFERENCE_INDEX + 5))
            return v;
    av_log(h->avctx, AV_LOG_ERROR, "mb_reference_index out of range\n");
    return AVERROR_INVALIDDATA;
}

/**
 * Decode one mv_diff_x or mv_diff_y (table 49, 8.4.4.2 g)): three unary
 * bins, then a bypass coded order 0 exp-Golomb tail and sign (8.4.4.1).
 * @param mvda same component of the left 8x8 block's mv_diff, magnitude
 */
static int aec_mv_diff(AVSContext *h, int base, int mvda, int *diff)
{
    int v;

    if (!ff_cavs_aec_decode_decision(&h->aec,
                                     base + (mvda < 2 ? 0 :
                                             mvda < 16 ? 1 : 2))) {
        *diff = 0;                  /* mvdSign is not coded for mvdAbs 0 */
        return 0;
    }
    if (!ff_cavs_aec_decode_decision(&h->aec, base + 3)) {
        v = 1;
    } else if (!ff_cavs_aec_decode_decision(&h->aec, base + 4)) {
        v = 2;
    } else {
        int even = ff_cavs_aec_decode_decision(&h->aec, base + 5);
        int k    = 0;

        while (!ff_cavs_aec_decode_bypass(&h->aec)) {
            /* mv_diff_x and mv_diff_y hold -4096 to 4095 (clause 7.2.5),
             * so (mvdAbs - 3) / 2 needs at most eleven leading zeroes */
            if (++k > 11) {
                av_log(h->avctx, AV_LOG_ERROR, "mv_diff out of range\n");
                return AVERROR_INVALIDDATA;
            }
        }
        v = (1 << k) - 1;
        while (k--)
            v += ff_cavs_aec_decode_bypass(&h->aec) << k;
        v = 3 + 2 * v + even;
    }
    if (ff_cavs_aec_decode_bypass(&h->aec))     /* mvdSign */
        v = -v;
    if (v < -4096 || v > 4095) {
        av_log(h->avctx, AV_LOG_ERROR, "mv_diff %d out of range\n", v);
        return AVERROR_INVALIDDATA;
    }
    *diff = v;
    return 0;
}

/** Decode a motion vector's mv_diff pair into the cache that
    ff_cavs_mv() reads and the next block takes its mvda from. */
static int aec_mv(AVSContext *h, enum cavs_mv_loc nP)
{
    int i;

    for (i = 0; i < 2; i++) {
        /* the left 8x8 block's mv_diff is one place back in the cache */
        int base = i ? AEC_MV_DIFF_Y : AEC_MV_DIFF_X;
        int diff;
        int ret = aec_mv_diff(h, base, FFABS(h->mvd[nP - 1][i]), &diff);

        if (ret < 0)
            return ret;
        h->mvd[nP][i] = diff;
    }
    return 0;
}

/** Decode intra_luma_pred_mode (table 47) and derive IntraLumaPredMode
    from predIntraPredMode per clause 9.4.4.2 a). */
static int aec_intra_luma_pred_mode(AVSContext *h, int pred)
{
    int v = 0;

    while (v < 4 &&
           !ff_cavs_aec_decode_decision(&h->aec, AEC_INTRA_LUMA_PRED_MODE + v))
        v++;

    if (v == 0)
        return pred;
    if (v == 4)
        v = 0;
    return v < pred ? v : v + 1;
}

/** Decode intra_chroma_pred_mode (table 48, 8.4.4.2 e)).
    @return IntraChromaPredMode, 0 to 3 (table 60) */
static int aec_intra_chroma_pred_mode(AVSContext *h)
{
    /* a and b of 8.4.4.2 e): the neighbour is available and not
     * Intra_Chroma_DC (INTRA_C_LP here); non-intra neighbours hold 0 */
    const int a = (h->flags & A_AVAIL) &&
                  h->left_c_pred_mode != INTRA_C_LP;
    const int b = (h->flags & B_AVAIL) &&
                  h->top_c_pred_mode[h->mbx] != INTRA_C_LP;
    int v;

    if (!ff_cavs_aec_decode_decision(&h->aec,
                                     AEC_INTRA_CHROMA_PRED_MODE + a + b))
        v = 0;
    else if (!ff_cavs_aec_decode_decision(&h->aec,
                                          AEC_INTRA_CHROMA_PRED_MODE + 3))
        v = 1;
    else
        v = 2 + ff_cavs_aec_decode_decision(&h->aec,
                                            AEC_INTRA_CHROMA_PRED_MODE + 3);

    h->left_c_pred_mode        = v;
    h->top_c_pred_mode[h->mbx] = v;
    return v;
}

/** Decode cbp (table 50, 8.4.4.2 h), clause 9.4.7).
    @return MbCBP, a six bit mask over the block order numbers */
static int aec_cbp(AVSContext *h)
{
    /* the neighbouring 8x8 block is available and has no coefficients */
    const int left = (h->flags & A_AVAIL) ? ~h->left_cbp        : 0;
    const int top  = (h->flags & B_AVAIL) ? ~h->top_cbp[h->mbx] : 0;
    int cbp = 0;
    int a, b;

    a = (left >> 1) & 1;                        /* left macroblock, block 1 */
    b = (top  >> 2) & 1;                        /* above macroblock, block 2 */
    cbp |= ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + a + 2 * b) << 0;

    a = ~cbp & 1;                               /* this macroblock, block 0 */
    b = (top >> 3) & 1;                         /* above macroblock, block 3 */
    cbp |= ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + a + 2 * b) << 1;

    a = (left >> 3) & 1;                        /* left macroblock, block 3 */
    b = ~cbp & 1;                               /* this macroblock, block 0 */
    cbp |= ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + a + 2 * b) << 2;

    a = (~cbp >> 2) & 1;                        /* this macroblock, block 2 */
    b = (~cbp >> 1) & 1;                        /* this macroblock, block 1 */
    cbp |= ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + a + 2 * b) << 3;

    /* chroma suffix: "0" leaves both bits clear, "11" sets both, "100" and
     * "101" set bit 4 and bit 5 respectively */
    if (ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + 4)) {
        if (ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + 5))
            cbp |= 3 << 4;
        else
            cbp |= 1 << (4 + ff_cavs_aec_decode_decision(&h->aec, AEC_CBP + 5));
    }

    h->left_cbp        = cbp;
    h->top_cbp[h->mbx] = cbp;
    return cbp;
}

/** Decode mb_qp_delta (table 44 unary, 8.4.4.2 i)) and map the code
    number to the signed delta of clause 9.4.8. */
static int aec_mb_qp_delta(AVSContext *h, int *delta)
{
    int ctx = AEC_MB_QP_DELTA + (h->prev_delta_qp != 0);
    int v   = 0;

    while (!ff_cavs_aec_decode_decision(&h->aec, ctx)) {
        v++;
        ctx = AEC_MB_QP_DELTA + (v == 1 ? 2 : 3);
        if (v > 64) {
            av_log(h->avctx, AV_LOG_ERROR, "mb_qp_delta out of range\n");
            return AVERROR_INVALIDDATA;
        }
    }

    if (v)
        v = (v & 1) ? (v + 1) / 2 : -((v + 1) / 2);
    *delta = v;
    return 0;
}

/**
 * Decode the trans_coefficient pairs of one 8x8 block (8.4.4.2 j) and k),
 * clause 9.5.2) and reconstruct it as the VLC path does.
 * @param chroma selects the chroma half of the context sets of table 51
 */
static int decode_residual_block_aec(AVSContext *h, int chroma, int qp,
                                     uint8_t *dst, ptrdiff_t stride)
{
    int16_t level_buf[65];
    uint8_t run_buf[65];
    int16_t *block = h->block;
    int lmax = 0, pos = 0, i, base, ret;

    /* table 51's "field mode" contexts are those of a field coded picture,
     * the condition under which 9.5.3 c) selects inverse scan method 2 */
    if (h->pic_structure)
        base = chroma ? AEC_COEFF_FRAME_CHROMA : AEC_COEFF_FRAME_LUMA;
    else
        base = chroma ? AEC_COEFF_FIELD_CHROMA : AEC_COEFF_FIELD_LUMA;

    for (i = 0; i < 65; i++) {
        /* priIdx and pos are bitstream derived, so the increments they
         * produce are bounded here, not only by the engine's assertions */
        const int pri = cavs_aec_pri_idx[FFMIN(lmax, 5)];
        int ctx_lvl = ff_cavs_aec_ctx_idx(base, AEC_COEFF_NUM_CTX,
                                          pri * 3 - (pri != 0));
        int ctx_run = ff_cavs_aec_ctx_idx(base + AEC_COEFF_RUN,
                                          AEC_COEFF_RUN_NUM_CTX, pri * 4);
        int abs_level, run_val;

        if (ctx_lvl < 0 || ctx_run < 0)
            return FFMIN(ctx_lvl, ctx_run);
        /* the engine keeps emitting MPS bins once the buffer runs out */
        if (get_bits_left(&h->gb) <= 0)
            return AVERROR_INVALIDDATA;

        /* coeffLevel, table 44 unary, secIdx per table 53 */
        if (!lmax) {
            abs_level = 0;
            if (!ff_cavs_aec_decode_decision(&h->aec, ctx_lvl)) {
                do {
                    abs_level++;
                    /* the increment below must not overflow level_buf[] */
                    if (abs_level >= 32767) {
                        av_log(h->avctx, AV_LOG_ERROR, "AbsLevel too large\n");
                        return AVERROR_INVALIDDATA;
                    }
                } while (!ff_cavs_aec_decode_decision(&h->aec, ctx_lvl + 1));
            }
            abs_level++;                        /* clause 9.5.2 a) */
        } else {
            /* the one context weighted bin of the standard (8.4.4.2 j));
             * ctxIdxIncW reduces to 14 + (pos >> 1) for pos 0..63 */
            int ctx_w = ff_cavs_aec_ctx_idx(base, AEC_COEFF_NUM_CTX,
                                            14 + (pos >> 1));
            if (ctx_w < 0)
                return ctx_w;
            if (ff_cavs_aec_decode_decision_w(&h->aec, ctx_lvl, ctx_w))
                break;                          /* AbsLevel 0, end of block */
            abs_level = 1;
            if (!ff_cavs_aec_decode_decision(&h->aec, ctx_lvl + 1)) {
                do {
                    abs_level++;
                    if (abs_level > 32767) {
                        av_log(h->avctx, AV_LOG_ERROR, "AbsLevel too large\n");
                        return AVERROR_INVALIDDATA;
                    }
                } while (!ff_cavs_aec_decode_decision(&h->aec, ctx_lvl + 2));
            }
        }

        /* coeffSign, bypass coded (8.4.4.1) */
        if (ff_cavs_aec_decode_bypass(&h->aec))
            level_buf[i] = -abs_level;
        else
            level_buf[i] = abs_level;

        /* coeffRun, table 44 unary, secIdx per table 53 */
        ctx_run += abs_level > 1 ? 2 : 0;
        run_val  = 0;
        if (!ff_cavs_aec_decode_decision(&h->aec, ctx_run)) {
            do {
                run_val++;
                if (run_val > 64) {
                    av_log(h->avctx, AV_LOG_ERROR, "RunVal too large\n");
                    return AVERROR_INVALIDDATA;
                }
            } while (!ff_cavs_aec_decode_decision(&h->aec, ctx_run + 1));
        }
        run_buf[i] = run_val + 1;   //scan positions consumed, 9.5.3 b)

        /* lMax and pos carried between pairs per their definitions in
         * 8.4.4.2 j), reset per block */
        lmax = FFMAX(lmax, abs_level);
        pos  = FFMIN(pos + run_val + 1, 63);
    }

    ret = dequant(h, level_buf, run_buf, block, dequant_mul[qp],
                  dequant_shift[qp], i);
    if (ret >= 0)
        h->cdsp.cavs_idct8_add(dst, block, stride);
    h->bdsp.clear_block(block);
    return ret;
}


/**
 * Position the bitstream reader on the start code that ends an AEC slice:
 * the engine reads ahead of the bin it is decoding, but holds at most one
 * byte of look ahead in valueT, so the prefix is found by scanning from
 * two bytes back. The scan does not consult h->stc_offset[], so on a
 * de-emulated payload it can stop at a coincidental prefix; then
 * check_for_slice() refuses the offset and the picture is aborted
 * cleanly by the "AEC picture without a slice header" guard.
 */
static void aec_end_of_slice(AVSContext *h)
{
    GetBitContext *gb = &h->gb;
    const uint8_t *buf = gb->buffer;
    int size = gb->size_in_bits >> 3;
    int i    = FFMAX((get_bits_count(gb) >> 3) - 2, 0);

    /* the engine is done with the bitstream until the next slice header
     * starts it again, which is what check_for_slice() waits for */
    h->aec.gb = NULL;

    for (; i + 2 < size; i++)
        if (!buf[i] && !buf[i + 1] && buf[i + 2] == 1)
            break;
    if (init_get_bits8(gb, buf, size) >= 0)
        skip_bits_long(gb, i * 8);
}

static inline int decode_residual_chroma(AVSContext *h)
{
    const int qp_u = ff_cavs_chroma_qp[av_clip(h->qp + h->chroma_qp_delta[0],
                                               0, 63)];
    const int qp_v = ff_cavs_chroma_qp[av_clip(h->qp + h->chroma_qp_delta[1],
                                               0, 63)];
    int ret;

    if (h->cbp & (1 << 4)) {
        if (h->aec_enable)
            ret = decode_residual_block_aec(h, 1, qp_u, h->cu, h->c_stride);
        else
            ret = decode_residual_block(h, &h->gb, chroma_dec, 0, qp_u,
                                        h->cu, h->c_stride);
        if (ret < 0)
            return ret;
    }
    if (h->cbp & (1 << 5)) {
        if (h->aec_enable)
            ret = decode_residual_block_aec(h, 1, qp_v, h->cv, h->c_stride);
        else
            ret = decode_residual_block(h, &h->gb, chroma_dec, 0, qp_v,
                                        h->cv, h->c_stride);
        if (ret < 0)
            return ret;
    }
    return 0;
}

static inline int decode_residual_inter(AVSContext *h)
{
    int block, ret;

    if (h->aec_enable) {
        int delta = 0;

        /* under AEC cbp carries MbCBP itself, for intra macroblocks too */
        h->cbp = aec_cbp(h);
        if (h->cbp && !h->qp_fixed) {
            ret = aec_mb_qp_delta(h, &delta);
            if (ret < 0)
                return ret;
            h->qp = (h->qp + (unsigned)delta) & 63;
        }
        h->prev_delta_qp = delta;
    } else {
        /* get coded block pattern */
        int cbp = get_ue_golomb(&h->gb);
        if (cbp > 63U) {
            av_log(h->avctx, AV_LOG_ERROR, "illegal inter cbp %d\n", cbp);
            return AVERROR_INVALIDDATA;
        }
        h->cbp = cbp_tab[cbp][1];

        /* get quantizer */
        if (h->cbp && !h->qp_fixed)
            h->qp = (h->qp + (unsigned)get_se_golomb(&h->gb)) & 63;
    }
    for (block = 0; block < 4; block++)
        if (h->cbp & (1 << block)) {
            if (h->aec_enable) {
                ret = decode_residual_block_aec(h, 0, h->qp,
                                                h->cy + h->luma_scan[block],
                                                h->l_stride);
                if (ret < 0)
                    return ret;
            } else {
                decode_residual_block(h, &h->gb, inter_dec, 0, h->qp,
                                      h->cy + h->luma_scan[block],
                                      h->l_stride);
            }
        }

    return decode_residual_chroma(h);
}

/*****************************************************************************
 *
 * macroblock level
 *
 ****************************************************************************/

static inline void set_mv_intra(AVSContext *h)
{
    h->mv[MV_FWD_X0] = ff_cavs_intra_mv;
    set_mvs(&h->mv[MV_FWD_X0], BLK_16X16);
    h->mv[MV_BWD_X0] = ff_cavs_intra_mv;
    set_mvs(&h->mv[MV_BWD_X0], BLK_16X16);
    if (h->cur.f->pict_type != AV_PICTURE_TYPE_B)
        h->col_type_base[h->mbidx] = I_8X8;
}

static int decode_mb_i(AVSContext *h, int cbp_code)
{
    GetBitContext *gb = &h->gb;
    unsigned pred_mode_uv;
    int block;
    uint8_t top[18];
    uint8_t *left = NULL;
    uint8_t *d;
    int ret;

    ff_cavs_init_mb(h);
    h->left_mb_type        = I_8X8;
    h->top_mb_type[h->mbx] = I_8X8;

    /* get intra prediction modes from stream */
    for (block = 0; block < 4; block++) {
        int nA, nB, predpred;
        int pos = scan3x3[block];

        nA = h->pred_mode_Y[pos - 1];
        nB = h->pred_mode_Y[pos - 3];
        predpred = FFMIN(nA, nB);
        if (predpred == NOT_AVAIL) // if either is not available
            predpred = INTRA_L_LP;
        if (h->aec_enable) {
            predpred = aec_intra_luma_pred_mode(h, predpred);
        } else if (!get_bits1(gb)) {
            int rem_mode = get_bits(gb, 2);
            predpred     = rem_mode + (rem_mode >= predpred);
        }
        h->pred_mode_Y[pos] = predpred;
    }
    if (h->aec_enable) {
        pred_mode_uv = aec_intra_chroma_pred_mode(h);
    } else {
        pred_mode_uv = get_ue_golomb_31(gb);
        if (pred_mode_uv > 6) {
            av_log(h->avctx, AV_LOG_ERROR, "illegal intra chroma pred mode\n");
            return AVERROR_INVALIDDATA;
        }
    }
    ff_cavs_modify_mb_i(h, &pred_mode_uv);

    /* get coded block pattern */
    if (h->aec_enable) {
        /* under AEC cbp is present for an intra macroblock too and carries
         * MbCBP itself (table 26, clause 9.4.7) */
        h->cbp = aec_cbp(h);
    } else {
        if (h->cur.f->pict_type == AV_PICTURE_TYPE_I)
            cbp_code = get_ue_golomb(gb);
        if (cbp_code > 63U) {
            av_log(h->avctx, AV_LOG_ERROR, "illegal intra cbp\n");
            return AVERROR_INVALIDDATA;
        }
        h->cbp = cbp_tab[cbp_code][0];
    }
    if (h->aec_enable) {
        int delta = 0;

        if (h->cbp && !h->qp_fixed) {
            ret = aec_mb_qp_delta(h, &delta);
            if (ret < 0)
                return ret;
            h->qp = (h->qp + (unsigned)delta) & 63;
        }
        /* a macroblock without mb_qp_delta resets PreviousDeltaQP:
         * CurrentQP == PreviousQP means mbQpDelta 0 (clause 9.4.8) */
        h->prev_delta_qp = delta;
    } else if (h->cbp && !h->qp_fixed) {
        h->qp = (h->qp + (unsigned)get_se_golomb(gb)) & 63; //qp_delta
    }

    /* luma intra prediction interleaved with residual decode/transform/add */
    for (block = 0; block < 4; block++) {
        d = h->cy + h->luma_scan[block];
        ff_cavs_load_intra_pred_luma(h, top, &left, block);
        h->intra_pred_l[h->pred_mode_Y[scan3x3[block]]]
            (d, top, left, h->l_stride);
        if (h->cbp & (1<<block)) {
            if (h->aec_enable)
                ret = decode_residual_block_aec(h, 0, h->qp, d, h->l_stride);
            else
                ret = decode_residual_block(h, gb, intra_dec, 1, h->qp, d,
                                            h->l_stride);
            if (ret < 0)
                return ret;
        }
    }

    /* chroma intra prediction */
    ff_cavs_load_intra_pred_chroma(h);
    h->intra_pred_c[pred_mode_uv](h->cu, &h->top_border_u[h->mbx * 10],
                                  h->left_border_u, h->c_stride);
    h->intra_pred_c[pred_mode_uv](h->cv, &h->top_border_v[h->mbx * 10],
                                  h->left_border_v, h->c_stride);

    ret = decode_residual_chroma(h);
    if (ret < 0)
        return ret;
    ff_cavs_filter(h, I_8X8);
    set_mv_intra(h);
    return 0;
}

static inline void set_intra_mode_default(AVSContext *h)
{
    if (h->stream_revision > 0) {
        h->pred_mode_Y[3] =  h->pred_mode_Y[6] = NOT_AVAIL;
        h->top_pred_Y[h->mbx * 2 + 0] = h->top_pred_Y[h->mbx * 2 + 1] = NOT_AVAIL;
    } else {
        h->pred_mode_Y[3] =  h->pred_mode_Y[6] = INTRA_L_LP;
        h->top_pred_Y[h->mbx * 2 + 0] = h->top_pred_Y[h->mbx * 2 + 1] = INTRA_L_LP;
    }
}

/**
 * Read one mb_reference_index into the mv cache and turn it into a
 * reference list index (table 26, clauses 7.2.5 and 9.4.5). Table 26
 * prints its presence predicate with one parenthesis missing; this is the
 * only bracketing consistent with picture_reference_flag (clause 7.2.3.2).
 * @return the reference list index, or a negative error code
 */
static int decode_ref_index(AVSContext *h, int list, enum cavs_mv_loc nP)
{
    const int base = h->ref_base[list];
    /* NumberOfReference of one direction, clause 9.4.5 figures 15 to 20 */
    const int max  = !h->pic_structure &&
                     h->cur.f->pict_type == AV_PICTURE_TYPE_P ? 3 : 1;
    int idx = 0;

    if (!h->ref_flag &&
        (h->cur.f->pict_type == AV_PICTURE_TYPE_P ||
         (h->cur.f->pict_type == AV_PICTURE_TYPE_B && !h->pic_structure))) {
        if (h->aec_enable) {
            /* a and b of 8.4.4.2 f): the neighbouring block's index is
             * greater than 0 (unusable neighbours hold a negative one) */
            int a = h->mv[nP - 1].ref_idx > 0;
            int b = h->mv[nP - 4].ref_idx > 0;

            idx = aec_ref_index(h, a + 2 * b, max);
        } else {
            idx = get_bits(&h->gb, max > 1 ? 2 : 1);
        }
        if (idx < 0 || idx > max) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "mb_reference_index %d out of range\n", idx);
            return AVERROR_INVALIDDATA;
        }
    }
    h->mv[nP].ref     = base + idx;
    h->mv[nP].ref_idx = idx;
    return base + idx;
}

/** Decode one motion vector: under AEC its mv_diff must be read before
    ff_cavs_mv() overwrites the cache its context comes from. */
static int decode_mv(AVSContext *h, enum cavs_mv_loc nP, enum cavs_mv_loc nC,
                     enum cavs_mv_pred mode, enum cavs_block size, int ref)
{
    if (h->aec_enable && mode < MV_PRED_PSKIP) {
        int ret = aec_mv(h, nP);
        if (ret < 0)
            return ret;
    }
    ff_cavs_mv(h, nP, nC, mode, size, ref);
    return 0;
}

#define MV(nP, nC, mode, size, ref)                             \
    do {                                                        \
        ret = decode_mv(h, nP, nC, mode, size, ref);            \
        if (ret < 0)                                            \
            return ret;                                         \
    } while (0)

#define REF(dst, list, nP)                                      \
    do {                                                        \
        ret = decode_ref_index(h, list, nP);                    \
        if (ret < 0)                                            \
            return ret;                                         \
        dst = ret;                                              \
    } while (0)

/** Derive WeightingPrediction for one inter macroblock (clause 9.4.1).
 *
 * A macroblock produced by mb_skip_run has no macroblock syntax and thus no
 * weighting_prediction bin when mb_weighting_flag is set.
 */
static int decode_weighting_prediction(AVSContext *h, int has_mb_syntax)
{
    h->weighting_prediction = h->slice_weighting_flag;
    if (!h->slice_weighting_flag || !h->mb_weighting_flag)
        return 0;

    h->weighting_prediction = 0;
    if (!has_mb_syntax)
        return 0;
    if (h->aec_enable) {
        if (!h->aec.gb)
            return AVERROR_INVALIDDATA;
        h->weighting_prediction = ff_cavs_aec_decode_decision(
                                      &h->aec,
                                      AEC_WEIGHTING_PREDICTION);
    } else {
        if (get_bits_left(&h->gb) < 1)
            return AVERROR_INVALIDDATA;
        h->weighting_prediction = get_bits1(&h->gb);
    }
    return 0;
}

static int decode_mb_p(AVSContext *h, enum cavs_mb mb_type, int has_mb_syntax)
{
    int ref[4];
    int ret;

    ff_cavs_init_mb(h);
    h->left_mb_type        = mb_type;
    h->top_mb_type[h->mbx] = mb_type;
    /* leave behind what a later intra macroblock's contexts read */
    h->left_c_pred_mode        = INTRA_C_LP;
    h->top_c_pred_mode[h->mbx] = INTRA_C_LP;
    if (mb_type == P_SKIP) {
        h->left_cbp        = 0;
        h->top_cbp[h->mbx] = 0;
        h->prev_delta_qp   = 0;
    }
    switch (mb_type) {
    case P_SKIP:
        MV(MV_FWD_X0, MV_FWD_C2, MV_PRED_PSKIP, BLK_16X16,
           h->ref_base[0] + h->pb_field_enhanced);
        break;
    case P_16X16:
        REF(ref[0], 0, MV_FWD_X0);
        MV(MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0]);
        break;
    case P_16X8:
        REF(ref[0], 0, MV_FWD_X0);
        REF(ref[2], 0, MV_FWD_X2);
        MV(MV_FWD_X0, MV_FWD_C2, MV_PRED_TOP,    BLK_16X8, ref[0]);
        MV(MV_FWD_X2, MV_FWD_A1, MV_PRED_LEFT,   BLK_16X8, ref[2]);
        break;
    case P_8X16:
        REF(ref[0], 0, MV_FWD_X0);
        REF(ref[1], 0, MV_FWD_X1);
        MV(MV_FWD_X0, MV_FWD_B3, MV_PRED_LEFT,     BLK_8X16, ref[0]);
        MV(MV_FWD_X1, MV_FWD_C2, MV_PRED_TOPRIGHT, BLK_8X16, ref[1]);
        break;
    case P_8X8:
        REF(ref[0], 0, MV_FWD_X0);
        REF(ref[1], 0, MV_FWD_X1);
        REF(ref[2], 0, MV_FWD_X2);
        REF(ref[3], 0, MV_FWD_X3);
        MV(MV_FWD_X0, MV_FWD_B3, MV_PRED_MEDIAN,   BLK_8X8, ref[0]);
        MV(MV_FWD_X1, MV_FWD_C2, MV_PRED_MEDIAN,   BLK_8X8, ref[1]);
        MV(MV_FWD_X2, MV_FWD_X1, MV_PRED_MEDIAN,   BLK_8X8, ref[2]);
        MV(MV_FWD_X3, MV_FWD_X0, MV_PRED_MEDIAN,   BLK_8X8, ref[3]);
    }
    ret = decode_weighting_prediction(h, has_mb_syntax);
    if (ret < 0)
        return ret;
    ret = ff_cavs_inter(h, mb_type);
    if (ret < 0)
        return ret;
    set_intra_mode_default(h);
    store_mvs(h);
    if (mb_type != P_SKIP) {
        ret = decode_residual_inter(h);
        if (ret < 0)
            return ret;
    }
    ff_cavs_filter(h, mb_type);
    h->col_type_base[h->mbidx] = mb_type;
    return 0;
}

/**
 * Decode one macroblock of a P coded macroblock layer, table 25.
 *
 * @param skip_count what is left of the current mb_skip_run, or -1 when the
 *                   next macroblock position starts a new one
 */
static int decode_mb_p_layer(AVSContext *h, int *skip_count)
{
    enum cavs_mb mb_type;
    int ret;

    if (h->skip_mode_flag && *skip_count < 0) {
        if (h->aec_enable) {
            *skip_count = aec_mb_skip_run(h);
        } else {
            if (get_bits_left(&h->gb) < 1)
                return AVERROR_INVALIDDATA;
            *skip_count = get_ue_golomb(&h->gb);
        }
        if (*skip_count < 0)
            return AVERROR_INVALIDDATA;
    }
    if (h->skip_mode_flag && (*skip_count)--) {
        ret = decode_mb_p(h, P_SKIP, 0);
        if (ret < 0)
            return ret;
        if (h->aec_enable && !*skip_count &&
            ff_cavs_aec_decode_stuffing_bit(&h->aec))
            aec_end_of_slice(h);
        return 0;
    }

    if (h->aec_enable) {
        ret = aec_mb_type_p(h);
        if (ret < 0)
            return ret;
        mb_type = ret + P_SKIP;
    } else {
        if (get_bits_left(&h->gb) < 1)
            return AVERROR_INVALIDDATA;
        mb_type = get_ue_golomb(&h->gb) + P_SKIP + h->skip_mode_flag;
    }
    if (mb_type > P_8X8)
        ret = decode_mb_i(h, mb_type - P_8X8 - 1);
    else
        ret = decode_mb_p(h, mb_type, 1);
    if (ret < 0)
        return ret;
    if (h->aec_enable && ff_cavs_aec_decode_stuffing_bit(&h->aec))
        aec_end_of_slice(h);
    return 0;
}

static int decode_mb_b(AVSContext *h, enum cavs_mb mb_type, int has_mb_syntax)
{
    int block;
    enum cavs_sub_mb sub_type[4];
    int flags;
    int ref[2][4];
    int ret;

    ff_cavs_init_mb(h);
    h->left_mb_type        = mb_type;
    h->top_mb_type[h->mbx] = mb_type;
    /* an inter macroblock leaves the neighbour state an AEC context model
     * reads exactly as decode_mb_p() does */
    h->left_c_pred_mode        = INTRA_C_LP;
    h->top_c_pred_mode[h->mbx] = INTRA_C_LP;
    if (mb_type == B_SKIP) {
        h->left_cbp        = 0;
        h->top_cbp[h->mbx] = 0;
        h->prev_delta_qp   = 0;
    }

    /* reset all MVs */
    h->mv[MV_FWD_X0] = ff_cavs_dir_mv;
    set_mvs(&h->mv[MV_FWD_X0], BLK_16X16);
    h->mv[MV_BWD_X0] = ff_cavs_dir_mv;
    set_mvs(&h->mv[MV_BWD_X0], BLK_16X16);
    switch (mb_type) {
    case B_SKIP:
    case B_DIRECT:
    {
        int col_mbidx = direct_col_mbidx(h);

        if (!h->col_type_base[col_mbidx]) {
            /* intra MB at co-location: in-plane prediction with the
             * default reference of each direction (9.9.1 b) 1)) */
            ff_cavs_mv(h, MV_FWD_X0, MV_FWD_C2, MV_PRED_BSKIP, BLK_16X16,
                       h->ref_base[0]);
            ff_cavs_mv(h, MV_BWD_X0, MV_BWD_C2, MV_PRED_BSKIP, BLK_16X16,
                       h->ref_base[1]);
        } else
            /* direct prediction from co-located P MB, block-wise */
            for (block = 0; block < 4; block++) {
                ret = mv_pred_direct(h, &h->mv[mv_scan[block]],
                                     &h->col_mv[col_mbidx * 4 + block]);
                if (ret < 0)
                    return ret;
            }
        break;
    }
    case B_FWD_16X16:
        REF(ref[0][0], 0, MV_FWD_X0);
        MV(MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0][0]);
        break;
    case B_SYM_16X16:
        REF(ref[0][0], 0, MV_FWD_X0);
        MV(MV_FWD_X0, MV_FWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[0][0]);
        mv_pred_sym(h, &h->mv[MV_FWD_X0], BLK_16X16);
        break;
    case B_BWD_16X16:
        REF(ref[1][0], 1, MV_BWD_X0);
        MV(MV_BWD_X0, MV_BWD_C2, MV_PRED_MEDIAN, BLK_16X16, ref[1][0]);
        break;
    case B_8X8:
#define TMP_UNUSED_INX  7
        flags = 0;
        for (block = 0; block < 4; block++) {
            if (h->aec_enable)
                sub_type[block] = aec_mb_part_type(h);
            else
                sub_type[block] = get_bits(&h->gb, 2);
        }
        /* all forward reference indices come first (table 26); a
         * symmetric block codes only its forward one */
        for (block = 0; block < 4; block++)
            if (sub_type[block] == B_SUB_FWD || sub_type[block] == B_SUB_SYM)
                REF(ref[0][block], 0, mv_scan[block]);
        for (block = 0; block < 4; block++)
            if (sub_type[block] == B_SUB_BWD)
                REF(ref[1][block], 1, mv_scan[block] + MV_BWD_OFFS);
        for (block = 0; block < 4; block++) {
            switch (sub_type[block]) {
            case B_SUB_DIRECT:
            {
                int col_mbidx = direct_col_mbidx(h);

                if (!h->col_type_base[col_mbidx]) {
                    /* intra MB at co-location, do in-plane prediction */
                    if(flags==0) {
                        // if col-MB is a Intra MB, current Block size is 16x16.
                        // AVS standard section 9.9.1
                        if(block>0){
                            h->mv[TMP_UNUSED_INX              ] = h->mv[MV_FWD_X0              ];
                            h->mv[TMP_UNUSED_INX + MV_BWD_OFFS] = h->mv[MV_FWD_X0 + MV_BWD_OFFS];
                        }
                        ff_cavs_mv(h, MV_FWD_X0, MV_FWD_C2,
                                   MV_PRED_BSKIP, BLK_8X8, h->ref_base[0]);
                        ff_cavs_mv(h, MV_FWD_X0+MV_BWD_OFFS,
                                   MV_FWD_C2+MV_BWD_OFFS,
                                   MV_PRED_BSKIP, BLK_8X8, h->ref_base[1]);
                        if(block>0) {
                            flags = mv_scan[block];
                            h->mv[flags              ] = h->mv[MV_FWD_X0              ];
                            h->mv[flags + MV_BWD_OFFS] = h->mv[MV_FWD_X0 + MV_BWD_OFFS];
                            h->mv[MV_FWD_X0              ] = h->mv[TMP_UNUSED_INX              ];
                            h->mv[MV_FWD_X0 + MV_BWD_OFFS] = h->mv[TMP_UNUSED_INX + MV_BWD_OFFS];
                        } else
                            flags = MV_FWD_X0;
                    } else {
                        h->mv[mv_scan[block]              ] = h->mv[flags              ];
                        h->mv[mv_scan[block] + MV_BWD_OFFS] = h->mv[flags + MV_BWD_OFFS];
                    }
                } else {
                    ret = mv_pred_direct(h, &h->mv[mv_scan[block]],
                                         &h->col_mv[col_mbidx * 4 + block]);
                    if (ret < 0)
                        return ret;
                }
                break;
            }
            case B_SUB_FWD:
                MV(mv_scan[block], mv_scan[block] - 3,
                   MV_PRED_MEDIAN, BLK_8X8, ref[0][block]);
                break;
            case B_SUB_SYM:
                MV(mv_scan[block], mv_scan[block] - 3,
                   MV_PRED_MEDIAN, BLK_8X8, ref[0][block]);
                mv_pred_sym(h, &h->mv[mv_scan[block]], BLK_8X8);
                break;
            }
        }
#undef TMP_UNUSED_INX
        for (block = 0; block < 4; block++) {
            if (sub_type[block] == B_SUB_BWD)
                MV(mv_scan[block] + MV_BWD_OFFS,
                   mv_scan[block] + MV_BWD_OFFS - 3,
                   MV_PRED_MEDIAN, BLK_8X8, ref[1][block]);
        }
        break;
    default:
        if (mb_type <= B_SYM_16X16) {
            av_log(h->avctx, AV_LOG_ERROR, "Invalid mb_type %d in B frame\n", mb_type);
            return AVERROR_INVALIDDATA;
        }
        av_assert2(mb_type < B_8X8);
        flags = ff_cavs_partition_flags[mb_type];
        if (mb_type & 1) { /* 16x8 macroblock types */
            if (flags & FWD0)
                REF(ref[0][0], 0, MV_FWD_X0);
            if (flags & FWD1)
                REF(ref[0][2], 0, MV_FWD_X2);
            if (flags & BWD0)
                REF(ref[1][0], 1, MV_BWD_X0);
            if (flags & BWD1)
                REF(ref[1][2], 1, MV_BWD_X2);
            if (flags & FWD0)
                MV(MV_FWD_X0, MV_FWD_C2, MV_PRED_TOP,  BLK_16X8, ref[0][0]);
            if (flags & SYM0)
                mv_pred_sym(h, &h->mv[MV_FWD_X0], BLK_16X8);
            if (flags & FWD1)
                MV(MV_FWD_X2, MV_FWD_A1, MV_PRED_LEFT, BLK_16X8, ref[0][2]);
            if (flags & SYM1)
                mv_pred_sym(h, &h->mv[MV_FWD_X2], BLK_16X8);
            if (flags & BWD0)
                MV(MV_BWD_X0, MV_BWD_C2, MV_PRED_TOP,  BLK_16X8, ref[1][0]);
            if (flags & BWD1)
                MV(MV_BWD_X2, MV_BWD_A1, MV_PRED_LEFT, BLK_16X8, ref[1][2]);
        } else {          /* 8x16 macroblock types */
            if (flags & FWD0)
                REF(ref[0][0], 0, MV_FWD_X0);
            if (flags & FWD1)
                REF(ref[0][1], 0, MV_FWD_X1);
            if (flags & BWD0)
                REF(ref[1][0], 1, MV_BWD_X0);
            if (flags & BWD1)
                REF(ref[1][1], 1, MV_BWD_X1);
            if (flags & FWD0)
                MV(MV_FWD_X0, MV_FWD_B3, MV_PRED_LEFT, BLK_8X16, ref[0][0]);
            if (flags & SYM0)
                mv_pred_sym(h, &h->mv[MV_FWD_X0], BLK_8X16);
            if (flags & FWD1)
                MV(MV_FWD_X1, MV_FWD_C2, MV_PRED_TOPRIGHT, BLK_8X16, ref[0][1]);
            if (flags & SYM1)
                mv_pred_sym(h, &h->mv[MV_FWD_X1], BLK_8X16);
            if (flags & BWD0)
                MV(MV_BWD_X0, MV_BWD_B3, MV_PRED_LEFT, BLK_8X16, ref[1][0]);
            if (flags & BWD1)
                MV(MV_BWD_X1, MV_BWD_C2, MV_PRED_TOPRIGHT, BLK_8X16, ref[1][1]);
        }
    }
    ret = decode_weighting_prediction(h, has_mb_syntax);
    if (ret < 0)
        return ret;
    ret = ff_cavs_inter(h, mb_type);
    if (ret < 0)
        return ret;
    set_intra_mode_default(h);
    if (mb_type != B_SKIP) {
        ret = decode_residual_inter(h);
        if (ret < 0)
            return ret;
    }
    ff_cavs_filter(h, mb_type);

    return 0;
}

#undef MV
#undef REF

/**
 * Decode one macroblock of a B coded macroblock layer, tables 25 and 26.
 * @param skip_count what is left of the current mb_skip_run, or -1 when the
 *                   next macroblock position starts a new one
 */
static int decode_mb_b_layer(AVSContext *h, int *skip_count)
{
    enum cavs_mb mb_type;
    int ret;

    if (h->skip_mode_flag && *skip_count < 0) {
        if (h->aec_enable) {
            *skip_count = aec_mb_skip_run(h);
        } else {
            if (get_bits_left(&h->gb) < 1)
                return AVERROR_INVALIDDATA;
            *skip_count = get_ue_golomb(&h->gb);
        }
        if (*skip_count < 0)
            return AVERROR_INVALIDDATA;
    }
    if (h->skip_mode_flag && (*skip_count)--) {
        ret = decode_mb_b(h, B_SKIP, 0);
        if (ret < 0)
            return ret;
        if (h->aec_enable && !*skip_count &&
            ff_cavs_aec_decode_stuffing_bit(&h->aec))
            aec_end_of_slice(h);
        return 0;
    }

    if (h->aec_enable) {
        ret = aec_mb_type_b(h);
        if (ret < 0)
            return ret;
        mb_type = ret + B_SKIP;
    } else {
        if (get_bits_left(&h->gb) < 1)
            return AVERROR_INVALIDDATA;
        mb_type = get_ue_golomb(&h->gb) + B_SKIP + h->skip_mode_flag;
    }
    if (mb_type > B_8X8)
        ret = decode_mb_i(h, mb_type - B_8X8 - 1);
    else
        ret = decode_mb_b(h, mb_type, 1);
    if (ret < 0)
        return ret;
    if (h->aec_enable && ff_cavs_aec_decode_stuffing_bit(&h->aec))
        aec_end_of_slice(h);
    return 0;
}

/*****************************************************************************
 *
 * slice level
 *
 ****************************************************************************/

static inline int decode_slice_header(AVSContext *h, GetBitContext *gb)
{
    int mby = h->stc;
    int i;

    if (h->stc > 0xAF)
        av_log(h->avctx, AV_LOG_ERROR, "unexpected start code 0x%02x\n", h->stc);

    /* pictures over 2800 lines carry the high bits of MbRow separately
     * (table 25, clause 7.2.4) */
    if (h->height > 2800)
        mby += get_bits(gb, 3) << 7; //slice_vertical_position_extension

    if (mby >= h->mb_height) {
        av_log(h->avctx, AV_LOG_ERROR, "MbRow %d is too large\n", mby);
        return AVERROR_INVALIDDATA;
    }

    h->mby   = mby;
    h->mbidx = h->mby * h->mb_width;
    /* the second field's first slice jumps to MbHeight / 2 (clause 3.24) */
    ff_cavs_set_mb_row(h);

    /* mark top macroblocks as unavailable */
    h->flags &= ~(B_AVAIL | C_AVAIL);
    if (!h->pic_qp_fixed) {
        if (get_bits_left(gb) < 7)
            return AVERROR_INVALIDDATA;
        h->qp_fixed = get_bits1(gb);
        h->qp       = get_bits(gb, 6);
    }
    h->slice_weighting_flag = 0;
    h->mb_weighting_flag    = 0;
    for (i = 0; i < FF_ARRAY_ELEMS(h->luma_scale); i++) {
        h->luma_scale[i] = h->chroma_scale[i] = 32;
        h->luma_shift[i] = h->chroma_shift[i] = 0;
    }
    /* second field slices of an intra picture carry the flag too */
    if ((h->cur.f->pict_type != AV_PICTURE_TYPE_I) ||
        (!h->pic_structure && h->mbidx >= h->mb_width * h->mb_height / 2)) {
        int nref;

        if (get_bits_left(gb) < 1)
            return AVERROR_INVALIDDATA;
        h->slice_weighting_flag = get_bits1(gb);
        if (h->slice_weighting_flag) {
            /* NumberOfReference, clause 9.4.5 */
            nref = h->cur.f->pict_type == AV_PICTURE_TYPE_I ? 1 :
                   h->pic_structure ? 2 : 4;
            if (get_bits_left(gb) < nref * 34 + 1)
                return AVERROR_INVALIDDATA;

            for (i = 0; i < nref; i++) {
                h->luma_scale[i] = get_bits(gb, 8);
                h->luma_shift[i] = sign_extend(get_bits(gb, 8), 8);
                if (!get_bits1(gb)) {
                    av_log(h->avctx, AV_LOG_ERROR,
                           "invalid luma weighting marker bit\n");
                    return AVERROR_INVALIDDATA;
                }
                h->chroma_scale[i] = get_bits(gb, 8);
                h->chroma_shift[i] = sign_extend(get_bits(gb, 8), 8);
                if (!get_bits1(gb)) {
                    av_log(h->avctx, AV_LOG_ERROR,
                           "invalid chroma weighting marker bit\n");
                    return AVERROR_INVALIDDATA;
                }
            }
            h->mb_weighting_flag = get_bits1(gb);
        }
    }
    return 0;
}

/**
 * Tell whether the byte aligned 00 00 01 at @p offset is a start code: in
 * a raw payload it always is (annex A), in a de-emulated one it may be a
 * coincidence, so only the offsets recorded during removal are trusted.
 */
static int is_start_code_offset(AVSContext *h, int offset)
{
    int i;

    if (h->gb.buffer != h->deemulated_buf)
        return 1;
    for (i = 0; i < h->nb_stc_offset; i++)
        if (h->stc_offset[i] == offset)
            return 1;
    return 0;
}

/**
 * Look for a slice start code at the current position and parse its header.
 * @return 1 when a new slice was entered, 0 otherwise, a negative error
 *         code when its header is invalid
 */
static inline int check_for_slice(AVSContext *h)
{
    GetBitContext *gb = &h->gb;
    int align;
    int ret;

    if (h->mbx)
        return 0;
    /* until aec_mb_stuffing_bit has ended an AEC slice (7.2.4) the bytes
     * at the reader's position are arithmetic code, and a byte aligned
     * 00 00 01 in them is a coincidence; aec_end_of_slice() releases gb */
    if (h->aec_enable && h->aec.gb)
        return 0;
    align = (-get_bits_count(gb)) & 7;
    /* check for stuffing byte */
    if (!align && (show_bits(gb, 8) == 0x80))
        align = 8;
    if ((show_bits_long(gb, 24 + align) & 0xFFFFFF) == 0x000001) {
        if (!is_start_code_offset(h, (get_bits_count(gb) + align) >> 3))
            return 0;
        skip_bits_long(gb, 24 + align);
        h->stc = get_bits(gb, 8);
        if (h->stc >= h->mb_height)
            return 0;
        ret = decode_slice_header(h, gb);
        if (ret < 0)
            return ret;
        if (h->aec_enable) {
            /* aec_byte_alignment_bit padding, then 8.4.1 */
            align_get_bits(gb);
            ff_cavs_aec_init(&h->aec, gb);
            h->prev_delta_qp = 0;   //reset with PreviousQP, clause 9.3
        }
        return 1;
    }
    return 0;
}

/*****************************************************************************
 *
 * frame level
 *
 ****************************************************************************/

/**
 * Build the reference list and the temporal distances of the current
 * frame or field (clause 9.4.5 figures 14 to 20, clause 9.4.6.1); cur.poc
 * is 2 * picture_distance, the DistanceIndex of a first field.
 */
static int set_ref_list(AVSContext *h)
{
    /* the first coded field is the first displayed one (clause 7.2.3.1),
     * and annex B.2 b) keeps top_field_first constant over the sequence */
    const int f1   = !h->top_field_first;   /* parity of a first field  */
    const int f2   =  h->top_field_first;   /* parity of a second field */
    const int b    = h->cur.f->pict_type == AV_PICTURE_TYPE_B;
    const int poc  = h->cur.poc;
    const int poc0 = h->DPB[0].poc;
    const int poc1 = h->DPB[1].poc;
    int i, nref;

#define REF(idx, frame, par, distance)          \
    do {                                        \
        h->ref[idx].f      = (frame);           \
        h->ref[idx].parity = (par);             \
        h->dist[idx]       = (distance) & 511;  \
    } while (0)

    h->ref_base[0] = h->ref_base[1] = 0;

    if (h->pic_structure) {
        /* figures 15 and 18: whole frames, most recent first; the
         * unreachable entries repeat DPB[1] to stay inside the DPB */
        REF(0, h->DPB[0].f, 0, b ? poc0 - poc : poc - poc0);
        for (i = 1; i < FF_ARRAY_ELEMS(h->ref); i++)
            REF(i, h->DPB[1].f, 0, poc - poc1);
        /* figure 18: a B frame picture predicts forward from DPB[1] and
         * backward from DPB[0], and codes no reference index at all */
        h->ref_base[0] = b;
        nref = 1;
    } else if (h->cur.f->pict_type == AV_PICTURE_TYPE_I) {
        /* figure 14: the second field of an intra picture references its
         * own first field, one field period away */
        for (i = 0; i < FF_ARRAY_ELEMS(h->ref); i++)
            REF(i, h->cur.f, f1, 1);
        nref = 1;
    } else if (!b) {
        if (!h->field) {
            /* figure 16: the four fields of the two previous frames */
            REF(0, h->DPB[0].f, f2, poc - poc0 - 1);
            REF(1, h->DPB[0].f, f1, poc - poc0);
            REF(2, h->DPB[1].f, f2, poc - poc1 - 1);
            REF(3, h->DPB[1].f, f1, poc - poc1);
        } else {
            /* figure 17: the first field of the picture itself, then the
             * three nearest fields before it */
            REF(0, h->cur.f,    f1, 1);
            REF(1, h->DPB[0].f, f2, poc - poc0);
            REF(2, h->DPB[0].f, f1, poc - poc0 + 1);
            REF(3, h->DPB[1].f, f2, poc - poc1);
        }
        nref = 4;
    } else {
        /* figures 19 and 20: two backward and two forward fields, each
         * direction nearest field first */
        REF(0, h->DPB[0].f, f1, poc0     - poc - h->field);
        REF(1, h->DPB[0].f, f2, poc0 + 1 - poc - h->field);
        REF(2, h->DPB[1].f, f2, poc + h->field - poc1 - 1);
        REF(3, h->DPB[1].f, f1, poc + h->field - poc1);
        h->ref_base[0] = 2;
        nref = 2;
    }
#undef REF

    for (i = 0; i < FF_ARRAY_ELEMS(h->ref); i++)
        h->scale_den[i] = h->dist[i] ? 512 / h->dist[i] : 0;

    /* bound the symmetric scaling factor of clause 9.9.1 c) for every
     * reference pair the picture can use */
    for (i = 0; b && i < nref; i++) {
        int fw     = h->ref_base[0] + i;
        int bw     = h->pic_structure ? h->ref_base[1]
                                      : h->ref_base[1] + 1 - i;
        int factor = h->dist[bw] * h->scale_den[fw];

        if (factor > 32768) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "symmetric scaling factor %d too large\n", factor);
            return AVERROR_INVALIDDATA;
        }
    }
    return 0;
}

static int get_se_golomb_checked(GetBitContext *gb, int *value)
{
    unsigned int suffix = 0;
    unsigned int code;
    int zeros = 0;

    while (1) {
        if (get_bits_left(gb) < 1)
            return AVERROR_INVALIDDATA;
        if (get_bits1(gb))
            break;
        if (++zeros >= 31)
            return AVERROR_INVALIDDATA;
    }
    if (get_bits_left(gb) < zeros)
        return AVERROR_INVALIDDATA;
    if (zeros)
        suffix = get_bits_long(gb, zeros);

    code = ((1U << zeros) - 1) + suffix;
    *value = code & 1 ? (int)(code / 2) + 1 : -(int)(code / 2);
    return 0;
}

static int decode_weighting_quant(AVSContext *h)
{
    int delta[2][6] = { 0 };
    int parameters[6] = { 128, 128, 128, 128, 128, 128 };
    int chroma_qp_disable;
    int param_index;
    int model;
    int selected = -1;

    h->weighting_quant   = 0;
    h->chroma_qp_delta[0] = 0;
    h->chroma_qp_delta[1] = 0;
    if (get_bits_left(&h->gb) < 1)
        return AVERROR_INVALIDDATA;
    if (!get_bits1(&h->gb))
        return 0;

    h->weighting_quant = 1;
    if (get_bits_left(&h->gb) < 5)
        return AVERROR_INVALIDDATA;
    skip_bits1(&h->gb); /* reserved_bits r(1), ignored per clause 5.8.4 */
    chroma_qp_disable = get_bits1(&h->gb);
    if (!chroma_qp_disable) {
        if (get_se_golomb_checked(&h->gb, &h->chroma_qp_delta[0]) < 0 ||
            get_se_golomb_checked(&h->gb, &h->chroma_qp_delta[1]) < 0)
            return AVERROR_INVALIDDATA;
        if (h->chroma_qp_delta[0] < -32 || h->chroma_qp_delta[0] > 31 ||
            h->chroma_qp_delta[1] < -32 || h->chroma_qp_delta[1] > 31)
            return AVERROR_INVALIDDATA;
    }
    if (get_bits_left(&h->gb) < 4)
        return AVERROR_INVALIDDATA;
    param_index = get_bits(&h->gb, 2);
    model       = get_bits(&h->gb, 2);
    if (param_index == 3 || model == 3)
        return AVERROR_INVALIDDATA;

    if (param_index == 1) {
        for (int i = 0; i < 6; i++) {
            if (get_se_golomb_checked(&h->gb, &delta[0][i]) < 0 ||
                delta[0][i] < -128 || delta[0][i] > 127)
                return AVERROR_INVALIDDATA;
        }
    }
    if (param_index == 2) {
        for (int i = 0; i < 6; i++) {
            if (get_se_golomb_checked(&h->gb, &delta[1][i]) < 0 ||
                delta[1][i] < -128 || delta[1][i] > 127)
                return AVERROR_INVALIDDATA;
        }
    }

    switch (param_index) {
    case 0:
        selected = 1;
        break;
    case 1:
        selected = 0;
        break;
    case 2:
        selected = 1;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (selected >= 0) {
        for (int i = 0; i < 6; i++) {
            int value = weighting_quant_default[selected][i] +
                        delta[selected][i];

            if (value < 0 || value > UINT8_MAX)
                return AVERROR_INVALIDDATA;
            parameters[i] = value;
        }
    }

    for (int i = 0; i < 64; i++)
        h->weighting_quant_matrix[i] =
            parameters[weighting_quant_model[model][i]];

    return 0;
}

static void cavs_frame_unref(AVSFrame *frame)
{
    av_frame_unref(frame->f);
    av_refstruct_unref(&frame->hwaccel_picture_private);
    frame->poc = 0;
}

static void cavs_rotate_dpb(AVSContext *h)
{
    cavs_frame_unref(&h->DPB[2]);
    FFSWAP(AVSFrame, h->cur, h->DPB[2]);
    FFSWAP(AVSFrame, h->DPB[1], h->DPB[2]);
    FFSWAP(AVSFrame, h->DPB[0], h->DPB[1]);
}

static int decode_pic(AVSContext *h)
{
    int ret;
    int field;
    int skip_count    = -1;

    if (!h->top_qp) {
        av_log(h->avctx, AV_LOG_ERROR, "No sequence header decoded yet\n");
        return AVERROR_INVALIDDATA;
    }

    cavs_frame_unref(&h->cur);

    skip_bits(&h->gb, 16);//bbv_delay
    if (h->profile == AV_PROFILE_CAVS_GUANGDIAN)
        skip_bits(&h->gb, 8); //marker_bit, bbv_delay_extension (tables 21, 22)
    if (h->stc == PIC_PB_START_CODE) {
        h->cur.f->pict_type = get_bits(&h->gb, 2) + AV_PICTURE_TYPE_I;
        if (h->cur.f->pict_type > AV_PICTURE_TYPE_B) {
            av_log(h->avctx, AV_LOG_ERROR, "illegal picture type\n");
            return AVERROR_INVALIDDATA;
        }
        /* make sure we have the reference frames we need */
        if (!h->DPB[0].f->data[0] ||
           (!h->DPB[1].f->data[0] && h->cur.f->pict_type == AV_PICTURE_TYPE_B))
            return AVERROR_INVALIDDATA;
    } else {
        h->cur.f->pict_type = AV_PICTURE_TYPE_I;
        if (get_bits1(&h->gb))
            skip_bits(&h->gb, 24);//time_code
        /* unconditional marker bit after time_code in this profile */
        if (h->profile == AV_PROFILE_CAVS_GUANGDIAN)
            h->stream_revision = 1;
        /* old sample clips were all progressive and no low_delay,
           bump stream revision if detected otherwise */
        else if (h->low_delay || !(show_bits(&h->gb, 9) & 1))
            h->stream_revision = 1;
        /* similarly test top_field_first and repeat_first_field */
        else if (show_bits(&h->gb, 11) & 3)
            h->stream_revision = 1;
        if (h->stream_revision > 0)
            skip_bits(&h->gb, 1); //marker_bit
    }

    if (get_bits_left(&h->gb) < 23)
        return AVERROR_INVALIDDATA;

    ret = ff_get_buffer(h->avctx, h->cur.f, h->cur.f->pict_type == AV_PICTURE_TYPE_B ?
                        0 : AV_GET_BUFFER_FLAG_REF);
    if (ret < 0)
        return ret;

    if (!h->edge_emu_buffer) {
        int alloc_size = FFALIGN(FFABS(h->cur.f->linesize[0]) + 32, 32);
        h->edge_emu_buffer = av_mallocz(alloc_size * 2 * 24);
        if (!h->edge_emu_buffer)
            return AVERROR(ENOMEM);
    }

    /* picture_distance; DistanceIndex is twice it (clause 9.4.6.1) */
    h->cur.poc = get_bits(&h->gb, 8) * 2;

    if (h->low_delay)
        get_ue_golomb(&h->gb); //bbv_check_times
    h->progressive   = get_bits1(&h->gb);
    h->pic_structure = 1;
    if (!h->progressive)
        h->pic_structure = get_bits1(&h->gb);
    /* clause 7.2.3: a progressive sequence has frame pictures only. Its
     * odd MbHeight would also make a field picture write 16 lines past
     * coded_height, so this is load bearing, not only conformance. */
    if (!h->pic_structure && h->progressive_seq) {
        av_log(h->avctx, AV_LOG_ERROR,
               "field picture in a progressive sequence\n");
        return AVERROR_INVALIDDATA;
    }
    /* a field coded picture uses inverse block scan method 2 (9.5.3 c)) */
    h->scantable = h->pic_structure ? h->permutated_scantable
                                    : h->permutated_scantable_field;
    h->weighting_scan = h->pic_structure ? h->weighting_scantable
                                         : h->weighting_scantable_field;
    if (!h->pic_structure && h->stc == PIC_PB_START_CODE)
        skip_bits1(&h->gb);     //advanced_pred_mode_disable
    h->top_field_first = get_bits1(&h->gb);
    skip_bits1(&h->gb);        //repeat_first_field
    h->pic_qp_fixed =
    h->qp_fixed = get_bits1(&h->gb);
    h->qp       = get_bits(&h->gb, 6);
    h->pb_field_enhanced = 0;
    if (h->cur.f->pict_type == AV_PICTURE_TYPE_I) {
        /* an intra picture carries skip_mode_flag exactly when it is
         * field coded (table 21): its second field is inter coded */
        h->skip_mode_flag = 0;
        if (!h->progressive && !h->pic_structure)
            h->skip_mode_flag = get_bits1(&h->gb);
        skip_bits(&h->gb, 4);   //reserved bits
    } else {
        if (!(h->cur.f->pict_type == AV_PICTURE_TYPE_B && h->pic_structure == 1))
            h->ref_flag        = get_bits1(&h->gb);
        else
            /* 1 for a frame coded B picture (clause 7.2.3.2) */
            h->ref_flag        = 1;
        /* no_forward_reference_flag (no effect on decoding),
         * pb_field_enhanced_flag, reserved r(2); table 22 */
        skip_bits1(&h->gb);
        h->pb_field_enhanced = get_bits1(&h->gb);
        skip_bits(&h->gb, 2);
        h->skip_mode_flag      = get_bits1(&h->gb);
    }
    h->loop_filter_disable     = get_bits1(&h->gb);
    if (!h->loop_filter_disable && get_bits1(&h->gb)) {
        h->alpha_offset        = get_se_golomb(&h->gb);
        h->beta_offset         = get_se_golomb(&h->gb);
        if (   h->alpha_offset < -64 || h->alpha_offset > 64
            || h-> beta_offset < -64 || h-> beta_offset > 64) {
            h->alpha_offset = h->beta_offset  = 0;
            return AVERROR_INVALIDDATA;
        }
    } else {
        h->alpha_offset = h->beta_offset  = 0;
    }

    /* AWQ parameters and aec_enable end the header (tables 21, 22) */
    h->aec_enable = 0;
    if (h->profile == AV_PROFILE_CAVS_GUANGDIAN) {
        ret = decode_weighting_quant(h);
        if (ret < 0)
            return ret;
        if (get_bits_left(&h->gb) < 1)
            return AVERROR_INVALIDDATA;
        h->aec_enable = get_bits1(&h->gb);
    } else {
        h->weighting_quant = 0;
        h->chroma_qp_delta[0] = 0;
        h->chroma_qp_delta[1] = 0;
    }

    if (h->pb_field_enhanced && h->pic_structure) {
        av_log(h->avctx, AV_LOG_ERROR,
               "pb_field_enhanced_flag set on a frame-coded picture\n");
        return AVERROR_INVALIDDATA;
    }
    /* Mapping an enhanced B field to a frame-coded future anchor requires a
     * different co-located block transform. Keep that combination explicit
     * until it is implemented rather than silently applying field mapping. */
    if (h->pb_field_enhanced &&
        h->cur.f->pict_type == AV_PICTURE_TYPE_B &&
        !(h->DPB[0].f->flags & AV_FRAME_FLAG_INTERLACED)) {
        avpriv_request_sample(h->avctx,
                              "Enhanced B field with frame-coded anchor");
        return AVERROR_PATCHWELCOME;
    }

    if (!h->pic_structure) {
        h->cur.f->flags |= AV_FRAME_FLAG_INTERLACED;
        if (h->top_field_first)
            h->cur.f->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;
    }

    /* the strides and sample pointers depend on picture_structure, so
     * this can only run once the header has been walked to its end */
    if ((ret = ff_cavs_init_pic(h)) < 0)
        return ret;
    if ((ret = set_ref_list(h)) < 0)
        return ret;

    field = h->field;
    ret   = 0;
    h->aec.gb = NULL;   /* started by the first slice header */
    do {
        ret = check_for_slice(h);
        if (ret < 0)
            break;
        if (ret)
            skip_count = -1;
        if (h->aec_enable && !h->aec.gb) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "AEC picture without a slice header\n");
            ret = AVERROR_INVALIDDATA;
            break;
        }
        /* the two fields have different references and distances */
        if (h->field != field) {
            field = h->field;
            if ((ret = set_ref_list(h)) < 0)
                break;
        }
        if (h->cur.f->pict_type == AV_PICTURE_TYPE_B) {
            ret = decode_mb_b_layer(h, &skip_count);
        } else if (h->cur.f->pict_type == AV_PICTURE_TYPE_P || h->field) {
            /* the second field of an intra picture is inter coded
             * (clause 9.4.2 a) 2), note to clause 3.80) */
            ret = decode_mb_p_layer(h, &skip_count);
        } else {
            ret = decode_mb_i(h, 0);
            /* aec_mb_stuffing_bit is 1 on the slice's last MB (7.2.4) */
            if (ret >= 0 && h->aec_enable &&
                ff_cavs_aec_decode_stuffing_bit(&h->aec))
                aec_end_of_slice(h);
        }
        if (ret < 0)
            break;
    } while (ff_cavs_next_mb(h));
    emms_c();
    if (ret >= 0 && h->cur.f->pict_type != AV_PICTURE_TYPE_B)
        cavs_rotate_dpb(h);
    return ret;
}

/*****************************************************************************
 *
 * headers and interface
 *
 ****************************************************************************/

static int decode_seq_header(AVSContext *h)
{
    int frame_rate_code;
    int width, height;
    int mb_width, mb_height;
    int chroma_format, sample_precision;
    int ret;

    h->profile = get_bits(&h->gb, 8);
    /* the sequence header is the same for both profiles (table 14) */
    if (h->profile != AV_PROFILE_CAVS_JIZHUN &&
        h->profile != AV_PROFILE_CAVS_GUANGDIAN) {
        avpriv_report_missing_feature(h->avctx, "Profile %#x", h->profile);
        return AVERROR_PATCHWELCOME;
    }
    h->avctx->profile = h->profile;
    h->level   = get_bits(&h->gb, 8);
    h->progressive_seq = get_bits1(&h->gb);

    width  = get_bits(&h->gb, 14);
    height = get_bits(&h->gb, 14);
    if ((h->width || h->height) && (h->width != width || h->height != height)) {
        avpriv_report_missing_feature(h->avctx,
                                      "Width/height changing in CAVS");
        return AVERROR_PATCHWELCOME;
    }
    if (width <= 0 || height <= 0) {
        av_log(h->avctx, AV_LOG_ERROR, "Dimensions invalid\n");
        return AVERROR_INVALIDDATA;
    }
    chroma_format    = get_bits(&h->gb, 2);
    sample_precision = get_bits(&h->gb, 3);
    /* Only 8 bit 4:2:0 is implemented; 4:2:2 changes the macroblock layer
     * (tables 25, 42). JiZhun samples predate the two fields being used in
     * earnest and keep being decoded as 8 bit 4:2:0 regardless. */
    if (h->profile == AV_PROFILE_CAVS_GUANGDIAN) {
        if (sample_precision != 1) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "sample_precision %d is not 8 bit\n", sample_precision);
            return AVERROR_INVALIDDATA;
        }
        if (chroma_format == 2) {
            avpriv_request_sample(h->avctx, "4:2:2 chroma");
            return AVERROR_PATCHWELCOME;
        }
        if (chroma_format != 1) {
            av_log(h->avctx, AV_LOG_ERROR,
                   "chroma_format %d is reserved\n", chroma_format);
            return AVERROR_INVALIDDATA;
        }
    }
    h->aspect_ratio = get_bits(&h->gb, 4);
    frame_rate_code = get_bits(&h->gb, 4);
    if (frame_rate_code == 0 || frame_rate_code > 13) {
        av_log(h->avctx, AV_LOG_WARNING,
               "frame_rate_code %d is invalid\n", frame_rate_code);
        frame_rate_code = 1;
    }

    skip_bits(&h->gb, 18); //bit_rate_lower
    skip_bits1(&h->gb);    //marker_bit
    skip_bits(&h->gb, 12); //bit_rate_upper
    h->low_delay =  get_bits1(&h->gb);

    mb_width  = (width  + 15) >> 4;
    /* an interlaced sequence is rounded up to an even number of
     * macroblock rows (GY/T 257.1-2012 clause 7.2.2) */
    mb_height = h->progressive_seq ? (height + 15) >> 4
                                   : ((height + 31) >> 5) * 2;

    /* the coded picture is the macroblock grid; horizontal_size and
     * vertical_size are the displayed part of it (clause 7.2.2) */
    ret = ff_set_dimensions(h->avctx, mb_width * 16, mb_height * 16);
    if (ret < 0)
        return ret;
    h->avctx->width  = width;
    h->avctx->height = height;

    h->width     = width;
    h->height    = height;
    h->mb_width  = mb_width;
    h->mb_height = mb_height;
    h->avctx->framerate = ff_mpeg12_frame_rate_tab[frame_rate_code];
    if (!h->top_qp)
        return ff_cavs_init_top_lines(h);
    return 0;
}

/** Tell whether annex A applies to the region a start code introduces:
    its exclusion list leaves the picture headers and the slices. */
static int annex_a_applies(int stc)
{
    return stc <= (SLICE_MAX_START_CODE & 0xFF) ||
           stc == (PIC_I_START_CODE  & 0xFF)    ||
           stc == (PIC_PB_START_CODE & 0xFF);
}

/**
 * Copy one region of a picture payload, removing the annex A escapes.
 *
 * Each escape shortens the region by two bits, which would leave the next
 * start code unaligned; the trailing stuffing of next_start_code() (clause
 * 5.8.2.4) carries no information, so it is dropped and rewritten, which
 * shortens the region by whole bytes and keeps the start code aligned.
 *
 * @param pb  destination, or NULL to only count the escapes
 * @return the number of escapes found
 */
static int deemulate_region(PutBitContext *pb, const uint8_t *buf, int size,
                            int deemulated)
{
    int escapes = 0;
    int stuffing = -1;
    int i;

    if (!deemulated) {
        for (i = 0; i < size; i++)
            if (pb)
                put_bits(pb, 8, buf[i]);
        return 0;
    }

    /* locate the stuffing bit: the last set bit of the region. A
     * conformant region cannot end in an escape (the escape exists
     * because coded data follows it), so truncating first cannot
     * hide one; in a damaged region the 02 passes through verbatim. */
    for (i = size; i > 0 && !buf[i - 1]; i--)
        ;
    if (i > 0) {
        stuffing = i - 1;
        size     = stuffing;
    }

    for (i = 0; i < size; i++) {
        if (i + 2 < size && !buf[i] && !buf[i + 1] && buf[i + 2] == 2) {
            /* drop the two low bits of the 02 (annex A) */
            if (pb) {
                put_bits(pb, 16, 0);
                put_bits(pb,  6, 0);
            }
            escapes++;
            i += 2;
        } else if (pb) {
            put_bits(pb, 8, buf[i]);
        }
    }

    if (pb) {
        if (stuffing >= 0) {
            int n = ff_ctz(buf[stuffing]) + 1;
            if (n < 8)
                put_bits(pb, 8 - n, buf[stuffing] >> n);
            put_bits(pb, 1, 1);     //stuffing_bit
        }
        align_put_bits(pb);         //stuffing_bit, until byte aligned
    }
    return escapes;
}

/**
 * Walk the payload region by region, removing the annex A escapes;
 * on the writing pass, record the offset of every start code prefix in
 * h->stc_offset for is_start_code_offset().
 * @return the number of escapes found
 */
static int deemulate_picture(AVSContext *h, PutBitContext *pb,
                             const uint8_t *buf, int size)
{
    int deemulated = 1;     /* buf starts inside a picture header */
    int escapes    = 0;
    int pos        = 0;

    while (pos < size) {
        int end;

        for (end = pos; end + 2 < size; end++)
            if (!buf[end] && !buf[end + 1] && buf[end + 2] == 1)
                break;
        if (end + 2 >= size)
            end = size;

        escapes += deemulate_region(pb, buf + pos, end - pos, deemulated);
        if (end == size)
            break;

        if (pb) {
            h->stc_offset[h->nb_stc_offset++] = put_bits_count(pb) >> 3;
            put_bits(pb, 24, 1);    //start code prefix
        }
        pos = end + 3;
        if (pos < size) {
            if (pb)
                put_bits(pb, 8, buf[pos]);
            deemulated = annex_a_applies(buf[pos]);
            pos++;
        }
    }
    return escapes;
}

/**
 * Remove the annex A pseudo start codes from a payload: in the
 * regions the annex applies to, a byte forming 00 00 02 with the two
 * before it loses its two least significant bits. Done once per picture;
 * a payload without escapes is decoded in place, with no copy.
 *
 * @param buf   payload starting right after a picture start code; it runs
 *              to the end of the packet, because the annex applies to any
 *              further pictures in it just the same
 * @return the size of h->deemulated_buf in bytes, 0 if buf has no escapes
 *         and can be decoded as it is, or a negative error code
 */
static int remove_pseudo_start_codes(AVSContext *h, const uint8_t *buf,
                                     int size)
{
    PutBitContext pb;

    if (size <= 0 || size > INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE ||
        !deemulate_picture(h, NULL, buf, size))
        return 0;

    av_fast_padded_malloc(&h->deemulated_buf, &h->deemulated_buf_size, size);
    if (!h->deemulated_buf)
        return AVERROR(ENOMEM);

    /* the payload holds fewer than size / 3 start code prefixes */
    av_fast_malloc(&h->stc_offset, &h->stc_offset_size,
                   (size / 3 + 1) * sizeof(*h->stc_offset));
    if (!h->stc_offset)
        return AVERROR(ENOMEM);
    h->nb_stc_offset = 0;

    /* removal only ever shortens the payload, so size bytes are enough */
    init_put_bits(&pb, h->deemulated_buf, size + AV_INPUT_BUFFER_PADDING_SIZE);
    deemulate_picture(h, &pb, buf, size);
    flush_put_bits(&pb);
    av_assert1(put_bytes_output(&pb) <= size);
    return put_bytes_output(&pb);
}

static const uint8_t *cavs_find_start_code(const uint8_t *buf,
                                           const uint8_t *end)
{
    while (end - buf >= 4) {
        if (!buf[0] && !buf[1] && buf[2] == 1)
            return buf;
        buf++;
    }
    return end;
}

static int cavs_configure_sequence(AVSContext *h, const uint8_t *buf,
                                   size_t size)
{
    AVCodecContext *avctx = h->avctx;
    AVSSequenceHeader sequence;
    int mb_width, mb_height;
    int grid_changed, size_changed;
    int ret;

    ret = ff_cavs_parse_sequence_header(buf, size, &sequence);
    if (ret < 0)
        return ret;
    if (h->sequence.valid &&
        (h->sequence.profile_id != sequence.profile_id ||
         h->sequence.width != sequence.width ||
         h->sequence.height != sequence.height)) {
        avpriv_report_missing_feature(avctx,
                                      "AVS sequence format changes");
        return AVERROR_PATCHWELCOME;
    }

    size_changed = (h->width || h->height) &&
                   (h->width != sequence.width ||
                    h->height != sequence.height);
    if (size_changed && avctx->hwaccel) {
        av_log(avctx, AV_LOG_ERROR,
               "CAVS hardware resolution changes require capture queue "
               "reinitialization\n");
        return AVERROR_INVALIDDATA;
    }

    mb_width  = (sequence.width + 15) >> 4;
    mb_height = sequence.flags & CAVS_SEQUENCE_FLAG_PROGRESSIVE ?
                (sequence.height + 15) >> 4 :
                ((sequence.height + 31) >> 5) * 2;
    grid_changed = h->mb_width &&
                   (h->mb_width != mb_width || h->mb_height != mb_height);

    ret = ff_set_dimensions(avctx, sequence.width, sequence.height);
    if (ret < 0)
        return ret;
    if (grid_changed) {
        ff_cavs_free_top_lines(h);
        av_freep(&h->edge_emu_buffer);
    }

    h->sequence = sequence;
    h->profile = sequence.profile_id;
    h->level = sequence.level_id;
    h->aspect_ratio = sequence.aspect_ratio;
    h->low_delay = !!(sequence.flags & CAVS_SEQUENCE_FLAG_LOW_DELAY);

    h->width = sequence.width;
    h->height = sequence.height;
    h->mb_width = mb_width;
    h->mb_height = mb_height;
    avctx->profile = sequence.profile_id;
    avctx->level = sequence.level_id;
    avctx->framerate = ff_mpeg12_frame_rate_tab[sequence.frame_rate_code];

    if (avctx->pix_fmt == AV_PIX_FMT_NONE) {
        const enum AVPixelFormat pix_fmts[] = {
#if CONFIG_CAVS_V4L2REQUEST_HWACCEL
            AV_PIX_FMT_DRM_PRIME,
#endif
            AV_PIX_FMT_YUV420P,
            AV_PIX_FMT_NONE,
        };

        avctx->pix_fmt = ff_get_format(avctx, pix_fmts);
        if (avctx->pix_fmt == AV_PIX_FMT_NONE)
            return AVERROR(EINVAL);
    }

    return 0;
}

enum CAVSSequenceScanResult {
    CAVS_SEQUENCE_SCAN_OK,
    CAVS_SEQUENCE_SCAN_BOUNDARY,
};

static int cavs_scan_sequence_headers(AVSContext *h, const uint8_t *buf,
                                      const uint8_t *end)
{
    const uint8_t *unit = cavs_find_start_code(buf, end);
    int boundary = 0;

    while (unit < end) {
        const uint8_t *next = cavs_find_start_code(unit + 4, end);
        uint8_t code = unit[3];
        int ret;

        if (code == (PIC_I_START_CODE & 0xff) ||
            code == (PIC_PB_START_CODE & 0xff)) {
            if (boundary) {
                av_log(h->avctx, AV_LOG_ERROR,
                       "AVS sequence boundary and picture share a packet\n");
                return AVERROR_INVALIDDATA;
            }
            break;
        }
        if (code == (CAVS_START_CODE & 0xff)) {
            if (boundary) {
                av_log(h->avctx, AV_LOG_ERROR,
                       "AVS sequence boundary and header share a packet\n");
                return AVERROR_INVALIDDATA;
            }
            ret = cavs_configure_sequence(h, unit + 4, next - unit - 4);
            if (ret < 0)
                return ret;
        } else if (code == (CAVS_END_CODE & 0xff) ||
                   code == (VIDEO_EDIT_START_CODE & 0xff)) {
            h->sequence.valid = 0;
            boundary = 1;
        }
        unit = next;
    }
    return boundary ? CAVS_SEQUENCE_SCAN_BOUNDARY : CAVS_SEQUENCE_SCAN_OK;
}

/** Drain the final anchor, then sever every reference at B1/B7. */
static int cavs_end_sequence_epoch(AVCodecContext *avctx, AVFrame *rframe,
                                   int *got_frame, int packet_size)
{
    AVSContext *h = avctx->priv_data;

    *got_frame = 0;
    cavs_frame_unref(&h->cur);
    if (!h->low_delay && h->DPB[0].f->data[0]) {
        av_frame_move_ref(rframe, h->DPB[0].f);
        av_refstruct_unref(&h->DPB[0].hwaccel_picture_private);
        h->DPB[0].poc = 0;
        *got_frame = 1;
    }
    for (int i = 0; i < FF_ARRAY_ELEMS(h->DPB); i++)
        cavs_frame_unref(&h->DPB[i]);
    h->got_keyframe  = 0;
    h->sequence.valid = 0;
    h->aec.gb         = NULL;
    if (FF_HW_HAS_CB(avctx, flush))
        FF_HW_SIMPLE_CALL(avctx, flush);

    return packet_size;
}

static int cavs_hw_picture_boundary(uint8_t code)
{
    return code >= 0xb0 && code != (USER_START_CODE & 0xff) &&
           code != (EXT_START_CODE & 0xff);
}

static int cavs_decode_frame_hw(AVCodecContext *avctx, AVFrame *rframe,
                                int *got_frame, AVPacket *avpkt)
{
    AVSContext *h = avctx->priv_data;
    const FFHWAccel *hwaccel = ffhwaccel(avctx->hwaccel);
    const uint8_t *buf = avpkt->data;
    const uint8_t *end = buf + avpkt->size;
    const uint8_t *picture = NULL, *picture_end = NULL;
    const uint8_t *frame_end = end;
    const uint8_t *unit;
    int intra, ret;

    *got_frame = 0;
    if (!avpkt->size) {
        if (!h->low_delay && h->DPB[0].f->data[0]) {
            av_frame_move_ref(rframe, h->DPB[0].f);
            av_refstruct_unref(&h->DPB[0].hwaccel_picture_private);
            h->DPB[0].poc = 0;
            *got_frame = 1;
        }
        return 0;
    }

    for (unit = cavs_find_start_code(buf, end); unit < end;) {
        const uint8_t *next = cavs_find_start_code(unit + 4, end);

        if (unit[3] == (PIC_I_START_CODE & 0xff) ||
            unit[3] == (PIC_PB_START_CODE & 0xff)) {
            picture = unit;
            picture_end = next;
            break;
        }
        unit = next;
    }
    if (!picture)
        return avpkt->size;
    if (!h->sequence.valid)
        return AVERROR_INVALIDDATA;

    intra = picture[3] == (PIC_I_START_CODE & 0xff);
    ret = ff_cavs_parse_picture_header(picture + 4,
                                       picture_end - picture - 4,
                                       &h->sequence, intra, &h->picture);
    if (ret < 0)
        return ret;

    if (intra && !h->got_keyframe) {
        for (int i = 0; i < FF_ARRAY_ELEMS(h->DPB); i++)
            cavs_frame_unref(&h->DPB[i]);
        h->got_keyframe = 1;
    } else if (!h->got_keyframe) {
        return picture_end - buf;
    }

    cavs_frame_unref(&h->cur);
    h->cur.f->pict_type = h->picture.picture_coding_type + AV_PICTURE_TYPE_I;
    if (intra)
        h->cur.f->flags |= AV_FRAME_FLAG_KEY;
    if (!(h->picture.flags & CAVS_PICTURE_FLAG_PROGRESSIVE_FRAME))
        h->cur.f->flags |= AV_FRAME_FLAG_INTERLACED;
    if (h->picture.flags & CAVS_PICTURE_FLAG_TOP_FIELD_FIRST)
        h->cur.f->flags |= AV_FRAME_FLAG_TOP_FIELD_FIRST;

    ret = ff_get_buffer(avctx, h->cur.f,
                        h->cur.f->pict_type == AV_PICTURE_TYPE_B ?
                        0 : AV_GET_BUFFER_FLAG_REF);
    if (ret < 0)
        return ret;
    ret = ff_hwaccel_frame_priv_alloc(avctx,
                                      &h->cur.hwaccel_picture_private);
    if (ret < 0)
        goto fail;

    ret = hwaccel->start_frame(avctx, avpkt->buf, picture,
                               picture_end - picture);
    if (ret < 0)
        goto fail;

    for (unit = picture_end; unit < end;) {
        const uint8_t *next = cavs_find_start_code(unit + 4, end);

        if (cavs_hw_picture_boundary(unit[3])) {
            frame_end = unit;
            break;
        }
        if (unit[3] <= (SLICE_MAX_START_CODE & 0xff)) {
            ret = hwaccel->decode_slice(avctx, unit, next - unit);
            if (ret < 0)
                goto fail;
        }
        unit = next;
    }

    ret = hwaccel->end_frame(avctx);
    if (ret < 0)
        goto fail;

    h->cur.poc = h->picture.picture_distance * 2;
    if (h->cur.f->pict_type != AV_PICTURE_TYPE_B) {
        cavs_rotate_dpb(h);
        if (h->DPB[!h->low_delay].f->data[0]) {
            ret = av_frame_ref(rframe, h->DPB[!h->low_delay].f);
            if (ret < 0)
                return ret;
            *got_frame = 1;
        }
    } else {
        av_frame_move_ref(rframe, h->cur.f);
        av_refstruct_unref(&h->cur.hwaccel_picture_private);
        h->cur.poc = 0;
        *got_frame = 1;
    }

    return frame_end - buf;

fail:
    cavs_frame_unref(&h->cur);
    return ret;
}

static av_cold void cavs_flush(AVCodecContext * avctx)
{
    AVSContext *h = avctx->priv_data;

    cavs_frame_unref(&h->cur);
    for (int i = 0; i < FF_ARRAY_ELEMS(h->DPB); i++)
        cavs_frame_unref(&h->DPB[i]);
    h->got_keyframe = 0;
    h->aec.gb = NULL;
    if (FF_HW_HAS_CB(avctx, flush))
        FF_HW_SIMPLE_CALL(avctx, flush);
}

static int cavs_decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                             int *got_frame, AVPacket *avpkt)
{
    AVSContext *h      = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    uint32_t stc       = -1;
    int input_size, ret;
    const uint8_t *buf_end;
    const uint8_t *buf_ptr;
    int frame_start = 0;

    if (buf_size == 0) {
        if (avctx->hwaccel)
            return cavs_decode_frame_hw(avctx, rframe, got_frame, avpkt);
        if (!h->low_delay && h->DPB[0].f->data[0]) {
            *got_frame = 1;
            av_frame_move_ref(rframe, h->DPB[0].f);
        }
        return 0;
    }

    h->stc = 0;

    buf_ptr = buf;
    buf_end = buf + buf_size;
    ret = cavs_scan_sequence_headers(h, buf, buf_end);
    if (ret < 0)
        return ret;
    if (ret == CAVS_SEQUENCE_SCAN_BOUNDARY)
        return cavs_end_sequence_epoch(avctx, rframe, got_frame, buf_size);
    if (avctx->hwaccel)
        return cavs_decode_frame_hw(avctx, rframe, got_frame, avpkt);

    for(;;) {
        buf_ptr = avpriv_find_start_code(buf_ptr, buf_end, &stc);
        if ((stc & 0xFFFFFE00) || buf_ptr == buf_end) {
            if (!h->stc)
                av_log(h->avctx, AV_LOG_WARNING, "no frame decoded\n");
            return FFMAX(0, buf_ptr - buf);
        }
        input_size = buf_end - buf_ptr;
        if ((ret = init_get_bits8(&h->gb, buf_ptr, input_size)) < 0)
            return ret;
        switch (stc) {
        case CAVS_START_CODE:
            ret = decode_seq_header(h);
            if (ret < 0)
                return ret;
            break;
        case PIC_I_START_CODE:
            if (!h->got_keyframe) {
                for (int i = 0; i < FF_ARRAY_ELEMS(h->DPB); i++)
                    cavs_frame_unref(&h->DPB[i]);
                h->got_keyframe = 1;
            }
            av_fallthrough;
        case PIC_PB_START_CODE:
            if (frame_start > 1)
                return AVERROR_INVALIDDATA;
            frame_start ++;
            if (*got_frame)
                av_frame_unref(rframe);
            *got_frame = 0;
            if (!h->got_keyframe)
                break;
            /* GY/T 257.1-2012 annex A pseudo start-code removal. */
            ret = remove_pseudo_start_codes(h, buf_ptr,
                                            (int)(buf_end - buf_ptr));
            if (ret < 0)
                return ret;
            if (ret) {
                ret = init_get_bits8(&h->gb, h->deemulated_buf, ret);
                if (ret < 0)
                    return ret;
            }
            h->stc = stc;
            if (decode_pic(h))
                break;
            *got_frame = 1;
            if (h->cur.f->pict_type != AV_PICTURE_TYPE_B) {
                if (h->DPB[!h->low_delay].f->data[0]) {
                    if ((ret = av_frame_ref(rframe, h->DPB[!h->low_delay].f)) < 0)
                        return ret;
                } else {
                    *got_frame = 0;
                }
            } else {
                av_frame_move_ref(rframe, h->cur.f);
            }
            break;
        case EXT_START_CODE:
            //mpeg_decode_extension(avctx, buf_ptr, input_size);
            break;
        case USER_START_CODE:
            //mpeg_decode_user_data(avctx, buf_ptr, input_size);
            break;
        default:
            /* decode_pic() consumes slice headers through check_for_slice(). */
            break;
        }
    }
}

const FFCodec ff_cavs_decoder = {
    .p.name         = "cavs",
    CODEC_LONG_NAME("Chinese AVS (Audio Video Standard) (AVS1-P2 JiZhun, AVS1-P16 GuangDian profiles)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CAVS,
    .priv_data_size = sizeof(AVSContext),
    .init           = ff_cavs_init,
    .close          = ff_cavs_end,
    FF_CODEC_DECODE_CB(cavs_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
    .flush          = cavs_flush,
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_cavs_profiles),
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .hw_configs     = (const AVCodecHWConfigInternal *const []) {
#if CONFIG_CAVS_V4L2REQUEST_HWACCEL
                       HWACCEL_V4L2REQUEST(cavs),
#endif
                       NULL
                   },
};
