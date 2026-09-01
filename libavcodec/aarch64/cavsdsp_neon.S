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

#include "libavutil/aarch64/asm.S"
#include "neon.S"

/*
 * One 8 point inverse transform pass over v16-v23, eight columns at a
 * time, in 16 bit lanes with wrapping arithmetic exactly like the SSE2
 * version; \bias is a vector added to a4/a5 before the >> \shift, which
 * matches the C code's rounding (its DC preload of 8 folds into the
 * second pass bias of 64 because (x + 64) >> 3 == (x >> 3) + 8).
 */
.macro cavs_idct8_1d bias, shift
        shl             v24.8h, v17.8h, #1      // 2*src1
        shl             v25.8h, v19.8h, #1      // 2*src3
        shl             v26.8h, v21.8h, #1      // 2*src5
        shl             v27.8h, v23.8h, #1      // 2*src7
        add             v28.8h, v24.8h, v17.8h  // 3*src1
        add             v29.8h, v25.8h, v19.8h  // 3*src3
        add             v30.8h, v26.8h, v21.8h  // 3*src5
        add             v31.8h, v27.8h, v23.8h  // 3*src7
        sub             v1.8h,  v28.8h, v27.8h  // a0 = 3*src1 - 2*src7
        add             v2.8h,  v29.8h, v26.8h  // a1 = 3*src3 + 2*src5
        sub             v3.8h,  v25.8h, v30.8h  // a2 = 2*src3 - 3*src5
        add             v4.8h,  v24.8h, v31.8h  // a3 = 2*src1 + 3*src7

        add             v5.8h,  v1.8h,  v2.8h
        add             v5.8h,  v5.8h,  v4.8h   // a0 + a1 + a3
        sub             v6.8h,  v1.8h,  v2.8h
        add             v6.8h,  v6.8h,  v3.8h   // a0 - a1 + a2
        sub             v24.8h, v4.8h,  v3.8h
        sub             v24.8h, v24.8h, v2.8h   // a3 - a2 - a1
        sub             v25.8h, v1.8h,  v3.8h
        sub             v25.8h, v25.8h, v4.8h   // a0 - a2 - a3
        shl             v5.8h,  v5.8h,  #1
        add             v5.8h,  v5.8h,  v2.8h   // b4
        shl             v6.8h,  v6.8h,  #1
        add             v6.8h,  v6.8h,  v1.8h   // b5
        shl             v24.8h, v24.8h, #1
        add             v24.8h, v24.8h, v4.8h   // b6
        shl             v25.8h, v25.8h, #1
        sub             v25.8h, v25.8h, v3.8h   // b7

        shl             v26.8h, v18.8h, #2      // 4*src2
        shl             v27.8h, v22.8h, #2      // 4*src6
        add             v28.8h, v26.8h, v18.8h  // 5*src2
        add             v29.8h, v27.8h, v22.8h  // 5*src6
        shl             v28.8h, v28.8h, #1      // 10*src2
        shl             v29.8h, v29.8h, #1      // 10*src6
        sub             v1.8h,  v26.8h, v29.8h  // a7 = 4*src2 - 10*src6
        add             v2.8h,  v27.8h, v28.8h  // a6 = 4*src6 + 10*src2
        add             v3.8h,  v16.8h, v20.8h
        sub             v4.8h,  v16.8h, v20.8h
        shl             v3.8h,  v3.8h,  #3
        shl             v4.8h,  v4.8h,  #3
        add             v3.8h,  v3.8h,  \bias   // a4 = 8*(src0 + src4) + bias
        add             v4.8h,  v4.8h,  \bias   // a5 = 8*(src0 - src4) + bias
        add             v26.8h, v3.8h,  v2.8h   // b0
        add             v27.8h, v4.8h,  v1.8h   // b1
        sub             v28.8h, v4.8h,  v1.8h   // b2
        sub             v29.8h, v3.8h,  v2.8h   // b3

        add             v16.8h, v26.8h, v5.8h   // b0 + b4
        add             v17.8h, v27.8h, v6.8h   // b1 + b5
        add             v18.8h, v28.8h, v24.8h  // b2 + b6
        add             v19.8h, v29.8h, v25.8h  // b3 + b7
        sub             v20.8h, v29.8h, v25.8h  // b3 - b7
        sub             v21.8h, v28.8h, v24.8h  // b2 - b6
        sub             v22.8h, v27.8h, v6.8h   // b1 - b5
        sub             v23.8h, v26.8h, v5.8h   // b0 - b4
        sshr            v16.8h, v16.8h, #\shift
        sshr            v17.8h, v17.8h, #\shift
        sshr            v18.8h, v18.8h, #\shift
        sshr            v19.8h, v19.8h, #\shift
        sshr            v20.8h, v20.8h, #\shift
        sshr            v21.8h, v21.8h, #\shift
        sshr            v22.8h, v22.8h, #\shift
        sshr            v23.8h, v23.8h, #\shift
.endm

/*
 * void ff_cavs_idct8_add_neon(uint8_t *dst, int16_t *block,
 *                             ptrdiff_t stride)
 *
 * Expects the coefficients transposed, like the SSE2 version: the caller
 * sets idct_perm = FF_IDCT_PERM_TRANSPOSE. The block is not cleared.
 */
function ff_cavs_idct8_add_neon, export=1
        ld1             {v16.8h, v17.8h, v18.8h, v19.8h}, [x1], #64
        ld1             {v20.8h, v21.8h, v22.8h, v23.8h}, [x1]
        movi            v0.8h,  #4
        cavs_idct8_1d   v0.8h, 3
        transpose_8x8H  v16, v17, v18, v19, v20, v21, v22, v23, v30, v31
        movi            v0.8h,  #64
        cavs_idct8_1d   v0.8h, 7

.irp    r, 16, 17, 18, 19, 20, 21, 22, 23
        ld1             {v1.8b}, [x0]
        uaddw           v2.8h,  v\r\().8h, v1.8b
        sqxtun          v1.8b,  v2.8h
        st1             {v1.8b}, [x0], x2
.endr
        ret
endfunc

/*
 * In-loop deblocking (clause 9.11 of GY/T 257.1-2012), all four edge
 * kernels. The per sample condition masks are computed in the byte
 * domain, the filter arithmetic in 16 bit lanes; (x + 4) >> 3 and
 * (x + 2) >> 2 of the C code are srshr/rshrn. p3/q3 are loaded by the
 * vertical kernels but never modified or stored.
 */

/* non-normative edge filter, bs < 2 (loop_filter_l1): p2/p1/p0/q0/q1/q2
   in v16-v21 as 16 byte lanes, alpha w2, beta w3, tc w4, per sample
   enable mask in v27; results replace v17-v20 */
.macro cavs_lf_l1_16
        dup             v25.16b, w2
        dup             v26.16b, w3
        uabd            v22.16b, v18.16b, v19.16b   // |p0 - q0|
        uabd            v23.16b, v17.16b, v18.16b   // |p1 - p0|
        uabd            v24.16b, v20.16b, v19.16b   // |q1 - q0|
        cmhi            v22.16b, v25.16b, v22.16b
        cmhi            v23.16b, v26.16b, v23.16b
        cmhi            v24.16b, v26.16b, v24.16b
        and             v22.16b, v22.16b, v23.16b
        and             v22.16b, v22.16b, v24.16b
        and             v22.16b, v22.16b, v27.16b   // filter this sample
        uabd            v23.16b, v16.16b, v18.16b   // |p2 - p0|
        uabd            v24.16b, v21.16b, v19.16b   // |q2 - q0|
        cmhi            v23.16b, v26.16b, v23.16b
        cmhi            v24.16b, v26.16b, v24.16b
        and             v23.16b, v23.16b, v22.16b   // p1 also filtered
        and             v24.16b, v24.16b, v22.16b   // q1 also filtered

        usubl           v0.8h,  v19.8b,  v18.8b     // q0 - p0
        usubl2          v1.8h,  v19.16b, v18.16b
        usubl           v2.8h,  v17.8b,  v20.8b     // p1 - q1
        usubl2          v3.8h,  v17.16b, v20.16b
        shl             v4.8h,  v0.8h,  #1
        shl             v5.8h,  v1.8h,  #1
        add             v0.8h,  v0.8h,  v4.8h       // 3 * (q0 - p0)
        add             v1.8h,  v1.8h,  v5.8h
        add             v0.8h,  v0.8h,  v2.8h
        add             v1.8h,  v1.8h,  v3.8h
        srshr           v0.8h,  v0.8h,  #3          // (x + 4) >> 3
        srshr           v1.8h,  v1.8h,  #3
        dup             v25.8h, w4                  // tc
        neg             v26.8h, v25.8h
        smin            v0.8h,  v0.8h,  v25.8h
        smin            v1.8h,  v1.8h,  v25.8h
        smax            v0.8h,  v0.8h,  v26.8h      // delta
        smax            v1.8h,  v1.8h,  v26.8h

        uxtl            v2.8h,  v18.8b
        uxtl2           v3.8h,  v18.16b
        add             v2.8h,  v2.8h,  v0.8h       // p0 + delta
        add             v3.8h,  v3.8h,  v1.8h
        sqxtun          v6.8b,  v2.8h
        sqxtun2         v6.16b, v3.8h               // new p0
        uxtl            v4.8h,  v19.8b
        uxtl2           v5.8h,  v19.16b
        sub             v4.8h,  v4.8h,  v0.8h       // q0 - delta
        sub             v5.8h,  v5.8h,  v1.8h
        sqxtun          v7.8b,  v4.8h
        sqxtun2         v7.16b, v5.8h               // new q0

        uxtl            v2.8h,  v6.8b               // new p0, 16 bit
        uxtl2           v3.8h,  v6.16b
        uxtl            v4.8h,  v7.8b               // new q0, 16 bit
        uxtl2           v5.8h,  v7.16b

        uxtl            v28.8h, v17.8b              // p1
        uxtl2           v29.8h, v17.16b
        sub             v0.8h,  v2.8h,  v28.8h      // new p0 - p1
        sub             v1.8h,  v3.8h,  v29.8h
        shl             v30.8h, v0.8h,  #1
        shl             v31.8h, v1.8h,  #1
        add             v0.8h,  v0.8h,  v30.8h
        add             v1.8h,  v1.8h,  v31.8h
        uaddw           v0.8h,  v0.8h,  v16.8b      // + p2
        uaddw2          v1.8h,  v1.8h,  v16.16b
        sub             v0.8h,  v0.8h,  v4.8h       // - new q0
        sub             v1.8h,  v1.8h,  v5.8h
        srshr           v0.8h,  v0.8h,  #3
        srshr           v1.8h,  v1.8h,  #3
        smin            v0.8h,  v0.8h,  v25.8h
        smin            v1.8h,  v1.8h,  v25.8h
        smax            v0.8h,  v0.8h,  v26.8h
        smax            v1.8h,  v1.8h,  v26.8h
        add             v0.8h,  v0.8h,  v28.8h      // p1 + delta
        add             v1.8h,  v1.8h,  v29.8h
        sqxtun          v30.8b,  v0.8h
        sqxtun2         v30.16b, v1.8h              // new p1

        uxtl            v28.8h, v20.8b              // q1
        uxtl2           v29.8h, v20.16b
        sub             v0.8h,  v28.8h, v4.8h       // q1 - new q0
        sub             v1.8h,  v29.8h, v5.8h
        shl             v4.8h,  v0.8h,  #1
        shl             v5.8h,  v1.8h,  #1
        add             v0.8h,  v0.8h,  v4.8h
        add             v1.8h,  v1.8h,  v5.8h
        add             v0.8h,  v0.8h,  v2.8h       // + new p0
        add             v1.8h,  v1.8h,  v3.8h
        usubw           v0.8h,  v0.8h,  v21.8b      // - q2
        usubw2          v1.8h,  v1.8h,  v21.16b
        srshr           v0.8h,  v0.8h,  #3
        srshr           v1.8h,  v1.8h,  #3
        smin            v0.8h,  v0.8h,  v25.8h
        smin            v1.8h,  v1.8h,  v25.8h
        smax            v0.8h,  v0.8h,  v26.8h
        smax            v1.8h,  v1.8h,  v26.8h
        sub             v0.8h,  v28.8h, v0.8h       // q1 - delta
        sub             v1.8h,  v29.8h, v1.8h
        sqxtun          v31.8b,  v0.8h
        sqxtun2         v31.16b, v1.8h              // new q1

        mov             v0.16b, v22.16b
        bsl             v0.16b, v6.16b,  v18.16b
        mov             v1.16b, v22.16b
        bsl             v1.16b, v7.16b,  v19.16b
        mov             v2.16b, v23.16b
        bsl             v2.16b, v30.16b, v17.16b
        mov             v3.16b, v24.16b
        bsl             v3.16b, v31.16b, v20.16b
        mov             v17.16b, v2.16b
        mov             v18.16b, v0.16b
        mov             v19.16b, v1.16b
        mov             v20.16b, v3.16b
.endm

/* normative edge filter, bs == 2 (loop_filter_l2): same register layout,
   no tc and no enable mask, alpha w2, beta w3, w7 clobbered */
.macro cavs_lf_l2_16
        dup             v25.16b, w2
        dup             v26.16b, w3
        uabd            v28.16b, v18.16b, v19.16b   // |p0 - q0|
        uabd            v23.16b, v17.16b, v18.16b   // |p1 - p0|
        uabd            v24.16b, v20.16b, v19.16b   // |q1 - q0|
        cmhi            v22.16b, v25.16b, v28.16b
        cmhi            v23.16b, v26.16b, v23.16b
        cmhi            v24.16b, v26.16b, v24.16b
        and             v22.16b, v22.16b, v23.16b
        and             v22.16b, v22.16b, v24.16b   // filter this sample
        lsr             w7, w2, #2
        add             w7, w7, #2                  // (alpha >> 2) + 2
        dup             v27.16b, w7
        cmhi            v28.16b, v27.16b, v28.16b   // |p0 - q0| < alpha2
        uabd            v23.16b, v16.16b, v18.16b   // |p2 - p0|
        uabd            v24.16b, v21.16b, v19.16b   // |q2 - q0|
        cmhi            v23.16b, v26.16b, v23.16b
        cmhi            v24.16b, v26.16b, v24.16b
        and             v23.16b, v23.16b, v28.16b
        and             v24.16b, v24.16b, v28.16b
        and             v23.16b, v23.16b, v22.16b   // strong p side
        and             v24.16b, v24.16b, v22.16b   // strong q side

        uaddl           v0.8h,  v18.8b,  v19.8b     // p0 + q0
        uaddl2          v1.8h,  v18.16b, v19.16b
        uxtl            v2.8h,  v17.8b              // p1
        uxtl2           v3.8h,  v17.16b
        uxtl            v4.8h,  v18.8b              // p0
        uxtl2           v5.8h,  v18.16b
        add             v6.8h,  v0.8h,  v2.8h       // p1 + p0 + q0
        add             v7.8h,  v1.8h,  v3.8h
        add             v28.8h, v6.8h,  v4.8h       // p1 + 2*p0 + q0
        add             v29.8h, v7.8h,  v5.8h
        add             v30.8h, v6.8h,  v2.8h       // 2*p1 + p0 + q0
        add             v31.8h, v7.8h,  v3.8h
        rshrn           v6.8b,  v28.8h, #2          // (x + 2) >> 2
        rshrn2          v6.16b, v29.8h, #2
        rshrn           v7.8b,  v30.8h, #2
        rshrn2          v7.16b, v31.8h, #2
        mov             v28.16b, v23.16b
        bsl             v28.16b, v6.16b, v7.16b     // p0 when filtered
        mov             v29.16b, v22.16b
        bsl             v29.16b, v28.16b, v18.16b   // new p0
        mov             v30.16b, v23.16b
        bsl             v30.16b, v7.16b, v17.16b    // new p1

        uxtl            v2.8h,  v20.8b              // q1
        uxtl2           v3.8h,  v20.16b
        uxtl            v4.8h,  v19.8b              // q0
        uxtl2           v5.8h,  v19.16b
        add             v6.8h,  v0.8h,  v2.8h       // q1 + q0 + p0
        add             v7.8h,  v1.8h,  v3.8h
        add             v4.8h,  v6.8h,  v4.8h       // q1 + 2*q0 + p0
        add             v5.8h,  v7.8h,  v5.8h
        add             v6.8h,  v6.8h,  v2.8h       // 2*q1 + q0 + p0
        add             v7.8h,  v7.8h,  v3.8h
        rshrn           v2.8b,  v4.8h,  #2
        rshrn2          v2.16b, v5.8h,  #2
        rshrn           v3.8b,  v6.8h,  #2
        rshrn2          v3.16b, v7.8h,  #2
        mov             v28.16b, v24.16b
        bsl             v28.16b, v2.16b, v3.16b     // q0 when filtered
        mov             v31.16b, v22.16b
        bsl             v31.16b, v28.16b, v19.16b   // new q0
        mov             v28.16b, v24.16b
        bsl             v28.16b, v3.16b, v20.16b    // new q1

        mov             v17.16b, v30.16b
        mov             v18.16b, v29.16b
        mov             v19.16b, v31.16b
        mov             v20.16b, v28.16b
.endm

/* build the 16 lane enable mask from bs1 (first 8) and bs2 (last 8) */
.macro cavs_lf_bs_mask
        cmp             w5, #0
        csetm           x7, ne
        cmp             w6, #0
        csetm           x8, ne
        mov             v27.d[0], x7
        mov             v27.d[1], x8
.endm


/* the chroma enable mask: bs1 covers the first 4 lanes, bs2 the next 4 */
.macro cavs_lf_bs_mask_c
        cmp             w5, #0
        csetm           w7, ne
        cmp             w6, #0
        csetm           w8, ne
        orr             x7, x7, x8, lsl #32
        mov             v27.d[0], x7
.endm

/* chroma variant of the bs < 2 filter (loop_filter_c1): 8 lanes,
   p1/p0/q0/q1 in v17-v20, mask in v27; only p0/q0 are modified */
.macro cavs_lf_c1_8
        dup             v25.8b, w2
        dup             v26.8b, w3
        uabd            v22.8b, v18.8b, v19.8b
        uabd            v23.8b, v17.8b, v18.8b
        uabd            v24.8b, v20.8b, v19.8b
        cmhi            v22.8b, v25.8b, v22.8b
        cmhi            v23.8b, v26.8b, v23.8b
        cmhi            v24.8b, v26.8b, v24.8b
        and             v22.8b, v22.8b, v23.8b
        and             v22.8b, v22.8b, v24.8b
        and             v22.8b, v22.8b, v27.8b
        usubl           v0.8h,  v19.8b, v18.8b      // q0 - p0
        usubl           v2.8h,  v17.8b, v20.8b      // p1 - q1
        shl             v4.8h,  v0.8h,  #1
        add             v0.8h,  v0.8h,  v4.8h
        add             v0.8h,  v0.8h,  v2.8h
        srshr           v0.8h,  v0.8h,  #3
        dup             v25.8h, w4
        neg             v26.8h, v25.8h
        smin            v0.8h,  v0.8h,  v25.8h
        smax            v0.8h,  v0.8h,  v26.8h      // delta
        uxtl            v2.8h,  v18.8b
        add             v2.8h,  v2.8h,  v0.8h
        sqxtun          v6.8b,  v2.8h
        uxtl            v4.8h,  v19.8b
        sub             v4.8h,  v4.8h,  v0.8h
        sqxtun          v7.8b,  v4.8h
        mov             v0.8b,  v22.8b
        bsl             v0.8b,  v6.8b,  v18.8b
        mov             v1.8b,  v22.8b
        bsl             v1.8b,  v7.8b,  v19.8b
        mov             v18.8b, v0.8b
        mov             v19.8b, v1.8b
.endm

/* chroma variant of the bs == 2 filter (loop_filter_c2): 8 lanes,
   p2..q2 in v16-v21; only p0/q0 are modified; w7 clobbered */
.macro cavs_lf_c2_8
        dup             v25.8b, w2
        dup             v26.8b, w3
        uabd            v28.8b, v18.8b, v19.8b      // |p0 - q0|
        uabd            v23.8b, v17.8b, v18.8b
        uabd            v24.8b, v20.8b, v19.8b
        cmhi            v22.8b, v25.8b, v28.8b
        cmhi            v23.8b, v26.8b, v23.8b
        cmhi            v24.8b, v26.8b, v24.8b
        and             v22.8b, v22.8b, v23.8b
        and             v22.8b, v22.8b, v24.8b
        lsr             w7, w2, #2
        add             w7, w7, #2                  // (alpha >> 2) + 2
        dup             v27.8b, w7
        cmhi            v28.8b, v27.8b, v28.8b      // |p0 - q0| < alpha2
        uabd            v23.8b, v16.8b, v18.8b      // |p2 - p0|
        uabd            v24.8b, v21.8b, v19.8b      // |q2 - q0|
        cmhi            v23.8b, v26.8b, v23.8b
        cmhi            v24.8b, v26.8b, v24.8b
        and             v23.8b, v23.8b, v28.8b
        and             v24.8b, v24.8b, v28.8b
        and             v23.8b, v23.8b, v22.8b      // strong p side
        and             v24.8b, v24.8b, v22.8b      // strong q side
        uaddl           v0.8h,  v18.8b, v19.8b      // p0 + q0
        uxtl            v2.8h,  v17.8b              // p1
        uxtl            v4.8h,  v18.8b              // p0
        add             v6.8h,  v0.8h,  v2.8h
        add             v28.8h, v6.8h,  v4.8h       // p1 + 2*p0 + q0
        add             v30.8h, v6.8h,  v2.8h       // 2*p1 + p0 + q0
        rshrn           v6.8b,  v28.8h, #2
        rshrn           v7.8b,  v30.8h, #2
        mov             v28.8b, v23.8b
        bsl             v28.8b, v6.8b,  v7.8b
        mov             v29.8b, v22.8b
        bsl             v29.8b, v28.8b, v18.8b      // new p0
        uxtl            v2.8h,  v20.8b              // q1
        uxtl            v4.8h,  v19.8b              // q0
        add             v6.8h,  v0.8h,  v2.8h
        add             v4.8h,  v6.8h,  v4.8h       // q1 + 2*q0 + p0
        add             v6.8h,  v6.8h,  v2.8h       // 2*q1 + q0 + p0
        rshrn           v2.8b,  v4.8h,  #2
        rshrn           v3.8b,  v6.8h,  #2
        mov             v28.8b, v24.8b
        bsl             v28.8b, v2.8b,  v3.8b
        mov             v31.8b, v22.8b
        bsl             v31.8b, v28.8b, v19.8b      // new q0
        mov             v18.8b, v29.8b
        mov             v19.8b, v31.8b
.endm

function ff_cavs_filter_lv_neon, export=1
        cmp             w5, #2
        b.eq            1f
        orr             w7, w5, w6
        cbz             w7, 9f
1:
        sub             x9, x0, #4
        ld1             {v24.8b},   [x9], x1
        ld1             {v16.8b},   [x9], x1
        ld1             {v17.8b},   [x9], x1
        ld1             {v18.8b},   [x9], x1
        ld1             {v19.8b},   [x9], x1
        ld1             {v20.8b},   [x9], x1
        ld1             {v21.8b},   [x9], x1
        ld1             {v25.8b},   [x9], x1
        ld1             {v24.d}[1], [x9], x1
        ld1             {v16.d}[1], [x9], x1
        ld1             {v17.d}[1], [x9], x1
        ld1             {v18.d}[1], [x9], x1
        ld1             {v19.d}[1], [x9], x1
        ld1             {v20.d}[1], [x9], x1
        ld1             {v21.d}[1], [x9], x1
        ld1             {v25.d}[1], [x9]

        transpose_8x16B v24, v16, v17, v18, v19, v20, v21, v25, v28, v29

        cmp             w5, #2
        b.eq            2f
        cavs_lf_bs_mask
        cavs_lf_l1_16
        b               3f
2:
        cavs_lf_l2_16
3:
        transpose_4x16B v17, v18, v19, v20, v28, v29, v30, v31

        sub             x9, x0, #2
        st1             {v17.s}[0], [x9], x1
        st1             {v18.s}[0], [x9], x1
        st1             {v19.s}[0], [x9], x1
        st1             {v20.s}[0], [x9], x1
        st1             {v17.s}[1], [x9], x1
        st1             {v18.s}[1], [x9], x1
        st1             {v19.s}[1], [x9], x1
        st1             {v20.s}[1], [x9], x1
        st1             {v17.s}[2], [x9], x1
        st1             {v18.s}[2], [x9], x1
        st1             {v19.s}[2], [x9], x1
        st1             {v20.s}[2], [x9], x1
        st1             {v17.s}[3], [x9], x1
        st1             {v18.s}[3], [x9], x1
        st1             {v19.s}[3], [x9], x1
        st1             {v20.s}[3], [x9]
9:
        ret
endfunc

function ff_cavs_filter_lh_neon, export=1
        cmp             w5, #2
        b.eq            1f
        orr             w7, w5, w6
        cbz             w7, 9f
1:
        sub             x9, x0, x1
        sub             x9, x9, x1, lsl #1          // d - 3 * stride
        ld1             {v16.16b}, [x9], x1         // p2
        ld1             {v17.16b}, [x9], x1         // p1
        ld1             {v18.16b}, [x9], x1         // p0
        ld1             {v19.16b}, [x9], x1         // q0
        ld1             {v20.16b}, [x9], x1         // q1
        ld1             {v21.16b}, [x9]             // q2

        cmp             w5, #2
        b.eq            2f
        cavs_lf_bs_mask
        cavs_lf_l1_16
        b               3f
2:
        cavs_lf_l2_16
3:
        sub             x9, x0, x1, lsl #1
        st1             {v17.16b}, [x9], x1
        st1             {v18.16b}, [x9], x1
        st1             {v19.16b}, [x9], x1
        st1             {v20.16b}, [x9]
9:
        ret
endfunc

function ff_cavs_filter_cv_neon, export=1
        cmp             w5, #2
        b.eq            1f
        orr             w7, w5, w6
        cbz             w7, 9f
1:
        sub             x9, x0, #4
        ld1             {v24.8b}, [x9], x1
        ld1             {v16.8b}, [x9], x1
        ld1             {v17.8b}, [x9], x1
        ld1             {v18.8b}, [x9], x1
        ld1             {v19.8b}, [x9], x1
        ld1             {v20.8b}, [x9], x1
        ld1             {v21.8b}, [x9], x1
        ld1             {v25.8b}, [x9]

        transpose_8x8B  v24, v16, v17, v18, v19, v20, v21, v25, v28, v29

        cmp             w5, #2
        b.eq            2f
        cavs_lf_bs_mask_c
        cavs_lf_c1_8
        b               3f
2:
        cavs_lf_c2_8
3:
        zip1            v0.16b, v18.16b, v19.16b

        sub             x9, x0, #1
        st1             {v0.h}[0], [x9], x1
        st1             {v0.h}[1], [x9], x1
        st1             {v0.h}[2], [x9], x1
        st1             {v0.h}[3], [x9], x1
        st1             {v0.h}[4], [x9], x1
        st1             {v0.h}[5], [x9], x1
        st1             {v0.h}[6], [x9], x1
        st1             {v0.h}[7], [x9]
9:
        ret
endfunc

function ff_cavs_filter_ch_neon, export=1
        cmp             w5, #2
        b.eq            1f
        orr             w7, w5, w6
        cbz             w7, 9f
1:
        sub             x9, x0, x1
        sub             x9, x9, x1, lsl #1          // d - 3 * stride
        ld1             {v16.8b}, [x9], x1          // p2
        ld1             {v17.8b}, [x9], x1          // p1
        ld1             {v18.8b}, [x9], x1          // p0
        ld1             {v19.8b}, [x9], x1          // q0
        ld1             {v20.8b}, [x9], x1          // q1
        ld1             {v21.8b}, [x9]              // q2

        cmp             w5, #2
        b.eq            2f
        cavs_lf_bs_mask_c
        cavs_lf_c1_8
        b               3f
2:
        cavs_lf_c2_8
3:
        sub             x9, x0, x1
        st1             {v18.8b}, [x9], x1
        st1             {v19.8b}, [x9]
9:
        ret
endfunc

/*
 * Sub sample interpolation (clause 9.9.2.2/9.9.2.3), the one dimensional
 * positions. The half sample filter (-1, 5, 5, -1) fits signed 16 bit
 * lanes as it stands; the quarter sample filters reach (96 + 42) * 255,
 * so like the SSE2 version they accumulate the positive and negative
 * taps separately in unsigned 16 bit lanes and combine with a saturating
 * subtract, which equals the C crop for every input. Loads may read up
 * to 3 bytes past the last used sample, like the h264 qpel NEON code.
 */

/* one 8 wide row of the h half sample filter; input v0 = 16 bytes from
   src - 2, result in \d (8 bytes) */
.macro cavs_h_hpel_row d
        ext             v1.16b, v0.16b, v0.16b, #1
        ext             v2.16b, v0.16b, v0.16b, #2
        ext             v3.16b, v0.16b, v0.16b, #3
        ext             v4.16b, v0.16b, v0.16b, #4
        uaddl           v5.8h,  v2.8b,  v3.8b
        uaddl           v6.8h,  v1.8b,  v4.8b
        shl             v7.8h,  v5.8h,  #2
        add             v5.8h,  v5.8h,  v7.8h
        sub             v5.8h,  v5.8h,  v6.8h
        sqrshrun        \d\().8b, v5.8h, #3
.endm

/* one 8 wide row of the left/right quarter sample filter; \e2/\e3 are
   the ext amounts of the two positive taps, \which selects the tap set */
.macro cavs_h_qpel_row d, which
        ext             v1.16b, v0.16b, v0.16b, #1
        ext             v2.16b, v0.16b, v0.16b, #2
        ext             v3.16b, v0.16b, v0.16b, #3
        ext             v4.16b, v0.16b, v0.16b, #4
.ifc \which, qpel_l
        // (-1, -2, 96, 42, -7) at src - 2 .. src + 2
        umull           v5.8h,  v2.8b,  v29.8b      // 96 * src0
        umlal           v5.8h,  v3.8b,  v30.8b      // + 42 * src1
        uaddl           v6.8h,  v0.8b,  v1.8b       // src-2 + src-1
        uaddw           v6.8h,  v6.8h,  v1.8b       // + src-1
        umlal           v6.8h,  v4.8b,  v31.8b      // + 7 * src2
.else
        // (-7, 42, 96, -2, -1) at src - 1 .. src + 3
        ext             v7.16b, v0.16b, v0.16b, #5
        umull           v5.8h,  v2.8b,  v30.8b      // 42 * src0
        umlal           v5.8h,  v3.8b,  v29.8b      // + 96 * src1
        umull           v6.8h,  v1.8b,  v31.8b      // 7 * src-1
        uaddw           v6.8h,  v6.8h,  v4.8b       // + src2
        uaddw           v6.8h,  v6.8h,  v4.8b       // + src2
        uaddw           v6.8h,  v6.8h,  v7.8b       // + src3
.endif
        movi            v7.8h,  #64
        add             v5.8h,  v5.8h,  v7.8h
        uqsub           v5.8h,  v5.8h,  v6.8h       // clip at 0
        ushr            v5.8h,  v5.8h,  #7
        uqxtn           \d\().8b, v5.8h             // clip at 255
.endm

/* 8x8 horizontal filter bodies; \op is put or avg */
.macro cavs_qpel8_h_fn type, op
function \op\()_cavs_qpel8_h_\type\()_neon
        sub             x1,  x1,  #2
        mov             w10, #8
1:      ld1             {v0.16b}, [x1], x2
.ifc \type, hpel
        cavs_h_hpel_row v16
.else
        cavs_h_qpel_row v16, \type
.endif
.ifc \op, avg
        ld1             {v17.8b}, [x0]
        urhadd          v16.8b, v16.8b, v17.8b
.endif
        st1             {v16.8b}, [x0], x2
        subs            w10, w10, #1
        b.ne            1b
        ret
endfunc
.endm

cavs_qpel8_h_fn hpel,   put
cavs_qpel8_h_fn qpel_l, put
cavs_qpel8_h_fn qpel_r, put
cavs_qpel8_h_fn hpel,   avg
cavs_qpel8_h_fn qpel_l, avg
cavs_qpel8_h_fn qpel_r, avg

/* 8x8 vertical filter bodies: a rolling window of source rows */
.macro cavs_qpel8_v_fn type, op
function \op\()_cavs_qpel8_v_\type\()_neon
.ifc \type, hpel
        sub             x9,  x1,  x2
        ld1             {v0.8b}, [x9], x2           // r-1
        ld1             {v1.8b}, [x9], x2           // r0
        ld1             {v2.8b}, [x9], x2           // r1
.endif
.ifc \type, qpel_l
        sub             x9,  x1,  x2,  lsl #1
        ld1             {v0.8b}, [x9], x2           // r-2
        ld1             {v1.8b}, [x9], x2           // r-1
        ld1             {v2.8b}, [x9], x2           // r0
        ld1             {v3.8b}, [x9], x2           // r1
.endif
.ifc \type, qpel_r
        sub             x9,  x1,  x2
        ld1             {v0.8b}, [x9], x2           // r-1
        ld1             {v1.8b}, [x9], x2           // r0
        ld1             {v2.8b}, [x9], x2           // r1
        ld1             {v3.8b}, [x9], x2           // r2
.endif
        mov             w10, #8
1:
.ifc \type, hpel
        ld1             {v3.8b}, [x9], x2           // r2
        uaddl           v5.8h,  v1.8b,  v2.8b
        uaddl           v6.8h,  v0.8b,  v3.8b
        shl             v7.8h,  v5.8h,  #2
        add             v5.8h,  v5.8h,  v7.8h
        sub             v5.8h,  v5.8h,  v6.8h
        sqrshrun        v16.8b, v5.8h,  #3
.endif
.ifc \type, qpel_l
        ld1             {v4.8b}, [x9], x2           // r2
        umull           v5.8h,  v2.8b,  v29.8b      // 96 * r0
        umlal           v5.8h,  v3.8b,  v30.8b      // + 42 * r1
        uaddl           v6.8h,  v0.8b,  v1.8b       // r-2 + r-1
        uaddw           v6.8h,  v6.8h,  v1.8b       // + r-1
        umlal           v6.8h,  v4.8b,  v31.8b      // + 7 * r2
        movi            v7.8h,  #64
        add             v5.8h,  v5.8h,  v7.8h
        uqsub           v5.8h,  v5.8h,  v6.8h
        ushr            v5.8h,  v5.8h,  #7
        uqxtn           v16.8b, v5.8h
.endif
.ifc \type, qpel_r
        ld1             {v4.8b}, [x9], x2           // r3
        umull           v5.8h,  v1.8b,  v30.8b      // 42 * r0
        umlal           v5.8h,  v2.8b,  v29.8b      // + 96 * r1
        umull           v6.8h,  v0.8b,  v31.8b      // 7 * r-1
        uaddw           v6.8h,  v6.8h,  v3.8b       // + r2
        uaddw           v6.8h,  v6.8h,  v3.8b       // + r2
        uaddw           v6.8h,  v6.8h,  v4.8b       // + r3
        movi            v7.8h,  #64
        add             v5.8h,  v5.8h,  v7.8h
        uqsub           v5.8h,  v5.8h,  v6.8h
        ushr            v5.8h,  v5.8h,  #7
        uqxtn           v16.8b, v5.8h
.endif
.ifc \op, avg
        ld1             {v17.8b}, [x0]
        urhadd          v16.8b, v16.8b, v17.8b
.endif
        st1             {v16.8b}, [x0], x2
        mov             v0.8b,  v1.8b
        mov             v1.8b,  v2.8b
        mov             v2.8b,  v3.8b
.ifnc \type, hpel
        mov             v3.8b,  v4.8b
.endif
        subs            w10, w10, #1
        b.ne            1b
        ret
endfunc
.endm

cavs_qpel8_v_fn hpel,   put
cavs_qpel8_v_fn qpel_l, put
cavs_qpel8_v_fn qpel_r, put
cavs_qpel8_v_fn hpel,   avg
cavs_qpel8_v_fn qpel_l, avg
cavs_qpel8_v_fn qpel_r, avg

/* the quarter sample tap constants shared by all qpel entry points */
.macro cavs_qpel_consts
        movi            v29.8b, #96
        movi            v30.8b, #42
        movi            v31.8b, #7
.endm

/* exported 8x8 entry points */
.macro cavs_qpel8_fn mc, core, op
function ff_\op\()_cavs_qpel8_\mc\()_neon, export=1
        cavs_qpel_consts
        b               \op\()_cavs_qpel8_\core\()_neon
endfunc
.endm

cavs_qpel8_fn mc20, h_hpel,   put
cavs_qpel8_fn mc10, h_qpel_l, put
cavs_qpel8_fn mc30, h_qpel_r, put
cavs_qpel8_fn mc02, v_hpel,   put
cavs_qpel8_fn mc01, v_qpel_l, put
cavs_qpel8_fn mc03, v_qpel_r, put
cavs_qpel8_fn mc20, h_hpel,   avg
cavs_qpel8_fn mc10, h_qpel_l, avg
cavs_qpel8_fn mc30, h_qpel_r, avg
cavs_qpel8_fn mc02, v_hpel,   avg
cavs_qpel8_fn mc01, v_qpel_l, avg
cavs_qpel8_fn mc03, v_qpel_r, avg

/* exported 16x16 entry points: the four 8x8 quadrants of the C code */
.macro cavs_qpel16_fn mc, core, op
function ff_\op\()_cavs_qpel16_\mc\()_neon, export=1
        stp             x19, x20, [sp, #-32]!
        stp             x21, x30, [sp, #16]
        cavs_qpel_consts
        mov             x19, x0
        mov             x20, x1
        mov             x21, x2
        bl              \op\()_cavs_qpel8_\core\()_neon
        add             x0,  x19, #8
        add             x1,  x20, #8
        mov             x2,  x21
        bl              \op\()_cavs_qpel8_\core\()_neon
        add             x0,  x19, x21, lsl #3
        add             x1,  x20, x21, lsl #3
        mov             x2,  x21
        bl              \op\()_cavs_qpel8_\core\()_neon
        add             x0,  x19, x21, lsl #3
        add             x1,  x20, x21, lsl #3
        add             x0,  x0,  #8
        add             x1,  x1,  #8
        mov             x2,  x21
        bl              \op\()_cavs_qpel8_\core\()_neon
        ldp             x21, x30, [sp, #16]
        ldp             x19, x20, [sp], #32
        ret
endfunc
.endm

cavs_qpel16_fn mc20, h_hpel,   put
cavs_qpel16_fn mc10, h_qpel_l, put
cavs_qpel16_fn mc30, h_qpel_r, put
cavs_qpel16_fn mc02, v_hpel,   put
cavs_qpel16_fn mc01, v_qpel_l, put
cavs_qpel16_fn mc03, v_qpel_r, put
cavs_qpel16_fn mc20, h_hpel,   avg
cavs_qpel16_fn mc10, h_qpel_l, avg
cavs_qpel16_fn mc30, h_qpel_r, avg
cavs_qpel16_fn mc02, v_hpel,   avg
cavs_qpel16_fn mc01, v_qpel_l, avg
cavs_qpel16_fn mc03, v_qpel_r, avg

/* full sample copies (mc00) */
function ff_put_cavs_qpel8_mc00_neon, export=1
.rept 8
        ld1             {v0.8b}, [x1], x2
        st1             {v0.8b}, [x0], x2
.endr
        ret
endfunc

function ff_avg_cavs_qpel8_mc00_neon, export=1
.rept 8
        ld1             {v0.8b}, [x1], x2
        ld1             {v1.8b}, [x0]
        urhadd          v0.8b,  v0.8b,  v1.8b
        st1             {v0.8b}, [x0], x2
.endr
        ret
endfunc

function ff_put_cavs_qpel16_mc00_neon, export=1
.rept 16
        ld1             {v0.16b}, [x1], x2
        st1             {v0.16b}, [x0], x2
.endr
        ret
endfunc

function ff_avg_cavs_qpel16_mc00_neon, export=1
.rept 16
        ld1             {v0.16b}, [x1], x2
        ld1             {v1.16b}, [x0]
        urhadd          v0.16b, v0.16b, v1.16b
        st1             {v0.16b}, [x0], x2
.endr
        ret
endfunc
