/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * The unpacking is libhydrasdr hydrasdr_unpack_samples_reference() and the
 * DDC libhydrasdr iqconverter_int16_opt_process_u16() with the 12-bit ADC
 * table, both from libhydrasdr 1.1.4; the arithmetic is kept as is so the
 * output matches libhydrasdr bit for bit.
 */

#include <string.h>

#include "raw12.h"

#define DC_ALPHA_Q15	32100	/* DC removal pole, 0.98 in Q15 */
#define FIR_SIZE	512
#define FIR_MASK	511
#define DLY_LEN		12	/* half-band center tap delay */
#define SYM_TAPS	12

/* libhydrasdr HB_KERNEL_INT16, the even taps of the 47-tap half-band
 * filter, Q15; the center tap 16384 is the halving of the delayed Q. */
static const int32_t kern[SYM_TAPS] = {
	-33, 56, -100, 166, -259, 389, -571, 829, -1220, 1885, -3353, 10389
};

void raw12_ddc_init(struct raw12_ddc *d)
{
	memset(d, 0, sizeof(*d));
}

static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
	       (uint32_t)p[3] << 24;
}

static void put32(uint8_t *p, uint32_t w)
{
	p[0] = (uint8_t)w;
	p[1] = (uint8_t)(w >> 8);
	p[2] = (uint8_t)(w >> 16);
	p[3] = (uint8_t)(w >> 24);
}

void raw12_unpack(const uint8_t *in, uint16_t c[RAW12_GROUP_CODES])
{
	uint32_t w0 = get32(in), w1 = get32(in + 4), w2 = get32(in + 8);

	c[0] = (uint16_t)((w0 >> 20) & 0xFFF);
	c[1] = (uint16_t)((w0 >> 8) & 0xFFF);
	c[2] = (uint16_t)(((w0 & 0xFF) << 4) | ((w1 >> 28) & 0xF));
	c[3] = (uint16_t)((w1 >> 16) & 0xFFF);
	c[4] = (uint16_t)((w1 >> 4) & 0xFFF);
	c[5] = (uint16_t)(((w1 & 0xF) << 8) | (w2 >> 24));
	c[6] = (uint16_t)((w2 >> 12) & 0xFFF);
	c[7] = (uint16_t)(w2 & 0xFFF);
}

void raw12_pack(const uint16_t c[RAW12_GROUP_CODES], uint8_t *out)
{
	uint32_t k[RAW12_GROUP_CODES];
	int i;

	for (i = 0; i < RAW12_GROUP_CODES; i++)
		k[i] = c[i] & 0xFFFu;
	put32(out, k[0] << 20 | k[1] << 8 | k[2] >> 4);
	put32(out + 4, (k[2] & 0xF) << 28 | k[3] << 16 | k[4] << 4 |
		       k[5] >> 8);
	put32(out + 8, (k[5] & 0xFF) << 24 | k[6] << 12 | k[7]);
}

/* Saturate to int16; values in range pass unchanged. */
static int16_t sat16(int32_t v)
{
	return (int16_t)(v > INT16_MAX ? INT16_MAX : (v < INT16_MIN ? INT16_MIN : v));
}

/* ADC code to int16 full scale, then the DC removal high-pass. Its gain
 * reaches 1.0103, so near ADC full scale the output saturates. */
static int16_t dc_remove(struct raw12_ddc *d, uint16_t code)
{
	int16_t x = (int16_t)(((int32_t)code - RAW12_CODE_MID) * 16);
	int32_t u = d->old_e + (int32_t)d->old_y * DC_ALPHA_Q15;
	int16_t s = (int16_t)(u >> 15);
	int16_t y = sat16((int32_t)x - d->old_x + s);

	d->old_e = u - (int32_t)s * 32768;
	d->old_x = x;
	d->old_y = y;
	return y;
}

/* One output of the symmetric half-band filter on the I branch. */
static int32_t fir(struct raw12_ddc *d, int32_t v)
{
	const int32_t *h;

	d->fir_head = (d->fir_head - 1) & FIR_MASK;
	d->fir[d->fir_head] = v;
	d->fir[d->fir_head + FIR_SIZE] = v;
	h = &d->fir[d->fir_head];
	return kern[0] * (h[0] + h[23]) + kern[1] * (h[1] + h[22]) +
	       kern[2] * (h[2] + h[21]) + kern[3] * (h[3] + h[20]) +
	       kern[4] * (h[4] + h[19]) + kern[5] * (h[5] + h[18]) +
	       kern[6] * (h[6] + h[17]) + kern[7] * (h[7] + h[16]) +
	       kern[8] * (h[8] + h[15]) + kern[9] * (h[9] + h[14]) +
	       kern[10] * (h[10] + h[13]) + kern[11] * (h[11] + h[12]);
}

/* The Q branch: the center tap, a delay of 12 samples. */
static int16_t delay(struct raw12_ddc *d, int16_t v)
{
	int16_t out = d->dly[d->dly_head];

	d->dly[d->dly_head] = v;
	if (++d->dly_head >= DLY_LEN)
		d->dly_head = 0;
	return out;
}

/* 4 codes to 2 complex samples; the fs/4 shift is the sign pattern
 * -I, -Q, +I, +Q. */
static void ddc4(struct raw12_ddc *d, const uint16_t *c, int16_t *iq)
{
	int16_t y0 = dc_remove(d, c[0]);
	int16_t y1 = dc_remove(d, c[1]);
	int16_t y2 = dc_remove(d, c[2]);
	int16_t y3 = dc_remove(d, c[3]);
	int32_t acc0 = fir(d, -(int32_t)y0);
	int32_t acc1 = fir(d, (int32_t)y2);

	iq[1] = delay(d, (int16_t)((-y1) >> 1));
	iq[3] = delay(d, (int16_t)(y3 >> 1));
	iq[0] = sat16(acc0 >> 15);
	iq[2] = sat16(acc1 >> 15);
}

void raw12_convert(struct raw12_ddc *d, const uint8_t *in, size_t groups,
		   int16_t *iq)
{
	uint16_t c[RAW12_GROUP_CODES];

	for (; groups; groups--, in += RAW12_GROUP_BYTES, iq += 8) {
		raw12_unpack(in, c);
		ddc4(d, c, iq);
		ddc4(d, c + 4, iq + 4);
	}
}

void raw12_fill(uint8_t *out, size_t groups)
{
	uint16_t c[RAW12_GROUP_CODES];
	int i;

	for (i = 0; i < RAW12_GROUP_CODES; i++)
		c[i] = RAW12_CODE_MID;
	for (; groups; groups--, out += RAW12_GROUP_BYTES)
		raw12_pack(c, out);
}
