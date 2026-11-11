/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

/*
 * Mode S PPM demodulator for 2.5, 5 and 10 MS/s complex input.
 *
 * The magnitude is integrated over one chip (0.5 us) starting at every
 * point of a 0.1 us grid: one grid point per sample at 10 MS/s, two at
 * 5 MS/s, four at 2.5 MS/s, with fractional weights for the sample cells a
 * chip covers partly. On that grid a chip is 5 points, a bit 10 and the
 * preamble 80 at every rate.
 *
 * The preamble is searched once per input sample with a 4-pulse / 4-gap
 * test, moved to the best grid point within the next microsecond and
 * checked on all 16 chips before the bits are sliced.
 */

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "demod.h"
#include "icao.h"

#define GRID_HZ		10000000.0
#define CHIP		5
#define BIT		10
#define PREAMBLE	80
#define LONG_FRAME	(PREAMBLE + MODES_LONG_BITS * BIT)
#define LOOKAHEAD	(LONG_FRAME + BIT + 8)
#define MAX_TAPS	6
#define ICAO_TTL_SEC	60

#define PRE_RATIO	1.6f	/* 4 pulses over 4 gap chips, search */
#define GATE_RATIO	2.0f	/* pulse mean over gap mean */
#define PULSE_RATIO	1.5f	/* third strongest pulse over gap mean */

struct demod {
	double rate;
	int f;			/* grid points per sample */
	int taps;
	float w[4][MAX_TAPS];
	float chip_len;		/* samples per chip */
	int fix;

	size_t max_block;
	size_t carry;		/* samples kept for the next call */
	size_t keep;		/* samples actually kept */
	float *mag;
	float *chip;
	uint64_t consumed;	/* samples given to process calls */
	uint64_t next_g;	/* absolute grid index of the next search */
	uint64_t resume_g;	/* first grid index after the last frame */

	struct icao_table icao;
	struct demod_stats st;
};

int demod_rate_supported(double rate)
{
	return rate == 2500000.0 || rate == 5000000.0 || rate == 10000000.0;
}

static void set_weights(struct demod *d)
{
	float len = d->chip_len;
	int q, k;

	d->taps = (int)ceilf(len + (float)(d->f - 1) / (float)d->f);
	for (q = 0; q < d->f; q++) {
		float s = (float)q / (float)d->f;

		for (k = 0; k < MAX_TAPS; k++) {
			float lo = (float)k > s ? (float)k : s;
			float hi = (float)(k + 1) < s + len ? (float)(k + 1) :
							       s + len;

			d->w[q][k] = hi > lo ? hi - lo : 0.0f;
		}
	}
}

struct demod *demod_create(double rate, size_t max_block)
{
	struct demod *d;

	if (!demod_rate_supported(rate) || max_block == 0)
		return NULL;
	d = calloc(1, sizeof(*d));
	if (!d)
		return NULL;
	d->rate = rate;
	d->f = (int)(GRID_HZ / rate);
	d->chip_len = (float)(rate * 0.5e-6);
	d->fix = 1;
	set_weights(d);

	d->max_block = max_block;
	d->carry = (size_t)(LOOKAHEAD / d->f) + (size_t)d->taps + 4;
	d->mag = malloc((d->carry + max_block) * sizeof(float));
	d->chip = malloc((d->carry + max_block) * (size_t)d->f * sizeof(float));
	if (!d->mag || !d->chip) {
		demod_destroy(d);
		return NULL;
	}
	icao_init(&d->icao, (uint64_t)(ICAO_TTL_SEC * rate));
	return d;
}

void demod_destroy(struct demod *d)
{
	if (!d)
		return;
	free(d->mag);
	free(d->chip);
	free(d);
}

void demod_set_fix(struct demod *d, int on)
{
	d->fix = on;
}

const struct demod_stats *demod_get_stats(const struct demod *d)
{
	return &d->st;
}

void demod_gap(struct demod *d, uint64_t samples)
{
	if (!samples)
		return;
	d->consumed += samples;
	d->keep = 0;
	d->next_g = d->consumed * (uint64_t)d->f;
}

/*
 * Chip energies for samples 0..last. The samples and weights are read into
 * locals first, so the stores to e cannot force them to be read again; with
 * f and taps constant at the call the loops unroll.
 */
static inline void energy_loop(const float *m, float *e, size_t last,
			       const float *w, int f, int taps)
{
	float lw[4][MAX_TAPS], s[MAX_TAPS];
	size_t n;
	int q, k;

	memcpy(lw, w, sizeof(lw));
	for (n = 0; n <= last; n++) {
		for (k = 0; k < taps; k++)
			s[k] = m[n + (size_t)k];
		for (q = 0; q < f; q++) {
			float acc = 0.0f;

			for (k = 0; k < taps; k++)
				acc += lw[q][k] * s[k];
			*e++ = acc;
		}
	}
}

/* Chip energy at every grid point that has all its taps in mag[]. */
static size_t chip_energy(struct demod *d, size_t mag_len)
{
	size_t last;

	if (mag_len < (size_t)d->taps)
		return 0;
	last = mag_len - (size_t)d->taps;
	/* the layouts of 10, 5 and 2.5 MS/s */
	if (d->f == 1 && d->taps == 5)
		energy_loop(d->mag, d->chip, last, &d->w[0][0], 1, 5);
	else if (d->f == 2 && d->taps == 3)
		energy_loop(d->mag, d->chip, last, &d->w[0][0], 2, 3);
	else if (d->f == 4 && d->taps == 2)
		energy_loop(d->mag, d->chip, last, &d->w[0][0], 4, 2);
	else
		energy_loop(d->mag, d->chip, last, &d->w[0][0], d->f, d->taps);
	return (last + 1) * (size_t)d->f;
}

static const int pulse_chip[4] = { 0, 2, 7, 9 };
static const int gap_chip[12] = { 1, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14, 15 };

static float preamble_ratio(const float *e, float *high, float *low)
{
	float h = 0.0f, l = 0.0f;
	int i;

	for (i = 0; i < 4; i++)
		h += e[pulse_chip[i] * CHIP];
	for (i = 0; i < 12; i++)
		l += e[gap_chip[i] * CHIP];
	*high = h * 0.25f;
	*low = l * (1.0f / 12.0f);
	return *high / (*low + 1e-12f);
}

/* Third strongest of the four preamble pulses. */
static float third_pulse(const float *e)
{
	float a[4];
	int i, j;

	for (i = 0; i < 4; i++)
		a[i] = e[pulse_chip[i] * CHIP];
	for (i = 1; i < 4; i++) {
		float v = a[i];

		for (j = i; j > 0 && a[j - 1] < v; j--)
			a[j] = a[j - 1];
		a[j] = v;
	}
	return a[2];
}

struct slice {
	uint8_t msg[MODES_LONG_BYTES];
	int df;
	int bits;
	float margin;		/* weakest bit decision */
	float level;
};

static void slice_bits(const float *e, float chip_len, struct slice *s)
{
	int n = MODES_LONG_BITS;
	float lvl = 0.0f;
	int i;

	memset(s->msg, 0, sizeof(s->msg));
	s->margin = 1e30f;
	for (i = 0; i < n; i++) {
		float a = e[i * BIT];
		float b = e[i * BIT + CHIP];
		float m = fabsf(a - b) / (a + b + 1e-12f);

		if (m < s->margin)
			s->margin = m;
		if (a > b) {
			s->msg[i >> 3] |= (uint8_t)(0x80u >> (i & 7));
			lvl += a;
		} else {
			lvl += b;
		}
		if (i == 4) {
			s->df = s->msg[0] >> 3;
			if (s->df < 16)
				n = MODES_SHORT_BITS;
		}
	}
	s->bits = n;
	s->level = lvl / ((float)n * chip_len);
}

static uint32_t frame_aa(const uint8_t *msg)
{
	return (uint32_t)msg[1] << 16 | (uint32_t)msg[2] << 8 | msg[3];
}

/*
 * 1 if the frame is valid, with *addr set. DF11 with a non-zero
 * interrogator code and the formats whose parity carries the address are
 * accepted only for an address already seen on a clean frame.
 */
static int validate(struct demod *d, const struct slice *s, uint64_t now,
		    uint32_t *addr)
{
	uint32_t r = modes_crc_residual(s->msg, s->bits);

	switch (s->df) {
	case 17:
	case 18:
		if (r)
			return 0;
		*addr = frame_aa(s->msg);
		icao_add(&d->icao, *addr, now);
		return 1;
	case 11:
		/* low 7 bits: CL (3) and IC (4), CL 5 to 7 do not exist */
		if (r >= 0x50)
			return 0;
		*addr = frame_aa(s->msg);
		if (r == 0) {
			icao_add(&d->icao, *addr, now);
			return 1;
		}
		return icao_known(&d->icao, *addr, now);
	case 0:
	case 4:
	case 5:
	case 16:
	case 20:
	case 21:
		*addr = r;
		return icao_known(&d->icao, r, now);
	default:
		return 0;
	}
}

/* Single-bit fix of a DF17/18 frame. Returns the bit, or -1. */
static int try_fix(struct slice *s)
{
	uint32_t r;
	int bit;

	if (s->df != 17 && s->df != 18)
		return -1;
	r = modes_crc_residual(s->msg, MODES_LONG_BITS);
	bit = modes_crc_find_bit(r, MODES_LONG_BITS);
	if (bit < 5)	/* never change the format field */
		return -1;
	s->msg[bit >> 3] ^= (uint8_t)(0x80u >> (bit & 7));
	return bit;
}

/* Slice offsets around the preamble peak, in grid points. */
static const int retry_off[5] = { 0, -1, 1, -2, 2 };

static void search(struct demod *d, size_t chip_len, uint64_t base_g,
		   demod_frame_fn fn, void *ctx)
{
	const float *e = d->chip;
	size_t last, g, start;
	size_t step = (size_t)d->f;

	if (chip_len <= LOOKAHEAD + 2)
		return;
	last = chip_len - LOOKAHEAD - 2;

	start = 0;
	if (d->next_g > base_g)
		start = (size_t)(d->next_g - base_g);
	if (d->resume_g > base_g + start)
		start = (size_t)(d->resume_g - base_g);
	if (start < 2)
		start = 2;

	for (g = start; g <= last; g += step) {
		struct slice s, best;
		float p, q, hi, lo, r, rbest;
		size_t lo_g, gp, peak, best_g, skip;
		uint64_t now;
		uint32_t addr = 0;
		int i, ok, fixed = -1;

		p = e[g] + e[g + 2 * CHIP] + e[g + 7 * CHIP] + e[g + 9 * CHIP];
		q = e[g + 4 * CHIP] + e[g + 5 * CHIP] + e[g + 11 * CHIP] +
		    e[g + 12 * CHIP];
		/* strict: digital silence (all zero) is no preamble */
		if (p <= PRE_RATIO * q)
			continue;

		/* best grid point from one sample back to one bit ahead */
		lo_g = g >= step ? g - step + 1 : g;
		peak = lo_g;
		rbest = -1.0f;
		for (gp = lo_g; gp <= g + BIT; gp++) {
			r = preamble_ratio(e + gp, &hi, &lo);
			if (r > rbest) {
				rbest = r;
				peak = gp;
			}
		}
		r = preamble_ratio(e + peak, &hi, &lo);
		if (hi < GATE_RATIO * lo ||
		    third_pulse(e + peak) < PULSE_RATIO * lo) {
			if (peak > g)
				g = peak;
			continue;
		}
		d->st.candidates++;

		ok = 0;
		memset(&best, 0, sizeof(best));
		best.df = -1;
		best.margin = -1.0f;
		best_g = peak;
		now = (base_g + peak) / (uint64_t)d->f;
		for (i = 0; i < 5 && !ok; i++) {
			int off = retry_off[i];

			gp = off < 0 ? peak - (size_t)-off : peak + (size_t)off;
			slice_bits(e + gp + PREAMBLE, d->chip_len, &s);
			ok = validate(d, &s, now, &addr);
			if (ok || ((s.df == 17 || s.df == 18) &&
				   s.margin > best.margin)) {
				best = s;
				best_g = gp;
			}
		}
		if (!ok && d->fix && best.df >= 0) {
			fixed = try_fix(&best);
			if (fixed >= 0) {
				addr = frame_aa(best.msg);
				ok = 1;
			}
		}
		peak = best_g;
		if (!ok) {
			d->st.rejected++;
			if (peak > g)
				g = peak;
			continue;
		}

		{
			struct modes_frame fr;

			memcpy(fr.msg, best.msg, sizeof(fr.msg));
			fr.bits = best.bits;
			fr.df = best.df;
			fr.addr = addr;
			fr.fixed_bit = fixed;
			fr.ts_12mhz = (base_g + peak) * 6u / 5u;
			fr.level = best.level;
			fr.snr_db = 20.0f * log10f(hi / (lo + 1e-12f));
			d->st.accepted++;
			if (fixed >= 0)
				d->st.fixed++;
			fn(ctx, &fr);
		}
		/*
		 * No reply has its data bits inside a frame with a clean
		 * parity: a stronger one would have broken that parity, a
		 * weaker one cannot be sliced under it. Its preamble may
		 * overlap the last 8 us, as a reply right behind a squitter
		 * does. Address/parity frames prove less, so the search
		 * resumes after their preamble.
		 */
		skip = modes_crc_residual(best.msg, best.bits) == 0 ?
		       (size_t)(best.bits * BIT) : PREAMBLE;
		d->resume_g = base_g + peak + skip;
		g = peak + skip - step;
	}
	d->next_g = base_g + g;
}

static void run(struct demod *d, size_t n, demod_frame_fn fn, void *ctx)
{
	size_t mag_len = d->keep + n;
	uint64_t base = d->consumed - d->keep;
	size_t chip_len;

	d->consumed += n;
	d->st.samples += n;
	chip_len = chip_energy(d, mag_len);
	search(d, chip_len, base * (uint64_t)d->f, fn, ctx);

	d->keep = mag_len < d->carry ? mag_len : d->carry;
	memmove(d->mag, d->mag + (mag_len - d->keep), d->keep * sizeof(float));
}

void demod_process_cs16(struct demod *d, const int16_t *iq, size_t n,
			demod_frame_fn fn, void *ctx)
{
	const float scale = 1.0f / 32768.0f;

	while (n) {
		size_t c = n < d->max_block ? n : d->max_block;
		float *m = d->mag + d->keep;
		size_t i;

		for (i = 0; i < c; i++) {
			float x = (float)iq[2 * i] * scale;
			float y = (float)iq[2 * i + 1] * scale;

			m[i] = sqrtf(x * x + y * y);
		}
		run(d, c, fn, ctx);
		iq += 2 * c;
		n -= c;
	}
}

void demod_process_cf32(struct demod *d, const float *iq, size_t n,
			demod_frame_fn fn, void *ctx)
{
	while (n) {
		size_t c = n < d->max_block ? n : d->max_block;
		float *m = d->mag + d->keep;
		size_t i;

		for (i = 0; i < c; i++) {
			float v = sqrtf(iq[2 * i] * iq[2 * i] +
					iq[2 * i + 1] * iq[2 * i + 1]);

			/* NaN or infinity from a damaged file reads as silence */
			m[i] = v <= FLT_MAX ? v : 0.0f;
		}
		run(d, c, fn, ctx);
		iq += 2 * c;
		n -= c;
	}
}
