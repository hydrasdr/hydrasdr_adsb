/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_DEMOD_H
#define HYDRASDR_ADSB_DEMOD_H

#include <stddef.h>
#include <stdint.h>

#include "crc.h"

/* Supported input rates: the HydraSDR 2.5, 5 and 10 MS/s streams. */
int demod_rate_supported(double rate);

struct modes_frame {
	uint8_t msg[MODES_LONG_BYTES];
	int bits;
	int df;
	uint32_t addr;
	int fixed_bit;		/* corrected bit index, -1 if none */
	uint64_t ts_12mhz;	/* start of the preamble, 12 MHz ticks */
	float level;		/* mean pulse magnitude, full scale 1.0 */
	float snr_db;		/* pulse chips over gap chips */
};

typedef void (*demod_frame_fn)(void *ctx, const struct modes_frame *f);

struct demod_stats {
	uint64_t samples;
	uint64_t candidates;	/* preambles that passed the gate */
	uint64_t accepted;
	uint64_t fixed;
	uint64_t rejected;	/* sliced but no valid frame */
};

struct demod;

/* max_block: largest number of samples given to one process call. */
struct demod *demod_create(double rate, size_t max_block);
void demod_destroy(struct demod *d);
void demod_set_fix(struct demod *d, int on);

/* Interleaved I/Q, int16 or float (full scale 1.0). */
void demod_process_cs16(struct demod *d, const int16_t *iq, size_t n,
			demod_frame_fn fn, void *ctx);
void demod_process_cf32(struct demod *d, const float *iq, size_t n,
			demod_frame_fn fn, void *ctx);

const struct demod_stats *demod_get_stats(const struct demod *d);

/*
 * samples were lost between two process calls: timestamps move on by that
 * many samples and no frame is sliced across the gap.
 */
void demod_gap(struct demod *d, uint64_t samples);

#endif
