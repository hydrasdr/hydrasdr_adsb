/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_INPUT_HYDRASDR_H
#define HYDRASDR_ADSB_INPUT_HYDRASDR_H

#include <stddef.h>
#include <stdint.h>

/*
 * Live samples from a HydraSDR device through libhydrasdr: int16 I/Q, or
 * with raw12 the packed 12-bit ADC stream (see raw12.h).
 */

struct live_config {
	uint64_t serial;	/* 0: first device found */
	uint32_t rate;
	uint64_t freq_hz;
	int gain;		/* preset value, -1: device linearity default */
	int sensitivity;	/* gain is a sensitivity preset, not linearity */
	int bias_tee;
	int raw12;		/* packed ADC stream instead of int16 I/Q */
};

/*
 * Called on the libhydrasdr thread with n complex samples, int16 I/Q or
 * n * RAW12_SAMPLE_BYTES bytes of .raw12 groups (n a multiple of 4); lost
 * is the number of samples the library dropped before them. Returns
 * non-zero to end the stream.
 */
typedef int (*live_block_fn)(void *ctx, const void *data, size_t n,
			     uint64_t lost);

struct live;

/* Opens and configures the device; prints what it found on stderr. */
struct live *live_open(const struct live_config *c);
const char *live_board(const struct live *l);
int live_start(struct live *l, live_block_fn fn, void *ctx);
int live_streaming(struct live *l);

/* Stops the stream (no callback runs after it), prints the USB counts. */
void live_close(struct live *l);

#endif
