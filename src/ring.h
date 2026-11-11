/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_RING_H
#define HYDRASDR_ADSB_RING_H

#include <stddef.h>
#include <stdint.h>

/*
 * Sample blocks from the libhydrasdr thread to the decoding thread, one
 * producer and one consumer. A sample is sample_bytes bytes of data (4 for
 * int16 I/Q) and aux_bytes bytes of aux (3 for the .raw12 groups the I/Q
 * came from, 0 for none). A push that finds every block full drops its
 * samples and adds them to the lost count of the next block, so the demod
 * keeps the stream time.
 */

struct ring_block {
	void *data;
	void *aux;		/* NULL without aux */
	size_t n;		/* complex samples */
	uint64_t lost;		/* samples lost just before these */
};

struct ring;

struct ring *ring_create(size_t blocks, size_t block_samples,
			 size_t sample_bytes, size_t aux_bytes);
void ring_destroy(struct ring *r);

/*
 * Producer: copies n samples of data and aux (NULL without aux) into
 * blocks; -1 once the ring is closed.
 */
int ring_push(struct ring *r, const void *data, const void *aux, size_t n,
	      uint64_t lost);

/*
 * Producer: 1 when a push of up to block_samples samples would find a free
 * block, 0 when every block is full, -1 once the ring is closed. Only the
 * consumer frees blocks, so a 1 holds until the next push.
 */
int ring_space(struct ring *r);

/* Producer: n samples dropped, carried as lost into the next block. */
void ring_drop(struct ring *r, size_t n);

/* Consumer: the oldest block, waiting up to timeout_ms; NULL if none. */
const struct ring_block *ring_peek(struct ring *r, int timeout_ms);
void ring_pop(struct ring *r);

/* Consumer: every later push fails, which ends the stream. */
void ring_close(struct ring *r);

/* Samples dropped because every block was full. */
uint64_t ring_dropped(struct ring *r);

#endif
