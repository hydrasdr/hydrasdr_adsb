/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_TRACK_H
#define HYDRASDR_ADSB_TRACK_H

#include <stdint.h>

#include "modes.h"

#define TICKS_PER_SEC	12000000ull	/* frame timestamps are 12 MHz */

enum pos_source {
	POS_NONE,
	POS_GLOBAL,		/* even/odd pair */
	POS_LOCAL,		/* one half against the last position */
	POS_RECEIVER,		/* one surface half against the receiver */
};

struct cpr_half {
	uint32_t lat;
	uint32_t lon;
	uint64_t t;
	int surface;
	int valid;
};

struct aircraft {
	uint32_t addr;
	enum addr_type addr_type;
	uint64_t first_seen;
	uint64_t last_seen;
	uint64_t messages;
	float level;		/* mean pulse magnitude, smoothed */

	struct modes_msg st;	/* latest value of every field */
	uint32_t valid;		/* MM_ flags ever received */
	uint64_t seen[21];	/* time of the last update per MM_ bit */

	struct cpr_half cpr[2];	/* even, odd */
	double lat;
	double lon;
	uint64_t pos_t;
	enum pos_source pos_src;
	int pos_surface;
	int pos_rejects;	/* consecutive fixes failing the speed test */

	struct aircraft *next;
};

struct tracker;

struct tracker *track_create(void);
void track_destroy(struct tracker *t);

/* Receiver position; max_km 0 disables the range check. */
void track_set_receiver(struct tracker *t, double lat, double lon,
			double max_km);

/*
 * Folds a decoded message into the table. Returns the aircraft, NULL if
 * the message carries no address. *pos_updated is set when this message
 * produced an accepted position.
 */
struct aircraft *track_update(struct tracker *t, const struct modes_msg *mm,
			      uint64_t now, float level, int *pos_updated);

/* Removes aircraft not heard for ttl ticks. */
void track_expire(struct tracker *t, uint64_t now, uint64_t ttl);

/* Iteration: first aircraft and next, NULL at the end. */
struct aircraft *track_first(struct tracker *t);
struct aircraft *track_next(struct tracker *t, struct aircraft *a);

int track_count(const struct tracker *t);

/* Fixes rejected by the range or speed tests (diagnostic). */
uint64_t track_rejected_fixes(const struct tracker *t);

#endif
