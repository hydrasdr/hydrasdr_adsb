/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_OUTPUT_H
#define HYDRASDR_ADSB_OUTPUT_H

#include <stddef.h>

#include "demod.h"
#include "modes.h"
#include "track.h"

#define OUT_MAX	256

/* "*HEX;\n", or "@TTTTTTTTTTTTHEX;\n" with mlat. Returns the length. */
size_t out_avr(char *buf, const struct modes_frame *f, int mlat);

/* Beast binary frame with 0x1a escaping. Returns the length. */
size_t out_beast(unsigned char *buf, const struct modes_frame *f);

/*
 * SBS BaseStation line, as dump1090 writes it on port 30003. pos is set
 * when this message produced the position stored in ac. Returns 0 for a
 * message with no SBS form.
 */
size_t out_sbs(char *buf, const struct modes_msg *mm,
	       const struct aircraft *ac, int pos);

#endif
