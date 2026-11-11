/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_INPUT_FILE_H
#define HYDRASDR_ADSB_INPUT_FILE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "raw12.h"

enum iq_format {
	IQ_NONE,
	IQ_CS16,	/* interleaved int16 */
	IQ_CF32,	/* interleaved float32 */
	IQ_CS12,	/* packed 12-bit, read back as int16 */
	IQ_RAW12,	/* .raw12 ADC groups through the DDC, read as int16 */
};

/*
 * .cs12: 3 bytes per complex sample, a little-endian 24-bit word with the
 * signed 12-bit I in bits 0..11 and Q in bits 12..23. Read as int16 at
 * code * 16, the int16 full scale.
 */
#define CS12_BYTES	3

struct iq_file {
	FILE *f;
	enum iq_format fmt;
	double rate;
	uint64_t left;		/* bytes of sample data left */
	/* IQ_RAW12 */
	struct raw12_ddc *ddc;
	uint8_t *groups;	/* read buffer */
	size_t g_next, g_have;	/* groups not yet converted */
	int fill_gaps;		/* the meta lists lost samples */
	uint8_t fill[RAW12_GROUP_BYTES];
	uint64_t gap;		/* fill samples read, not yet taken */
};

/*
 * Opens a 2-channel WAV (PCM16 or float32), a SigMF recording (data or
 * meta path, "x.sigmf-data"/"x.sigmf-meta" or "x.cs16"/"x.cs16.sigmf-meta",
 * ci16_le, ci12_le or cf32_le), or raw samples described by fmt and rate.
 * A raw file with fmt IQ_NONE is cs12 when its name ends in ".cs12", raw12
 * when it ends in ".raw12" and cs16 otherwise. A .raw12 file takes its
 * I/Q rate from rate, else from "FILE.sigmf-meta", else from the
 * "<rate>SPS" token of its name as HydraSDR Studio writes it.
 * Returns 0 on success.
 */
int iq_file_open(struct iq_file *in, const char *path, enum iq_format fmt,
		 double rate);
void iq_file_close(struct iq_file *in);

/*
 * Reads up to max complex samples into buf: max * 8 bytes of floats for
 * IQ_CF32, max * 4 bytes of int16 for the other formats. IQ_RAW12 reads
 * whole groups of 4 samples, so max must be at least 4. Returns the number
 * read.
 *
 * When the meta of a .raw12 file lists lost samples, the fill groups that
 * stand for them go through the DDC like the others but come back as a gap,
 * not as samples. One call returns samples or adds to the gap, never both,
 * so a gap taken after a call comes before the samples of the next one.
 */
size_t iq_file_read(struct iq_file *in, void *buf, size_t max);

/* Samples of the gap read since the last call; 0 for every other format. */
uint64_t iq_file_take_gap(struct iq_file *in);

#endif
