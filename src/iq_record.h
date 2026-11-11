/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_IQ_RECORD_H
#define HYDRASDR_ADSB_IQ_RECORD_H

#include <stddef.h>
#include <stdint.h>

/*
 * .raw12 recorder: the packed ADC groups as the device sent them (see
 * raw12.h), counted in complex samples, 3 bytes each. The meta file
 * "FILE.sigmf-meta" is written on close with the start time, the I/Q
 * sample rate and frequency, and one annotation per stretch of lost
 * samples.
 */

struct iq_record;

/* hw: board name, recorder: program name and version; both may be NULL. */
struct iq_record *iq_record_open(const char *path, double rate,
				 double freq_hz, const char *hw,
				 const char *recorder);

/* n samples, n * 3 bytes of groups; 0 on success, -1 once a write has
 * failed. */
int iq_record_write(struct iq_record *r, const uint8_t *groups, size_t n);

/* The next n samples written stand for lost ones (raw12_fill codes, so the
 * file keeps the stream's timing); listed in the meta file. */
void iq_record_lost(struct iq_record *r, uint64_t n);

uint64_t iq_record_samples(const struct iq_record *r);

/* Writes the meta file and closes; 0 if every write succeeded. */
int iq_record_close(struct iq_record *r);

#endif
