/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 */

#ifndef HYDRASDR_ADSB_RAW12_H
#define HYDRASDR_ADSB_RAW12_H

#include <stddef.h>
#include <stdint.h>

/*
 * .raw12: the packed 12-bit ADC stream of a HydraSDR as the device sends it
 * with USB packing (libhydrasdr HYDRASDR_SAMPLE_RAW), the HydraSDR Studio
 * .raw12 format. 12 bytes, three little-endian 32-bit words, carry 8 real
 * ADC codes, which the DDC turns into 4 complex samples.
 *
 * The DDC is the libhydrasdr 1.1.4 int16 converter ("int16_opt": DC
 * removal, fs/4 shift, 47-tap half-band filter, decimation by 2), so its
 * output is the INT16_IQ stream libhydrasdr gives for the same codes.
 */

#define RAW12_GROUP_BYTES	12
#define RAW12_GROUP_CODES	8
#define RAW12_GROUP_SAMPLES	4	/* complex samples out of one group */
#define RAW12_SAMPLE_BYTES	3	/* bytes per complex sample */
#define RAW12_CODE_MID		2048	/* ADC code of 0 V */

struct raw12_ddc {
	int32_t fir[1024];	/* history, written twice to skip the wrap */
	int16_t dly[12];
	int32_t fir_head;
	int32_t dly_head;
	int32_t old_e;
	int16_t old_x;
	int16_t old_y;
};

void raw12_ddc_init(struct raw12_ddc *d);

/* The 8 codes of one group, in stream order. */
void raw12_unpack(const uint8_t *in, uint16_t codes[RAW12_GROUP_CODES]);

/* The group carrying codes, the inverse of raw12_unpack. */
void raw12_pack(const uint16_t codes[RAW12_GROUP_CODES], uint8_t *out);

/* groups * 12 bytes in, groups * 4 complex samples (int16 I, Q) out. */
void raw12_convert(struct raw12_ddc *d, const uint8_t *in, size_t groups,
		   int16_t *iq);

/* groups of RAW12_CODE_MID codes, standing for lost samples */
void raw12_fill(uint8_t *out, size_t groups);

#endif
