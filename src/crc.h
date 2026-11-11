/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_CRC_H
#define HYDRASDR_ADSB_CRC_H

#include <stdint.h>

#define MODES_LONG_BITS		112
#define MODES_SHORT_BITS	56
#define MODES_LONG_BYTES	(MODES_LONG_BITS / 8)
#define MODES_SHORT_BYTES	(MODES_SHORT_BITS / 8)

void modes_crc_init(void);

/*
 * Mode S parity check. Returns the 24-bit remainder of the data bits XORed
 * with the parity field: 0 for a clean DF11/17/18 frame, the address for an
 * error free DF0/4/5/16/20/21 reply.
 */
uint32_t modes_crc_residual(const uint8_t *msg, int bits);

/*
 * Index of the only bit whose syndrome equals residual, or -1 when no bit or
 * more than one bit matches. bits is 56 or 112.
 */
int modes_crc_find_bit(uint32_t residual, int bits);

#endif
