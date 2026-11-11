/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_ICAO_H
#define HYDRASDR_ADSB_ICAO_H

#include <stdint.h>

/*
 * Addresses seen on a CRC-clean frame (DF11 IID 0, DF17, DF18), used to
 * accept replies whose parity carries the address (DF0/4/5/16/20/21) and
 * DF11 replies with a non-zero interrogator code.
 */

#define ICAO_TABLE_SIZE	4096	/* power of two */

struct icao_table {
	uint32_t addr[ICAO_TABLE_SIZE];
	uint64_t seen[ICAO_TABLE_SIZE];
	uint64_t ttl;
};

void icao_init(struct icao_table *t, uint64_t ttl);
void icao_add(struct icao_table *t, uint32_t addr, uint64_t now);
int icao_known(const struct icao_table *t, uint32_t addr, uint64_t now);

#endif
