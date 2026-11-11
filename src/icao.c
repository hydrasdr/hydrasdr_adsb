/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <string.h>

#include "icao.h"

#define ICAO_PROBES	16

static uint32_t icao_hash(uint32_t a)
{
	a = (a ^ (a >> 13)) * 0x5bd1e995u;
	return (a ^ (a >> 15)) & (ICAO_TABLE_SIZE - 1);
}

void icao_init(struct icao_table *t, uint64_t ttl)
{
	memset(t, 0, sizeof(*t));
	t->ttl = ttl;
}

/*
 * Linear probing over a short window. An expired slot is reused, so the
 * table never fills with old aircraft; when all probes are live the oldest
 * one is replaced.
 */
void icao_add(struct icao_table *t, uint32_t addr, uint64_t now)
{
	uint32_t h = icao_hash(addr);
	uint32_t victim = h;
	uint64_t oldest = UINT64_MAX;
	int i;

	addr |= 0x80000000u;	/* slot in use */
	for (i = 0; i < ICAO_PROBES; i++) {
		uint32_t s = (h + (uint32_t)i) & (ICAO_TABLE_SIZE - 1);

		if (t->addr[s] == addr) {
			t->seen[s] = now;
			return;
		}
		if (t->addr[s] == 0 || now - t->seen[s] > t->ttl) {
			victim = s;
			oldest = 0;
		} else if (oldest && t->seen[s] < oldest) {
			victim = s;
			oldest = t->seen[s];
		}
	}
	t->addr[victim] = addr;
	t->seen[victim] = now;
}

int icao_known(const struct icao_table *t, uint32_t addr, uint64_t now)
{
	uint32_t h = icao_hash(addr);
	int i;

	addr |= 0x80000000u;
	for (i = 0; i < ICAO_PROBES; i++) {
		uint32_t s = (h + (uint32_t)i) & (ICAO_TABLE_SIZE - 1);

		if (t->addr[s] == addr)
			return now - t->seen[s] <= t->ttl;
	}
	return 0;
}
