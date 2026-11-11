/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_CPR_H
#define HYDRASDR_ADSB_CPR_H

#include <stdint.h>

/* Number of longitude zones at a latitude. */
int cpr_nl(double lat);

/*
 * Global decode of an even (0) and an odd (1) airborne CPR pair. odd_newer
 * selects the half the position belongs to. Returns 0 on success.
 */
int cpr_global_air(uint32_t lat0, uint32_t lon0, uint32_t lat1,
		   uint32_t lon1, int odd_newer, double *lat, double *lon);

/*
 * Global decode of a surface pair. The four solutions are 90 degrees
 * apart; the one nearest the reference (within 45 NM) is returned.
 */
int cpr_global_surface(uint32_t lat0, uint32_t lon0, uint32_t lat1,
		       uint32_t lon1, int odd_newer, double ref_lat,
		       double ref_lon, double *lat, double *lon);

/*
 * Local decode of one half against a reference that must lie within half
 * a zone of the true position (180 NM airborne, 45 NM surface).
 */
int cpr_local(uint32_t cpr_lat, uint32_t cpr_lon, int odd, int surface,
	      double ref_lat, double ref_lon, double *lat, double *lon);

/* Great circle distance in km. */
double cpr_distance_km(double lat0, double lon0, double lat1, double lon1);

#endif
