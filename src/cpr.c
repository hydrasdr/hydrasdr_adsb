/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

/* Compact Position Reporting, DO-260B appendix A. */

#include <math.h>

#include "cpr.h"

#define PI		3.14159265358979323846
#define CPR_SCALE	131072.0	/* 2^17 */
#define NZ		15

static double mod_pos(double x, double y)
{
	double r = fmod(x, y);

	return r < 0.0 ? r + y : r;
}

int cpr_nl(double lat)
{
	double a, c;

	lat = fabs(lat);
	if (lat == 0.0)
		return 59;
	if (lat == 87.0)
		return 2;
	if (lat > 87.0)
		return 1;
	a = 1.0 - cos(PI / (2.0 * NZ));
	c = cos(PI / 180.0 * lat);
	return (int)floor(2.0 * PI / acos(1.0 - a / (c * c)));
}

static double wrap_lon(double lon)
{
	return lon >= 180.0 ? lon - 360.0 : lon;
}

/* Latitudes of both halves; span is 360 airborne, 90 surface. */
static void global_lat(uint32_t lat0, uint32_t lat1, double span,
		       double *rlat0, double *rlat1)
{
	double y0 = lat0 / CPR_SCALE, y1 = lat1 / CPR_SCALE;
	double j = floor(59.0 * y0 - 60.0 * y1 + 0.5);

	*rlat0 = span / 60.0 * (mod_pos(j, 60.0) + y0);
	*rlat1 = span / 59.0 * (mod_pos(j, 59.0) + y1);
}

/* Longitude of the selected half, before the 90 degree ambiguity. */
static double global_lon(uint32_t lon0, uint32_t lon1, int odd_newer,
			 double rlat, double span)
{
	double x0 = lon0 / CPR_SCALE, x1 = lon1 / CPR_SCALE;
	int nl = cpr_nl(rlat);
	int ni = nl - odd_newer;
	double m = floor(x0 * (nl - 1) - x1 * nl + 0.5);

	if (ni < 1)
		ni = 1;
	return span / ni * (mod_pos(m, ni) + (odd_newer ? x1 : x0));
}

int cpr_global_air(uint32_t lat0, uint32_t lon0, uint32_t lat1,
		   uint32_t lon1, int odd_newer, double *lat, double *lon)
{
	double rlat0, rlat1, rlat;

	global_lat(lat0, lat1, 360.0, &rlat0, &rlat1);
	if (rlat0 >= 270.0)
		rlat0 -= 360.0;
	if (rlat1 >= 270.0)
		rlat1 -= 360.0;
	if (fabs(rlat0) > 90.0 || fabs(rlat1) > 90.0 ||
	    cpr_nl(rlat0) != cpr_nl(rlat1))
		return -1;
	rlat = odd_newer ? rlat1 : rlat0;
	*lat = rlat;
	*lon = wrap_lon(global_lon(lon0, lon1, odd_newer, rlat, 360.0));
	return 0;
}

int cpr_global_surface(uint32_t lat0, uint32_t lon0, uint32_t lat1,
		       uint32_t lon1, int odd_newer, double ref_lat,
		       double ref_lon, double *lat, double *lon)
{
	double rlat0, rlat1, rlat, rlon, best = 1e30;
	int k;

	global_lat(lat0, lat1, 90.0, &rlat0, &rlat1);
	/* the decode gives the northern solution; the southern is 90 below */
	rlat = odd_newer ? rlat1 : rlat0;
	if (fabs(rlat - 90.0 - ref_lat) < fabs(rlat - ref_lat)) {
		rlat0 -= 90.0;
		rlat1 -= 90.0;
		rlat -= 90.0;
	}
	if (cpr_nl(rlat0) != cpr_nl(rlat1))
		return -1;
	rlon = global_lon(lon0, lon1, odd_newer, rlat, 90.0);
	for (k = 0; k < 4; k++) {
		double l = mod_pos(rlon + 90.0 * k + 180.0, 360.0) - 180.0;
		double d = cpr_distance_km(ref_lat, ref_lon, rlat, l);

		if (d < best) {
			best = d;
			*lon = l;
		}
	}
	*lat = rlat;
	return best <= 83.0 ? 0 : -1;	/* 45 NM */
}

int cpr_local(uint32_t cpr_lat, uint32_t cpr_lon, int odd, int surface,
	      double ref_lat, double ref_lon, double *lat, double *lon)
{
	double span = surface ? 90.0 : 360.0;
	double y = cpr_lat / CPR_SCALE, x = cpr_lon / CPR_SCALE;
	double dlat = span / (60.0 - odd);
	double dlon, j, m, rlat, rlon;
	int nl;

	j = floor(ref_lat / dlat) +
	    floor(0.5 + mod_pos(ref_lat, dlat) / dlat - y);
	rlat = dlat * (j + y);
	if (fabs(rlat) > 90.0)
		return -1;
	nl = cpr_nl(rlat) - odd;
	dlon = span / (nl < 1 ? 1 : nl);
	m = floor(ref_lon / dlon) +
	    floor(0.5 + mod_pos(ref_lon, dlon) / dlon - x);
	rlon = dlon * (m + x);
	if (rlon >= 180.0)
		rlon -= 360.0;
	else if (rlon < -180.0)
		rlon += 360.0;
	*lat = rlat;
	*lon = rlon;
	return 0;
}

double cpr_distance_km(double lat0, double lon0, double lat1, double lon1)
{
	double p0 = lat0 * PI / 180.0, p1 = lat1 * PI / 180.0;
	double dp = p1 - p0, dl = (lon1 - lon0) * PI / 180.0;
	double a = sin(dp / 2) * sin(dp / 2) +
		   cos(p0) * cos(p1) * sin(dl / 2) * sin(dl / 2);

	return 6371.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}
