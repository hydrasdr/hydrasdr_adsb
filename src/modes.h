/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_MODES_H
#define HYDRASDR_ADSB_MODES_H

#include <stdint.h>

/* Fields present in a decoded message. */
enum {
	MM_ALT_BARO	= 1u << 0,
	MM_ALT_GEOM	= 1u << 1,
	MM_SQUAWK	= 1u << 2,
	MM_CALLSIGN	= 1u << 3,
	MM_CATEGORY	= 1u << 4,
	MM_CPR		= 1u << 5,
	MM_GS		= 1u << 6,	/* ground speed */
	MM_HEADING	= 1u << 7,
	MM_AIRSPEED	= 1u << 8,
	MM_VRATE	= 1u << 9,
	MM_GEOM_DELTA	= 1u << 10,	/* GNSS minus baro altitude */
	MM_GROUND	= 1u << 11,	/* airborne / on ground known */
	MM_FS		= 1u << 12,	/* alert and SPI known */
	MM_EMERGENCY	= 1u << 13,
	MM_VERSION	= 1u << 14,
	MM_NACP		= 1u << 15,
	MM_SIL		= 1u << 16,
	MM_SEL_ALT	= 1u << 17,
	MM_QNH		= 1u << 18,
	MM_SEL_HEADING	= 1u << 19,
	MM_TRACK	= 1u << 20,	/* track over ground */
};

enum addr_type {
	ADDR_ICAO,		/* 24-bit aircraft address */
	ADDR_NON_ICAO,		/* DF18 anonymous or track file number */
};

struct modes_msg {
	uint8_t raw[14];
	int bits;
	int df;
	uint32_t addr;
	enum addr_type addr_type;
	int tisb;		/* DF18 CF 2, 3, 5 or 6: relayed, not first party */
	int ca;			/* DF11/17 capability, DF18 control field */
	int tc;			/* ADS-B type code, -1 if no ME field */
	int st;			/* ADS-B subtype */
	uint32_t valid;		/* MM_ flags */

	int alt_baro;		/* ft */
	int alt_geom;		/* ft */
	int geom_delta;		/* ft */
	int squawk;		/* 4 octal digits, 7700 for 7700 */
	char callsign[9];
	int category;		/* 0xA0..0xD7: set letter and number */
	int on_ground;
	int alert;
	int spi;

	int cpr_odd;
	int cpr_surface;
	uint32_t cpr_lat;
	uint32_t cpr_lon;

	double gs;		/* kt */
	double track;		/* deg */
	double heading;		/* deg */
	int heading_true;
	double airspeed;	/* kt */
	int airspeed_tas;
	int vrate;		/* ft/min */
	int vrate_geom;		/* GNSS source */

	int emergency;		/* TC 28 emergency state */
	int version;		/* ADS-B version */
	int nacp;
	int sil;
	int sel_alt;		/* ft */
	int sel_alt_fms;
	double qnh;		/* hPa */
	double sel_heading;	/* deg */
};

/*
 * Decodes a frame already validated by the demodulator. addr is the
 * address it validated (from the frame or from the parity).
 */
void modes_decode(const uint8_t *msg, int bits, uint32_t addr,
		  struct modes_msg *mm);

/* Altitude in ft from the 13-bit AC field (DF0/4/16/20). 0 if invalid. */
int modes_ac13_ft(uint32_t ac13, int *alt);
/* Altitude in ft from the 12-bit ADS-B field. 0 if invalid. */
int modes_ac12_ft(uint32_t ac12, int *alt);
/* Mode A code from the 13-bit ID field, as 4 octal digits in decimal. */
int modes_id13_squawk(uint32_t id13);

#endif
