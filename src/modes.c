/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

/*
 * Mode S field decoding (ICAO Annex 10 Vol IV, DO-260B). Bit numbers in
 * this file count from 0 at the first transmitted bit.
 */

#include <math.h>
#include <string.h>

#include "modes.h"

#define RAD2DEG	(180.0 / 3.14159265358979323846)

static uint32_t bits(const uint8_t *m, int first, int n)
{
	uint32_t v = 0;
	int i;

	for (i = first; i < first + n; i++)
		v = v << 1 | (((uint32_t)m[i >> 3] >> (7 - (i & 7))) & 1u);
	return v;
}

/* 11-bit Gillham code ordered D2 D4 A1 A2 A4 B1 B2 B4 C1 C2 C4. */
static int gillham_ft(uint32_t g, int *alt)
{
	/* C1 C2 C4 give the 100 ft step 1..5; 000, 101, 111 are illegal */
	static const int c_pos[8] = { 0, 1, 3, 2, 5, 0, 4, 0 };
	int pos = c_pos[g & 7];
	uint32_t band = (g >> 3) & 0xFF;
	int step;

	if (!pos)
		return 0;
	band ^= band >> 4;
	band ^= band >> 2;
	band ^= band >> 1;
	/* the C sequence runs backwards in odd 500 ft bands */
	step = (band & 1) ? 3 - pos : pos - 3;
	*alt = -1000 + (int)band * 500 + step * 100;
	return 1;
}

int modes_ac12_ft(uint32_t f, int *alt)
{
	uint32_t g;

	if (!f)
		return 0;
	if (f & 0x10) {
		/* Q = 1: 25 ft steps, the 11 bits around Q form the count */
		uint32_t n = ((f & 0xFE0) >> 1) | (f & 0xF);

		*alt = (int)n * 25 - 1000;
		return 1;
	}
	/* Q = 0: C1 A1 C2 A2 C4 A4 B1 Q B2 D2 B4 D4 to Gillham order */
	g = ((f >> 2) & 1) << 10 | (f & 1) << 9 | ((f >> 10) & 1) << 8 |
	    ((f >> 8) & 1) << 7 | ((f >> 6) & 1) << 6 | ((f >> 5) & 1) << 5 |
	    ((f >> 3) & 1) << 4 | ((f >> 1) & 1) << 3 | ((f >> 11) & 1) << 2 |
	    ((f >> 9) & 1) << 1 | ((f >> 7) & 1);
	return gillham_ft(g, alt);
}

int modes_ac13_ft(uint32_t f, int *alt)
{
	if (f & 0x40)	/* M = 1: metric, not used in service */
		return 0;
	return modes_ac12_ft(((f & 0x1F80) >> 1) | (f & 0x3F), alt);
}

int modes_id13_squawk(uint32_t f)
{
	int a = (int)(((f >> 7) & 1) << 2 | ((f >> 9) & 1) << 1 | ((f >> 11) & 1));
	int b = (int)(((f >> 1) & 1) << 2 | ((f >> 3) & 1) << 1 | ((f >> 5) & 1));
	int c = (int)(((f >> 8) & 1) << 2 | ((f >> 10) & 1) << 1 | ((f >> 12) & 1));
	int d = (int)((f & 1) << 2 | ((f >> 2) & 1) << 1 | ((f >> 4) & 1));

	return a * 1000 + b * 100 + c * 10 + d;
}

/* DF4/5/20/21 flight status */
static void flight_status(struct modes_msg *mm, int fs)
{
	if (fs > 5)
		return;
	mm->alert = fs == 2 || fs == 3 || fs == 4;
	mm->spi = fs == 4 || fs == 5;
	mm->valid |= MM_FS;
	if (fs < 4) {
		mm->on_ground = fs == 1 || fs == 3;
		mm->valid |= MM_GROUND;
	}
}

static void me_ident(const uint8_t *m, struct modes_msg *mm)
{
	static const char cs[] =
		"#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
	int i, n = 8;

	mm->category = (0x0E - mm->tc) << 4 | (int)bits(m, 37, 3);
	mm->valid |= MM_CATEGORY;

	for (i = 0; i < 8; i++) {
		char c = cs[bits(m, 40 + 6 * i, 6)];

		if (c == '#')
			return;
		mm->callsign[i] = c;
	}
	while (n > 0 && mm->callsign[n - 1] == ' ')
		n--;
	mm->callsign[n] = '\0';
	mm->valid |= MM_CALLSIGN;
}

static void me_cpr(const uint8_t *m, struct modes_msg *mm, int surface)
{
	mm->cpr_surface = surface;
	mm->cpr_odd = (int)bits(m, 53, 1);
	mm->cpr_lat = bits(m, 54, 17);
	mm->cpr_lon = bits(m, 71, 17);
	mm->valid |= MM_CPR;
}

static void me_airborne_pos(const uint8_t *m, struct modes_msg *mm)
{
	uint32_t ss = bits(m, 37, 2);
	int alt;

	if (mm->tc != 0) {
		mm->on_ground = 0;
		mm->valid |= MM_GROUND;
	}
	if (ss == 1 || ss == 2)
		mm->alert = 1;
	if (ss == 3)
		mm->spi = 1;

	if (modes_ac12_ft(bits(m, 40, 12), &alt)) {
		if (mm->tc >= 20) {
			mm->alt_geom = alt;
			mm->valid |= MM_ALT_GEOM;
		} else {
			mm->alt_baro = alt;
			mm->valid |= MM_ALT_BARO;
		}
	}
	if (mm->tc != 0)
		me_cpr(m, mm, 0);
}

/* Surface movement code to kt, DO-260B table. */
static int movement_kt(int mov, double *kt)
{
	if (mov == 1)
		*kt = 0.0;
	else if (mov <= 8)
		*kt = 0.125 + (mov - 2) * 0.125;
	else if (mov <= 12)
		*kt = 1.0 + (mov - 9) * 0.25;
	else if (mov <= 38)
		*kt = 2.0 + (mov - 13) * 0.5;
	else if (mov <= 93)
		*kt = 15.0 + (mov - 39) * 1.0;
	else if (mov <= 108)
		*kt = 70.0 + (mov - 94) * 2.0;
	else if (mov <= 123)
		*kt = 100.0 + (mov - 109) * 5.0;
	else if (mov == 124)
		*kt = 175.0;
	else
		return 0;
	return mov != 0;
}

static void me_surface_pos(const uint8_t *m, struct modes_msg *mm)
{
	double kt;

	mm->on_ground = 1;
	mm->valid |= MM_GROUND;
	if (movement_kt((int)bits(m, 37, 7), &kt)) {
		mm->gs = kt;
		mm->valid |= MM_GS;
	}
	if (bits(m, 44, 1)) {
		mm->track = bits(m, 45, 7) * 360.0 / 128.0;
		mm->valid |= MM_TRACK;
	}
	me_cpr(m, mm, 1);
}

static void me_velocity(const uint8_t *m, struct modes_msg *mm)
{
	uint32_t vr, diff;

	mm->st = (int)bits(m, 37, 3);
	if (mm->st < 1 || mm->st > 4)
		return;

	if (mm->st <= 2) {
		int scale = mm->st == 2 ? 4 : 1;
		int ew = (int)bits(m, 46, 10);
		int ns = (int)bits(m, 57, 10);

		if (ew && ns) {
			double vx = (ew - 1) * scale;
			double vy = (ns - 1) * scale;

			if (bits(m, 45, 1))
				vx = -vx;
			if (bits(m, 56, 1))
				vy = -vy;
			mm->gs = sqrt(vx * vx + vy * vy);
			mm->track = atan2(vx, vy) * RAD2DEG;
			if (mm->track < 0.0)
				mm->track += 360.0;
			mm->valid |= MM_GS | MM_TRACK;
		}
	} else {
		int scale = mm->st == 4 ? 4 : 1;
		int as = (int)bits(m, 57, 10);

		if (bits(m, 45, 1)) {
			mm->heading = bits(m, 46, 10) * 360.0 / 1024.0;
			mm->heading_true = 0;
			mm->valid |= MM_HEADING;
		}
		if (as) {
			mm->airspeed = (as - 1) * scale;
			mm->airspeed_tas = (int)bits(m, 56, 1);
			mm->valid |= MM_AIRSPEED;
		}
	}

	vr = bits(m, 69, 9);
	if (vr) {
		mm->vrate = ((int)vr - 1) * 64 * (bits(m, 68, 1) ? -1 : 1);
		mm->vrate_geom = !bits(m, 67, 1);
		mm->valid |= MM_VRATE;
	}
	diff = bits(m, 81, 7);
	if (diff) {
		mm->geom_delta = ((int)diff - 1) * 25 * (bits(m, 80, 1) ? -1 : 1);
		mm->valid |= MM_GEOM_DELTA;
	}
}

static void me_status(const uint8_t *m, struct modes_msg *mm)
{
	mm->st = (int)bits(m, 37, 3);
	if (mm->st != 1)
		return;
	mm->emergency = (int)bits(m, 40, 3);
	mm->valid |= MM_EMERGENCY;
	if (bits(m, 43, 13)) {
		mm->squawk = modes_id13_squawk(bits(m, 43, 13));
		mm->valid |= MM_SQUAWK;
	}
}

static void me_target_state(const uint8_t *m, struct modes_msg *mm)
{
	uint32_t v;

	mm->st = (int)bits(m, 37, 2);
	if (mm->st != 1)	/* version 1 layout not decoded */
		return;
	v = bits(m, 41, 11);
	if (v) {
		mm->sel_alt = ((int)v - 1) * 32;
		mm->sel_alt_fms = (int)bits(m, 40, 1);
		mm->valid |= MM_SEL_ALT;
	}
	v = bits(m, 52, 9);
	if (v) {
		mm->qnh = 800.0 + (v - 1) * 0.8;
		mm->valid |= MM_QNH;
	}
	if (bits(m, 61, 1)) {
		mm->sel_heading = bits(m, 62, 9) * 180.0 / 256.0;
		mm->valid |= MM_SEL_HEADING;
	}
	mm->nacp = (int)bits(m, 71, 4);
	mm->sil = (int)bits(m, 76, 2);
	mm->valid |= MM_NACP | MM_SIL;
}

static void me_op_status(const uint8_t *m, struct modes_msg *mm)
{
	mm->st = (int)bits(m, 37, 3);
	if (mm->st > 1)
		return;
	mm->version = (int)bits(m, 72, 3);
	mm->valid |= MM_VERSION;
	if (mm->version == 0)	/* version 0 has no NACp or SIL here */
		return;
	mm->nacp = (int)bits(m, 76, 4);
	mm->sil = (int)bits(m, 82, 2);
	mm->valid |= MM_NACP | MM_SIL;
}

static void decode_me(const uint8_t *m, struct modes_msg *mm)
{
	mm->tc = (int)bits(m, 32, 5);
	switch (mm->tc) {
	case 0:
		me_airborne_pos(m, mm);
		break;
	case 1: case 2: case 3: case 4:
		me_ident(m, mm);
		break;
	case 5: case 6: case 7: case 8:
		me_surface_pos(m, mm);
		break;
	case 9: case 10: case 11: case 12: case 13: case 14: case 15:
	case 16: case 17: case 18: case 20: case 21: case 22:
		me_airborne_pos(m, mm);
		break;
	case 19:
		me_velocity(m, mm);
		break;
	case 28:
		me_status(m, mm);
		break;
	case 29:
		me_target_state(m, mm);
		break;
	case 31:
		me_op_status(m, mm);
		break;
	default:
		break;
	}
}

void modes_decode(const uint8_t *m, int nbits, uint32_t addr,
		  struct modes_msg *mm)
{
	int alt, fs;

	memset(mm, 0, sizeof(*mm));
	memcpy(mm->raw, m, (size_t)nbits / 8);
	mm->bits = nbits;
	mm->df = m[0] >> 3;
	mm->addr = addr;
	mm->addr_type = ADDR_ICAO;
	mm->tc = -1;

	switch (mm->df) {
	case 0:
	case 16:
		mm->on_ground = (int)bits(m, 5, 1);
		mm->valid |= MM_GROUND;
		if (modes_ac13_ft(bits(m, 19, 13), &alt)) {
			mm->alt_baro = alt;
			mm->valid |= MM_ALT_BARO;
		}
		break;
	case 4:
	case 20:
		fs = (int)bits(m, 5, 3);
		flight_status(mm, fs);
		if (modes_ac13_ft(bits(m, 19, 13), &alt)) {
			mm->alt_baro = alt;
			mm->valid |= MM_ALT_BARO;
		}
		break;
	case 5:
	case 21:
		fs = (int)bits(m, 5, 3);
		flight_status(mm, fs);
		if (bits(m, 19, 13)) {
			mm->squawk = modes_id13_squawk(bits(m, 19, 13));
			mm->valid |= MM_SQUAWK;
		}
		break;
	case 11:
		mm->ca = (int)bits(m, 5, 3);
		if (mm->ca == 4 || mm->ca == 5) {
			mm->on_ground = mm->ca == 4;
			mm->valid |= MM_GROUND;
		}
		break;
	case 17:
		mm->ca = (int)bits(m, 5, 3);
		decode_me(m, mm);
		break;
	case 18:
		mm->ca = (int)bits(m, 5, 3);
		/* CF 1 and 5: not an ICAO address; 2, 3, 5, 6: relayed */
		if (mm->ca == 1 || mm->ca == 5)
			mm->addr_type = ADDR_NON_ICAO;
		mm->tisb = mm->ca == 2 || mm->ca == 3 || mm->ca == 5 ||
			   mm->ca == 6;
		/* CF 3 (coarse TIS-B) and 4 (management) use other layouts */
		if (mm->ca != 3 && mm->ca != 4 && mm->ca != 7)
			decode_me(m, mm);
		break;
	default:
		break;
	}
}
