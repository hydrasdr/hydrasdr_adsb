/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

/*
 * Aircraft table. A position is accepted only when it is consistent with
 * the previous one: an even/odd pair must be of the same kind (airborne or
 * surface) and close in time, a single half is decoded against the
 * aircraft's own recent position (or, on the surface, the receiver), and
 * a fix implying an impossible speed is dropped before it is stored.
 */

#include <stdlib.h>
#include <string.h>

#include "cpr.h"
#include "track.h"

#define BUCKETS		1024
/*
 * A receiver sees a few hundred; the limit bounds memory and the bucket
 * chains when a transmitter forges addresses.
 */
#define MAX_AIRCRAFT	4096
#define PAIR_AIR_SEC	10
#define PAIR_SURF_SEC	25
#define LOCAL_AIR_SEC	300	/* 92 km at 600 kt: well inside 180 NM */
#define LOCAL_SURF_SEC	60
#define STALE_POS_SEC	600
#define MAX_AIR_KT	1100.0
#define MAX_SURF_KT	250.0
#define SLACK_AIR_KM	0.5
#define SLACK_SURF_KM	0.1
#define REJECTS_RESET	3

struct tracker {
	struct aircraft *bucket[BUCKETS];
	int count;
	int have_rx;
	double rx_lat;
	double rx_lon;
	double max_km;
	uint64_t rejected;
};

static unsigned slot(uint32_t addr, enum addr_type ty)
{
	return (((addr * 2654435761u) >> 20) ^ (unsigned)ty) & (BUCKETS - 1);
}

struct tracker *track_create(void)
{
	return calloc(1, sizeof(struct tracker));
}

void track_destroy(struct tracker *t)
{
	int i;

	if (!t)
		return;
	for (i = 0; i < BUCKETS; i++) {
		while (t->bucket[i]) {
			struct aircraft *a = t->bucket[i];

			t->bucket[i] = a->next;
			free(a);
		}
	}
	free(t);
}

void track_set_receiver(struct tracker *t, double lat, double lon,
			double max_km)
{
	t->have_rx = 1;
	t->rx_lat = lat;
	t->rx_lon = lon;
	t->max_km = max_km;
}

static struct aircraft *find_or_create(struct tracker *t, uint32_t addr,
				       enum addr_type ty, uint64_t now)
{
	unsigned s = slot(addr, ty);
	struct aircraft *a;

	for (a = t->bucket[s]; a; a = a->next)
		if (a->addr == addr && a->addr_type == ty)
			return a;
	if (t->count >= MAX_AIRCRAFT)
		return NULL;
	a = calloc(1, sizeof(*a));
	if (!a)
		return NULL;
	a->addr = addr;
	a->addr_type = ty;
	a->first_seen = now;
	a->next = t->bucket[s];
	t->bucket[s] = a;
	t->count++;
	return a;
}

static int bit_index(uint32_t flag)
{
	int i = 0;

	while (flag > 1) {
		flag >>= 1;
		i++;
	}
	return i;
}

static void copy_fields(struct aircraft *a, const struct modes_msg *mm,
			uint64_t now)
{
	struct modes_msg *s = &a->st;
	uint32_t v = mm->valid;
	uint32_t f;

	if (v & MM_ALT_BARO)
		s->alt_baro = mm->alt_baro;
	if (v & MM_ALT_GEOM)
		s->alt_geom = mm->alt_geom;
	if (v & MM_GEOM_DELTA)
		s->geom_delta = mm->geom_delta;
	if (v & MM_SQUAWK)
		s->squawk = mm->squawk;
	if (v & MM_CALLSIGN)
		memcpy(s->callsign, mm->callsign, sizeof(s->callsign));
	if (v & MM_CATEGORY)
		s->category = mm->category;
	if (v & MM_GROUND)
		s->on_ground = mm->on_ground;
	if (v & MM_FS) {
		s->alert = mm->alert;
		s->spi = mm->spi;
	}
	if (v & MM_GS)
		s->gs = mm->gs;
	if (v & MM_TRACK)
		s->track = mm->track;
	if (v & MM_HEADING) {
		s->heading = mm->heading;
		s->heading_true = mm->heading_true;
	}
	if (v & MM_AIRSPEED) {
		s->airspeed = mm->airspeed;
		s->airspeed_tas = mm->airspeed_tas;
	}
	if (v & MM_VRATE) {
		s->vrate = mm->vrate;
		s->vrate_geom = mm->vrate_geom;
	}
	if (v & MM_EMERGENCY)
		s->emergency = mm->emergency;
	if (v & MM_VERSION)
		s->version = mm->version;
	if (v & MM_NACP)
		s->nacp = mm->nacp;
	if (v & MM_SIL)
		s->sil = mm->sil;
	if (v & MM_SEL_ALT) {
		s->sel_alt = mm->sel_alt;
		s->sel_alt_fms = mm->sel_alt_fms;
	}
	if (v & MM_QNH)
		s->qnh = mm->qnh;
	if (v & MM_SEL_HEADING)
		s->sel_heading = mm->sel_heading;

	a->valid |= v;
	for (f = 1; f && f <= v; f <<= 1)
		if (v & f)
			a->seen[bit_index(f)] = now;
}

/* 1 when the new fix is reachable from the stored one. */
static int plausible(const struct aircraft *a, double lat, double lon,
		     int surface, uint64_t now)
{
	double dt = (double)(now - a->pos_t) / TICKS_PER_SEC;
	double kt = surface ? MAX_SURF_KT : MAX_AIR_KT;
	double km = surface ? SLACK_SURF_KM : SLACK_AIR_KM;

	if ((a->valid & MM_GS) && now - a->seen[bit_index(MM_GS)] <
	    30 * TICKS_PER_SEC && a->st.gs * 1.5 + 100.0 < kt)
		kt = a->st.gs * 1.5 + 100.0;
	km += kt * 1.852 * dt / 3600.0;
	return cpr_distance_km(a->lat, a->lon, lat, lon) <= km;
}

static int has_recent_pos(const struct aircraft *a, uint64_t now,
			  unsigned secs)
{
	return a->pos_src != POS_NONE && now - a->pos_t <= secs * TICKS_PER_SEC;
}

static int update_position(struct tracker *t, struct aircraft *a,
			   const struct modes_msg *mm, uint64_t now)
{
	struct cpr_half *h = &a->cpr[mm->cpr_odd];
	const struct cpr_half *o = &a->cpr[!mm->cpr_odd];
	int surf = mm->cpr_surface;
	unsigned pair_sec = surf ? PAIR_SURF_SEC : PAIR_AIR_SEC;
	enum pos_source src = POS_NONE;
	double lat = 0.0, lon = 0.0;
	const struct cpr_half *e, *d;

	h->lat = mm->cpr_lat;
	h->lon = mm->cpr_lon;
	h->t = now;
	h->surface = surf;
	h->valid = 1;
	e = &a->cpr[0];
	d = &a->cpr[1];

	if (o->valid && o->surface == surf &&
	    now - o->t <= pair_sec * TICKS_PER_SEC) {
		if (!surf) {
			if (!cpr_global_air(e->lat, e->lon, d->lat, d->lon,
					    mm->cpr_odd, &lat, &lon))
				src = POS_GLOBAL;
		} else if (has_recent_pos(a, now, LOCAL_SURF_SEC)) {
			if (!cpr_global_surface(e->lat, e->lon, d->lat, d->lon,
						mm->cpr_odd, a->lat, a->lon,
						&lat, &lon))
				src = POS_GLOBAL;
		} else if (t->have_rx) {
			if (!cpr_global_surface(e->lat, e->lon, d->lat, d->lon,
						mm->cpr_odd, t->rx_lat,
						t->rx_lon, &lat, &lon))
				src = POS_GLOBAL;
		}
	}
	if (src == POS_NONE &&
	    has_recent_pos(a, now, surf ? LOCAL_SURF_SEC : LOCAL_AIR_SEC)) {
		if (!cpr_local(mm->cpr_lat, mm->cpr_lon, mm->cpr_odd, surf,
			       a->lat, a->lon, &lat, &lon))
			src = POS_LOCAL;
	}
	if (src == POS_NONE && surf && t->have_rx) {
		if (!cpr_local(mm->cpr_lat, mm->cpr_lon, mm->cpr_odd, 1,
			       t->rx_lat, t->rx_lon, &lat, &lon) &&
		    cpr_distance_km(t->rx_lat, t->rx_lon, lat, lon) <= 83.0)
			src = POS_RECEIVER;
	}
	if (src == POS_NONE)
		return 0;

	if (t->have_rx && t->max_km > 0.0 &&
	    cpr_distance_km(t->rx_lat, t->rx_lon, lat, lon) > t->max_km) {
		t->rejected++;
		return 0;
	}
	if (has_recent_pos(a, now, STALE_POS_SEC) &&
	    !plausible(a, lat, lon, surf, now)) {
		t->rejected++;
		/* the stored fix is the wrong one after repeated pairs */
		if (src != POS_GLOBAL || ++a->pos_rejects < REJECTS_RESET)
			return 0;
	}
	a->lat = lat;
	a->lon = lon;
	a->pos_t = now;
	a->pos_src = src;
	a->pos_surface = surf;
	a->pos_rejects = 0;
	return 1;
}

struct aircraft *track_update(struct tracker *t, const struct modes_msg *mm,
			      uint64_t now, float level, int *pos_updated)
{
	struct aircraft *a;

	*pos_updated = 0;
	a = find_or_create(t, mm->addr, mm->addr_type, now);
	if (!a)
		return NULL;
	a->last_seen = now;
	a->messages++;
	a->level = a->messages == 1 ? level : a->level * 0.9f + level * 0.1f;

	copy_fields(a, mm, now);
	if (mm->valid & MM_CPR)
		*pos_updated = update_position(t, a, mm, now);
	return a;
}

void track_expire(struct tracker *t, uint64_t now, uint64_t ttl)
{
	int i;

	for (i = 0; i < BUCKETS; i++) {
		struct aircraft **p = &t->bucket[i];

		while (*p) {
			struct aircraft *a = *p;

			if (now - a->last_seen > ttl) {
				*p = a->next;
				free(a);
				t->count--;
			} else {
				p = &a->next;
			}
		}
	}
}

static struct aircraft *scan_from(struct tracker *t, unsigned b)
{
	for (; b < BUCKETS; b++)
		if (t->bucket[b])
			return t->bucket[b];
	return NULL;
}

struct aircraft *track_first(struct tracker *t)
{
	return scan_from(t, 0);
}

struct aircraft *track_next(struct tracker *t, struct aircraft *a)
{
	if (a->next)
		return a->next;
	return scan_from(t, slot(a->addr, a->addr_type) + 1);
}

int track_count(const struct tracker *t)
{
	return t->count;
}

uint64_t track_rejected_fixes(const struct tracker *t)
{
	return t->rejected;
}
