/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include "output.h"

static const char hexd[] = "0123456789ABCDEF";

size_t out_avr(char *buf, const struct modes_frame *f, int mlat)
{
	size_t n = 0;
	int i;

	if (mlat) {
		buf[n++] = '@';
		for (i = 44; i >= 0; i -= 4)
			buf[n++] = hexd[(f->ts_12mhz >> i) & 15];
	} else {
		buf[n++] = '*';
	}
	for (i = 0; i < f->bits / 8; i++) {
		buf[n++] = hexd[f->msg[i] >> 4];
		buf[n++] = hexd[f->msg[i] & 15];
	}
	buf[n++] = ';';
	buf[n++] = '\n';
	return n;
}

static size_t put_esc(unsigned char *buf, size_t n, unsigned char b)
{
	buf[n++] = b;
	if (b == 0x1a)
		buf[n++] = b;
	return n;
}

/* Full scale level to 0..255; NaN, negative and overrange are clamped. */
static unsigned char level_byte(float level)
{
	float v = level * 255.0f + 0.5f;

	if (!(v > 0.0f))
		return 0;
	if (v >= 255.0f)
		return 255;
	return (unsigned char)v;
}

size_t out_beast(unsigned char *buf, const struct modes_frame *f)
{
	size_t n = 0;
	int i;

	buf[n++] = 0x1a;
	buf[n++] = f->bits == 112 ? '3' : '2';
	for (i = 40; i >= 0; i -= 8)
		n = put_esc(buf, n, (unsigned char)(f->ts_12mhz >> i));
	n = put_esc(buf, n, level_byte(f->level));
	for (i = 0; i < f->bits / 8; i++)
		n = put_esc(buf, n, f->msg[i]);
	return n;
}

static void wall_clock(char *date, char *tod)
{
	struct tm tm;
	unsigned ms;
	time_t t;
#ifdef _WIN32
	FILETIME ft;
	unsigned long long v;

	GetSystemTimeAsFileTime(&ft);
	v = ((unsigned long long)ft.dwHighDateTime << 32 | ft.dwLowDateTime) /
	    10000ull - 11644473600000ull;
	t = (time_t)(v / 1000);
	ms = (unsigned)(v % 1000);
	localtime_s(&tm, &t);
#else
	struct timeval tv;

	gettimeofday(&tv, NULL);
	t = tv.tv_sec;
	ms = (unsigned)(tv.tv_usec / 1000);
	localtime_r(&t, &tm);
#endif
	strftime(date, 11, "%Y/%m/%d", &tm);
	strftime(tod, 9, "%H:%M:%S", &tm);
	snprintf(tod + 8, 5, ".%03u", ms);
}

static int sbs_type(const struct modes_msg *mm)
{
	switch (mm->df) {
	case 0:
	case 16:
		return 7;
	case 4:
	case 20:
		return 5;
	case 5:
	case 21:
		return 6;
	case 11:
		return 8;
	case 17:
	case 18:
		if (mm->tc >= 1 && mm->tc <= 4)
			return 1;
		if (mm->tc >= 5 && mm->tc <= 8)
			return 2;
		if ((mm->tc >= 9 && mm->tc <= 18) ||
		    (mm->tc >= 20 && mm->tc <= 22))
			return 3;
		if (mm->tc == 19 && (mm->valid & (MM_GS | MM_VRATE)))
			return 4;
		return 0;
	default:
		return 0;
	}
}

/* Bounded line writer: once a field does not fit, the line is dropped. */
struct line {
	char *p;
	size_t left;
};

static void put(struct line *l, const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(l->p, l->left, fmt, ap);
	va_end(ap);
	if (n < 0 || (size_t)n >= l->left) {
		l->left = 0;
		return;
	}
	l->p += n;
	l->left -= (size_t)n;
}

static void flag(struct line *l, int known, int v)
{
	if (known)
		put(l, ",%d", v ? -1 : 0);
	else
		put(l, ",");
}

size_t out_sbs(char *buf, const struct modes_msg *mm,
	       const struct aircraft *ac, int pos)
{
	struct line l = { buf, OUT_MAX };
	char date[11], tod[13];
	int type = sbs_type(mm);
	uint32_t v = mm->valid;
	int emerg;

	if (!type)
		return 0;
	wall_clock(date, tod);
	put(&l, "MSG,%d,1,1,%s%06X,1,%s,%s,%s,%s", type,
	    mm->addr_type == ADDR_NON_ICAO ? "~" : "",
	    (unsigned)(mm->addr & 0xFFFFFF), date, tod, date, tod);

	if (v & MM_CALLSIGN)
		put(&l, ",%.8s", mm->callsign);
	else
		put(&l, ",");
	if (v & MM_ALT_BARO)
		put(&l, ",%d", mm->alt_baro);
	else
		put(&l, ",");
	if (v & MM_GS)
		put(&l, ",%.0f", mm->gs);
	else
		put(&l, ",");
	if (v & MM_TRACK)
		put(&l, ",%.0f", mm->track >= 359.5 ? 0.0 : mm->track);
	else
		put(&l, ",");
	if (pos && ac)
		put(&l, ",%.5f,%.5f", ac->lat, ac->lon);
	else
		put(&l, ",,");
	if (v & MM_VRATE)
		put(&l, ",%d", mm->vrate);
	else
		put(&l, ",");
	if (v & MM_SQUAWK)
		put(&l, ",%04d", mm->squawk);
	else
		put(&l, ",");

	emerg = (v & MM_SQUAWK) && (mm->squawk == 7500 ||
				    mm->squawk == 7600 || mm->squawk == 7700);
	flag(&l, (v & MM_FS) != 0, mm->alert);
	flag(&l, (v & MM_SQUAWK) != 0, emerg);
	flag(&l, (v & MM_FS) != 0, mm->spi);
	flag(&l, (v & MM_GROUND) != 0, mm->on_ground);
	put(&l, "\r\n");
	if (!l.left)
		return 0;
	return (size_t)(l.p - buf);
}
