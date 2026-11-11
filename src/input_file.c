/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <stdlib.h>
#include <string.h>

#include "input_file.h"
#include "raw12.h"

#define RAW12_READ_GROUPS	16384	/* 192 KB read buffer */

static size_t sample_bytes(enum iq_format fmt)
{
	if (fmt == IQ_CF32)
		return 8;
	if (fmt == IQ_RAW12)
		return RAW12_SAMPLE_BYTES;
	return fmt == IQ_CS12 ? CS12_BYTES : 4;
}

static int16_t cs12_code(uint32_t v)
{
	int32_t c = (int32_t)(v & 0xFFF);

	if (c >= 0x800)
		c -= 0x1000;
	return (int16_t)(c * 16);
}

static int ends_with(const char *s, const char *tail)
{
	size_t n = strlen(s), t = strlen(tail);

	return n >= t && strcmp(s + n - t, tail) == 0;
}

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
	       (uint32_t)p[3] << 24;
}

static uint16_t le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | p[1] << 8);
}

/* 64-bit count: a 0xFFFFFFFF chunk size plus its pad byte does not wrap. */
static int skip(FILE *f, uint64_t n)
{
	uint8_t tmp[512];

	while (n) {
		size_t c = n < sizeof(tmp) ? (size_t)n : sizeof(tmp);

		if (fread(tmp, 1, c, f) != c)
			return -1;
		n -= c;
	}
	return 0;
}

/* 1: WAV parsed, 0: not a WAV, -1: unsupported WAV. */
static int parse_wav(struct iq_file *in)
{
	uint8_t h[16];
	int fmt_ok = 0;

	if (fread(h, 1, 12, in->f) != 12 || memcmp(h, "RIFF", 4) ||
	    memcmp(h + 8, "WAVE", 4))
		return 0;

	for (;;) {
		uint32_t sz;

		if (fread(h, 1, 8, in->f) != 8)
			return -1;
		sz = le32(h + 4);
		if (!memcmp(h, "fmt ", 4)) {
			uint16_t tag, ch, bits;

			if (sz < 16 || fread(h, 1, 16, in->f) != 16)
				return -1;
			tag = le16(h);
			ch = le16(h + 2);
			bits = le16(h + 14);
			in->rate = le32(h + 4);
			fmt_ok = 0;
			if (ch == 2 && tag == 1 && bits == 16) {
				in->fmt = IQ_CS16;
				fmt_ok = 1;
			} else if (ch == 2 && tag == 3 && bits == 32) {
				in->fmt = IQ_CF32;
				fmt_ok = 1;
			}
			if (skip(in->f, (uint64_t)sz - 16 + (sz & 1)))
				return -1;
		} else if (!memcmp(h, "data", 4)) {
			/* 0 and 0xFFFFFFFF: length not known by the writer */
			in->left = (sz == 0 || sz == 0xFFFFFFFFu) ?
				UINT64_MAX : sz;
			return fmt_ok ? 1 : -1;
		} else if (skip(in->f, (uint64_t)sz + (sz & 1))) {
			return -1;
		}
	}
}

/* Value of a top level key in the SigMF meta text, or NULL. */
static const char *json_value(const char *buf, const char *key)
{
	const char *p = strstr(buf, key);

	if (!p)
		return NULL;
	p = strchr(p + strlen(key), ':');
	if (!p)
		return NULL;
	p++;
	while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
		p++;
	return p;
}

static int parse_sigmf_meta(const char *path, struct iq_file *in)
{
	char buf[16384];
	const char *p;
	size_t n;
	FILE *f = fopen(path, "rb");

	if (!f)
		return -1;
	n = fread(buf, 1, sizeof(buf) - 1, f);
	fclose(f);
	buf[n] = '\0';

	p = json_value(buf, "\"core:sample_rate\"");
	if (!p)
		return -1;
	in->rate = strtod(p, NULL);
	/* the annotations of the lost stretches in a .raw12 meta */
	in->fill_gaps = json_value(buf, "\"core:sample_count\"") != NULL;

	p = json_value(buf, "\"core:datatype\"");
	if (p && !strncmp(p, "\"ci16_le\"", 9))
		in->fmt = IQ_CS16;
	else if (p && !strncmp(p, "\"ci12_le\"", 9))
		in->fmt = IQ_CS12;
	else if (p && !strncmp(p, "\"cf32_le\"", 9))
		in->fmt = IQ_CF32;
	else
		return -1;
	return in->rate > 0.0 ? 0 : -1;
}

/*
 * SigMF pair from either file name. HydraSDR Studio writes "x.cs16" with
 * "x.cs16.sigmf-meta"; the standard names are "x.sigmf-data" and
 * "x.sigmf-meta".
 */
static int open_sigmf(struct iq_file *in, const char *path)
{
	static const char ext_meta[] = ".sigmf-meta";
	static const char ext_data[] = ".sigmf-data";
	const size_t ext = sizeof(ext_meta) - 1;
	size_t n = strlen(path);
	size_t size = n + sizeof(ext_meta);
	char *meta = malloc(size);
	char *data = malloc(size);
	int is_meta = ends_with(path, ext_meta);
	int ret = -1;

	if (!meta || !data)
		goto out;

	if (is_meta) {
		snprintf(meta, size, "%s", path);
		snprintf(data, size, "%.*s", (int)(n - ext), path);
	} else if (ends_with(path, ext_data)) {
		snprintf(meta, size, "%.*s%s", (int)(n - ext), path, ext_meta);
		snprintf(data, size, "%s", path);
	} else {
		snprintf(meta, size, "%s%s", path, ext_meta);
		snprintf(data, size, "%s", path);
	}

	if (parse_sigmf_meta(meta, in))
		goto out;
	in->f = fopen(data, "rb");
	if (!in->f && is_meta) {
		snprintf(data, size, "%.*s%s", (int)(n - ext), path, ext_data);
		in->f = fopen(data, "rb");
	}
	if (!in->f)
		goto out;
	in->left = UINT64_MAX;
	ret = 0;
out:
	free(meta);
	free(data);
	return ret;
}

/* core:sample_rate and the lost stretches of "path.sigmf-meta", into m. */
static void raw12_meta(const char *path, struct iq_file *m)
{
	static const char ext[] = ".sigmf-meta";
	size_t size = strlen(path) + sizeof(ext);
	char *meta = malloc(size);

	memset(m, 0, sizeof(*m));
	if (!meta)
		return;
	snprintf(meta, size, "%s%s", path, ext);
	/* both are read before the datatype check fails */
	parse_sigmf_meta(meta, m);
	free(meta);
}

/* The rate of a HydraSDR Studio name, "<freq>Hz_<rate>SPS_...", or 0. */
static double name_rate(const char *path)
{
	const char *b = path, *p, *s;
	double r = 0.0;

	for (p = path; *p; p++)
		if (*p == '/' || *p == '\\')
			b = p + 1;
	for (s = strstr(b, "SPS"); s; s = strstr(s + 3, "SPS")) {
		for (p = s; p > b && p[-1] >= '0' && p[-1] <= '9'; p--)
			;
		if (p == s || (p > b && p[-1] != '_'))
			continue;
		for (r = 0.0; p < s; p++)
			r = r * 10.0 + (*p - '0');
		return r;
	}
	return 0.0;
}

static int open_raw12(struct iq_file *in, const char *path, double rate)
{
	struct iq_file m;

	raw12_meta(path, &m);
	if (rate <= 0.0)
		rate = m.rate;
	if (rate <= 0.0)
		rate = name_rate(path);
	if (rate <= 0.0)
		return -1;
	in->ddc = malloc(sizeof(*in->ddc));
	in->groups = malloc(RAW12_READ_GROUPS * RAW12_GROUP_BYTES);
	in->f = fopen(path, "rb");
	if (!in->ddc || !in->groups || !in->f) {
		iq_file_close(in);
		return -1;
	}
	raw12_ddc_init(in->ddc);
	in->fill_gaps = m.fill_gaps;
	raw12_fill(in->fill, 1);
	in->fmt = IQ_RAW12;
	in->rate = rate;
	in->left = UINT64_MAX;
	return 0;
}

int iq_file_open(struct iq_file *in, const char *path, enum iq_format fmt,
		 double rate)
{
	int w;

	memset(in, 0, sizeof(*in));
	if (fmt == IQ_RAW12 || (fmt == IQ_NONE && ends_with(path, ".raw12")))
		return open_raw12(in, path, rate);

	if (ends_with(path, ".sigmf-meta") || ends_with(path, ".sigmf-data"))
		return open_sigmf(in, path);

	in->f = fopen(path, "rb");
	if (!in->f)
		return -1;

	w = parse_wav(in);
	if (w == 1)
		return 0;
	if (w < 0) {
		iq_file_close(in);
		return -1;
	}

	/* raw samples, described by a sibling meta file or the options */
	fclose(in->f);
	in->f = NULL;
	if (fmt == IQ_NONE && rate <= 0.0 && open_sigmf(in, path) == 0)
		return 0;
	if (rate <= 0.0)
		return -1;
	in->f = fopen(path, "rb");
	if (!in->f)
		return -1;
	if (fmt == IQ_NONE)
		fmt = ends_with(path, ".cs12") ? IQ_CS12 : IQ_CS16;
	in->fmt = fmt;
	in->rate = rate;
	in->left = UINT64_MAX;
	return 0;
}

void iq_file_close(struct iq_file *in)
{
	if (in->f)
		fclose(in->f);
	in->f = NULL;
	free(in->ddc);
	in->ddc = NULL;
	free(in->groups);
	in->groups = NULL;
	in->g_next = in->g_have = 0;
}

static int is_fill(const struct iq_file *in, const uint8_t *g)
{
	return in->fill_gaps && !memcmp(g, in->fill, RAW12_GROUP_BYTES);
}

/*
 * Runs of real groups go to out, runs of fill groups through out to the
 * gap. A run of the other kind ends the call, so one call returns samples
 * or adds to the gap, never both.
 */
static size_t read_raw12(struct iq_file *in, int16_t *out, size_t max)
{
	size_t room = max / RAW12_GROUP_SAMPLES, done = 0, gapped = 0;

	while (done < room) {
		const uint8_t *g;
		size_t c, lim;
		int fill;

		if (in->g_next == in->g_have) {
			in->g_have = fread(in->groups, RAW12_GROUP_BYTES,
					   RAW12_READ_GROUPS, in->f);
			in->g_next = 0;
			if (!in->g_have)
				break;
		}
		g = in->groups + in->g_next * RAW12_GROUP_BYTES;
		fill = is_fill(in, g);
		if (fill ? done : gapped)
			break;
		lim = in->g_have - in->g_next;
		if (lim > room - done)
			lim = room - done;
		for (c = 1; c < lim &&
		     is_fill(in, g + c * RAW12_GROUP_BYTES) == fill; c++)
			;
		/* a fill run lands at out + 0 and is dropped there */
		raw12_convert(in->ddc, g, c,
			      out + done * RAW12_GROUP_SAMPLES * 2);
		in->g_next += c;
		if (fill)
			gapped += c;
		else
			done += c;
	}
	in->gap += (uint64_t)gapped * RAW12_GROUP_SAMPLES;
	return done * RAW12_GROUP_SAMPLES;
}

uint64_t iq_file_take_gap(struct iq_file *in)
{
	uint64_t g = in->gap;

	in->gap = 0;
	return g;
}

size_t iq_file_read(struct iq_file *in, void *buf, size_t max)
{
	size_t sz = sample_bytes(in->fmt);
	uint8_t *packed = (uint8_t *)buf + max;
	int16_t *out = buf;
	size_t got, k;

	if ((uint64_t)max * sz > in->left)
		max = (size_t)(in->left / sz);
	if (in->fmt == IQ_RAW12)
		return read_raw12(in, out, max);
	if (in->fmt != IQ_CS12) {
		got = fread(buf, sz, max, in->f);
		in->left -= (uint64_t)got * sz;
		return got;
	}

	/*
	 * Packed samples land at byte max, int16 pairs are written from byte
	 * 0: sample k is read at max + 3k before 4k..4k+3 is written, and
	 * 4k + 3 < max + 3k + 3 for every k < max.
	 */
	got = fread(packed, CS12_BYTES, max, in->f);
	in->left -= (uint64_t)got * CS12_BYTES;
	for (k = 0; k < got; k++) {
		const uint8_t *p = packed + CS12_BYTES * k;
		uint32_t w = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
			     (uint32_t)p[2] << 16;

		out[2 * k] = cs12_code(w);
		out[2 * k + 1] = cs12_code(w >> 12);
	}
	return got;
}
