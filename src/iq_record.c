/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "iq_record.h"
#include "raw12.h"

#define FILE_BUFFER	(1 << 20)
#define MAX_GAPS	64	/* lost stretches listed in the meta file */
#define NAME_LEN	64
#define SIGMF_VERSION	"1.2.6"

struct gap {
	uint64_t start;
	uint64_t count;
};

struct iq_record {
	FILE *f;
	char *path;
	char *vbuf;
	double rate;
	double freq_hz;
	char hw[NAME_LEN];
	char recorder[NAME_LEN];
	char datetime[32];
	uint64_t samples;
	uint64_t lost;
	uint64_t gaps;
	struct gap gap[MAX_GAPS];
	int failed;
};

/* Printable ASCII only: the name comes from the device. */
static void copy_name(char *dst, size_t size, const char *src)
{
	size_t n = 0;

	for (; src && *src && n + 1 < size; src++)
		if (*src >= 0x20 && *src < 0x7F)
			dst[n++] = *src;
	dst[n] = '\0';
}

static const char *base_name(const char *path)
{
	const char *b = path, *p;

	for (p = path; *p; p++)
		if (*p == '/' || *p == '\\')
			b = p + 1;
	return b;
}

static void put_string(FILE *f, const char *s)
{
	fputc('"', f);
	for (; *s; s++) {
		unsigned char c = (unsigned char)*s;

		if (c == '"' || c == '\\')
			fprintf(f, "\\%c", c);
		else if (c < 0x20)
			fprintf(f, "\\u%04x", c);
		else
			fputc(c, f);
	}
	fputc('"', f);
}

struct iq_record *iq_record_open(const char *path, double rate,
				 double freq_hz, const char *hw,
				 const char *recorder)
{
	struct iq_record *r = calloc(1, sizeof(*r));
	time_t now = time(NULL);
	struct tm *tm = gmtime(&now);
	size_t n = strlen(path) + 1;

	if (!r)
		return NULL;
	r->path = malloc(n);
	r->vbuf = malloc(FILE_BUFFER);
	if (!r->path || !r->vbuf)
		goto fail;
	memcpy(r->path, path, n);
	r->f = fopen(path, "wb");
	if (!r->f)
		goto fail;
	setvbuf(r->f, r->vbuf, _IOFBF, FILE_BUFFER);
	r->rate = rate;
	r->freq_hz = freq_hz;
	copy_name(r->hw, sizeof(r->hw), hw);
	copy_name(r->recorder, sizeof(r->recorder), recorder);
	if (!tm || !strftime(r->datetime, sizeof(r->datetime),
			     "%Y-%m-%dT%H:%M:%SZ", tm))
		r->datetime[0] = '\0';
	return r;
fail:
	free(r->path);
	free(r->vbuf);
	free(r);
	return NULL;
}

int iq_record_write(struct iq_record *r, const uint8_t *groups, size_t n)
{
	if (!r->failed && n &&
	    fwrite(groups, RAW12_SAMPLE_BYTES, n, r->f) != n)
		r->failed = 1;
	r->samples += n;
	return r->failed ? -1 : 0;
}

void iq_record_lost(struct iq_record *r, uint64_t n)
{
	if (!n)
		return;
	if (r->gaps < MAX_GAPS) {
		r->gap[r->gaps].start = r->samples;
		r->gap[r->gaps].count = n;
	}
	r->gaps++;
	r->lost += n;
}

uint64_t iq_record_samples(const struct iq_record *r)
{
	return r->samples;
}

static int write_meta(const struct iq_record *r)
{
	static const char ext[] = ".sigmf-meta";
	size_t size = strlen(r->path) + sizeof(ext);
	char *path = malloc(size);
	uint64_t i;
	FILE *f;
	int bad;

	if (!path)
		return -1;
	snprintf(path, size, "%s%s", r->path, ext);
	f = fopen(path, "w");
	free(path);
	if (!f)
		return -1;

	/* no core:datatype: SigMF has none for packed 12-bit real codes, and
	 * a reader that needs one refuses the file instead of misreading it */
	fprintf(f, "{\n    \"global\": {\n"
		"        \"core:version\": \"" SIGMF_VERSION "\",\n"
		"        \"core:sample_rate\": %.0f,\n"
		"        \"core:dataset\": ", r->rate);
	put_string(f, base_name(r->path));
	if (r->hw[0]) {
		fprintf(f, ",\n        \"core:hw\": ");
		put_string(f, r->hw);
	}
	if (r->recorder[0]) {
		fprintf(f, ",\n        \"core:recorder\": ");
		put_string(f, r->recorder);
	}
	fprintf(f, ",\n        \"core:description\": \"HydraSDR .raw12: "
		"packed 12-bit ADC codes, 8 in 12 bytes, at twice "
		"core:sample_rate, which is the I/Q rate after the libhydrasdr "
		"DDC");
	if (r->lost)
		fprintf(f, "; %llu samples lost in %llu stretches, recorded "
			"as mid-scale codes", (unsigned long long)r->lost,
			(unsigned long long)r->gaps);
	fputc('"', f);
	fprintf(f, "\n    },\n    \"captures\": [\n        {\n"
		"            \"core:sample_start\": 0,\n"
		"            \"core:frequency\": %.0f", r->freq_hz);
	if (r->datetime[0])
		fprintf(f, ",\n            \"core:datetime\": \"%s\"",
			r->datetime);
	fprintf(f, "\n        }\n    ],\n    \"annotations\": [");
	for (i = 0; i < r->gaps && i < MAX_GAPS; i++)
		fprintf(f, "%s\n        {\n"
			"            \"core:sample_start\": %llu,\n"
			"            \"core:sample_count\": %llu,\n"
			"            \"core:comment\": \"lost, recorded as "
			"mid-scale codes\"\n"
			"        }", i ? "," : "",
			(unsigned long long)r->gap[i].start,
			(unsigned long long)r->gap[i].count);
	fprintf(f, "%s]\n}\n", r->gaps ? "\n    " : "");
	bad = ferror(f);
	return fclose(f) || bad ? -1 : 0;
}

int iq_record_close(struct iq_record *r)
{
	int bad = r->failed;

	if (fclose(r->f))
		bad = 1;
	if (write_meta(r))
		bad = 1;
	free(r->vbuf);
	free(r->path);
	free(r);
	return bad ? -1 : 0;
}
