/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

/*
 * hydrasdr_adsb: Mode S / ADS-B 1090 MHz receiver for HydraSDR.
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "crc.h"
#include "demod.h"
#include "input_file.h"
#include "iq_record.h"
#include "modes.h"
#include "net.h"
#include "output.h"
#include "track.h"

#ifdef HAVE_LIBHYDRASDR
#include "input_hydrasdr.h"
#include "raw12.h"
#include "ring.h"
#endif

#ifndef HYDRASDR_ADSB_VERSION
#define HYDRASDR_ADSB_VERSION "dev"
#endif

#define BLOCK_SAMPLES	131072
#define AIRCRAFT_TTL	(60 * TICKS_PER_SEC)
#define MAX_RATE_HZ	1e9
#define MAX_RANGE_KM	20000.0	/* half the Earth's circumference */
#define MAX_DURATION	1e7	/* seconds, about 115 days */
#define ADSB_FREQ_HZ	1090000000u
#define LIVE_RATE_HZ	5000000.0
#define RING_BLOCKS	64	/* 16 MB: 0.42 s at 10 MS/s */
#define RING_SAMPLES	65536	/* one libhydrasdr transfer */
#define RING_WAIT_MS	100
#define FILL_GROUPS	4096	/* lost samples go out in 16384 at a time */

static volatile sig_atomic_t stop;

enum { SRV_BEAST, SRV_SBS, SRV_RAW, SRV_COUNT };

static const char *const srv_name[SRV_COUNT] = {
	"Beast output", "SBS output", "AVR output"
};

struct options {
	const char *ifile;
	enum iq_format iformat;
	double rate;
	int raw;
	int mlat;
	int fix;
	int have_lat;
	int have_lon;
	double lat;
	double lon;
	double max_range_km;
	int net;
	const char *net_bind;
	int port[SRV_COUNT];
	int throttle;
	uint64_t serial;
	int gain;
	int sensitivity;
	int bias_tee;
	const char *record;
	double duration;
};

#ifdef HAVE_LIBHYDRASDR
/* State of the libhydrasdr thread, which the decoding thread never reads. */
struct producer {
	struct ring *ring;
	int raw12;
	struct raw12_ddc ddc;	/* with --record: the ADC stream to I/Q */
	int16_t *iq;		/* RING_SAMPLES complex samples */
	const uint8_t *fill;	/* FILL_GROUPS groups of fill codes */
};
#endif

struct app {
	struct options opt;
	double rate;
	struct demod *dm;
	struct tracker *track;
	struct net_server *srv[SRV_COUNT];
	struct iq_record *rec;
#ifdef HAVE_LIBHYDRASDR
	struct ring *ring;
	struct producer prod;
	uint8_t *fill;		/* FILL_GROUPS groups of fill codes */
#endif
	uint64_t pos;		/* stream position in samples, lost included */
	uint64_t lost;
	uint64_t max_samples;	/* 0: no limit */
	uint64_t busy_us;
	int rec_failed;
	uint64_t frames;
	uint64_t positions;
	uint64_t aircraft_new;
};

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

static void usage(void)
{
	fprintf(stderr,
		"hydrasdr_adsb " HYDRASDR_ADSB_VERSION "\n"
		"usage: hydrasdr_adsb [options]    receive from a HydraSDR\n"
		"       hydrasdr_adsb --ifile FILE [options]\n"
		"  --serial HEX      device serial number (default: first found)\n"
		"  --rate HZ         2500000, 5000000 (default) or 10000000\n"
		"  --linearity-gain N\n"
		"                    linearity gain preset (default: the device\n"
		"                    default)\n"
		"  --sensitivity-gain N\n"
		"                    sensitivity gain preset, instead of linearity\n"
		"  --bias-tee        power the antenna through the RF input\n"
		"  --record FILE     write the ADC stream to FILE, a .raw12 name,\n"
		"                    with FILE.sigmf-meta\n"
		"  --duration SEC    stop after SEC seconds\n"
		"  --ifile FILE      read a recording: SigMF, 2-channel WAV, or raw\n"
		"  --iformat FMT     raw sample format: cs16, cs12, cf32, raw12\n"
		"                    (default: from a .cs12 or .raw12 name, cs16\n"
		"                    otherwise)\n"
		"  --raw             print frames as *HEX; on stdout\n"
		"  --mlat            with --raw: @TIMESTAMP before each frame\n"
		"  --no-fix          no single-bit error correction\n"
		"  --lat DEG         receiver latitude\n"
		"  --lon DEG         receiver longitude\n"
		"  --max-range KM    drop positions farther from the receiver\n"
		"                    (default 450 with --lat/--lon, 0 = off)\n"
		"  --net             serve Beast, SBS and AVR over TCP\n"
		"  --net-bind-address ADDR  (default 127.0.0.1)\n"
		"  --net-bo-port N   Beast output port (default 30005, 0 = off)\n"
		"  --net-sbs-port N  SBS BaseStation port (default 30003)\n"
		"  --net-ro-port N   AVR output port (default 30002)\n"
		"  --throttle        read a recording at its real rate\n"
		"  --help\n");
}

static void on_frame(void *ctx, const struct modes_frame *f)
{
	struct app *a = ctx;
	unsigned char buf[OUT_MAX];
	struct modes_msg mm;
	struct aircraft *ac;
	size_t n;
	int pos;

	a->frames++;
	if (a->opt.raw) {
		n = out_avr((char *)buf, f, a->opt.mlat);
		fwrite(buf, 1, n, stdout);
	}
	if (net_clients(a->srv[SRV_BEAST]))
		net_send(a->srv[SRV_BEAST], buf, out_beast(buf, f));
	if (net_clients(a->srv[SRV_RAW]))
		net_send(a->srv[SRV_RAW], buf, out_avr((char *)buf, f, 0));

	modes_decode(f->msg, f->bits, f->addr, &mm);
	ac = track_update(a->track, &mm, f->ts_12mhz, f->level, &pos);
	if (ac && ac->messages == 1)
		a->aircraft_new++;
	a->positions += (uint64_t)pos;
	if (net_clients(a->srv[SRV_SBS])) {
		n = out_sbs((char *)buf, &mm, ac, pos);
		if (n)
			net_send(a->srv[SRV_SBS], buf, n);
	}
}

/* Stream time in 12 MHz ticks. */
static uint64_t stream_ticks(const struct app *a)
{
	return (uint64_t)((double)a->pos / a->rate * TICKS_PER_SEC);
}

static void process(struct app *a, const void *iq, size_t n, int cf32)
{
	if (cf32)
		demod_process_cf32(a->dm, iq, n, on_frame, a);
	else
		demod_process_cs16(a->dm, iq, n, on_frame, a);
	a->pos += n;
	track_expire(a->track, stream_ticks(a), AIRCRAFT_TTL);
	if (a->opt.net)
		net_poll(a->srv, SRV_COUNT, 0);
}

/* Serves the network until the monotonic clock reaches until_us. */
static void wait_until(struct app *a, uint64_t until_us)
{
	uint64_t t;

	while (!stop && (t = net_now_us()) < until_us)
		net_poll(a->srv, SRV_COUNT, (int)((until_us - t + 999) / 1000));
}

static int run_file(struct app *a)
{
	struct iq_file in;
	void *buf;
	uint64_t start;

	if (iq_file_open(&in, a->opt.ifile, a->opt.iformat, a->opt.rate)) {
		fprintf(stderr, "%s: cannot open or unknown format\n",
			a->opt.ifile);
		return 1;
	}
	a->rate = in.rate;
	a->dm = demod_create(in.rate, BLOCK_SAMPLES);
	buf = malloc(BLOCK_SAMPLES * 8);
	if (!a->dm || !buf) {
		fprintf(stderr, "%s: sample rate %.0f not supported\n",
			a->opt.ifile, in.rate);
		free(buf);
		iq_file_close(&in);
		return 1;
	}
	demod_set_fix(a->dm, a->opt.fix);

	start = net_now_us();
	while (!stop) {
		uint64_t t = net_now_us();
		size_t n = iq_file_read(&in, buf, BLOCK_SAMPLES);
		uint64_t gap = iq_file_take_gap(&in);

		if (!n && !gap)
			break;
		if (gap) {
			/* lost when recorded: skipped as the live run did */
			a->lost += gap;
			a->pos += gap;
			demod_gap(a->dm, gap);
		}
		if (n)
			process(a, buf, n, in.fmt == IQ_CF32);
		a->busy_us += net_now_us() - t;
		if (a->opt.throttle)
			wait_until(a, start + stream_ticks(a) /
				   (TICKS_PER_SEC / 1000000));
	}
	free(buf);
	iq_file_close(&in);
	if (a->lost)
		fprintf(stderr, "lost when recorded: %llu samples (%.3f s)\n",
			(unsigned long long)a->lost, (double)a->lost / a->rate);
	return 0;
}

#ifdef HAVE_LIBHYDRASDR
/*
 * libhydrasdr thread. The samples go to the ring and nothing else, so the
 * conversion and the decoding run on two cores. With --record the .raw12
 * groups go through the DDC here, where libhydrasdr converts them without
 * --record, and travel in the ring next to their I/Q for the file.
 */

/*
 * Lost samples go through the DDC as fill codes, the output dropped, as a
 * replay of the recording runs them; n is a multiple of 4.
 */
static void ddc_fill(struct producer *p, uint64_t n)
{
	while (n) {
		size_t c = n < FILL_GROUPS * RAW12_GROUP_SAMPLES ? (size_t)n :
			   FILL_GROUPS * RAW12_GROUP_SAMPLES;

		raw12_convert(&p->ddc, p->fill, c / RAW12_GROUP_SAMPLES, p->iq);
		n -= c;
	}
}

static int on_block(void *ctx, const void *data, size_t n, uint64_t lost)
{
	struct producer *p = ctx;
	const uint8_t *g = data;

	if (!p->raw12)
		return ring_push(p->ring, data, NULL, n, lost) ? 1 : 0;
	/* whole groups of 4 samples */
	lost = (lost + RAW12_GROUP_SAMPLES - 1) / RAW12_GROUP_SAMPLES *
	       RAW12_GROUP_SAMPLES;
	ddc_fill(p, lost);
	while (n) {
		size_t c = n < RING_SAMPLES ? n : RING_SAMPLES;
		int space = ring_space(p->ring);

		if (space < 0)
			return 1;
		if (space) {
			raw12_convert(&p->ddc, g, c / RAW12_GROUP_SAMPLES, p->iq);
			if (ring_push(p->ring, p->iq, g, c, lost))
				return 1;
		} else {
			/* the decoder is behind: dropped as lost samples */
			ddc_fill(p, c);
			ring_push(p->ring, NULL, NULL, 0, lost);
			ring_drop(p->ring, c);
		}
		lost = 0;
		g += c * RAW12_SAMPLE_BYTES;
		n -= c;
	}
	return 0;
}

/* With --record: lost samples go to the recording as fill codes. */
static int record_fill(struct app *a, uint64_t n)
{
	iq_record_lost(a->rec, n);
	while (n) {
		size_t c = n < FILL_GROUPS * RAW12_GROUP_SAMPLES ? (size_t)n :
			   FILL_GROUPS * RAW12_GROUP_SAMPLES;

		if (iq_record_write(a->rec, a->fill, c))
			return -1;
		n -= c;
	}
	return 0;
}

/* Decoding thread. Returns non-zero at the end of the run. */
static int consume(struct app *a, const struct ring_block *b)
{
	uint64_t lost = b->lost, t = net_now_us();
	size_t n = b->n;

	if (lost) {
		a->lost += lost;
		a->pos += lost;
		demod_gap(a->dm, lost);
		if (a->rec && record_fill(a, lost))
			a->rec_failed = 1;
	}
	if (a->max_samples) {
		if (a->pos >= a->max_samples)
			return 1;
		if (n > a->max_samples - a->pos)
			n = (size_t)(a->max_samples - a->pos);
	}
	if (a->rec && !a->rec_failed && iq_record_write(a->rec, b->aux, n))
		a->rec_failed = 1;
	if (a->rec_failed) {
		fprintf(stderr, "%s: write failed\n", a->opt.record);
		return 1;
	}
	process(a, b->data, n, 0);
	if (a->opt.raw)
		fflush(stdout);
	a->busy_us += net_now_us() - t;
	return a->max_samples && a->pos >= a->max_samples;
}

static int run_live(struct app *a)
{
	struct live_config c;
	struct live *l;

	memset(&c, 0, sizeof(c));
	c.serial = a->opt.serial;
	c.rate = (uint32_t)a->opt.rate;
	c.freq_hz = ADSB_FREQ_HZ;
	c.gain = a->opt.gain;
	c.sensitivity = a->opt.sensitivity;
	c.bias_tee = a->opt.bias_tee;
	c.raw12 = a->opt.record != NULL;

	a->rate = a->opt.rate;
	a->dm = demod_create(a->rate, BLOCK_SAMPLES);
	a->ring = ring_create(RING_BLOCKS, RING_SAMPLES, 2 * sizeof(int16_t),
			      c.raw12 ? RAW12_SAMPLE_BYTES : 0);
	if (!a->dm || !a->ring)
		return 1;
	a->prod.ring = a->ring;
	a->prod.raw12 = c.raw12;
	if (c.raw12) {
		a->prod.iq = malloc(RING_SAMPLES * 2 * sizeof(int16_t));
		a->fill = malloc(FILL_GROUPS * RAW12_GROUP_BYTES);
		if (!a->prod.iq || !a->fill)
			return 1;
		raw12_ddc_init(&a->prod.ddc);
		raw12_fill(a->fill, FILL_GROUPS);
		a->prod.fill = a->fill;
	}
	demod_set_fix(a->dm, a->opt.fix);
	if (a->opt.duration > 0.0)
		a->max_samples = (uint64_t)(a->opt.duration * a->rate) /
				 RAW12_GROUP_SAMPLES * RAW12_GROUP_SAMPLES;

	l = live_open(&c);
	if (!l)
		return 1;
	if (a->opt.record) {
		a->rec = iq_record_open(a->opt.record, a->rate, ADSB_FREQ_HZ,
					live_board(l),
					"hydrasdr_adsb " HYDRASDR_ADSB_VERSION);
		if (!a->rec) {
			fprintf(stderr, "%s: cannot create\n", a->opt.record);
			live_close(l);
			return 1;
		}
	}
	if (live_start(l, on_block, &a->prod)) {
		live_close(l);
		return 1;
	}
	while (!stop) {
		const struct ring_block *b = ring_peek(a->ring, RING_WAIT_MS);
		int end;

		if (!b) {
			if (!live_streaming(l))
				break;
			if (a->opt.net)
				net_poll(a->srv, SRV_COUNT, 0);
			continue;
		}
		end = consume(a, b);
		ring_pop(a->ring);
		if (end)
			break;
	}
	/* the next push fails and ends the stream; no callback runs after
	 * live_close */
	ring_close(a->ring);
	live_close(l);

	if (a->rec) {
		if (iq_record_close(a->rec))
			a->rec_failed = 1;
		if (a->rec_failed)
			fprintf(stderr, "%s: incomplete\n", a->opt.record);
	}
	if (a->lost)
		fprintf(stderr, "lost %llu samples (%.3f s)\n",
			(unsigned long long)a->lost,
			(double)a->lost / a->rate);
	if (ring_dropped(a->ring))
		fprintf(stderr, "decoder behind: %llu samples dropped\n",
			(unsigned long long)ring_dropped(a->ring));
	return a->rec_failed;
}
#else
static int run_live(struct app *a)
{
	(void)a;
	fprintf(stderr, "built without libhydrasdr: use --ifile\n");
	return 1;
}
#endif

/* The whole string, finite and within [lo, hi]; NaN fails the test. */
static int parse_num(const char *s, double lo, double hi, double *out)
{
	char *end;
	double v = strtod(s, &end);

	if (end == s || *end || !(v >= lo && v <= hi))
		return -1;
	*out = v;
	return 0;
}

static int parse_port(const char *s, int *port)
{
	char *end;
	long v = strtol(s, &end, 10);

	if (end == s || *end || v < 0 || v > 65535)
		return -1;
	*port = (int)v;
	return 0;
}

/* Up to 16 hex digits, optional 0x, not zero. */
static int parse_serial(const char *s, uint64_t *out)
{
	uint64_t v = 0;
	int n = 0;

	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		s += 2;
	for (; *s; s++, n++) {
		int d;

		if (*s >= '0' && *s <= '9')
			d = *s - '0';
		else if (*s >= 'a' && *s <= 'f')
			d = *s - 'a' + 10;
		else if (*s >= 'A' && *s <= 'F')
			d = *s - 'A' + 10;
		else
			return -1;
		if (n == 16)
			return -1;
		v = v << 4 | (uint64_t)d;
	}
	if (!v)
		return -1;
	*out = v;
	return 0;
}

static int parse_args(int argc, char **argv, struct options *o)
{
	double g;
	int i;

	o->fix = 1;
	o->max_range_km = 450.0;
	o->net_bind = "127.0.0.1";
	o->port[SRV_BEAST] = 30005;
	o->port[SRV_SBS] = 30003;
	o->port[SRV_RAW] = 30002;
	o->gain = -1;
	for (i = 1; i < argc; i++) {
		const char *a = argv[i];
		const char *v = i + 1 < argc ? argv[i + 1] : NULL;

		if (!strcmp(a, "--help") || !strcmp(a, "-h")) {
			usage();
			return 0;
		} else if (!strcmp(a, "--raw")) {
			o->raw = 1;
		} else if (!strcmp(a, "--mlat")) {
			o->mlat = 1;
		} else if (!strcmp(a, "--no-fix")) {
			o->fix = 0;
		} else if (!strcmp(a, "--ifile") && v) {
			o->ifile = v;
			i++;
		} else if (!strcmp(a, "--iformat") && v) {
			if (!strcmp(v, "cs16"))
				o->iformat = IQ_CS16;
			else if (!strcmp(v, "cs12"))
				o->iformat = IQ_CS12;
			else if (!strcmp(v, "cf32"))
				o->iformat = IQ_CF32;
			else if (!strcmp(v, "raw12"))
				o->iformat = IQ_RAW12;
			else
				goto bad;
			i++;
		} else if (!strcmp(a, "--rate") && v) {
			if (parse_num(v, 0.0, MAX_RATE_HZ, &o->rate) ||
			    !demod_rate_supported(o->rate))
				goto bad;
			i++;
		} else if (!strcmp(a, "--serial") && v) {
			if (parse_serial(v, &o->serial))
				goto bad;
			i++;
		} else if ((!strcmp(a, "--linearity-gain") ||
			    !strcmp(a, "--sensitivity-gain")) && v) {
			/* one preset only */
			if (o->gain >= 0 || parse_num(v, 0.0, 255.0, &g) ||
			    g != (int)g)
				goto bad;
			o->gain = (int)g;
			o->sensitivity = a[2] == 's';
			i++;
		} else if (!strcmp(a, "--bias-tee")) {
			o->bias_tee = 1;
		} else if (!strcmp(a, "--record") && v) {
			o->record = v;
			i++;
		} else if (!strcmp(a, "--duration") && v) {
			if (parse_num(v, 0.0, MAX_DURATION, &o->duration))
				goto bad;
			i++;
		} else if (!strcmp(a, "--lat") && v) {
			if (parse_num(v, -90.0, 90.0, &o->lat))
				goto bad;
			o->have_lat = 1;
			i++;
		} else if (!strcmp(a, "--lon") && v) {
			if (parse_num(v, -180.0, 180.0, &o->lon))
				goto bad;
			o->have_lon = 1;
			i++;
		} else if (!strcmp(a, "--max-range") && v) {
			if (parse_num(v, 0.0, MAX_RANGE_KM, &o->max_range_km))
				goto bad;
			i++;
		} else if (!strcmp(a, "--net")) {
			o->net = 1;
		} else if (!strcmp(a, "--net-bind-address") && v) {
			o->net_bind = v;
			i++;
		} else if (!strcmp(a, "--net-bo-port") && v) {
			if (parse_port(v, &o->port[SRV_BEAST]))
				goto bad;
			i++;
		} else if (!strcmp(a, "--net-sbs-port") && v) {
			if (parse_port(v, &o->port[SRV_SBS]))
				goto bad;
			i++;
		} else if (!strcmp(a, "--net-ro-port") && v) {
			if (parse_port(v, &o->port[SRV_RAW]))
				goto bad;
			i++;
		} else if (!strcmp(a, "--throttle")) {
			o->throttle = 1;
		} else {
			goto bad;
		}
	}
	if (o->have_lat != o->have_lon) {
		usage();
		return 2;
	}
	if (o->ifile && (o->serial || o->gain >= 0 || o->bias_tee ||
			 o->record || o->duration > 0.0)) {
		fprintf(stderr, "--serial, the gain presets, --bias-tee, "
			"--record and --duration are for live input\n");
		return 2;
	}
	if (o->record && (strlen(o->record) < 7 ||
			  strcmp(o->record + strlen(o->record) - 6, ".raw12"))) {
		fprintf(stderr, "--record writes .raw12: the name must end in "
			".raw12\n");
		return 2;
	}
	if (!o->ifile && o->rate <= 0.0)
		o->rate = LIVE_RATE_HZ;
	return -1;
bad:
	fprintf(stderr, "bad option: %s\n", argv[i]);
	usage();
	return 2;
}

int main(int argc, char **argv)
{
	static struct app app;
	const struct demod_stats *st;
	double secs, proc;
	int rc, i;

	rc = parse_args(argc, argv, &app.opt);
	if (rc >= 0)
		return rc;

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
#ifdef SIGPIPE
	signal(SIGPIPE, SIG_IGN);
#endif
	modes_crc_init();

	app.track = track_create();
	if (!app.track)
		return 1;
	if (app.opt.have_lat)
		track_set_receiver(app.track, app.opt.lat, app.opt.lon,
				   app.opt.max_range_km);

	if (app.opt.net) {
		if (net_init()) {
			fprintf(stderr, "network init failed\n");
			return 1;
		}
		for (i = 0; i < SRV_COUNT; i++) {
			if (!app.opt.port[i])
				continue;
			app.srv[i] = net_listen(srv_name[i], app.opt.net_bind,
						app.opt.port[i]);
			if (!app.srv[i])
				return 1;
		}
	}

	rc = app.opt.ifile ? run_file(&app) : run_live(&app);
	fflush(stdout);

	/* Give clients up to one second to take what is queued. */
	for (i = 0; i < 10 && net_pending(app.srv, SRV_COUNT); i++)
		net_poll(app.srv, SRV_COUNT, 100);
	for (i = 0; i < SRV_COUNT; i++)
		net_close(app.srv[i]);
	if (app.opt.net)
		net_cleanup();

	if (app.dm && (app.pos || !rc)) {
		st = demod_get_stats(app.dm);
		secs = (double)app.pos / app.rate;
		proc = (double)app.busy_us / 1e6;
		fprintf(stderr,
			"%.1f s at %.1f MS/s, %.2f s processing "
			"(%.1fx real time)\n"
			"frames %llu, fixed %llu, preambles %llu, "
			"rejected %llu\n"
			"aircraft %llu, positions %llu, rejected fixes %llu\n",
			secs, app.rate / 1e6, proc,
			proc > 0.0 ? secs / proc : 0.0,
			(unsigned long long)st->accepted,
			(unsigned long long)st->fixed,
			(unsigned long long)st->candidates,
			(unsigned long long)st->rejected,
			(unsigned long long)app.aircraft_new,
			(unsigned long long)app.positions,
			(unsigned long long)track_rejected_fixes(app.track));
	}
	demod_destroy(app.dm);
#ifdef HAVE_LIBHYDRASDR
	ring_destroy(app.ring);
	free(app.prod.iq);
	free(app.fill);
#endif
	track_destroy(app.track);
	return rc;
}
