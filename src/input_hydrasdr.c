/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hydrasdr.h>

#include "input_hydrasdr.h"
#include "raw12.h"

struct live {
	struct hydrasdr_device *dev;
	uint64_t freq_hz;
	int bias_tee;
	int started;
	int done;		/* the callback asked for the end */
	int raw12;
	uint64_t dropped;	/* library count seen so far */
	live_block_fn fn;
	void *ctx;
	char board[HYDRASDR_DEVICE_NAME_MAX_LEN];
};

/* Printable ASCII only: the strings come from the device. */
static void copy_name(char *dst, size_t size, const char *src, size_t len)
{
	size_t n = 0, i;

	for (i = 0; i < len && src[i] && n + 1 < size; i++)
		if (src[i] >= 0x20 && src[i] < 0x7F)
			dst[n++] = src[i];
	dst[n] = '\0';
}

static int fail(const char *what, int err)
{
	fprintf(stderr, "%s: %s (%d)\n", what,
		hydrasdr_error_name((enum hydrasdr_error)err), err);
	return -1;
}

static int set_rate(struct live *l, uint32_t rate)
{
	uint32_t count = 0, *rates, i;
	int err, found = 0;

	err = hydrasdr_get_samplerates(l->dev, &count, 0);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_get_samplerates", err);
	if (count == 0 || count > 256)
		return -1;
	rates = malloc(count * sizeof(*rates));
	if (!rates)
		return -1;
	/*
	 * For RAW the list holds the ADC rates, twice the I/Q rates, while
	 * hydrasdr_set_samplerate takes the I/Q rate (libhydrasdr 1.1.3).
	 */
	err = hydrasdr_get_samplerates(l->dev, rates, count);
	for (i = 0; err == HYDRASDR_SUCCESS && i < count; i++)
		found |= rates[i] == (l->raw12 ? 2 * (uint64_t)rate : rate);
	if (!found) {
		fprintf(stderr, "%s does not list %u S/s; it lists:", l->board,
			rate);
		for (i = 0; err == HYDRASDR_SUCCESS && i < count; i++)
			fprintf(stderr, " %u", l->raw12 ? rates[i] / 2 : rates[i]);
		fprintf(stderr, "\n");
	}
	free(rates);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_get_samplerates", err);
	if (!found)
		return -1;
	err = hydrasdr_set_samplerate(l->dev, rate);
	return err == HYDRASDR_SUCCESS ? 0 : fail("hydrasdr_set_samplerate",
						    err);
}

/* Linearity or sensitivity preset: the two gain controls every device maps. */
static int set_gain(struct live *l, const hydrasdr_device_info_t *info,
		    int sensitivity, int gain)
{
	const hydrasdr_gain_range_t *g = sensitivity ?
		&info->sensitivity_gain : &info->linearity_gain;
	uint32_t cap = sensitivity ? HYDRASDR_CAP_SENSITIVITY_GAIN :
				     HYDRASDR_CAP_LINEARITY_GAIN;
	const char *name = sensitivity ? "sensitivity" : "linearity";
	int err;

	if (!(info->features & cap)) {
		if (gain < 0)
			return 0;
		fprintf(stderr, "%s has no %s gain\n", l->board, name);
		return -1;
	}
	if (gain < 0)
		gain = g->default_value;
	if (gain < g->min_value || gain > g->max_value) {
		fprintf(stderr, "%s gain %d out of range %u..%u\n", name, gain,
			g->min_value, g->max_value);
		return -1;
	}
	err = hydrasdr_set_gain(l->dev, sensitivity ?
				HYDRASDR_GAIN_TYPE_SENSITIVITY :
				HYDRASDR_GAIN_TYPE_LINEARITY, (uint8_t)gain);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_set_gain", err);
	fprintf(stderr, "%s gain %d (%u..%u)\n", name, gain, g->min_value,
		g->max_value);
	return 0;
}

/*
 * 12-bit packing over USB when the device has it: a quarter less USB
 * traffic, 30 MB/s instead of 40 at 10 MS/s on an RFOne.
 */
static int set_packing(struct live *l, const hydrasdr_device_info_t *info)
{
	int err;

	if (!(info->features & HYDRASDR_CAP_PACKING))
		return 0;
	err = hydrasdr_set_packing(l->dev, 1);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_set_packing", err);
	fprintf(stderr, "USB packing on\n");
	return 0;
}

/*
 * .raw12 holds 12-bit codes packed 8 in 12 bytes at twice the I/Q rate,
 * which the DDC of raw12.c turns into I/Q: the stream must be that one.
 */
static int check_raw12(struct live *l, const hydrasdr_device_info_t *info)
{
	hydrasdr_device_info_t *now = malloc(sizeof(*now));
	int err, ok;

	if (!now)
		return -1;
	err = hydrasdr_get_device_info(l->dev, now);
	ok = err == HYDRASDR_SUCCESS && (info->features & HYDRASDR_CAP_PACKING) &&
	     now->current_adc_bits == 12 && now->current_packing == 1 &&
	     now->current_decimation_factor == 1;
	if (!ok)
		fprintf(stderr, "%s: .raw12 needs a packed 12-bit ADC stream "
			"(ADC %u bits, packing %u, decimation %u)\n", l->board,
			now->current_adc_bits, now->current_packing,
			now->current_decimation_factor);
	free(now);
	return ok ? 0 : -1;
}

/* The first RF port, as hydrasdr_rx selects: the port stays set between
 * programs, so it is set every time. */
static int set_rf_port(struct live *l, const hydrasdr_device_info_t *info)
{
	int err;

	if (!(info->features & HYDRASDR_CAP_RF_PORT_SELECT))
		return 0;
	err = hydrasdr_set_rf_port(l->dev, HYDRASDR_RF_PORT_RX0);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_set_rf_port", err);
	return 0;
}

/* The bias tee is set either way: a previous program may have left it on. */
static int set_bias_tee(struct live *l, const hydrasdr_device_info_t *info,
			int on)
{
	int err;

	if (!(info->features & HYDRASDR_CAP_BIAS_TEE)) {
		if (!on)
			return 0;
		fprintf(stderr, "%s has no bias tee\n", l->board);
		return -1;
	}
	err = hydrasdr_set_rf_bias(l->dev, (uint8_t)(on ? 1 : 0));
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_set_rf_bias", err);
	l->bias_tee = on;
	if (on)
		fprintf(stderr, "bias tee on\n");
	return 0;
}

struct live *live_open(const struct live_config *c)
{
	struct live *l = calloc(1, sizeof(*l));
	hydrasdr_device_info_t *info = malloc(sizeof(*info));
	char fw[HYDRASDR_DEVICE_INFO_FW_VERSION_MAX_LEN];
	hydrasdr_lib_version_t v;
	int err;

	if (!l || !info)
		goto out;
	l->freq_hz = c->freq_hz;
	err = c->serial ? hydrasdr_open_sn(&l->dev, c->serial) :
			  hydrasdr_open(&l->dev);
	if (err != HYDRASDR_SUCCESS) {
		l->dev = NULL;
		fail(c->serial ? "hydrasdr_open_sn" : "hydrasdr_open", err);
		goto out;
	}
	err = hydrasdr_get_device_info(l->dev, info);
	if (err != HYDRASDR_SUCCESS) {
		fail("hydrasdr_get_device_info", err);
		goto out;
	}
	copy_name(l->board, sizeof(l->board), info->board_name,
		  sizeof(info->board_name));
	copy_name(fw, sizeof(fw), info->firmware_version,
		  sizeof(info->firmware_version));
	hydrasdr_lib_version(&v);
	fprintf(stderr, "%s, serial %08X%08X, firmware %s, libhydrasdr "
		"%u.%u.%u\n", l->board, info->part_serial.serial_no[2],
		info->part_serial.serial_no[3], fw, v.major_version,
		v.minor_version, v.revision);
	if (!(info->features & HYDRASDR_CAP_RX)) {
		fprintf(stderr, "%s cannot receive\n", l->board);
		goto out;
	}
	l->raw12 = c->raw12;
	err = hydrasdr_set_sample_type(l->dev, l->raw12 ? HYDRASDR_SAMPLE_RAW :
						     HYDRASDR_SAMPLE_INT16_IQ);
	if (err != HYDRASDR_SUCCESS) {
		fail("hydrasdr_set_sample_type", err);
		goto out;
	}
	if (set_rate(l, c->rate) || set_packing(l, info) ||
	    (l->raw12 && check_raw12(l, info)) ||
	    set_bias_tee(l, info, c->bias_tee) || set_rf_port(l, info) ||
	    set_gain(l, info, c->gain >= 0 && c->sensitivity, c->gain))
		goto out;
	free(info);
	return l;
out:
	free(info);
	live_close(l);
	return NULL;
}

const char *live_board(const struct live *l)
{
	return l->board;
}

static int on_transfer(hydrasdr_transfer *t)
{
	struct live *l = t->ctx;
	uint64_t lost = 0, dropped = t->dropped_samples;
	size_t n = (size_t)t->sample_count;

	if (l->done)
		return 1;
	if (t->sample_type != (l->raw12 ? HYDRASDR_SAMPLE_RAW :
					  HYDRASDR_SAMPLE_INT16_IQ) ||
	    t->sample_count < 0 ||
	    (l->raw12 && n % RAW12_GROUP_CODES)) {
		l->done = 1;
		return 1;
	}
	/* RAW counts ADC codes, two per complex sample */
	if (l->raw12) {
		n /= 2;
		dropped /= 2;
	}
	/*
	 * The library counts a buffer when it drops it, so the count can
	 * include buffers queued after this one: a gap is placed at most one
	 * buffer early.
	 */
	if (dropped > l->dropped) {
		lost = dropped - l->dropped;
		l->dropped = dropped;
	}
	if (l->fn(l->ctx, t->samples, n, lost))
		l->done = 1;
	return l->done;
}

/* Tuned after the start, in the order hydrasdr_rx uses. */
int live_start(struct live *l, live_block_fn fn, void *ctx)
{
	int err;

	l->fn = fn;
	l->ctx = ctx;
	err = hydrasdr_start_rx(l->dev, on_transfer, l);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_start_rx", err);
	l->started = 1;
	err = hydrasdr_set_freq(l->dev, l->freq_hz);
	if (err != HYDRASDR_SUCCESS)
		return fail("hydrasdr_set_freq", err);
	return 0;
}

int live_streaming(struct live *l)
{
	return hydrasdr_is_streaming(l->dev) == HYDRASDR_TRUE;
}

void live_close(struct live *l)
{
	hydrasdr_streaming_stats_t st;

	if (!l)
		return;
	if (l->dev && l->started) {
		hydrasdr_stop_rx(l->dev);
		if (hydrasdr_get_streaming_stats(l->dev, &st) ==
		    HYDRASDR_SUCCESS)
			fprintf(stderr, "USB buffers %llu, dropped %llu\n",
				(unsigned long long)st.buffers_received,
				(unsigned long long)st.buffers_dropped);
	}
	if (l->dev && l->bias_tee)
		hydrasdr_set_rf_bias(l->dev, 0);
	if (l->dev)
		hydrasdr_close(l->dev);
	free(l);
}
