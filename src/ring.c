/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600	/* SRW locks and condition variables */
#endif
#include <windows.h>
#else
#include <pthread.h>
#include <time.h>
#endif

#include "ring.h"

/*
 * count, dropped and closed change under the lock. The producer alone
 * writes the block at head while it is free, the consumer alone reads the
 * block at tail while it is filled; the count update under the lock orders
 * the block contents between the two threads.
 */
struct ring {
#ifdef _WIN32
	SRWLOCK lock;
	CONDITION_VARIABLE cond;
#else
	pthread_mutex_t lock;
	pthread_cond_t cond;
#endif
	struct ring_block *b;
	unsigned char *mem;
	size_t blocks;
	size_t block_samples;
	size_t sample_bytes;
	size_t aux_bytes;
	size_t count;
	size_t head;		/* producer */
	size_t tail;		/* consumer */
	uint64_t pending;	/* producer: lost samples for the next block */
	uint64_t dropped;
	int closed;
};

#ifdef _WIN32
static void lock(struct ring *r)
{
	AcquireSRWLockExclusive(&r->lock);
}

static void unlock(struct ring *r)
{
	ReleaseSRWLockExclusive(&r->lock);
}

static void wake(struct ring *r)
{
	WakeConditionVariable(&r->cond);
}

static void wait_ms(struct ring *r, int ms)
{
	SleepConditionVariableSRW(&r->cond, &r->lock, (DWORD)ms, 0);
}
#else
static void lock(struct ring *r)
{
	pthread_mutex_lock(&r->lock);
}

static void unlock(struct ring *r)
{
	pthread_mutex_unlock(&r->lock);
}

static void wake(struct ring *r)
{
	pthread_cond_signal(&r->cond);
}

static void wait_ms(struct ring *r, int ms)
{
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += ms / 1000;
	ts.tv_nsec += (long)(ms % 1000) * 1000000L;
	if (ts.tv_nsec >= 1000000000L) {
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000L;
	}
	pthread_cond_timedwait(&r->cond, &r->lock, &ts);
}
#endif

struct ring *ring_create(size_t blocks, size_t block_samples,
			 size_t sample_bytes, size_t aux_bytes)
{
	size_t bytes = sample_bytes + aux_bytes, i;
	struct ring *r;

	if (!blocks || !block_samples || !sample_bytes ||
	    aux_bytes > SIZE_MAX - sample_bytes ||
	    block_samples > SIZE_MAX / bytes / blocks)
		return NULL;
	r = calloc(1, sizeof(*r));
	if (!r)
		return NULL;
	r->b = calloc(blocks, sizeof(*r->b));
	r->mem = malloc(blocks * block_samples * bytes);
	if (!r->b || !r->mem) {
		free(r->b);
		free(r->mem);
		free(r);
		return NULL;
	}
	for (i = 0; i < blocks; i++) {
		unsigned char *m = r->mem + i * block_samples * bytes;

		r->b[i].data = m;
		r->b[i].aux = aux_bytes ? m + block_samples * sample_bytes :
					  NULL;
	}
	r->blocks = blocks;
	r->block_samples = block_samples;
	r->sample_bytes = sample_bytes;
	r->aux_bytes = aux_bytes;
#ifdef _WIN32
	InitializeSRWLock(&r->lock);
	InitializeConditionVariable(&r->cond);
#else
	pthread_mutex_init(&r->lock, NULL);
	pthread_cond_init(&r->cond, NULL);
#endif
	return r;
}

void ring_destroy(struct ring *r)
{
	if (!r)
		return;
#ifndef _WIN32
	pthread_cond_destroy(&r->cond);
	pthread_mutex_destroy(&r->lock);
#endif
	free(r->mem);
	free(r->b);
	free(r);
}

int ring_push(struct ring *r, const void *data, const void *aux, size_t n,
	      uint64_t lost)
{
	const unsigned char *p = data, *q = aux;

	r->pending += lost;
	while (n) {
		size_t c = n < r->block_samples ? n : r->block_samples;
		struct ring_block *b = &r->b[r->head];
		int closed, full;

		lock(r);
		closed = r->closed;
		full = r->count == r->blocks;
		if (full && !closed)
			r->dropped += c;
		unlock(r);
		if (closed)
			return -1;

		if (full) {
			r->pending += c;
		} else {
			memcpy(b->data, p, c * r->sample_bytes);
			if (q)
				memcpy(b->aux, q, c * r->aux_bytes);
			b->n = c;
			b->lost = r->pending;
			r->pending = 0;
			r->head = (r->head + 1) % r->blocks;
			lock(r);
			r->count++;
			wake(r);
			unlock(r);
		}
		p += c * r->sample_bytes;
		if (q)
			q += c * r->aux_bytes;
		n -= c;
	}
	return 0;
}

int ring_space(struct ring *r)
{
	int s;

	lock(r);
	s = r->closed ? -1 : r->count < r->blocks;
	unlock(r);
	return s;
}

void ring_drop(struct ring *r, size_t n)
{
	r->pending += n;
	lock(r);
	r->dropped += n;
	unlock(r);
}

const struct ring_block *ring_peek(struct ring *r, int timeout_ms)
{
	size_t count;

	lock(r);
	if (!r->count && timeout_ms > 0)
		wait_ms(r, timeout_ms);
	count = r->count;
	unlock(r);
	return count ? &r->b[r->tail] : NULL;
}

void ring_pop(struct ring *r)
{
	lock(r);
	r->count--;
	unlock(r);
	r->tail = (r->tail + 1) % r->blocks;
}

void ring_close(struct ring *r)
{
	lock(r);
	r->closed = 1;
	unlock(r);
}

uint64_t ring_dropped(struct ring *r)
{
	uint64_t d;

	lock(r);
	d = r->dropped;
	unlock(r);
	return d;
}
