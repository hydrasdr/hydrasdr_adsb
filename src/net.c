/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
typedef int io_len_t;
#define BAD_SOCK	INVALID_SOCKET
#define sock_close	closesocket
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
typedef int sock_t;
typedef size_t io_len_t;
#define BAD_SOCK	(-1)
#define sock_close	close
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL	0
#endif

#include "net.h"

#define MAX_CLIENTS	16
#define QUEUE_SIZE	(256 * 1024)

struct client {
	sock_t fd;
	unsigned char *q;
	size_t len;
};

struct net_server {
	const char *name;
	sock_t fd;
	int n;
	struct client c[MAX_CLIENTS];
};

static int set_nonblock(sock_t s)
{
#ifdef _WIN32
	u_long on = 1;

	/* FIONBIO is an unsigned constant, the argument a long */
	return ioctlsocket(s, (long)FIONBIO, &on) == 0 ? 0 : -1;
#else
	int fl = fcntl(s, F_GETFL, 0);

	return fl < 0 ? -1 : fcntl(s, F_SETFL, fl | O_NONBLOCK);
#endif
}

/* select() cannot watch a POSIX descriptor at or above FD_SETSIZE. */
static int fd_fits(sock_t s)
{
#ifdef _WIN32
	(void)s;
	return 1;
#else
	return s < FD_SETSIZE;
#endif
}

static int would_block(void)
{
#ifdef _WIN32
	return WSAGetLastError() == WSAEWOULDBLOCK;
#else
	return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
}

int net_init(void)
{
#ifdef _WIN32
	WSADATA wsa;

	return WSAStartup(MAKEWORD(2, 2), &wsa) == 0 ? 0 : -1;
#else
	return 0;
#endif
}

void net_cleanup(void)
{
#ifdef _WIN32
	WSACleanup();
#endif
}

struct net_server *net_listen(const char *name, const char *bind_addr,
			      int port)
{
	struct sockaddr_in sa;
	struct net_server *s;
	int on = 1;

	s = calloc(1, sizeof(*s));
	if (!s)
		return NULL;
	s->name = name;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons((unsigned short)port);
	if (inet_pton(AF_INET, bind_addr, &sa.sin_addr) != 1) {
		fprintf(stderr, "%s: bad address %s\n", name, bind_addr);
		free(s);
		return NULL;
	}
	s->fd = socket(AF_INET, SOCK_STREAM, 0);
	if (s->fd == BAD_SOCK) {
		free(s);
		return NULL;
	}
#ifdef _WIN32
	/* SO_REUSEADDR on Windows would let another socket take the port */
	setsockopt(s->fd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (const char *)&on,
		   sizeof(on));
#else
	setsockopt(s->fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&on,
		   sizeof(on));
#endif
	if (!fd_fits(s->fd) ||
	    bind(s->fd, (struct sockaddr *)&sa, sizeof(sa)) ||
	    listen(s->fd, 8) || set_nonblock(s->fd)) {
		fprintf(stderr, "%s: cannot listen on %s:%d\n", name, bind_addr,
			port);
		sock_close(s->fd);
		free(s);
		return NULL;
	}
	fprintf(stderr, "%s on %s:%d\n", name, bind_addr, port);
	return s;
}

static void drop(struct net_server *s, int i)
{
	sock_close(s->c[i].fd);
	free(s->c[i].q);
	s->c[i] = s->c[--s->n];
}

void net_close(struct net_server *s)
{
	if (!s)
		return;
	while (s->n)
		drop(s, s->n - 1);
	sock_close(s->fd);
	free(s);
}

void net_send(struct net_server *s, const void *data, size_t len)
{
	int i;

	if (!s)
		return;
	for (i = s->n - 1; i >= 0; i--) {
		struct client *c = &s->c[i];

		if (c->len + len > QUEUE_SIZE) {
			fprintf(stderr, "%s: client too slow, disconnected\n",
				s->name);
			drop(s, i);
			continue;
		}
		memcpy(c->q + c->len, data, len);
		c->len += len;
	}
}

int net_clients(const struct net_server *s)
{
	return s ? s->n : 0;
}

/* At most MAX_CLIENTS accepts per poll: a connect flood cannot hold it. */
static void accept_clients(struct net_server *s)
{
	int k;

	for (k = 0; k < MAX_CLIENTS; k++) {
		sock_t fd = accept(s->fd, NULL, NULL);

		if (fd == BAD_SOCK)
			return;
		if (s->n == MAX_CLIENTS || !fd_fits(fd) || set_nonblock(fd)) {
			sock_close(fd);
			continue;
		}
		s->c[s->n].fd = fd;
		s->c[s->n].len = 0;
		s->c[s->n].q = malloc(QUEUE_SIZE);
		if (!s->c[s->n].q) {
			sock_close(fd);
			continue;
		}
		s->n++;
	}
}

/* 0 to keep the client, -1 to drop it. */
static int flush(struct client *c)
{
	while (c->len) {
		int r = (int)send(c->fd, (const char *)c->q,
				  (io_len_t)c->len, MSG_NOSIGNAL);

		if (r < 0)
			return would_block() ? 0 : -1;
		memmove(c->q, c->q + r, c->len - (size_t)r);
		c->len -= (size_t)r;
	}
	return 0;
}

/* Input is not used; a read of 0 means the peer closed. */
static int drain(struct client *c)
{
	char buf[512];
	int r = (int)recv(c->fd, buf, (io_len_t)sizeof(buf), 0);

	if (r == 0)
		return -1;
	if (r < 0)
		return would_block() ? 0 : -1;
	return 0;
}

void net_poll(struct net_server **srv, int n, int timeout_ms)
{
	struct timeval tv;
	fd_set rd, wr;
	sock_t maxfd = 0;
	int i, j, any = 0;

	FD_ZERO(&rd);
	FD_ZERO(&wr);
	for (i = 0; i < n; i++) {
		struct net_server *s = srv[i];

		if (!s)
			continue;
		FD_SET(s->fd, &rd);
		if (s->fd > maxfd)
			maxfd = s->fd;
		any = 1;
		for (j = 0; j < s->n; j++) {
			FD_SET(s->c[j].fd, &rd);
			if (s->c[j].len)
				FD_SET(s->c[j].fd, &wr);
			if (s->c[j].fd > maxfd)
				maxfd = s->c[j].fd;
		}
	}
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;
	if (!any) {
#ifdef _WIN32
		Sleep((DWORD)timeout_ms);
#else
		select(0, NULL, NULL, NULL, &tv);
#endif
		return;
	}
	if (select((int)maxfd + 1, &rd, &wr, NULL, &tv) <= 0)
		return;

	for (i = 0; i < n; i++) {
		struct net_server *s = srv[i];

		if (!s)
			continue;
		if (FD_ISSET(s->fd, &rd))
			accept_clients(s);
		for (j = s->n - 1; j >= 0; j--) {
			struct client *c = &s->c[j];
			int bad = 0;

			if (FD_ISSET(c->fd, &rd))
				bad = drain(c);
			if (!bad && c->len)
				bad = flush(c);
			if (bad)
				drop(s, j);
		}
	}
}

size_t net_pending(struct net_server **srv, int n)
{
	size_t len = 0;
	int i, j;

	for (i = 0; i < n; i++) {
		if (!srv[i])
			continue;
		for (j = 0; j < srv[i]->n; j++)
			len += srv[i]->c[j].len;
	}
	return len;
}

uint64_t net_now_us(void)
{
#ifdef _WIN32
	static uint64_t freq;
	LARGE_INTEGER v;
	uint64_t c;

	if (!freq) {
		QueryPerformanceFrequency(&v);
		freq = (uint64_t)v.QuadPart;
	}
	QueryPerformanceCounter(&v);
	c = (uint64_t)v.QuadPart;
	return c / freq * 1000000u + c % freq * 1000000u / freq;
#else
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
#endif
}
