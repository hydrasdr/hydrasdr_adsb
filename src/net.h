/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026, Benjamin Vernoux <bvernoux@hydrasdr.com> */

#ifndef HYDRASDR_ADSB_NET_H
#define HYDRASDR_ADSB_NET_H

#include <stddef.h>
#include <stdint.h>

/*
 * TCP output servers. Sockets are non-blocking and serviced by
 * net_poll() from the receive loop; a client whose queue overflows is
 * disconnected.
 */

struct net_server;

int net_init(void);
void net_cleanup(void);

/* Returns NULL and prints the reason on failure. */
struct net_server *net_listen(const char *name, const char *bind_addr,
			      int port);
void net_close(struct net_server *s);

/* Queues len bytes for every connected client. */
void net_send(struct net_server *s, const void *data, size_t len);
int net_clients(const struct net_server *s);

/*
 * Accepts clients, flushes queues and drops closed connections on all
 * servers. Waits up to timeout_ms for socket activity.
 */
void net_poll(struct net_server **srv, int n, int timeout_ms);

/* Bytes still queued for clients on all servers. */
size_t net_pending(struct net_server **srv, int n);

/* Monotonic clock in microseconds. */
uint64_t net_now_us(void);

#endif
