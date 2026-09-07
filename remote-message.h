/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __USTEER_REMOTE_MESSAGE_H
#define __USTEER_REMOTE_MESSAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <libubox/blob.h>

/* Leave room for IPv6 and UDP even on a 1280-byte link. */
#define USTEER_REMOTE_MAX_PAYLOAD 1200

typedef int (*usteer_message_emit)(struct blob_attr *data, void *priv);

size_t usteer_message_payload_limit(unsigned int mtu, bool ipv6);
int usteer_message_send(struct blob_attr *data, size_t limit,
			usteer_message_emit emit, void *priv);

#endif
