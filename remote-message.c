/* SPDX-License-Identifier: GPL-2.0-only */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "remote-message.h"
#include "remote.h"

size_t usteer_message_payload_limit(unsigned int mtu, bool ipv6)
{
	unsigned int overhead = ipv6 ? 48 : 28;

	if (mtu <= overhead)
		return 0;

	return mtu - overhead < USTEER_REMOTE_MAX_PAYLOAD ?
		mtu - overhead : USTEER_REMOTE_MAX_PAYLOAD;
}

/* The input is a locally constructed message, not an untrusted receive buffer. */
static struct blob_attr *message_field(struct blob_attr *data, unsigned int id)
{
	struct blob_attr *cur;
	int rem;

	blob_for_each_attr(cur, data, rem)
		if (blob_id(cur) == id)
			return cur;

	return NULL;
}

static size_t metadata_size(struct blob_attr *data, unsigned int skip)
{
	struct blob_attr *cur;
	size_t len = sizeof(*data);
	int rem;

	blob_for_each_attr(cur, data, rem)
		if (blob_id(cur) != skip)
			len += blob_pad_len(cur);

	return len;
}

static size_t copy_metadata(char *out, struct blob_attr *data, unsigned int skip)
{
	struct blob_attr *cur;
	size_t len = sizeof(*data);
	int rem;

	memcpy(out, data, sizeof(*data));
	blob_for_each_attr(cur, data, rem) {
		if (blob_id(cur) == skip)
			continue;
		memcpy(out + len, cur, blob_pad_len(cur));
		len += blob_pad_len(cur);
	}

	return len;
}

static int emit_chunk(char *out, size_t len, size_t nodes, size_t node,
		      size_t stations, usteer_message_emit emit, void *priv)
{
	blob_set_raw_len((struct blob_attr *)out, len);
	blob_set_raw_len((struct blob_attr *)(out + nodes), len - nodes);
	blob_set_raw_len((struct blob_attr *)(out + node), len - node);
	blob_set_raw_len((struct blob_attr *)(out + stations), len - stations);
	return emit((struct blob_attr *)out, priv);
}

int usteer_message_send(struct blob_attr *data, size_t limit,
			usteer_message_emit emit, void *priv)
{
	struct blob_attr *nodes, *node, *stations, *sta;
	size_t top_len, node_len, overhead, nodes_off, node_off, sta_off, len;
	int rem, sta_rem, ret = 0;
	char *out;

	if (blob_pad_len(data) <= limit)
		return emit(data, priv);

	nodes = message_field(data, APMSG_NODES);
	if (!nodes)
		return -EINVAL;

	top_len = metadata_size(data, APMSG_NODES) + sizeof(*nodes);
	if (top_len > limit)
		return -EMSGSIZE;

	/* Validate the entire update before emitting any part of it. Metadata and
	 * individual station records are indivisible in the existing protocol.
	 * Never silently truncate them or fall back to IP fragmentation.
	 */
	blob_for_each_attr(node, nodes, rem) {
		stations = message_field(node, APMSG_NODE_STATIONS);
		if (!stations)
			return -EINVAL;

		node_len = metadata_size(node, APMSG_NODE_STATIONS) + sizeof(*stations);
		if (node_len > limit - top_len)
			return -EMSGSIZE;

		overhead = top_len + node_len;
		blob_for_each_attr(sta, stations, sta_rem)
			if (blob_pad_len(sta) > limit - overhead)
				return -EMSGSIZE;
	}

	out = malloc(limit);
	if (!out)
		return -ENOMEM;

	nodes_off = copy_metadata(out, data, APMSG_NODES);
	memcpy(out + nodes_off, nodes, sizeof(*nodes));
	node_off = nodes_off + sizeof(*nodes);
	blob_for_each_attr(node, nodes, rem) {
		stations = message_field(node, APMSG_NODE_STATIONS);
		sta_off = node_off + copy_metadata(out + node_off, node, APMSG_NODE_STATIONS);
		memcpy(out + sta_off, stations, sizeof(*stations));
		overhead = sta_off + sizeof(*stations);
		len = overhead;

		blob_for_each_attr(sta, stations, sta_rem) {
			if (blob_pad_len(sta) > limit - len) {
				ret = emit_chunk(out, len, nodes_off, node_off, sta_off, emit, priv);
				if (ret)
					goto out;
				len = overhead;
			}
			memcpy(out + len, sta, blob_pad_len(sta));
			len += blob_pad_len(sta);
		}
		ret = emit_chunk(out, len, nodes_off, node_off, sta_off, emit, priv);
		if (ret)
			goto out;
	}
out:
	free(out);
	return ret;
}
