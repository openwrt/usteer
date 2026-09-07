/* SPDX-License-Identifier: GPL-2.0-only */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <unistd.h>
#include "remote-message.h"
#include "remote.h"

static struct blob_buf input;
static unsigned int seen[3][200];
static unsigned int nodes_seen[3], emitted, fail_at;
static size_t cap;
static bool expect_identical;
static unsigned int station_extra;
static int udp_tx, udp_rx;

static struct blob_attr *field(struct blob_attr *data, unsigned int id)
{
	struct blob_attr *cur;
	int rem;
	blob_for_each_attr(cur, data, rem)
		if (blob_id(cur) == id)
			return cur;
	return NULL;
}

static void make_message(unsigned int count, unsigned int metadata)
{
	unsigned int n, i;
	void *nodes, *node, *stations, *station;
	char extra[2048] = {}, name[16];
	uint8_t addr[6] = {};

	assert(metadata <= sizeof(extra));
	blob_buf_init(&input, 0);
	blob_put_int32(&input, APMSG_ID, 123);
	blob_put_int32(&input, APMSG_SEQ, 456);
	blob_put(&input, APMSG_HOST_INFO, NULL, 0);
	nodes = blob_nest_start(&input, APMSG_NODES);
	for (n = 0; n < 3; n++) {
		node = blob_nest_start(&input, 0);
		snprintf(name, sizeof(name), "radio%u", n);
		blob_put_string(&input, APMSG_NODE_NAME, name);
		blob_put_string(&input, APMSG_NODE_SSID, "HomeNET");
		addr[0] = n;
		blob_put(&input, APMSG_NODE_BSSID, addr, 6);
		blob_put_int32(&input, APMSG_NODE_FREQ, 2412);
		blob_put_int32(&input, APMSG_NODE_N_ASSOC, n == 2 ? 0 : count);
		blob_put(&input, APMSG_NODE_NODE_INFO, extra, n == 2 ? metadata : 0);
		blob_put(&input, 31, "abc", 3);
		stations = blob_nest_start(&input, APMSG_NODE_STATIONS);
		for (i = 0; n != 2 && i < count; i++) {
			station = blob_nest_start(&input, 0);
			addr[1] = i;
			blob_put(&input, APMSG_STA_ADDR, addr, 6);
			blob_put_int32(&input, APMSG_STA_SIGNAL, -40);
			blob_put_int32(&input, APMSG_STA_TIMEOUT, 1000);
			blob_put_int32(&input, APMSG_STA_SEEN, 10);
			blob_put_int32(&input, APMSG_STA_LAST_CONNECTED, 20);
			blob_put_int8(&input, APMSG_STA_CONNECTED, 1);
			if (station_extra)
				blob_put(&input, 31, extra, station_extra);
			blob_nest_end(&input, station);
		}
		blob_nest_end(&input, stations);
		blob_nest_end(&input, node);
	}
	blob_nest_end(&input, nodes);
}

static int receive_message(struct blob_attr *data, void *priv)
{
	struct apmsg msg;
	struct apmsg_node node_msg = {};
	struct apmsg_sta sta_msg;
	struct blob_attr *node, *sta, *cur, *original, *original_nodes;
	int rem, sta_rem, attr_rem, orig_rem;
	unsigned int n;

	assert(priv == &input);
	assert(blob_pad_len(data) <= cap);
	emitted++;
	if (fail_at && emitted == fail_at)
		return -EIO;
	if (expect_identical)
		assert(!memcmp(data, input.head, blob_pad_len(input.head)));
	/* Use the unchanged upstream parser, including mandatory-field checks. */
	assert(parse_apmsg(&msg, data));
	assert(msg.id == 123 && msg.seq == 456);
	assert(msg.host_info && !blob_len(msg.host_info));
	original_nodes = field(input.head, APMSG_NODES);
	blob_for_each_attr(node, msg.nodes, rem) {
		assert(parse_apmsg_node(&node_msg, node));
		n = (unsigned char)node_msg.bssid[0];
		assert(n < 3);
		nodes_seen[n]++;
		original = NULL;
		blob_for_each_attr(cur, original_nodes, orig_rem)
			if (((unsigned char *)blob_data(field(cur, APMSG_NODE_BSSID)))[0] == n)
				original = cur;
		assert(original);
		blob_for_each_attr(cur, original, attr_rem) {
			struct blob_attr *copy;
			if (blob_id(cur) == APMSG_NODE_STATIONS)
				continue;
			copy = field(node, blob_id(cur));
			assert(copy && blob_pad_len(copy) == blob_pad_len(cur));
			assert(!memcmp(copy, cur, blob_pad_len(cur)));
		}
		blob_for_each_attr(sta, node_msg.stations, sta_rem) {
			assert(parse_apmsg_sta(&sta_msg, sta));
			assert(sta_msg.addr[0] == n && sta_msg.addr[1] < 200);
			assert(sta_msg.signal == -40 && sta_msg.timeout == 1000);
			assert(sta_msg.seen == 10 && sta_msg.last_connected == 20);
			assert(sta_msg.connected);
			assert(++seen[n][sta_msg.addr[1]] == 1);
		}
	}
	return 0;
}

static void reset(size_t limit)
{
	memset(seen, 0, sizeof(seen));
	memset(nodes_seen, 0, sizeof(nodes_seen));
	emitted = fail_at = 0;
	expect_identical = false;
	cap = limit;
}

static int udp_emit(struct blob_attr *data, void *priv)
{
	unsigned int received[512];
	ssize_t len = blob_pad_len(data);

	assert(send(udp_tx, data, len, 0) == len);
	assert(recv(udp_rx, received, sizeof(received), 0) == len);
	assert(!memcmp(received, data, len));
	return receive_message((struct blob_attr *)received, priv);
}

static void test_udp(int family)
{
	struct sockaddr_storage addr = {};
	struct sockaddr_in *v4 = (void *)&addr;
	struct sockaddr_in6 *v6 = (void *)&addr;
	struct timeval timeout = { .tv_sec = 2 };
	socklen_t len = family == AF_INET ? sizeof(*v4) : sizeof(*v6);
	unsigned int n, i;

	addr.ss_family = family;
	if (family == AF_INET)
		v4->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	else
		v6->sin6_addr = in6addr_loopback;
	udp_rx = socket(family, SOCK_DGRAM, 0);
	udp_tx = socket(family, SOCK_DGRAM, 0);
	assert(udp_rx >= 0 && udp_tx >= 0);
	assert(!bind(udp_rx, (void *)&addr, len));
	assert(!getsockname(udp_rx, (void *)&addr, &len));
	assert(!connect(udp_tx, (void *)&addr, len));
	assert(!setsockopt(udp_rx, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)));
	make_message(150, 16);
	reset(1200);
	assert(!usteer_message_send(input.head, cap, udp_emit, &input));
	for (n = 0; n < 2; n++)
		for (i = 0; i < 150; i++)
			assert(seen[n][i] == 1);
	assert(nodes_seen[2]);
	close(udp_rx);
	close(udp_tx);
}

int main(void)
{
	unsigned int n, i;
	size_t limit;
	int ret;
	void *original;

	assert(usteer_message_payload_limit(1500, false) == 1200);
	assert(usteer_message_payload_limit(1280, true) == 1200);
	assert(usteer_message_payload_limit(576, false) == 548);
	assert(usteer_message_payload_limit(1000, true) == 952);
	assert(!usteer_message_payload_limit(28, false));
	assert(!usteer_message_payload_limit(48, true));
	assert(!usteer_message_payload_limit(0, true));

	make_message(1, 0);
	reset(blob_pad_len(input.head));
	expect_identical = true;
	assert(!usteer_message_send(input.head, cap, receive_message, &input));
	assert(emitted == 1);
	reset(blob_pad_len(input.head) - 1);
	assert(!usteer_message_send(input.head, cap, receive_message, &input));
	assert(emitted > 1);

	make_message(150, 16);
	original = malloc(blob_pad_len(input.head));
	assert(original);
	memcpy(original, input.head, blob_pad_len(input.head));
	for (limit = 200; limit <= 1200; limit++) {
		reset(limit);
		assert(!usteer_message_send(input.head, cap, receive_message, &input));
		assert(!memcmp(original, input.head, blob_pad_len(input.head)));
		for (n = 0; n < 3; n++) {
			assert(nodes_seen[n]);
			for (i = 0; i < 150; i++)
				assert(seen[n][i] == (n != 2));
		}
	}
	free(original);
	reset(1200);
	fail_at = 2;
	assert(usteer_message_send(input.head, cap, receive_message, &input) == -EIO);
	assert(emitted == 2);

	/* Metadata and individual records cannot be truncated to fit. */
	for (limit = 0; limit < 100; limit++) {
		reset(limit);
		ret = usteer_message_send(input.head, cap, receive_message, &input);
		assert(ret == -EMSGSIZE && !emitted);
	}
	make_message(1, 1400);
	reset(1200);
	assert(usteer_message_send(input.head, cap, receive_message, &input) == -EMSGSIZE);
	assert(!emitted);
	station_extra = 1400;
	make_message(1, 0);
	reset(1200);
	assert(usteer_message_send(input.head, cap, receive_message, &input) == -EMSGSIZE);
	assert(!emitted);
	station_extra = 0;

	make_message(0, 0);
	reset(200);
	assert(!usteer_message_send(input.head, cap, receive_message, &input));
	assert(nodes_seen[0] && nodes_seen[1] && nodes_seen[2]);

	/* A host-only update remains a complete, valid old-format message. */
	blob_buf_init(&input, 0);
	blob_put_int32(&input, APMSG_ID, 123);
	blob_put_int32(&input, APMSG_SEQ, 456);
	blob_put(&input, APMSG_HOST_INFO, NULL, 0);
	blob_put(&input, APMSG_NODES, NULL, 0);
	reset(1200);
	expect_identical = true;
	assert(!usteer_message_send(input.head, cap, receive_message, &input));
	assert(emitted == 1);
	reset(20);
	assert(usteer_message_send(input.head, cap, receive_message, &input) == -EMSGSIZE);
	assert(!emitted);

	test_udp(AF_INET);
	test_udp(AF_INET6);
	blob_buf_free(&input);
	puts("remote-message: all tests passed");
	return 0;
}
