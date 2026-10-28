/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>

#include "known.h"
#include "node.h"

#define USTEER_KNOWN_DIR	"/etc/usteer/known"
#define USTEER_KNOWN_SAVE_INTERVAL	(5 * 60 * 1000)

static struct uloop_timeout save_timer;

static void
known_path(char *buf, size_t len, const char *name)
{
	int i, n;

	n = snprintf(buf, len, USTEER_KNOWN_DIR "/");
	for (i = 0; name[i] && n < (int) len - 1; i++, n++)
		buf[n] = isalnum((unsigned char) name[i]) ? name[i] : '_';
	buf[n] = 0;
}

void
usteer_known_node_init(struct usteer_node *node, const char *name)
{
	char path[192];
	unsigned int addr[6];
	int signal;
	unsigned long long ts;
	FILE *f;

	INIT_LIST_HEAD(&node->known_sta);
	node->known_name = NULL;

	/* Remote nodes (name == NULL) are never persisted to disk - their
	 * known-station data only ever arrives over the wire, and only
	 * stays around for as long as that peer keeps broadcasting.
	 *
	 * known_stations can be enabled per-SSID, but the SSID isn't known
	 * yet this early in node bring-up, so it isn't checked here -
	 * loading a leftover file for a node whose SSID turns out to have
	 * the feature disabled is harmless (it will simply never be
	 * consulted; see find_known_candidate() in policy.c). */
	if (!name)
		return;

	node->known_name = strdup(name);

	known_path(path, sizeof(path), name);
	f = fopen(path, "r");
	if (!f)
		return;

	while (fscanf(f, "%2x:%2x:%2x:%2x:%2x:%2x %d %llu\n",
		      &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5],
		      &signal, &ts) == 8) {
		struct usteer_known_sta *ks;
		uint8_t mac[6];
		int i;

		for (i = 0; i < 6; i++)
			mac[i] = addr[i];

		ks = calloc(1, sizeof(*ks));
		memcpy(ks->addr, mac, sizeof(ks->addr));
		ks->signal = signal;
		ks->timestamp = ts;
		list_add(&ks->list, &node->known_sta);
	}

	fclose(f);
	usteer_known_prune(node);

	MSG(INFO, "Loaded %s\n", path);
}

void
usteer_known_node_free(struct usteer_node *node)
{
	struct usteer_known_sta *ks, *tmp;

	list_for_each_entry_safe(ks, tmp, &node->known_sta, list) {
		list_del(&ks->list);
		free(ks);
	}

	free(node->known_name);
	node->known_name = NULL;
}

struct usteer_known_sta *
usteer_known_find(struct usteer_node *node, const uint8_t *addr)
{
	struct usteer_known_sta *ks;

	list_for_each_entry(ks, &node->known_sta, list) {
		if (!memcmp(ks->addr, addr, 6))
			return ks;
	}

	return NULL;
}

void
usteer_known_delete(struct usteer_node *node, const uint8_t *addr)
{
	struct usteer_known_sta *ks = usteer_known_find(node, addr);

	if (!ks)
		return;

	list_del(&ks->list);
	free(ks);
}

/*
 * Most recent known-signal timestamp for addr across every node this
 * process currently has visibility into: our own local nodes, plus any
 * remote node that is presently, actively broadcasting. A station that
 * is confirmed alive anywhere in the network keeps ALL of its recorded
 * entries alive - including on nodes it hasn't personally been near in
 * a while - because those are exactly the "known good elsewhere"
 * candidates the whole feature exists to preserve. Only a station that
 * nobody has seen anywhere for known_stations_timeout is actually gone.
 */
static uint64_t
known_last_seen_anywhere(const uint8_t *addr)
{
	struct usteer_remote_node *rn;
	struct usteer_node *node;
	struct usteer_known_sta *ks;
	uint64_t last = 0;

	for_each_local_node(node) {
		ks = usteer_known_find(node, addr);
		if (ks && ks->timestamp > last)
			last = ks->timestamp;
	}

	for_each_remote_node(rn) {
		ks = usteer_known_find(&rn->node, addr);
		if (ks && ks->timestamp > last)
			last = ks->timestamp;
	}

	return last;
}

void
usteer_known_prune(struct usteer_node *node)
{
	struct usteer_known_sta *ks, *tmp;
	uint32_t timeout = config.known_stations_timeout;
	uint64_t max_age, last_seen;

	if (!timeout)
		return;

	max_age = (uint64_t) timeout * 1000;
	list_for_each_entry_safe(ks, tmp, &node->known_sta, list) {
		last_seen = known_last_seen_anywhere(ks->addr);
		if (current_time - last_seen <= max_age)
			continue;

		list_del(&ks->list);
		free(ks);
	}
}

void
usteer_known_update(struct usteer_node *node, const uint8_t *addr, int signal)
{
	struct usteer_known_sta *ks;

	if (!config.known_stations)
		return;

	ks = usteer_known_find(node, addr);
	if (!ks) {
		ks = calloc(1, sizeof(*ks));
		memcpy(ks->addr, addr, sizeof(ks->addr));
		ks->signal = signal;
		ks->timestamp = current_time;
		list_add(&ks->list, &node->known_sta);
		return;
	}

	/* signal == 0 is the "unknown, worth exploring" placeholder (see
	 * remote.c, interface_add_station()) - a real reading is always
	 * negative dBm in practice, so it unconditionally supersedes a
	 * placeholder. Otherwise refresh on any reading at least as good
	 * as the stored one, so a station that stays put doesn't age out
	 * even though its signal never improves further. */
	if (ks->signal == 0 || signal >= ks->signal) {
		ks->signal = signal;
		ks->timestamp = current_time;
	}
}

static void
usteer_known_node_save(struct usteer_node *node)
{
	struct usteer_known_sta *ks;
	char path[192];
	FILE *f;

	if (!node->known_name || list_empty(&node->known_sta))
		return;

	known_path(path, sizeof(path), node->known_name);
	f = fopen(path, "w");
	if (!f)
		return;

	list_for_each_entry(ks, &node->known_sta, list) {
		fprintf(f, MAC_ADDR_FMT " %d %llu\n",
			MAC_ADDR_DATA(ks->addr), ks->signal,
			(unsigned long long) ks->timestamp);
	}

	fclose(f);
}

static void
usteer_known_save_timer(struct uloop_timeout *t)
{
	struct usteer_node *node;

	uloop_timeout_set(t, USTEER_KNOWN_SAVE_INTERVAL);

	for_each_local_node(node) {
		if (!config.known_stations)
			continue;

		usteer_known_prune(node);
		usteer_known_node_save(node);
	}
}

void
usteer_known_init(void)
{
	/* plain mkdir() does not create intermediate directories */
	mkdir("/etc/usteer", 0755);
	mkdir(USTEER_KNOWN_DIR, 0755);

	save_timer.cb = usteer_known_save_timer;
	uloop_timeout_set(&save_timer, USTEER_KNOWN_SAVE_INTERVAL);
}
