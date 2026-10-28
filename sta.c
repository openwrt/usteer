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
 *
 *   Copyright (C) 2020 embedd.ch 
 *   Copyright (C) 2020 Felix Fietkau <nbd@nbd.name> 
 *   Copyright (C) 2020 John Crispin <john@phrozen.org> 
 */

#include "usteer.h"
#include "known.h"
#include "ssid_config.h"

static int
avl_macaddr_cmp(const void *k1, const void *k2, void *ptr)
{
	return memcmp(k1, k2, 6);
}

AVL_TREE(stations, avl_macaddr_cmp, false, NULL);
static struct usteer_timeout_queue tq;

static void
usteer_sta_del(struct sta *sta)
{
	MSG(DEBUG, "Delete station " MAC_ADDR_FMT "\n",
	    MAC_ADDR_DATA(sta->addr));

	avl_delete(&stations, &sta->avl);
	usteer_measurement_report_sta_cleanup(sta);
	free(sta);
}

static void
usteer_sta_info_del(struct sta_info *si)
{
	struct sta *sta = si->sta;

	MSG(DEBUG, "Delete station " MAC_ADDR_FMT " entry for node %s\n",
	    MAC_ADDR_DATA(sta->addr), usteer_node_name(si->node));

	usteer_timeout_cancel(&tq, &si->timeout);
	list_del(&si->list);
	list_del(&si->node_list);
	free(si);

	if (list_empty(&sta->nodes))
		usteer_sta_del(sta);
}

void
usteer_sta_node_cleanup(struct usteer_node *node)
{
	struct sta_info *si, *tmp;

	free(node->rrm_nr);
	node->rrm_nr = NULL;

	list_for_each_entry_safe(si, tmp, &node->sta_info, node_list)
		usteer_sta_info_del(si);
}

static void
usteer_sta_info_timeout(struct usteer_timeout_queue *q, struct usteer_timeout *t)
{
	struct sta_info *si = container_of(t, struct sta_info, timeout);

	usteer_sta_info_del(si);
}

void
usteer_sta_update_aggressiveness(struct sta_info *si)
{
	struct blob_attr *cur;
	int rem;
	char sta_mac[18];
	char config_entry[20];
	char config_mac[18];
	const char *ssid = si->node->ssid;

	sprintf(sta_mac, MAC_ADDR_FMT, MAC_ADDR_DATA(si->sta->addr));
	si->aggressiveness = SSID_CFG(ssid, aggressiveness);
	blobmsg_for_each_attr(cur, SSID_CFG(ssid, aggressiveness_mac_list), rem) {
		strcpy(config_entry, blobmsg_get_string(cur));
		if (strlen(config_entry) != 19)
			continue;
		strncpy(config_mac, config_entry, 18);
		config_mac[17] = '\0';
		if (strcmp(config_mac, sta_mac) != 0)
			continue;
		si->aggressiveness = config_entry[18] - '0';
		break;
	}
}

/* Station infos only evaluate their aggressiveness when they are created.
 * Stations that were already known when a new config arrives (or that were
 * created before the first config was loaded) would keep the old value,
 * so re-evaluate all of them after every config update. */
void
usteer_sta_update_all_aggressiveness(void)
{
	struct sta_info *si;
	struct sta *sta;

	avl_for_each_element(&stations, sta, avl)
		list_for_each_entry(si, &sta->nodes, list)
			usteer_sta_update_aggressiveness(si);
}

struct sta_info *
usteer_sta_info_get(struct sta *sta, struct usteer_node *node, bool *create)
{
	struct sta_info *si;

	list_for_each_entry(si, &sta->nodes, list) {
		if (si->node != node)
			continue;

		if (create)
			*create = false;

		return si;
	}

	if (!create)
		return NULL;

	MSG(DEBUG, "Create station " MAC_ADDR_FMT " entry for node %s\n",
	    MAC_ADDR_DATA(sta->addr), usteer_node_name(node));

	si = calloc(1, sizeof(*si));
	si->node = node;
	si->sta = sta;
	list_add(&si->list, &sta->nodes);
	list_add(&si->node_list, &node->sta_info);
	si->created = current_time;
	*create = true;

	usteer_sta_update_aggressiveness(si);

	/* Node is by default not connected. */
	usteer_sta_disconnected(si);

	return si;
}


void
usteer_sta_info_update_timeout(struct sta_info *si, int timeout)
{
	if (si->connected == STA_CONNECTED)
		usteer_timeout_cancel(&tq, &si->timeout);
	else if (timeout > 0)
		usteer_timeout_set(&tq, &si->timeout, timeout);
	else
		usteer_sta_info_del(si);
}

struct sta *
usteer_sta_get(const uint8_t *addr, bool create)
{
	struct sta *sta;

	sta = avl_find_element(&stations, addr, sta, avl);
	if (sta)
		return sta;

	if (!create)
		return NULL;

	MSG(DEBUG, "Create station entry " MAC_ADDR_FMT "\n", MAC_ADDR_DATA(addr));
	sta = calloc(1, sizeof(*sta));
	memcpy(sta->addr, addr, sizeof(sta->addr));
	sta->avl.key = sta->addr;
	avl_insert(&stations, &sta->avl);
	INIT_LIST_HEAD(&sta->nodes);
	INIT_LIST_HEAD(&sta->measurements);

	return sta;
}

void usteer_sta_disconnected(struct sta_info *si)
{
	si->connected = STA_NOT_CONNECTED;
	si->kick_time = 0;
	si->connected_since = 0;
	usteer_sta_info_update_timeout(si, SSID_CFG(si->node->ssid, local_sta_timeout));
}

void
usteer_sta_info_update(struct sta_info *si, int signal, bool avg)
{
	/* ignore probe request signal when connected */
	if (si->connected == STA_CONNECTED && si->signal != NO_SIGNAL && !avg)
		signal = NO_SIGNAL;

	if (signal != NO_SIGNAL) {
		si->signal = signal;
		usteer_band_steering_sta_update(si);

		/* Only record known-station data from the periodic nl80211
		 * poll of stations hostapd currently reports as associated
		 * (avg == true; see nl80211_update_sta()) - not from bare
		 * PROBE/AUTH/ASSOC events (avg == false), which are
		 * frequently sent from randomized, throwaway MAC addresses
		 * (privacy MAC randomization on phones/tablets scanning
		 * nearby networks without ever connecting) and would
		 * otherwise flood the known-station list with junk that is
		 * never seen again. Note si->connected is not yet updated
		 * to STA_CONNECTED at this point in the nl80211 poll cycle
		 * (see usteer_local_node_set_assoc()), so it can't be used
		 * as the gate here. */
		if (si->node->type == NODE_TYPE_LOCAL && avg)
			usteer_known_update(si->node, si->sta->addr, signal);
	}

	si->seen = current_time;

	if (si->node->freq < 4000)
		si->sta->seen_2ghz = 1;
	else
		si->sta->seen_5ghz = 1;

	usteer_sta_info_update_timeout(si, SSID_CFG(si->node->ssid, local_sta_timeout));
}

bool
usteer_handle_sta_event(struct usteer_node *node, const uint8_t *addr,
		       enum usteer_event_type type, int freq, int signal)
{
	struct sta *sta;
	struct sta_info *si;
	uint32_t diff;
	bool ret;
	bool create;

	sta = usteer_sta_get(addr, true);
	if (!sta)
		return -1;

	si = usteer_sta_info_get(sta, node, &create);
	usteer_sta_info_update(si, signal, false);
	si->stats[type].requests++;

	diff = si->stats[type].blocked_last_time - current_time;
	if (diff > SSID_CFG(node->ssid, sta_block_timeout))
		si->stats[type].blocked_cur = 0;

	ret = usteer_check_request(si, type);
	if (!ret) {
		si->stats[type].blocked_cur++;
		si->stats[type].blocked_total++;
		si->stats[type].blocked_last_time = current_time;
	} else {
		si->stats[type].blocked_cur = 0;
	}

	if (create)
		usteer_send_sta_update(si);

	return ret;
}

bool
usteer_sta_supports_beacon_measurement_mode(struct sta_info *si, enum usteer_beacon_measurement_mode mode)
{
	switch (mode) {
		case BEACON_MEASUREMENT_PASSIVE:
			return si->rrm & (1 << 4);
		case BEACON_MEASUREMENT_ACTIVE:
			return si->rrm & (1 << 5);
		case BEACON_MEASUREMENT_TABLE:
			return si->rrm & (1 << 6);
	}

	return false;
}

bool
usteer_sta_supports_link_measurement(struct sta_info *si)
{
	return si->rrm & (1 << 0);
}

static void __usteer_init usteer_sta_init(void)
{
	usteer_timeout_init(&tq);
	tq.cb = usteer_sta_info_timeout;
}
