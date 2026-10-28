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
#include "node.h"
#include "event.h"
#include "known.h"
#include "ssid_config.h"

static bool
below_assoc_threshold(struct usteer_node *node_cur, struct usteer_node *node_new)
{
	int n_assoc_cur = node_cur->n_assoc;
	int n_assoc_new = node_new->n_assoc;
	bool ref_5g = node_cur->freq > 4000;
	bool node_5g = node_new->freq > 4000;

	uint32_t load_balancing_threshold = SSID_CFG(node_cur->ssid, load_balancing_threshold);

	if (!load_balancing_threshold)
		return false;

	if (ref_5g && !node_5g)
		n_assoc_new += SSID_CFG(node_cur->ssid, band_steering_threshold);
	else if (!ref_5g && node_5g)
		n_assoc_cur += SSID_CFG(node_cur->ssid, band_steering_threshold);

	n_assoc_new += load_balancing_threshold;

	return n_assoc_new <= n_assoc_cur;
}

static bool
better_signal_strength(const char *ssid, int signal_cur, int signal_new)
{
	uint32_t threshold = SSID_CFG(ssid, signal_diff_threshold);
	const bool is_better = signal_new - signal_cur > (int) threshold;

	if (!threshold)
		return false;

	return is_better;
}

static bool
below_load_threshold(struct usteer_node *node)
{
	return node->n_assoc >= SSID_CFG(node->ssid, load_kick_min_clients) &&
	       node->load > SSID_CFG(node->ssid, load_kick_threshold);
}

static bool
has_better_load(struct usteer_node *node_cur, struct usteer_node *node_new)
{
	return !below_load_threshold(node_cur) && below_load_threshold(node_new);
}

bool
usteer_policy_node_below_max_assoc(struct usteer_node *node)
{
	return !node->max_assoc || node->n_assoc < node->max_assoc;
}

static bool
over_min_signal(struct usteer_node *node, int signal)
{
	int32_t min_snr = SSID_CFG(node->ssid, min_snr);
	int32_t roam_trigger_snr = SSID_CFG(node->ssid, roam_trigger_snr);

	if (min_snr && signal < usteer_snr_to_signal(node, min_snr))
		return false;

	if (roam_trigger_snr && signal < usteer_snr_to_signal(node, roam_trigger_snr))
		return false;

	return true;
}

static uint32_t
is_better_candidate(struct sta_info *si_cur, struct sta_info *si_new)
{
	struct usteer_node *current_node = si_cur->node;
	struct usteer_node *new_node = si_new->node;
	int current_signal = si_cur->signal;
	int new_signal = si_new->signal;
	uint32_t reasons = 0;

	if (!usteer_policy_node_below_max_assoc(new_node))
		return 0;

	if (!over_min_signal(new_node, new_signal))
		return 0;

	if (below_assoc_threshold(current_node, new_node) &&
	    !below_assoc_threshold(new_node, current_node))
		reasons |= (1 << UEV_SELECT_REASON_NUM_ASSOC);

	if (better_signal_strength(current_node->ssid, current_signal, new_signal))
		reasons |= (1 << UEV_SELECT_REASON_SIGNAL);

	if (has_better_load(current_node, new_node) &&
		!has_better_load(new_node, current_node))
		reasons |= (1 << UEV_SELECT_REASON_LOAD);

	return reasons;
}

/*
 * Optional fallback (known_stations, per-SSID or global): stations that never probe or
 * report RRM measurements once associated never generate live sta_info
 * data on any node but their current one, so the scan in
 * find_better_candidate() below can never find them a candidate no
 * matter how bad their signal gets. Consult the persisted
 * best-signal-ever-seen record instead.
 *
 * A node only ever has known-station data if it is a local node, or a
 * remote node that is currently, actively broadcasting on the usteer
 * remote protocol (see remote.c) - a peer that stops sending updates has
 * its remote node, and with it its known-station list, removed by the
 * existing remote_node_timeout logic. So this can never point a station
 * towards a peer that isn't confirmed alive right now.
 */
static struct sta_info *
find_known_candidate(struct sta_info *si_ref, struct uevent *ev)
{
	struct sta_info *si, *candidate = NULL;
	struct usteer_remote_node *rn;
	struct usteer_node *node;
	struct usteer_known_sta *ks;
	uint32_t reasons;
	bool create;

	if (!config.known_stations)
		return NULL;

	for_each_local_node(node) {
		if (node == si_ref->node ||
		    strcmp(node->ssid, si_ref->node->ssid) != 0)
			continue;

		ks = usteer_known_find(node, si_ref->sta->addr);
		if (!ks)
			continue;

		si = usteer_sta_info_get(si_ref->sta, node, &create);
		if (!si)
			continue;
		si->signal = ks->signal;

		reasons = is_better_candidate(si_ref, si);
		if (!(reasons & (1 << UEV_SELECT_REASON_SIGNAL)))
			continue;

		if (candidate && si->signal <= candidate->signal)
			continue;

		candidate = si;
		if (ev) {
			ev->si_other = si;
			ev->select_reasons = reasons;
		}
	}

	for_each_remote_node(rn) {
		node = &rn->node;
		if (strcmp(node->ssid, si_ref->node->ssid) != 0)
			continue;

		ks = usteer_known_find(node, si_ref->sta->addr);
		if (!ks)
			continue;

		si = usteer_sta_info_get(si_ref->sta, node, &create);
		if (!si)
			continue;
		si->signal = ks->signal;

		reasons = is_better_candidate(si_ref, si);
		if (!(reasons & (1 << UEV_SELECT_REASON_SIGNAL)))
			continue;

		if (candidate && si->signal <= candidate->signal)
			continue;

		candidate = si;
		if (ev) {
			ev->si_other = si;
			ev->select_reasons = reasons;
		}
	}

	return candidate;
}

static struct sta_info *
find_better_candidate(struct sta_info *si_ref, struct uevent *ev, uint32_t required_criteria, uint64_t max_age, bool *exploratory)
{
	struct sta_info *si, *candidate = NULL;
	struct sta *sta = si_ref->sta;
	uint32_t reasons;

	if (exploratory)
		*exploratory = false;

	list_for_each_entry(si, &sta->nodes, list) {
		if (si == si_ref)
			continue;

		if (current_time - si->seen > SSID_CFG(si_ref->node->ssid, seen_policy_timeout))
			continue;

		if (strcmp(si->node->ssid, si_ref->node->ssid) != 0)
			continue;

		if (max_age && max_age < current_time - si->seen)
			continue;

		reasons = is_better_candidate(si_ref, si);
		if (!reasons)
			continue;

		if (!(reasons & required_criteria))
			continue;

		if (ev) {
			ev->si_other = si;
			ev->select_reasons = reasons;
		}

		if (!candidate || si->signal > candidate->signal)
			candidate = si;
	}

	if (!candidate && (required_criteria & (1 << UEV_SELECT_REASON_SIGNAL))) {
		candidate = find_known_candidate(si_ref, ev);
		if (candidate && exploratory)
			*exploratory = true;
	}

	return candidate;
}

int
usteer_snr_to_signal(struct usteer_node *node, int snr)
{
	int noise = -95;

	if (snr < 0)
		return snr;

	if (node->noise)
		noise = node->noise;

	return noise + snr;
}

bool
usteer_check_request(struct sta_info *si, enum usteer_event_type type)
{
	struct uevent ev = {
		.si_cur = si,
	};
	int32_t min_snr = SSID_CFG(si->node->ssid, min_snr);
	int min_signal;
	bool ret = true;
	bool exploratory;

	if (type == EVENT_TYPE_PROBE && !SSID_CFG(si->node->ssid, probe_steering))
		goto out;

	if (type == EVENT_TYPE_AUTH)
		goto out;

	if (type == EVENT_TYPE_ASSOC) {
		/* Check if assoc request has lower signal than min_signal.
		 * If this is the case, block assoc even when assoc steering is enabled.
		 *
		 * Otherwise, the client potentially ends up in a assoc - kick loop.
		 */
		if (min_snr && si->signal < usteer_snr_to_signal(si->node, min_snr)) {
			ev.reason = UEV_REASON_LOW_SIGNAL;
			ev.threshold.cur = si->signal;
			ev.threshold.ref = usteer_snr_to_signal(si->node, min_snr);
			ret = false;
			goto out;
		} else if (!SSID_CFG(si->node->ssid, assoc_steering)) {
			goto out;
		}
	}

	min_signal = usteer_snr_to_signal(si->node, SSID_CFG(si->node->ssid, min_connect_snr));
	if (si->signal < min_signal) {
		ev.reason = UEV_REASON_LOW_SIGNAL;
		ev.threshold.cur = si->signal;
		ev.threshold.ref = min_signal;
		ret = false;
		goto out;
	}

	uint32_t initial_connect_delay = SSID_CFG(si->node->ssid, initial_connect_delay);
	if (current_time - si->created < initial_connect_delay) {
		ev.reason = UEV_REASON_CONNECT_DELAY;
		ev.threshold.cur = current_time - si->created;
		ev.threshold.ref = initial_connect_delay;
		ret = false;
		goto out;
	}

	/* A candidate found only via the known-devices fallback (signal
	 * never actually observed on that node, just a "worth exploring"
	 * placeholder - see find_known_candidate()) must never be grounds
	 * to reject a probe/assoc request outright: this station may have
	 * no better option right now, and rejecting on a pure guess risks
	 * an assoc-reject loop instead of just a missed steering
	 * opportunity. */
	if (!find_better_candidate(si, &ev, UEV_SELECT_REASON_ALL, 0, &exploratory) || exploratory)
		goto out;

	ev.reason = UEV_REASON_BETTER_CANDIDATE;
	ev.node_cur = si->node;
	ret = false;

out:
	switch (type) {
	case EVENT_TYPE_PROBE:
		ev.type = ret ? UEV_PROBE_REQ_ACCEPT : UEV_PROBE_REQ_DENY;
		break;
	case EVENT_TYPE_ASSOC:
		ev.type = ret ? UEV_ASSOC_REQ_ACCEPT : UEV_ASSOC_REQ_DENY;
		break;
	case EVENT_TYPE_AUTH:
		ev.type = ret ? UEV_AUTH_REQ_ACCEPT : UEV_AUTH_REQ_DENY;
		break;
	default:
		break;
	}

	if (!ret && si->stats[type].blocked_cur >= SSID_CFG(si->node->ssid, max_retry_band)) {
		ev.reason = UEV_REASON_RETRY_EXCEEDED;
		ev.threshold.cur = si->stats[type].blocked_cur;
		ev.threshold.ref = SSID_CFG(si->node->ssid, max_retry_band);
	}
	usteer_event(&ev);

	return ret;
}

static bool
is_more_kickable(struct sta_info *si_cur, struct sta_info *si_new)
{
	if (!si_cur)
		return true;

	if (si_new->kick_count > si_cur->kick_count)
		return false;

	return si_cur->signal > si_new->signal;
}

static void
usteer_roam_set_state(struct sta_info *si, enum roam_trigger_state state,
		      struct uevent *ev)
{
	/* NOP in case we remain idle */
	if (si->roam_state == state && si->roam_state == ROAM_TRIGGER_IDLE) {
		si->roam_tries = 0;
		return;
	}

	si->roam_event = current_time;

	if (si->roam_state == state) {
		si->roam_tries++;
	} else {
		si->roam_tries = 0;
	}

	si->roam_state = state;
	usteer_event(ev);
}

static void
usteer_roam_sm_start_scan(struct sta_info *si, struct uevent *ev)
{
	uint32_t roam_scan_timeout = SSID_CFG(si->node->ssid, roam_scan_timeout);

	/* Start scanning in case we are not timeout-constrained or timeout has expired */
	if (!roam_scan_timeout ||
	    current_time > si->roam_scan_timeout_start + roam_scan_timeout) {
		usteer_roam_set_state(si, ROAM_TRIGGER_SCAN, ev);
		return;
	}

	/* We are currently in scan timeout / cooldown.
	 * Check if we are in ROAM_TRIGGER_IDLE state. Enter this state if not.
	 */
	if (si->roam_state == ROAM_TRIGGER_IDLE)
		return;

	/* Enter idle state */
	usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, ev);
}

static struct sta_info *
usteer_roam_sm_found_better_node(struct sta_info *si, struct uevent *ev, enum roam_trigger_state next_state,
				  bool *exploratory)
{
	uint64_t max_age = 2 * SSID_CFG(si->node->ssid, roam_scan_interval);
	struct sta_info *candidate;

	if (max_age > current_time - si->roam_scan_start)
		max_age = current_time - si->roam_scan_start;

	candidate = find_better_candidate(si, ev, (1 << UEV_SELECT_REASON_SIGNAL), max_age, exploratory);
	if (candidate)
		usteer_roam_set_state(si, next_state, ev);

	return candidate;
}

static bool
usteer_roam_trigger_sm(struct usteer_local_node *ln, struct sta_info *si)
{
	struct sta_info *candidate;
	uint32_t disassoc_timer;
	uint32_t validity_period;
	bool exploratory = false;
	uint32_t roam_scan_tries = SSID_CFG(si->node->ssid, roam_scan_tries);
	struct uevent ev = {
		.si_cur = si,
	};

	switch (si->roam_state) {
	case ROAM_TRIGGER_SCAN:
		if (!si->roam_tries) {
			si->roam_scan_start = current_time;
		}

		/* Check if we've found a better node regardless of the scan-interval */
		if (usteer_roam_sm_found_better_node(si, &ev, ROAM_TRIGGER_SCAN_DONE, NULL))
			break;

		/* Only scan every scan-interval */
		if (current_time - si->roam_event < SSID_CFG(si->node->ssid, roam_scan_interval))
			break;

		/* Check if no node was found within roam_scan_tries tries */
		if (roam_scan_tries && si->roam_tries >= roam_scan_tries) {
			if (!SSID_CFG(si->node->ssid, roam_scan_timeout)) {
				usteer_roam_set_state(si, ROAM_TRIGGER_SCAN_DONE, &ev);
			} else {
				/* Set timeout until roam_scans are paused */
				si->roam_scan_timeout_start = current_time;
				usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, &ev);
			}
			break;
		}

		/* Send beacon-request to client */
		usteer_ubus_trigger_client_scan(si);
		usteer_roam_sm_start_scan(si, &ev);
		break;

	case ROAM_TRIGGER_IDLE:
		usteer_roam_sm_start_scan(si, &ev);
		break;

	case ROAM_TRIGGER_SCAN_DONE:
		/* Roaming time over, switch back to ROAM_TRIGGER_IDLE */
		if (si->roam_transition_start && current_time - si->roam_transition_start > SSID_CFG(si->node->ssid, roam_kick_delay)) {
			si->roam_transition_start = 0;
			usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, &ev);
			break;
		}

		candidate = usteer_roam_sm_found_better_node(si, &ev, ROAM_TRIGGER_SCAN_DONE, &exploratory);
		/* Kick back in case no better node is found */
		if (!candidate) {
			usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, &ev);
			break;
		}

		if (!candidate->node->rrm_nr)
			MSG(VERBOSE, "Candidates node rrm nr not returned from hostapd. Neighbor list empty!");

		if (!si->roam_transition_start)
			si->roam_transition_start = current_time;
		si->roam_transition_request_validity_end = current_time + 10000;
		validity_period = 10000 / usteer_local_node_get_beacon_interval(ln); /* ~ 10 seconds */
		/* A candidate from the known-devices fallback (see
		 * find_known_candidate()) is a guess, not a confirmed better
		 * signal: the client is free to ignore it, and it must never
		 * be force-kicked - that would disconnect a station that may
		 * still be perfectly happy where it is, on the strength of a
		 * placeholder reading that was never actually observed. */
		if (si->aggressiveness >= 2 && !exploratory) {
			if (!si->kick_time)
				si->kick_time = current_time + SSID_CFG(si->node->ssid, roam_kick_delay);
			if (si->aggressiveness >= 3)
				disassoc_timer = (si->kick_time - current_time) / usteer_local_node_get_beacon_interval(ln);
			else
				disassoc_timer = 0;
			usteer_ubus_bss_transition_request(si, 1, true, disassoc_timer, true, validity_period, candidate->node);
			/* Countdown end */
			if (disassoc_timer < validity_period) {
				si->roam_transition_start = 0;
				usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, &ev);
			}
		} else {
			usteer_ubus_bss_transition_request(si, 1, false, 0, true, validity_period, candidate->node);
			usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, &ev);
		}
		break;
	}

	return false;
}

bool usteer_policy_can_perform_roam(struct sta_info *si)
{
	/* Only trigger for connected STAs */
	if (si->connected != STA_CONNECTED)
		return false;

	/* The node SSID may not have been known yet when the station info was
	 * created, so evaluate the per-MAC aggressiveness again here. */
	usteer_sta_update_aggressiveness(si);

	/* Only trigger for STA with active roaming */
	if (!si->aggressiveness)
		return false;

	/* Skip on pending kick */
	if (si->kick_time && si->kick_time <= current_time)
		return false;

	/* Skip if in validity period */
	if (current_time < si->roam_transition_request_validity_end)
		return false;

	/* Skip on rejected transition */
	if (si->bss_transition_response.status_code && current_time - si->bss_transition_response.timestamp < SSID_CFG(si->node->ssid, steer_reject_timeout))
		return false;

	/* Skip if transition accepted */
	if (!si->bss_transition_response.status_code && current_time - si->bss_transition_response.timestamp < SSID_CFG(si->node->ssid, roam_kick_delay))
		return false;

	/* Skip on previous kick attempt */
	if (current_time - si->roam_kick < SSID_CFG(si->node->ssid, roam_trigger_interval))
		return false;

	/* Skip if connection is established shorter than the trigger-interval */
	if (current_time - si->connected_since < SSID_CFG(si->node->ssid, roam_trigger_interval))
		return false;

	return true;
}

static bool
usteer_local_node_roam_sm_active(struct sta_info *si, int min_signal)
{
	if (!usteer_policy_can_perform_roam(si))
		return false;

	/* Signal has to be below scan / roam threshold */
	if (si->signal >= min_signal)
		return false;

	return true;
}

static void
usteer_local_node_roam_check(struct usteer_local_node *ln, struct uevent *ev)
{
	struct sta_info *si;
	int32_t roam_scan_snr = SSID_CFG(ln->node.ssid, roam_scan_snr);
	int32_t roam_trigger_snr = SSID_CFG(ln->node.ssid, roam_trigger_snr);
	int min_signal;

	if (roam_scan_snr)
		min_signal = roam_scan_snr;
	else if (roam_trigger_snr)
		min_signal = roam_trigger_snr;
	else
		return;

	usteer_update_time();
	min_signal = usteer_snr_to_signal(&ln->node, min_signal);

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (!usteer_local_node_roam_sm_active(si, min_signal)) {
			usteer_roam_set_state(si, ROAM_TRIGGER_IDLE, ev);
			continue;
		}

		/*
		 * If the state machine kicked a client, other clients should wait
		 * until the next turn
		 */
		if (usteer_roam_trigger_sm(ln, si))
			return;
	}
}

static void
usteer_local_node_snr_kick(struct usteer_local_node *ln)
{
	int32_t min_snr = SSID_CFG(ln->node.ssid, min_snr);
	unsigned int min_count = DIV_ROUND_UP(SSID_CFG(ln->node.ssid, min_snr_kick_delay), SSID_CFG(ln->node.ssid, local_sta_update));
	struct uevent ev = {
		.node_local = &ln->node,
	};
	struct sta_info *si;
	int min_signal;

	if (!min_snr)
		return;

	min_signal = usteer_snr_to_signal(&ln->node, min_snr);
	ev.threshold.ref = min_signal;

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (si->connected != STA_CONNECTED)
			continue;

		if (si->signal >= min_signal) {
			si->below_min_snr = 0;
			continue;
		} else {
			si->below_min_snr++;
		}

		if (si->below_min_snr <= min_count)
			continue;

		ev.type = UEV_SIGNAL_KICK;
		ev.threshold.cur = si->signal;
		ev.count = si->kick_count;
		usteer_event(&ev);

		usteer_ubus_kick_client(si, KICK_REASON_UNSPECIFIED);
		return;
	}
}

static void
usteer_local_node_load_kick(struct usteer_local_node *ln)
{
	struct usteer_node *node = &ln->node;
	struct sta_info *kick1 = NULL, *kick2 = NULL;
	struct sta_info *candidate = NULL;
	struct sta_info *si;
	struct uevent ev = {
		.node_local = &ln->node,
	};
	bool load_kick_enabled = SSID_CFG(node->ssid, load_kick_enabled);
	uint32_t load_kick_threshold = SSID_CFG(node->ssid, load_kick_threshold);
	uint32_t load_kick_delay = SSID_CFG(node->ssid, load_kick_delay);
	uint32_t load_kick_min_clients = SSID_CFG(node->ssid, load_kick_min_clients);
	unsigned int min_count = DIV_ROUND_UP(load_kick_delay, SSID_CFG(node->ssid, local_sta_update));

	if (!load_kick_enabled || !load_kick_threshold || !load_kick_delay)
		return;

	if (node->load < load_kick_threshold) {
		if (!ln->load_thr_count)
			return;

		ln->load_thr_count = 0;
		ev.type = UEV_LOAD_KICK_RESET;
		ev.threshold.cur = node->load;
		ev.threshold.ref = load_kick_threshold;
		goto out;
	}

	if (++ln->load_thr_count <= min_count) {
		if (ln->load_thr_count > 1)
			return;

		ev.type = UEV_LOAD_KICK_TRIGGER;
		ev.threshold.cur = node->load;
		ev.threshold.ref = load_kick_threshold;
		goto out;
	}

	ln->load_thr_count = 0;
	if (node->n_assoc < load_kick_min_clients) {
		ev.type = UEV_LOAD_KICK_MIN_CLIENTS;
		ev.threshold.cur = node->n_assoc;
		ev.threshold.ref = load_kick_min_clients;
		goto out;
	}

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		struct sta_info *tmp;

		if (si->connected != STA_CONNECTED)
			continue;

		if (is_more_kickable(kick1, si))
			kick1 = si;

		tmp = find_better_candidate(si, NULL, (1 << UEV_SELECT_REASON_LOAD), 0, NULL);
		if (!tmp)
			continue;

		if (is_more_kickable(kick2, si)) {
			kick2 = si;
			candidate = tmp;
		}
	}

	if (!kick1) {
		ev.type = UEV_LOAD_KICK_NO_CLIENT;
		goto out;
	}

	if (kick2)
		kick1 = kick2;

	kick1->kick_count++;

	ev.type = UEV_LOAD_KICK_CLIENT;
	ev.si_cur = kick1;
	ev.si_other = candidate;
	ev.count = kick1->kick_count;

	usteer_ubus_kick_client(kick1, SSID_CFG(node->ssid, load_kick_reason_code));

out:
	usteer_event(&ev);
}

static void
usteer_local_node_perform_kick(struct usteer_local_node *ln)
{
	struct sta_info *si;

	list_for_each_entry(si, &ln->node.sta_info, node_list) {
		if (!si->kick_time || si->kick_time > current_time)
			continue;

		usteer_ubus_kick_client(si, KICK_REASON_BSS_TRANSITION);
	}
}

void
usteer_local_node_kick(struct usteer_local_node *ln)
{
	struct uevent ev = {
		.node_local = &ln->node,
	};

	usteer_local_node_perform_kick(ln);

	usteer_local_node_snr_kick(ln);
	usteer_local_node_load_kick(ln);
	usteer_local_node_roam_check(ln, &ev);
}
