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

#ifndef __USTEER_SSID_CONFIG_H
#define __USTEER_SSID_CONFIG_H

#include "usteer.h"

/*
 * Optional per-SSID overrides of the roaming-policy subset of
 * usteer_config. Every field here mirrors a same-named field in
 * usteer_config exactly (same type, same meaning) - the only
 * difference is that these apply to stations on one specific SSID
 * instead of network-wide.
 *
 * A SSID with no entry here simply uses the global usteer_config value
 * for every field, unchanged - existing single-SSID or "same policy
 * everywhere" setups need no configuration change at all. Use
 * SSID_CFG(ssid, field) everywhere a call site used to read
 * config.field directly, to transparently get the per-SSID override
 * when one exists.
 */
struct usteer_ssid_config {
	struct avl_node avl; /* key: heap-allocated copy of the SSID string */

	/* Field order mirrors struct usteer_config's declaration order. */
	uint32_t sta_block_timeout;
	uint32_t local_sta_timeout;
	uint32_t local_sta_update;

	uint32_t max_retry_band;
	uint32_t seen_policy_timeout;

	bool assoc_steering;
	bool probe_steering;

	uint32_t max_neighbor_reports;

	uint32_t band_steering_threshold;
	uint32_t load_balancing_threshold;

	uint32_t aggressiveness;
	struct blob_attr *aggressiveness_mac_list;

	int32_t min_snr;
	uint32_t min_snr_kick_delay;
	int32_t min_connect_snr;
	uint32_t signal_diff_threshold;

	uint32_t steer_reject_timeout;

	int32_t roam_scan_snr;
	uint32_t roam_process_timeout;

	uint32_t roam_scan_tries;
	uint32_t roam_scan_timeout;
	uint32_t roam_scan_interval;

	int32_t roam_trigger_snr;
	uint32_t roam_trigger_interval;

	uint32_t roam_kick_delay;

	uint32_t band_steering_interval;
	int32_t band_steering_min_snr;
	uint32_t band_steering_signal_threshold;

	uint32_t initial_connect_delay;

	bool load_kick_enabled;
	uint32_t load_kick_threshold;
	uint32_t load_kick_delay;
	uint32_t load_kick_min_clients;
	uint32_t load_kick_reason_code;

	/* NULL if unset - unlike every other field here, the global
	 * fallback for this one is NOT config.node_up_script (that struct
	 * field is dead; the real global value is a private static in
	 * local_node.c, set via config_set_node_up_script()), so callers
	 * can't use the SSID_CFG() macro for it and must fall back to that
	 * global manually - see usteer_local_node_up_script_run() call
	 * sites. */
	char *node_up_script;
};

extern struct avl_tree ssid_configs;

/* NULL if ssid has no override - callers fall back to global config. */
struct usteer_ssid_config *usteer_ssid_config_get(const char *ssid);

/*
 * ssid may be NULL or empty (e.g. a node whose SSID hasn't been
 * populated yet) - always falls back to the global value in that case.
 */
#define SSID_CFG(ssid, field) \
	({ \
		struct usteer_ssid_config *__sc = (ssid) ? usteer_ssid_config_get(ssid) : NULL; \
		__sc ? __sc->field : config.field; \
	})

void config_set_ssid_configs(struct blob_attr *data);
void config_get_ssid_configs(struct blob_buf *buf);

#endif
