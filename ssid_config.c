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

#include <libubox/avl-cmp.h>

#include "ssid_config.h"

AVL_TREE(ssid_configs, avl_strcmp, false, NULL);

struct usteer_ssid_config *
usteer_ssid_config_get(const char *ssid)
{
	struct usteer_ssid_config *sc;

	if (!ssid || !*ssid)
		return NULL;

	return avl_find_element(&ssid_configs, ssid, sc, avl);
}

static void
usteer_ssid_config_free(struct usteer_ssid_config *sc)
{
	avl_delete(&ssid_configs, &sc->avl);
	free(sc->aggressiveness_mac_list);
	free(sc->node_up_script);
	/* sc->avl.key (ssid_buf) is NOT a separate allocation - it was
	 * carved out of the same calloc_a() block as sc itself, so freeing
	 * sc below also releases it. Freeing it separately here corrupts
	 * the heap (this was crashing every second set_config call, right
	 * after the first reconfiguration's cleanup pass ran). */
	free(sc);
}

/* Enum/policy/set/get order mirrors struct usteer_config's declaration order. */
enum {
	SCFG_SSID,
	SCFG_STA_BLOCK_TIMEOUT,
	SCFG_LOCAL_STA_TIMEOUT,
	SCFG_LOCAL_STA_UPDATE,
	SCFG_MAX_RETRY_BAND,
	SCFG_SEEN_POLICY_TIMEOUT,
	SCFG_ASSOC_STEERING,
	SCFG_PROBE_STEERING,
	SCFG_MAX_NEIGHBOR_REPORTS,
	SCFG_BAND_STEERING_THRESHOLD,
	SCFG_LOAD_BALANCING_THRESHOLD,
	SCFG_AGGRESSIVENESS,
	SCFG_AGGRESSIVENESS_MAC_LIST,
	SCFG_MIN_SNR,
	SCFG_MIN_SNR_KICK_DELAY,
	SCFG_MIN_CONNECT_SNR,
	SCFG_SIGNAL_DIFF_THRESHOLD,
	SCFG_STEER_REJECT_TIMEOUT,
	SCFG_ROAM_SCAN_SNR,
	SCFG_ROAM_PROCESS_TIMEOUT,
	SCFG_ROAM_SCAN_TRIES,
	SCFG_ROAM_SCAN_TIMEOUT,
	SCFG_ROAM_SCAN_INTERVAL,
	SCFG_ROAM_TRIGGER_SNR,
	SCFG_ROAM_TRIGGER_INTERVAL,
	SCFG_ROAM_KICK_DELAY,
	SCFG_BAND_STEERING_INTERVAL,
	SCFG_BAND_STEERING_MIN_SNR,
	SCFG_BAND_STEERING_SIGNAL_THRESHOLD,
	SCFG_INITIAL_CONNECT_DELAY,
	SCFG_LOAD_KICK_ENABLED,
	SCFG_LOAD_KICK_THRESHOLD,
	SCFG_LOAD_KICK_DELAY,
	SCFG_LOAD_KICK_MIN_CLIENTS,
	SCFG_LOAD_KICK_REASON_CODE,
	SCFG_NODE_UP_SCRIPT,
	__SCFG_MAX
};

static const struct blobmsg_policy ssid_cfg_policy[__SCFG_MAX] = {
	[SCFG_SSID] = { "ssid", BLOBMSG_TYPE_STRING },
	[SCFG_STA_BLOCK_TIMEOUT] = { "sta_block_timeout", BLOBMSG_TYPE_INT32 },
	[SCFG_LOCAL_STA_TIMEOUT] = { "local_sta_timeout", BLOBMSG_TYPE_INT32 },
	[SCFG_LOCAL_STA_UPDATE] = { "local_sta_update", BLOBMSG_TYPE_INT32 },
	[SCFG_MAX_RETRY_BAND] = { "max_retry_band", BLOBMSG_TYPE_INT32 },
	[SCFG_SEEN_POLICY_TIMEOUT] = { "seen_policy_timeout", BLOBMSG_TYPE_INT32 },
	[SCFG_ASSOC_STEERING] = { "assoc_steering", BLOBMSG_TYPE_BOOL },
	[SCFG_PROBE_STEERING] = { "probe_steering", BLOBMSG_TYPE_BOOL },
	[SCFG_MAX_NEIGHBOR_REPORTS] = { "max_neighbor_reports", BLOBMSG_TYPE_INT32 },
	[SCFG_BAND_STEERING_THRESHOLD] = { "band_steering_threshold", BLOBMSG_TYPE_INT32 },
	[SCFG_LOAD_BALANCING_THRESHOLD] = { "load_balancing_threshold", BLOBMSG_TYPE_INT32 },
	[SCFG_AGGRESSIVENESS] = { "aggressiveness", BLOBMSG_TYPE_INT32 },
	[SCFG_AGGRESSIVENESS_MAC_LIST] = { "aggressiveness_mac_list", BLOBMSG_TYPE_ARRAY },
	[SCFG_MIN_SNR] = { "min_snr", BLOBMSG_TYPE_INT32 },
	[SCFG_MIN_SNR_KICK_DELAY] = { "min_snr_kick_delay", BLOBMSG_TYPE_INT32 },
	[SCFG_MIN_CONNECT_SNR] = { "min_connect_snr", BLOBMSG_TYPE_INT32 },
	[SCFG_SIGNAL_DIFF_THRESHOLD] = { "signal_diff_threshold", BLOBMSG_TYPE_INT32 },
	[SCFG_STEER_REJECT_TIMEOUT] = { "steer_reject_timeout", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_SCAN_SNR] = { "roam_scan_snr", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_PROCESS_TIMEOUT] = { "roam_process_timeout", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_SCAN_TRIES] = { "roam_scan_tries", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_SCAN_TIMEOUT] = { "roam_scan_timeout", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_SCAN_INTERVAL] = { "roam_scan_interval", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_TRIGGER_SNR] = { "roam_trigger_snr", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_TRIGGER_INTERVAL] = { "roam_trigger_interval", BLOBMSG_TYPE_INT32 },
	[SCFG_ROAM_KICK_DELAY] = { "roam_kick_delay", BLOBMSG_TYPE_INT32 },
	[SCFG_BAND_STEERING_INTERVAL] = { "band_steering_interval", BLOBMSG_TYPE_INT32 },
	[SCFG_BAND_STEERING_MIN_SNR] = { "band_steering_min_snr", BLOBMSG_TYPE_INT32 },
	[SCFG_BAND_STEERING_SIGNAL_THRESHOLD] = { "band_steering_signal_threshold", BLOBMSG_TYPE_INT32 },
	[SCFG_INITIAL_CONNECT_DELAY] = { "initial_connect_delay", BLOBMSG_TYPE_INT32 },
	[SCFG_LOAD_KICK_ENABLED] = { "load_kick_enabled", BLOBMSG_TYPE_BOOL },
	[SCFG_LOAD_KICK_THRESHOLD] = { "load_kick_threshold", BLOBMSG_TYPE_INT32 },
	[SCFG_LOAD_KICK_DELAY] = { "load_kick_delay", BLOBMSG_TYPE_INT32 },
	[SCFG_LOAD_KICK_MIN_CLIENTS] = { "load_kick_min_clients", BLOBMSG_TYPE_INT32 },
	[SCFG_LOAD_KICK_REASON_CODE] = { "load_kick_reason_code", BLOBMSG_TYPE_INT32 },
	[SCFG_NODE_UP_SCRIPT] = { "node_up_script", BLOBMSG_TYPE_STRING },
};

#define SCFG_INT(_sc, _tb, _field, _idx) \
	(_sc)->_field = (_tb)[_idx] ? blobmsg_get_u32(_tb[_idx]) : config._field
#define SCFG_SINT(_sc, _tb, _field, _idx) \
	(_sc)->_field = (_tb)[_idx] ? (int32_t) blobmsg_get_u32(_tb[_idx]) : config._field
#define SCFG_BOOL(_sc, _tb, _field, _idx) \
	(_sc)->_field = (_tb)[_idx] ? blobmsg_get_bool(_tb[_idx]) : config._field
/* Blob-typed fields can't be shared with the global config by pointer
 * (config._field can be freed/replaced by a later config_set_*() call) -
 * always take an owned copy, either of the override or of the current
 * global value, so it stays valid for the lifetime of this sc entry. */
#define SCFG_LIST(_sc, _tb, _field, _idx) \
	do { \
		struct blob_attr *__src = (_tb)[_idx] ? (_tb)[_idx] : config._field; \
		(_sc)->_field = __src ? blob_memdup(__src) : NULL; \
	} while (0)

void config_set_ssid_configs(struct blob_attr *data)
{
	struct usteer_ssid_config *sc, *tmp;
	struct blob_attr *cur;
	int rem;

	avl_for_each_element_safe(&ssid_configs, sc, avl, tmp)
		usteer_ssid_config_free(sc);

	if (!data)
		return;

	blobmsg_for_each_attr(cur, data, rem) {
		struct blob_attr *tb[__SCFG_MAX];
		char *ssid_buf;

		blobmsg_parse(ssid_cfg_policy, __SCFG_MAX, tb, blobmsg_data(cur), blobmsg_data_len(cur));
		if (!tb[SCFG_SSID])
			continue;

		sc = calloc_a(sizeof(*sc), &ssid_buf, strlen(blobmsg_get_string(tb[SCFG_SSID])) + 1);
		strcpy(ssid_buf, blobmsg_get_string(tb[SCFG_SSID]));
		sc->avl.key = ssid_buf;

		SCFG_INT(sc, tb, sta_block_timeout, SCFG_STA_BLOCK_TIMEOUT);
		SCFG_INT(sc, tb, local_sta_timeout, SCFG_LOCAL_STA_TIMEOUT);
		SCFG_INT(sc, tb, local_sta_update, SCFG_LOCAL_STA_UPDATE);
		SCFG_INT(sc, tb, max_retry_band, SCFG_MAX_RETRY_BAND);
		SCFG_INT(sc, tb, seen_policy_timeout, SCFG_SEEN_POLICY_TIMEOUT);
		SCFG_BOOL(sc, tb, assoc_steering, SCFG_ASSOC_STEERING);
		SCFG_BOOL(sc, tb, probe_steering, SCFG_PROBE_STEERING);
		SCFG_INT(sc, tb, max_neighbor_reports, SCFG_MAX_NEIGHBOR_REPORTS);
		SCFG_INT(sc, tb, band_steering_threshold, SCFG_BAND_STEERING_THRESHOLD);
		SCFG_INT(sc, tb, load_balancing_threshold, SCFG_LOAD_BALANCING_THRESHOLD);
		SCFG_INT(sc, tb, aggressiveness, SCFG_AGGRESSIVENESS);
		SCFG_LIST(sc, tb, aggressiveness_mac_list, SCFG_AGGRESSIVENESS_MAC_LIST);
		SCFG_SINT(sc, tb, min_snr, SCFG_MIN_SNR);
		SCFG_INT(sc, tb, min_snr_kick_delay, SCFG_MIN_SNR_KICK_DELAY);
		SCFG_SINT(sc, tb, min_connect_snr, SCFG_MIN_CONNECT_SNR);
		SCFG_INT(sc, tb, signal_diff_threshold, SCFG_SIGNAL_DIFF_THRESHOLD);
		SCFG_INT(sc, tb, steer_reject_timeout, SCFG_STEER_REJECT_TIMEOUT);
		SCFG_SINT(sc, tb, roam_scan_snr, SCFG_ROAM_SCAN_SNR);
		SCFG_INT(sc, tb, roam_process_timeout, SCFG_ROAM_PROCESS_TIMEOUT);
		SCFG_INT(sc, tb, roam_scan_tries, SCFG_ROAM_SCAN_TRIES);
		SCFG_INT(sc, tb, roam_scan_timeout, SCFG_ROAM_SCAN_TIMEOUT);
		SCFG_INT(sc, tb, roam_scan_interval, SCFG_ROAM_SCAN_INTERVAL);
		SCFG_SINT(sc, tb, roam_trigger_snr, SCFG_ROAM_TRIGGER_SNR);
		SCFG_INT(sc, tb, roam_trigger_interval, SCFG_ROAM_TRIGGER_INTERVAL);
		SCFG_INT(sc, tb, roam_kick_delay, SCFG_ROAM_KICK_DELAY);
		SCFG_INT(sc, tb, band_steering_interval, SCFG_BAND_STEERING_INTERVAL);
		SCFG_SINT(sc, tb, band_steering_min_snr, SCFG_BAND_STEERING_MIN_SNR);
		SCFG_INT(sc, tb, band_steering_signal_threshold, SCFG_BAND_STEERING_SIGNAL_THRESHOLD);
		SCFG_INT(sc, tb, initial_connect_delay, SCFG_INITIAL_CONNECT_DELAY);
		SCFG_BOOL(sc, tb, load_kick_enabled, SCFG_LOAD_KICK_ENABLED);
		SCFG_INT(sc, tb, load_kick_threshold, SCFG_LOAD_KICK_THRESHOLD);
		SCFG_INT(sc, tb, load_kick_delay, SCFG_LOAD_KICK_DELAY);
		SCFG_INT(sc, tb, load_kick_min_clients, SCFG_LOAD_KICK_MIN_CLIENTS);
		SCFG_INT(sc, tb, load_kick_reason_code, SCFG_LOAD_KICK_REASON_CODE);
		/* No global fallback copy here (unlike SCFG_LIST) - the real
		 * global value isn't config.node_up_script (dead field), so
		 * leave unset overrides NULL and let call sites fall back to
		 * the actual global themselves. */
		if (tb[SCFG_NODE_UP_SCRIPT])
			sc->node_up_script = strdup(blobmsg_get_string(tb[SCFG_NODE_UP_SCRIPT]));

		avl_insert(&ssid_configs, &sc->avl);
	}
}

void config_get_ssid_configs(struct blob_buf *buf)
{
	struct usteer_ssid_config *sc;
	void *c, *e;

	c = blobmsg_open_array(buf, "ssid_configs");
	avl_for_each_element(&ssid_configs, sc, avl) {
		e = blobmsg_open_table(buf, NULL);
		blobmsg_add_string(buf, "ssid", sc->avl.key);
		blobmsg_add_u32(buf, "sta_block_timeout", sc->sta_block_timeout);
		blobmsg_add_u32(buf, "local_sta_timeout", sc->local_sta_timeout);
		blobmsg_add_u32(buf, "local_sta_update", sc->local_sta_update);
		blobmsg_add_u32(buf, "max_retry_band", sc->max_retry_band);
		blobmsg_add_u32(buf, "seen_policy_timeout", sc->seen_policy_timeout);
		blobmsg_add_u8(buf, "assoc_steering", sc->assoc_steering);
		blobmsg_add_u8(buf, "probe_steering", sc->probe_steering);
		blobmsg_add_u32(buf, "max_neighbor_reports", sc->max_neighbor_reports);
		blobmsg_add_u32(buf, "band_steering_threshold", sc->band_steering_threshold);
		blobmsg_add_u32(buf, "load_balancing_threshold", sc->load_balancing_threshold);
		blobmsg_add_u32(buf, "aggressiveness", sc->aggressiveness);
		if (sc->aggressiveness_mac_list)
			blobmsg_add_blob(buf, sc->aggressiveness_mac_list);
		blobmsg_add_u32(buf, "min_snr", sc->min_snr);
		blobmsg_add_u32(buf, "min_snr_kick_delay", sc->min_snr_kick_delay);
		blobmsg_add_u32(buf, "min_connect_snr", sc->min_connect_snr);
		blobmsg_add_u32(buf, "signal_diff_threshold", sc->signal_diff_threshold);
		blobmsg_add_u32(buf, "steer_reject_timeout", sc->steer_reject_timeout);
		blobmsg_add_u32(buf, "roam_scan_snr", sc->roam_scan_snr);
		blobmsg_add_u32(buf, "roam_process_timeout", sc->roam_process_timeout);
		blobmsg_add_u32(buf, "roam_scan_tries", sc->roam_scan_tries);
		blobmsg_add_u32(buf, "roam_scan_timeout", sc->roam_scan_timeout);
		blobmsg_add_u32(buf, "roam_scan_interval", sc->roam_scan_interval);
		blobmsg_add_u32(buf, "roam_trigger_snr", sc->roam_trigger_snr);
		blobmsg_add_u32(buf, "roam_trigger_interval", sc->roam_trigger_interval);
		blobmsg_add_u32(buf, "roam_kick_delay", sc->roam_kick_delay);
		blobmsg_add_u32(buf, "band_steering_interval", sc->band_steering_interval);
		blobmsg_add_u32(buf, "band_steering_min_snr", sc->band_steering_min_snr);
		blobmsg_add_u32(buf, "band_steering_signal_threshold", sc->band_steering_signal_threshold);
		blobmsg_add_u32(buf, "initial_connect_delay", sc->initial_connect_delay);
		blobmsg_add_u8(buf, "load_kick_enabled", sc->load_kick_enabled);
		blobmsg_add_u32(buf, "load_kick_threshold", sc->load_kick_threshold);
		blobmsg_add_u32(buf, "load_kick_delay", sc->load_kick_delay);
		blobmsg_add_u32(buf, "load_kick_min_clients", sc->load_kick_min_clients);
		blobmsg_add_u32(buf, "load_kick_reason_code", sc->load_kick_reason_code);
		if (sc->node_up_script)
			blobmsg_add_string(buf, "node_up_script", sc->node_up_script);
		blobmsg_close_table(buf, e);
	}
	blobmsg_close_array(buf, c);
}
