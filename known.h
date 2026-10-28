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

#ifndef __USTEER_KNOWN_H
#define __USTEER_KNOWN_H

#include "usteer.h"

/*
 * Optional, opt-in (config.known_stations) persisted memory of the best
 * signal ever locally observed for a station on a given node.
 *
 * This exists because some stations (cheap/embedded WiFi stacks: home
 * appliances, IoT sensors, ...) never probe or report RRM neighbor/beacon
 * measurements once associated, so usteer's normal candidate search
 * (which only ever compares stations it has *live* signal data for on
 * another node) can never find a better node for them, no matter how bad
 * their current signal gets.
 *
 * For a stationary station this historical reading remains physically
 * valid indefinitely, so it is kept on disk and re-loaded on restart.
 * A remote node's known-station data only ever exists for as long as
 * that node is actively broadcasting on the usteer remote protocol (see
 * remote.c) - a peer that stops sending updates has its remote node (and
 * with it, its known-station list) removed by the existing
 * remote_node_timeout logic, so a station can never be pointed at a peer
 * that is not confirmed alive in the current update cycle.
 *
 * Cold start: whenever a node learns, via the normal remote exchange,
 * that some station is connected to a peer (interface_add_station() in
 * remote.c), it also seeds a signal == 0 placeholder for that station on
 * each of its own local nodes that share its SSID and don't already have
 * any entry. 0 is never a real reading (real dBm signal is always
 * negative), so it always looks like a large improvement to the normal
 * comparison logic - the peer that actually holds the connection ends up
 * offering this node as a candidate and pushing the station over via the
 * ordinary roam path. usteer_known_update() then unconditionally
 * replaces the placeholder with whatever real signal is measured there,
 * after which the station is judged on its merits like any other - sent
 * straight back if the untested node turns out to be worse.
 */
struct usteer_known_sta {
	struct list_head list;

	uint8_t addr[6];
	int signal;
	uint64_t timestamp; /* current_time (ms) of this reading */
};

void usteer_known_node_init(struct usteer_node *node, const char *name);
void usteer_known_node_free(struct usteer_node *node);

/* Record a fresh, locally-observed signal reading (keep-if-better). */
void usteer_known_update(struct usteer_node *node, const uint8_t *addr, int signal);

/* Find the best known-signal record for addr on the given node, if any. */
struct usteer_known_sta *usteer_known_find(struct usteer_node *node, const uint8_t *addr);

/* Manually remove a single record, e.g. on user request from the UI. */
void usteer_known_delete(struct usteer_node *node, const uint8_t *addr);

/* Drop entries older than config.known_stations_timeout (0 = never). */
void usteer_known_prune(struct usteer_node *node);

void usteer_known_init(void);

#endif
