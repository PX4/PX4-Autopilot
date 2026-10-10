/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once

#include <stdint.h>

namespace sensors
{
namespace gnss_order
{

/**
 * Fixed position of each receiver in sensors_status_gnss.order, which GPS_RAW_INT and GPS2_RAW report. An operator reads
 * them as fixed receivers, so the order doesn't follow the selection, and a position held for a receiver that hasn't
 * published yet stays free, so that a late receiver doesn't shift the others.
 *
 * With a configured preference, the preferred receiver comes first. Without one, a receiver whose device ID is in a
 * SENS_GNSSn_ID slot takes position n, which is stable across boots. The others take the free positions as they first
 * publish; when only held positions are left (a receiver matches no configured slot), one not taken by a publishing
 * receiver, so that every receiver is still reported.
 *
 * @param first_publication 1 for the first receiver to publish, 2 for the next, 0 before it publishes
 * @param slot SENS_GNSSn_ID slot that has the receiver's device ID, -1 for none
 * @param slot_configured SENS_GNSSn_ID is set
 * @param preferred receiver SENS_GNSS_PRIME designates, -1 for none or not yet published
 * @param preference_configured SENS_GNSS_PRIME names a receiver, whether or not it has published
 * @param order position of each receiver, -1 until it publishes
 */
template<int N>
void receiverOrder(const uint8_t (&first_publication)[N], const int8_t (&slot)[N], const bool (&slot_configured)[N],
		   int preferred, bool preference_configured, int8_t (&order)[N])
{
	bool held[N] {};  // for a receiver that may not have published yet
	bool taken[N] {}; // by a receiver that has published

	for (int i = 0; i < N; i++) {
		order[i] = -1;
	}

	if (preference_configured) {
		held[0] = true;

		if ((preferred >= 0) && (preferred < N) && (first_publication[preferred] != 0)) {
			order[preferred] = 0;
			taken[0] = true;
		}

	} else {
		for (int i = 0; i < N; i++) {
			held[i] = slot_configured[i];
		}

		for (int i = 0; i < N; i++) {
			if ((first_publication[i] != 0) && (slot[i] >= 0) && (slot[i] < N) && !taken[slot[i]]) {
				order[i] = slot[i];
				taken[slot[i]] = true;
			}
		}
	}

	for (int publication = 1; publication <= N; publication++) {
		for (int i = 0; i < N; i++) {
			if ((first_publication[i] != publication) || (order[i] >= 0)) {
				continue;
			}

			int position = -1;

			for (int p = 0; (p < N) && (position < 0); p++) {
				if (!taken[p] && !held[p]) {
					position = p;
				}
			}

			for (int p = 0; (p < N) && (position < 0); p++) {
				if (!taken[p]) {
					position = p;
				}
			}

			if (position >= 0) {
				order[i] = position;
				taken[position] = true;
			}
		}
	}
}

} // namespace gnss_order
} // namespace sensors
