/****************************************************************************
 *
 *	Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

/**
 * @file spektrum_rssi.h
 *
 * RSSI dBm to percentage conversion for Spektrum telemetry receivers
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 */

#pragma once

#include <math.h>

#define MIN_RSSI_DBM (-92.0f)
#define MAX_RSSI_DBM (-42.0f)

class SpektrumRssi
{
	int8_t lu_dbm_percent[129];

public:
	SpektrumRssi()
	{
		for (int i = 0; i <= 128; i++) {
			float rssi_dbm = -1.0f * (float)i;
			float percent;

			if (rssi_dbm > MAX_RSSI_DBM) {
				percent = 100.0f;

			} else if (rssi_dbm < MIN_RSSI_DBM) {
				percent = 0.0f;

			} else {
				percent = 100.0f * log10f(1 + (rssi_dbm - MIN_RSSI_DBM) * (9.0f / (MAX_RSSI_DBM - MIN_RSSI_DBM)));
			}

			lu_dbm_percent[i] = (int8_t)roundf(percent);
		}
	}

	int8_t dbm_to_percent(int8_t dbm) const
	{
		return lu_dbm_percent[abs(dbm)];
	}
};
