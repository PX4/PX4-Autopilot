/**
 *  @file gps_metadata_monitor.cpp
 *  @brief Perform selective CN0 analysis to determine indoor vs outdoor localization
 */
/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "gps_metadata_monitor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <cmath>
#include <lib/parameters/param.h>

#include "devices/src/ashtech.h"
#include "devices/src/emlid_reach.h"
#include "devices/src/mtk.h"
#include "devices/src/ubx.h"


GPSDriverUBXModal::GPSDriverUBXModal(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
				     struct vehicle_gps_position_s *gps_position,
				     struct satellite_info_s *satellite_info,
				     uint8_t dynamic_model) :
	GPSDriverUBX(gpsInterface,
		     callback,
		     callback_user,
		     gps_position,
		     satellite_info,
		     dynamic_model)
	, _position(gps_position)
	, _sat_info(satellite_info)
{
	init();
}

GPSDriverUBXModal::~GPSDriverUBXModal()
{
}

void GPSDriverUBXModal::init()
{

	_min_sat_elevation = 0.f;
	_min_sat_avg_cno = 0.f;
	param_get(param_find("GPS_MIN_EL"), &_min_sat_elevation);
	param_get(param_find("GPS_MIN_CNO"), &_min_sat_avg_cno);

	GPS_WARN("ModalAi: GPS metadata configuration is Min_sat_elevation:  %f, Min_sat_CNO:  %f", (double)_min_sat_elevation,
		 (double) _min_sat_avg_cno);

}



//
// Check if the monitored satellite database is full or not.
// return true = ready to perform analysis
// return false = not ready
// basically indicates if we launching INDOORS
//
bool
GPSDriverUBXModal::monitorGPSSignalQualitySize()
{
	return _sats_being_monitored >= CFG_MIN_SATS_TO_MONITOR;
}

//
// get the average CN0 from selective analysis
// Return the selective CN0 signal quality in dB >0, larger the number the better the quality
//
int16_t
GPSDriverUBXModal::monitorGPSSignalQuality()
{
	static bool reset_data = true;

	// main return value
	static float avgSNR = -1.0;
	static float lastAvgSNR = -1.0;
	static bool cnoChanged = false;

	// if the fix is not a lock, rset the database
	// very likely we are indoors
	if (_position->fix_type < 3) {
		reset_data = true;

	} else {

		if (cnoChanged) {
			cnoChanged = false;

			for (int k = 0; k < satellite_info_s::SAT_INFO_MAX_SATELLITES; k++) {
				GPS_INFO("GPS inventory sat\t#%u\tused %u\tsnr %u\televation %u\tsvid %u",
					 (unsigned) k,
					 (unsigned)_sat_info->used[k],
					 (unsigned)_sat_info->snr[k],
					 (unsigned)_sat_info->elevation[k],
					 (unsigned)_sat_info->svid[k]);
			}
		}

		// rebuild the selective monitoring database
		if (reset_data) {
			avgSNR = 0;
			reset_data = false;
			memset(_monitored_sats, 0, sizeof(_monitored_sats));
			memset(_monitored_sats_id, 0, sizeof(_monitored_sats_id));
			_sats_being_monitored = 0;
		}

		// build a database of sats that meet the elevation requirement
		if (_sats_being_monitored < CFG_MIN_SATS_TO_MONITOR) {
			// wait until we grab enough sats to do the calculation based on the overhead criteria
			for (int i = 0; i < satellite_info_s::SAT_INFO_MAX_SATELLITES; i++) {
				if (_sat_info->used[i]) {
					if (_sat_info->elevation[i] > _min_sat_elevation && _sat_info->snr[i] > _min_sat_avg_cno) {
						// add to manifest
						bool is_exists = false;

						for (int j = 0; j < CFG_MIN_SATS_TO_MONITOR; j++) {
							if (_monitored_sats_id[j] == _sat_info->svid[i]) {
								is_exists = true;
								break;
							}
						}

						if (!is_exists) {
							GPS_INFO("Adding sat %d", _sats_being_monitored);
							_monitored_sats[_sats_being_monitored] = i;
							_monitored_sats_id[_sats_being_monitored] = _sat_info->svid[i];
							_sats_being_monitored++;

							if (_sats_being_monitored >= CFG_MIN_SATS_TO_MONITOR) {
								break;
							}
						}
					}
				}
			}

		} else {	/* we have enough sats to conduct selective cn0 analysis */
			for (int i = 0; i < CFG_MIN_SATS_TO_MONITOR; i++) {
				if (_sat_info->elevation[_monitored_sats[i]] > _min_sat_elevation) {
					avgSNR +=  _sat_info->snr[_monitored_sats[i]];

				} else {
					PX4_WARN("Tracking satellite has disappeared from horizon for unknown reason--please check location! %d %d",
						 _sat_info->svid[_monitored_sats[i]], _monitored_sats_id[i]);
					reset_data = true; // Force a reset of the table to monitor
					break;

				}
			}

			if (!reset_data) {
				avgSNR /= CFG_MIN_SATS_TO_MONITOR;

				if (avgSNR < _min_sat_avg_cno) {
					avgSNR = 0; // force failure
				}

				if ((lastAvgSNR > 0.0f && (int) avgSNR == 0) || (lastAvgSNR <= 0.0f && avgSNR > 0.0f)) {
					cnoChanged = true;
				}

				lastAvgSNR = avgSNR;
			}

		}
	}

	return avgSNR;
}
