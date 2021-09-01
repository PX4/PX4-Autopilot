/**
 *  @file gps_metadata_monitor.h
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


#ifndef SRC_DRIVERS_GPS_GPS_METADATA_MONITOR_H_
#define SRC_DRIVERS_GPS_GPS_METADATA_MONITOR_H_

#include "devices/src/ubx.h"

/*
 * minimum satellites to conduct CN0 analysis.
 * This is current minimum for a valid 3D fix
 */
#define CFG_MIN_SATS_TO_MONITOR 5

class GPSDriverUBXModal : public GPSDriverUBX
{
public:

	GPSDriverUBXModal(Interface gpsInterface,
			  GPSCallbackPtr callback,
			  void *callback_user,
			  struct vehicle_gps_position_s *gps_position,
			  struct satellite_info_s *satellite_info,
			  uint8_t dynamic_model = 7);

	virtual ~GPSDriverUBXModal();

	void init();

	// methods for monitoring CN0 signal quality
	// See https://downloads.hindawi.com/journals/misy/2012/109129.pdf
	int16_t monitorGPSSignalQuality();
	bool monitorGPSSignalQualitySize();


protected:
	struct vehicle_gps_position_s *_position {nullptr};
	struct satellite_info_s *_sat_info {nullptr};

	// Variables for CN0 monitor feature
	uint8_t _monitored_sats[CFG_MIN_SATS_TO_MONITOR];
	uint8_t _monitored_sats_id[CFG_MIN_SATS_TO_MONITOR];
	uint8_t _sats_being_monitored 	= 0;
	float _min_sat_elevation 	= 0;
	float _min_sat_avg_cno 		= 0;

};


#endif /* SRC_DRIVERS_GPS_GPS_METADATA_MONITOR_H_ */
