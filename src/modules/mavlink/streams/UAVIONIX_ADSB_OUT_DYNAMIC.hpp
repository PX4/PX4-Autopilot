/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#ifndef UAVIONIX_ADSB_OUT_DYNAMIC_HPP
#define UAVIONIX_ADSB_OUT_DYNAMIC_HPP

#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

class MavlinkStreamUavionixADSBOutDynamic : public ModuleParams, public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamUavionixADSBOutDynamic(mavlink); }

	static constexpr const char *get_name_static() { return "UAVIONIX_ADSB_OUT_DYNAMIC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamUavionixADSBOutDynamic(Mavlink *mavlink) : ModuleParams(nullptr), MavlinkStream(mavlink) {}


	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ADSB_SQUAWK>)		_adsb_squawk,
		(ParamInt<px4::params::ADSB_EMERGC>)		_adsb_emergc
	);


	bool send() override
	{

		vehicle_status_s vehicle_status;
		_vehicle_status_sub.copy(&vehicle_status);

		sensor_gps_s vehicle_gps_position;
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		vehicle_air_data_s vehicle_air_data;
		_vehicle_air_data_sub.copy(&vehicle_air_data);

		// Required update for dynamic message is 5 [Hz]
		mavlink_uavionix_adsb_out_dynamic_t dynamic_msg = {
			.utcTime = static_cast<uint32_t>(vehicle_gps_position.time_utc_usec / 1000000ULL),
			.gpsLat = static_cast<int32_t>(round(vehicle_gps_position.latitude_deg * 1e7)),
			.gpsLon = static_cast<int32_t>(round(vehicle_gps_position.longitude_deg * 1e7)),
			.gpsAlt = static_cast<int32_t>(round(vehicle_gps_position.altitude_ellipsoid_m * 1e3)), // convert [m] to [mm]
			.baroAltMSL = static_cast<int32_t>(vehicle_air_data.baro_pressure_pa / 100.0f), // convert [Pa] to [mBar]
			.accuracyHor = static_cast<uint32_t>(vehicle_gps_position.eph * 1000.0f), // convert [m] to [mm]
			.accuracyVert = static_cast<uint16_t>(vehicle_gps_position.epv * 100.0f), // convert [m] to [cm]
			.accuracyVel = static_cast<uint16_t>(vehicle_gps_position.s_variance_m_s * 1000.f), // convert [m/s] to [mm/s],
			.velVert = static_cast<int16_t>(-1.0f * vehicle_gps_position.vel_d_m_s * 100.0f), // convert [m/s] to [cm/s]
			.velNS = static_cast<int16_t>(vehicle_gps_position.vel_n_m_s * 100.0f), // convert [m/s] to [cm/s]
			.VelEW = static_cast<int16_t>(vehicle_gps_position.vel_e_m_s * 100.0f), // convert [m/s] to [cm/s]
			.state = UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND,
			.squawk = static_cast<uint16_t>(_adsb_squawk.get()),
			.gpsFix = vehicle_gps_position.fix_type,
			.numSats = vehicle_gps_position.satellites_used,
			.emergencyStatus = static_cast<uint8_t>(_adsb_emergc.get())
		};

		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			dynamic_msg.state |= ~UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
		}

		mavlink_msg_uavionix_adsb_out_dynamic_send_struct(_mavlink->get_channel(), &dynamic_msg);

		return true;
	}
};

#endif // UAVIONIX_ADSB_OUT_DYNAMIC_HPP
