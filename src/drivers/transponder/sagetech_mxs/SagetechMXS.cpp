/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "SagetechMXS.hpp"
#define GPS_SUB_TEST 0
#define ADSB_PUB_TEST 0
#define SG_HW_TEST


SagetechMXS::SagetechMXS(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	strncpy(_port, port, sizeof(_port) -1);
	_port[sizeof(_port)-1] = '\0';
}

SagetechMXS::~SagetechMXS()
{
	// stop();
	perf_free(_loop_elapsed_perf);
	perf_free(_loop_count_perf);
	perf_free(_loop_interval_perf);
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

bool SagetechMXS::init()
{
	ScheduleOnInterval(UPDATE_INTERVAL_US);	// 50Hz
	return PX4_OK;
}

void SagetechMXS::Run()
{
	// Thread Stop
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	perf_begin(_loop_elapsed_perf);
	perf_count(_loop_interval_perf);

	// Count the number of times the loop has executed
	perf_count(_loop_count_perf);
	_loop_count = perf_event_count(_loop_count_perf);

	/*************************
	 * 20 Hz Timer
	 * ***********************/
	open_serial_port();

	// Parse Bytes
	int bytes_available {};
	int ret = ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
	(void) ret;
	// PX4_INFO("Bytes in buffer: %d. Ret code: %d", bytes_available, ret);
	if (bytes_available > 0) {
		while (bytes_available > 0) {
			uint16_t data;
			// tcflush(_fd, TCOFLUSH);
			ret = read(_fd, &data, 1);
			// PX4_INFO("GOT BYTE: %02x", (uint8_t)data);
			parse_byte((uint8_t)data);
			bytes_available -= 1;
		}
	}

	/******************
	 * 5 Hz Timer
	 * ****************/
	if (!(_loop_count % FIVE_HZ_MOD)) {		// 5Hz Timer (GPS Flight)
		// PX4_INFO("5 Hz callback");

		// Check if vehicle is landed or in air
		if (_vehicle_land_detected_sub.updated()) {
			_vehicle_land_detected_sub.copy(&_landed);
		}

		// Check GPS for updates at 5Hz
		if (_sensor_gps_sub.updated()) {
			_sensor_gps_sub.copy(&_gps);
		}

		// If Vehicle is in air send GPS messages at 5Hz
		if (!_landed.landed) {
			send_gps_msg();
		}

	}

	/***********************
	 * 1 Hz Timer
	 * *********************/

	if (!(_loop_count % ONE_HZ_MOD)) {		// 1Hz Timer (Operating Message/GPS Ground)
		// PX4_INFO("1 Hz callback");

		// If Vehicle is grounded send GPS message at 1 Hz
		if (_landed.landed) {
			send_gps_msg();
		}
		// Send 1Hz Operating Message
		send_operating_msg();

		// Update Parameters
		if (_parameter_update_sub.updated()) {
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			ModuleParams::updateParams();
		}
	}


	/************************
	 * 2 Hz Timer
	 * **********************/

	if(!(_loop_count % TWO_HZ_MOD)) {
		// PX4_INFO("2 Hz Callback");
	}

	/************************
	 * 8.2 Second Timer
	 * **********************/

	if (!(_loop_count % EIGHT_TWO_SEC_MOD)) {	// 8.2 second timer (Flight ID)
		// PX4_INFO("8.2 second callback");
		send_flight_id_msg();
	}

	perf_end(_loop_elapsed_perf);
}

int SagetechMXS::print_status()
{
	perf_print_counter(_loop_elapsed_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

void SagetechMXS::print_info()
{
	perf_print_counter(_loop_elapsed_perf);
	perf_print_counter(_loop_interval_perf);
}

/***************************
 * ADSB Vehicle List Functions
****************************/

bool SagetechMXS::get_vehicle_by_ICAO(const uint32_t icao, transponder_report_s &vehicle) const
{
	transponder_report_s temp_vehicle;
	temp_vehicle.icao_address = icao;

	uint16_t i;
	if (find_index(temp_vehicle, &i)) {
		memcpy(&vehicle, &vehicle_list[i], sizeof(transponder_report_s));
		return true;
	}
	return false;
}

bool SagetechMXS::find_index(const transponder_report_s &vehicle, uint16_t *index) const
{
	for (uint16_t i = 0; i < vehicle_count; i++) {
		if (vehicle_list[i].icao_address == vehicle.icao_address) {
			*index = i;
			return true;
		}
	}
	return false;
}

void SagetechMXS::set_vehicle(const uint16_t index, const transponder_report_s &vehicle)
{
	if (index >= MAX_VEHICLES_TRACKED) {
		return; // out of range
	}
	vehicle_list[index] = vehicle;
}

void SagetechMXS::determine_furthest_aircraft(void)
{
	float max_distance = 0;
	uint16_t max_distance_index = 0;

	for (uint16_t index = 0; index < vehicle_count; index++) {
		const float distance = get_distance_to_next_waypoint(_gps.lat, _gps.lon, vehicle_list[index].lat, vehicle_list[index].lon);
		if (max_distance < distance || index == 0) {
			max_distance = distance;
			max_distance_index = index;
		}
	}
	furthest_vehicle_index = max_distance_index;
	furthest_vehicle_distance = max_distance;
}

void SagetechMXS::delete_vehicle(const uint16_t index)
{
	if (index >= vehicle_count) {
		return;
	}

	if (index == furthest_vehicle_index && furthest_vehicle_distance > 0) {
		furthest_vehicle_distance = 0;
		furthest_vehicle_index = 0;
	}
	if (index != vehicle_count-1) {
		vehicle_list[index] = vehicle_list[vehicle_count-1];
	}
	vehicle_count--;
}

void SagetechMXS::handle_vehicle(const transponder_report_s &vehicle)
{
	// needs to handle updating the vehicle list, keeping track of which vehicles to drop
	// and which to keep, allocating new vehicles, and publishing to the transponder_report topic
	uint16_t index = MAX_VEHICLES_TRACKED + 1; // Make invalid to start with.
	const float my_loc_distance_to_vehicle = get_distance_to_next_waypoint(_gps.lat, _gps.lon, vehicle.lat, vehicle.lon);
	const bool is_tracked_in_list = find_index(vehicle, &index);
	const uint16_t required_flags_position = transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
		transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	if (!(vehicle.flags & required_flags_position)) {
		if (is_tracked_in_list) {
			delete_vehicle(index);	// If the vehicle is tracked in our list but doesn't have the right flags remove it
		}
	return;
	} else if (is_tracked_in_list) {	// If the vehicle is in the list update it with the index found
		set_vehicle(index, vehicle);
	} else if (vehicle_count < MAX_VEHICLES_TRACKED) {	// If the vehicle is not in the list, and the vehicle count is less than the max count
								// then add it to the vehicle_count index (after the last vehicle) and increment vehicle_count
		set_vehicle(vehicle_count, vehicle);
		vehicle_count++;
	} else {	// Buffer is full. If new vehicle is closer, replace furthest with new vehicle
		if (_gps.fix_type == 0) {	// Invalid GPS fix
			furthest_vehicle_distance = 0;
			furthest_vehicle_index = 0;
		} else {
			if (furthest_vehicle_distance <= 0) {
				determine_furthest_aircraft();
			}

			if (my_loc_distance_to_vehicle < furthest_vehicle_distance) {
				set_vehicle(furthest_vehicle_index, vehicle);
				furthest_vehicle_distance = 0;
				furthest_vehicle_index = 0;
			}
		}
	}

	const uint16_t required_flags_avoidance = required_flags_position | transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
		transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	if (vehicle.flags & required_flags_avoidance) {
		_transponder_report_pub.publish(vehicle);
	}
}

/*************************************
 * Handlers for Received Messages
 * ***********************************/

void SagetechMXS::handle_ack(const sg_ack_t ack)
{
	if ((ack.ackId != last.msg.id) || (ack.ackType != last.msg.type)) {
		// The message id doesn't match the last message sent.
	}
	// System health
	if (ack.failXpdr && !last.failXpdr) {
		// The transponder failed.
	}
	if (ack.failSystem && !last.failSystem) {
		// System Failure Indicator
	}
    last.failXpdr = ack.failXpdr;
    last.failSystem = ack.failSystem;
}

void SagetechMXS::handle_svr(sg_svr_t svr)
{
	if (svr.addrType != svrAdrIcaoUnknown && svr.addrType != svrAdrIcao && svr.addrType != svrAdrIcaoSurface) {
		return; // invalid icao
	}

	transponder_report_s t{};
	// Check if vehicle already exist in buffer
	if(!get_vehicle_by_ICAO(svr.addr, t)) {
		memset(&t, 0, sizeof(t));
		t.icao_address = svr.addr;
	}

	t.timestamp = hrt_absolute_time();
	t.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK;
	t.flags |= transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;

	//Set data from svr message
	if (svr.validity.position) {
		t.lat = svr.lat;
		t.lon = svr.lon;
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	}

	if (svr.validity.geoAlt || svr.validity.baroAlt) {
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
		if (svr.validity.geoAlt) {
			t.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
			t.altitude = (svr.airborne.geoAlt * SAGETECH_SCALE_FEET_TO_M); //Convert from Feet to Meters
		} else {
			t.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
			t.altitude = (svr.airborne.baroAlt * SAGETECH_SCALE_FEET_TO_M);	//Convert from Feet to Meters
		}
	}

	if (svr.validity.baroVRate || svr.validity.geoVRate) {
		t.ver_velocity = (svr.airborne.vrate * SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC); //Convert from feet/min to meters/second
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	}

	if (svr.type == svrSurface) {
		if (svr.validity.surfSpeed) {
			t.hor_velocity = svr.surface.speed * SAGETECH_SCALE_KNOTS_TO_M_PER_SEC;
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
		}
		if (svr.validity.surfHeading) {
			t.heading = matrix::wrap_pi((float)svr.surface.heading*(M_PI_F/180.0f)+M_PI_F);
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
		}
	}

	if (svr.type == svrAirborne) {
		if (svr.validity.airSpeed) {
			t.hor_velocity = (svr.airborne.speed * SAGETECH_SCALE_KNOTS_TO_M_PER_SEC);	//Convert from knots to meters/second
			t.heading = matrix::wrap_pi((float)svr.airborne.heading*(M_PI_F/180.0f)+M_PI_F);
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
		}
	}

	PX4_INFO("SVR ICAO %x, SVR Air Heading %f", (int) t.icao_address, (double) t.heading);
	handle_vehicle(t);
}

void SagetechMXS::handle_msr(sg_msr_t msr)
{

	transponder_report_s t{};
	if (!get_vehicle_by_ICAO(msr.addr, t)) {
		// new vehicle creation isn't allowed here since position isn't provided
		return;
	}

	t.timestamp = hrt_absolute_time();
	t.flags |= transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;

	if(strlen(msr.callsign)) {
		snprintf(t.callsign, sizeof(t.callsign), "%-8s", msr.callsign);
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;
	} else {
		t.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;
	}

	handle_vehicle(t);
}

/**************************************
 * Message Sending Functions
 **************************************/

int SagetechMXS::msg_write(const uint8_t *data, const uint16_t len) const
{
	int ret = 0;
	if (_fd >= 0) {
		tcflush(_fd, TCIFLUSH);
		ret = write(_fd, data, len);
		// PX4_INFO("WRITING DATA OF LEN %d OUT", len);
	} else {
		return PX4_ERROR;
	}
	if (ret != len) {
		perf_count(_comms_errors);
		PX4_INFO("write fail. Expected %d Got %d", len, ret);
		return PX4_ERROR;
	}
	return PX4_OK;
}

void SagetechMXS::send_data_req(const sg_datatype_t dataReqType)
{
	sg_datareq_t dataReq {};
	dataReq.reqType = dataReqType;
	last.msg.type = SG_MSG_TYPE_HOST_DATAREQ;
	uint8_t txComBuffer[SG_MSG_LEN_DATAREQ] {};
	sgEncodeDataReq(txComBuffer, &dataReq, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_DATAREQ);
}

void SagetechMXS::send_install_msg()
{
	// MXS must be in OFF mode to change ICAO or Registration
	if (mxs_state.op.opMode != modeOff) {
		// gcs().send_text(MAV_SEVERITY_WARNING, "ADSB Sagetech MXS: unable to send installation data while not in OFF mode.");
		return;
	}

	mxs_state.inst.icao = _adsb_icao.get();
	mxs_state.inst.emitter = convert_emitter_type_to_sg(_adsb_emit_type.get());
	mxs_state.inst.size = (sg_size_t)_adsb_len_width.get();
	mxs_state.inst.maxSpeed = (sg_airspeed_t)_adsb_max_speed.get();
	mxs_state.inst.antenna = sg_antenna_t::antBottom;

	last.msg.type = SG_MSG_TYPE_HOST_INSTALL;
	uint8_t txComBuffer[SG_MSG_LEN_INSTALL] {};
	sgEncodeInstall(txComBuffer, &mxs_state.inst, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_INSTALL);
}

void SagetechMXS::send_flight_id_msg()
{
#ifdef SG_HW_TEST
	snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightId), "%-8s", "MXSTEST0");
#endif

	last.msg.type = SG_MSG_TYPE_HOST_FLIGHT;
	uint8_t txComBuffer[SG_MSG_LEN_FLIGHT] {};
	sgEncodeFlightId(txComBuffer, &mxs_state.fid, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_FLIGHT);
}

void SagetechMXS::send_operating_msg()
{
#ifdef SG_HW_TEST
	// TODO: Parameterize Squawk
	mxs_state.op.squawk = convert_base_to_decimal(8, _adsb_squawk.get());
	mxs_state.op.opMode = (sg_op_mode_t)_mxs_op_mode.get();
	// sg_op_mode_t::modeStby;

	mxs_state.op.savePowerUp = true;                                                  // Save power-up state in non-volatile
	mxs_state.op.enableSqt = true;                                                    // Enable extended squitters
	mxs_state.op.enableXBit = false;                                                  // Enable the x-bit
	mxs_state.op.milEmergency = false;                                                // Broadcast a military emergency
	mxs_state.op.emergcType = sg_emergc_t::emergcNone;                                // Enumerated civilian emergency type

	mxs_state.op.altUseIntrnl = true;                                                 // True = Report altitude from internal pressure sensor (will ignore other bits in the field)
	mxs_state.op.altHostAvlbl = false;
	mxs_state.op.altRes25 = !mxs_state.inst.altRes100;                                // Host Altitude Resolution from install

	mxs_state.op.altitude = _gps.alt * SAGETECH_SCALE_MM_TO_FT;                       // Height above sealevel in feet

	mxs_state.op.identOn = false;

	if (_gps.vel_ned_valid) {
		mxs_state.op.climbValid = true;
		mxs_state.op.climbRate = _gps.vel_d_m_s * SAGETECH_SCALE_M_PER_SEC_TO_FT_PER_MIN;
		mxs_state.op.airspdValid = true;
		mxs_state.op.headingValid = true;
	} else {
		// PX4_WARN("send_operating_msg: Invalid NED");
		mxs_state.op.climbValid = false;
		mxs_state.op.climbRate = -CLIMB_RATE_LIMIT;
		mxs_state.op.airspdValid = false;
		mxs_state.op.headingValid = false;
	}

	const uint16_t speed_knots = _gps.vel_m_s * SAGETECH_SCALE_M_PER_SEC_TO_KNOTS;
	double heading = math::degrees(matrix::wrap_2pi(_gps.cog_rad));
	mxs_state.op.airspd = speed_knots;
	mxs_state.op.heading = heading;
#endif

	last.msg.type = SG_MSG_TYPE_HOST_OPMSG;
	uint8_t txComBuffer[SG_MSG_LEN_OPMSG] {};
	sgEncodeOperating(txComBuffer, &mxs_state.op, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_OPMSG);
}

// #define SG_PAYLOAD_LEN_GPS  SG_MSG_LEN_GPS - 5  /// the payload length.

// #define PBASE                    4   /// the payload offset.
// #define OFFSET_LONGITUDE         0   /// the longitude offset in the payload.
// #define OFFSET_LATITUDE         11   /// the latitude offset in the payload.
// #define OFFSET_SPEED            21   /// the ground speed offset in the payload.
// #define OFFSET_TRACK            27   /// the ground track offset in the payload.
// #define OFFSET_STATUS           35   /// the hemisphere/data status offset in the payload.
// #define OFFSET_TIME             36   /// the time of fix offset in the payload.
// #define OFFSET_HEIGHT           46   /// the GNSS height offset in the payload.
// #define OFFSET_HPL              50   /// the horizontal protection limit offset in the payload.
// #define OFFSET_HFOM             54   /// the horizontal figure of merit offset in the payload.
// #define OFFSET_VFOM             58   /// the vertical figure of merit offset in the payload.
// #define OFFSET_NACV             62   /// the navigation accuracy for velocity offset in the payload.

// #define LEN_LNG                 11   /// bytes in the longitude field
// #define LEN_LAT                 10   /// bytes in the latitude field
// #define LEN_SPD                  6   /// bytes in the speed over ground field
// #define LEN_TRK                  8   /// bytes in the ground track field
// #define LEN_TIME                10   /// bytes in the time of fix field

// static void checkGPSInputs(sg_gps_t *gps)
// {
// 	// Validate longitude
// 	for (int i = 0; i < LEN_LNG; ++i) {
// 		if (i == 5) {
// 			if (!(gps->longitude[i] == 0x2E)){
// 				PX4_ERR("A period is expected to separate minutes from fractions of minutes.");
// 			}
// 		} else {
// 			if (!(0x30 <= gps->longitude[i] && gps->longitude[i] <= 0x39)) {
// 				PX4_ERR("Longitude contains an invalid character");
// 			}
// 		}
// 	}

// 	// Validate latitude
// 	for (int i = 0; i < LEN_LAT; ++i) {
// 		if (i == 4) {
// 			if (!(gps->latitude[i] == 0x2E)) {
// 				PX4_ERR("A period is expected to separate minutes from fractions of minutes.");
// 			}
// 		} else {
// 			if(!(0x30 <= gps->latitude[i] && gps->latitude[i] <= 0x39)) {
// 				PX4_ERR("Latitude contains an invalid character");
// 			}
// 		}
// 	}

// 	// Validate speed over ground
// 	bool spdDecimal = false;
// 	(void) spdDecimal;
// 	for (int i = 0; i < LEN_SPD; ++i) {
// 		if (gps->grdSpeed[i] == 0x2E) {
// 			if (!(spdDecimal == false)) {
// 				PX4_ERR("Only one period should be used in speed over ground.");
// 			}
// 			spdDecimal = true;
// 		} else {
// 			if (!(0x30 <= gps->grdSpeed[i] && gps->grdSpeed[i] <= 0x39)) {
// 				PX4_ERR("Ground speed contains an invalid character");
// 			}
// 		}
// 	}

// 	if (!(spdDecimal == true)) {
// 		PX4_ERR("Use a period in ground speed to signify the start of fractional knots.");
// 	}

// 	// Validate ground track
// 	for (int i = 0; i < LEN_TRK; ++i) {
// 		if (i == 3) {
// 			if (!(gps->grdTrack[i] == 0x2E)) {
// 				PX4_ERR("A period is expected to signify the start of fractional degrees.");
// 			}
// 		} else {
// 			if(!(0x30 <= gps->grdTrack[i] && gps->grdTrack[i] <= 0x39)) {
// 				PX4_ERR("Ground track contains an invalid character");
// 			}
// 		}
// 	}

// 	// Validate time of fix
// 	bool tofSpaces = false;
// 	for (int i = 0; i < LEN_TIME; ++i) {
// 		if (i == 6) {
// 			if(!(gps->timeOfFix[i] == 0x2E)) {
// 				PX4_ERR("A period is expected to signify the start of fractional seconds.");
// 			}
// 		} else if (i == 0 && gps->timeOfFix[i] == 0x20) {
// 			tofSpaces = true;
// 		} else {
// 			if (tofSpaces) {
// 				if(!(gps->timeOfFix[i] == 0x20)) {
// 					PX4_ERR("All characters must be filled with spaces.");
// 				}
// 			} else {
// 				if(!(0x30 <= gps->timeOfFix[i] && gps->timeOfFix[i] <= 0x39)) {
// 					PX4_ERR("Time of Fix contains an invalid character");
// 				}
// 			}
// 		}
// 	}

// 	// Validate height
// 	if(!((float)-1200.0 <= gps->height && gps->height <= (float)160000.0)) {
// 		PX4_ERR("GPS height is not within the troposphere");
// 	}

// 	// Validate hpl
// 	if(!(0 <= gps->hpl)) {
// 		PX4_ERR("HPL cannot be negative");
// 	}

// 	// Validate hfom
// 	if(!(0 <= gps->hfom)) {
// 		PX4_ERR("HFOM cannot be negative");
// 	}

// 	// Validate vfom
// 	if(!(0 <= gps->vfom)) {
// 		PX4_ERR("VFOM cannot be negative");
// 	}

// 	// Validate status
// 	if(!(nacvUnknown <= gps->nacv && gps->nacv <= nacv0dot3)) {
// 		PX4_ERR("NACv is not an enumerated value");
// 	}
// }

void SagetechMXS::send_gps_msg()
{
	sg_gps_t gps {};

	gps.hpl = SAGETECH_HPL_UNKNOWN;                                                     // HPL over 37,040m means unknown
	gps.hfom = _gps.eph >= 0 ? _gps.eph : 0;
	gps.vfom = _gps.epv >= 0 ? _gps.epv : 0;
	gps.nacv = sg_nacv_t::nacvUnknown;
	if (_gps.s_variance_m_s >= (float)10.0 || _gps.s_variance_m_s < 0) {
		gps.nacv = sg_nacv_t::nacvUnknown;
	}
	else if (_gps.s_variance_m_s >= (float)3.0) {
		gps.nacv = sg_nacv_t::nacv10dot0;
	}
	else if (_gps.s_variance_m_s >= (float)1.0) {
		gps.nacv = sg_nacv_t::nacv3dot0;
	}
	else if (_gps.s_variance_m_s >= (float)0.3) {
		gps.nacv = sg_nacv_t::nacv1dot0;
	}
	else { //if (_gps.s_variance_m_s >= 0.0)
		gps.nacv = sg_nacv_t::nacv0dot3;
	}

	// Get Vehicle Longitude and Latitude and Convert to string
	const int32_t longitude = _gps.lon;
	const int32_t latitude =  _gps.lat;
	const double lon_deg = longitude * 1.0E-7 * (longitude < 0 ? -1 : 1);
	const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
	snprintf((char*)&gps.longitude, 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes, unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

	const double lat_deg = latitude *  1.0E-7 * (latitude < 0 ? -1 : 1);
	const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
	snprintf((char*)&gps.latitude, 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes, unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

	const float speed_knots = _gps.vel_m_s * SAGETECH_SCALE_M_PER_SEC_TO_KNOTS;
	snprintf((char*)&gps.grdSpeed, 7, "%03u.%02u", (unsigned)speed_knots, unsigned((speed_knots - (int)speed_knots) * (float)1.0E2));

	const float heading = math::degrees(matrix::wrap_2pi(_gps.cog_rad));

	snprintf((char*)&gps.grdTrack, 9, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * (float)1.0E4));

	gps.latNorth = (latitude >= 0 ? true: false);
	gps.lngEast = (longitude >= 0 ? true: false);

	gps.gpsValid = (_gps.fix_type < 2) ? false : true;  // If the status is not OK, gpsValid is false.

	const time_t time_sec = _gps.time_utc_usec * 1E-6;
	struct tm* tm = gmtime(&time_sec);
	snprintf((char*)&gps.timeOfFix, 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (_gps.time_utc_usec % 1000000) * 1.0e-6);
	PX4_INFO("send_gps_msg: ToF %s, Longitude %s, Latitude %s, Grd Speed %s, Grd Track %s", gps.timeOfFix, gps.longitude, gps.latitude, gps.grdSpeed, gps.grdTrack);

	gps.height = _gps.alt_ellipsoid * 1E-3;

	// checkGPSInputs(&gps);
	last.msg.type = SG_MSG_TYPE_HOST_GPS;
	uint8_t txComBuffer[SG_MSG_LEN_GPS] {};
	sgEncodeGPS(txComBuffer, &gps, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_GPS);
}



void SagetechMXS::send_targetreq_msg()
{
	mxs_state.treq.reqType = sg_reporttype_t::reportAuto;
	mxs_state.treq.transmitPort = sg_transmitport_t::transmitCom1;
	mxs_state.treq.maxTargets = MAX_VEHICLES_TRACKED;
	// TODO: have way to track special target. will need to change up tracked list to do so.
	// mxs_state.treq.icao = _frontend._special_ICAO_target.get();
	mxs_state.treq.stateVector = true;
	mxs_state.treq.modeStatus = true;
	mxs_state.treq.targetState = false;
	mxs_state.treq.airRefVel = false;
	mxs_state.treq.tisb = false;
	mxs_state.treq.military = false;
	mxs_state.treq.commA = false;
	mxs_state.treq.ownship = true;

	last.msg.type = SG_MSG_TYPE_HOST_TARGETREQ;
	uint8_t txComBuffer[SG_MSG_LEN_TARGETREQ] {};
	sgEncodeTargetReq(txComBuffer, &mxs_state.treq, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_TARGETREQ);
}

/***********************************************
 * Byte Parsing and Packet Handling
 * *********************************************/

void SagetechMXS::handle_packet(const Packet &msg)
{
	switch(msg.type) {
		case MsgType::ACK:
			// PX4_INFO("GOT ACK PACKET");
			if(sgDecodeAck((uint8_t*) &msg, &mxs_state.ack)) {
				handle_ack(mxs_state.ack);
			}
			break;
		case MsgType::Installation_Response:
			// TODO: set up installation data here
		case MsgType::FlightID_Response: {
			// PX4_INFO("GOT FID RESP PACKET");
			sg_flightid_t fid{};
			if (sgDecodeFlightId((uint8_t*) &msg, &fid)) {
				// TODO: Do something with this?
			}
			break;
		}
		case MsgType::ADSB_StateVector_Report: {
			PX4_INFO("GOT SVR PACKET");
			sg_svr_t svr{};
			if (sgDecodeSVR((uint8_t*) &msg, &svr)) {
				handle_svr(svr);
			}
			break;
		}
		case MsgType::ADSB_ModeStatus_Report: {
			// PX4_INFO("GOT MSR PACKET");
			sg_msr_t msr{};
			if (sgDecodeMSR((uint8_t*) &msg, &msr)) {
				handle_msr(msr);
			}
			break;
		}
		case MsgType::FlightID:
			// PX4_INFO("GOT FID PACKET");
			break;
		case MsgType::Installation:
		case MsgType::Operating:
		case MsgType::GPS_Data:
		case MsgType::Data_Request:
		case MsgType::Target_Request:
		case MsgType::Mode:
		case MsgType::Status_Response:
		case MsgType::RESERVED_0x84:
		case MsgType::RESERVED_0x85:
		case MsgType::Mode_Settings:
		case MsgType::RESERVED_0x8D:
		case MsgType::Version_Response:
		case MsgType::Serial_Number_Response:
		case MsgType::Target_Summary_Report:
		case MsgType::ADSB_Target_State_Report:
		case MsgType::ADSB_Air_Ref_Vel_Report:
			// Not handling the rest of these.
			PX4_INFO("GOT DIFFERENT PACKET");
			break;
	}
}

bool SagetechMXS::parse_byte(const uint8_t data)
{
	switch (_message_in.state) {
		default:
		case ParseState::WaitingFor_Start:
			if (data == SG_MSG_START_BYTE) {
				_message_in.checksum = data; // initialize checksum here
				_message_in.state = ParseState::WaitingFor_MsgType;
			}
			break;
		case ParseState::WaitingFor_MsgType:
			_message_in.checksum += data;
			_message_in.packet.type = static_cast<MsgType>(data);
			_message_in.state = ParseState::WaitingFor_MsgId;
			break;
		case ParseState::WaitingFor_MsgId:
			_message_in.checksum += data;
			_message_in.packet.id = data;
			_message_in.state = ParseState::WaitingFor_PayloadLen;
			break;
		case ParseState::WaitingFor_PayloadLen:
			_message_in.checksum += data;
			_message_in.packet.payload_length = data;
			// PX4_INFO("Packet Payload Length: %d", _message_in.packet.payload_length);
			// maybe useful to add a length check here. Very few errors are due to length though...
			_message_in.index = 0;
			_message_in.state = (data == 0) ? ParseState::WaitingFor_Checksum : ParseState::WaitingFor_PayloadContents;
			break;
		case ParseState::WaitingFor_PayloadContents:
			_message_in.checksum += data;
			_message_in.packet.payload[_message_in.index++] = data;
			if (_message_in.index >= _message_in.packet.payload_length) {	// Note: yeah, this is right since it will increment index after writing the last byte and it will equal the payload length
				_message_in.state = ParseState::WaitingFor_Checksum;
			}
			break;
		case ParseState::WaitingFor_Checksum:
			// PX4_INFO("Payload Bytes Got: %d", _message_in.index);
			_message_in.state = ParseState::WaitingFor_Start;
			if (_message_in.checksum == data) {
				// append the checksum to the payload and zero out the payload index
				_message_in.packet.payload[_message_in.index] = data;
				_message_in.index = 0;
				// TODO: handle_packet
				handle_packet(_message_in.packet);
			} else if (data == SG_MSG_START_BYTE) {
				PX4_INFO("ERROR: Byte dropped. Tossing Packet.");
				_message_in.state = ParseState::WaitingFor_MsgType;
				_message_in.checksum = data;
			} else {
				PX4_INFO("ERROR: Checksum Mismatch. Expected %02x. Received %02x.", _message_in.checksum, data);
			}
			break;
	}
	return false;
}

/******************************
 * Helper Functions
 * ***************************/

uint32_t SagetechMXS::convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber)
{
    // Our only sensible input bases are 16 and 8
    if (baseIn != 8 && baseIn != 16) {
	return inputNumber;
    }
    uint32_t outputNumber = 0;
    for (uint8_t i=0; i < 10; i++) {
	outputNumber += (inputNumber % 10) * powf(baseIn, i);
	inputNumber /= 10;
	if (inputNumber == 0) break;
    }
    return outputNumber;
}

int SagetechMXS::open_serial_port() {

	if (_fd < 0) {	// Open port if not open
		_fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_fd < 0) {
			PX4_ERR("Opening port %s failed %i", _port, errno);
			return PX4_ERROR;
		}
	} else {
		return PX4_OK;
	}

	// UART Configuration
	struct termios uart_config {};
	int termios_state = -1;

	if(tcgetattr(_fd, &uart_config)) {
		PX4_ERR("Unable to get UART Configuration");
		close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	uart_config.c_oflag &= ~ONLCR;
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	// uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
	// uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);
	// uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);
	// uart_config.c_oflag &= ~ONLCR;
	// uart_config.c_cflag &= ~(CSTOPB | PARENB);

	unsigned baud = B57600;
	if ((cfsetispeed(&uart_config, baud) < 0) || (cfsetospeed(&uart_config, baud) < 0)) {
		PX4_ERR("ERR SET BAUD %s: %d\n", _port, termios_state);
		px4_close(_fd);
		return PX4_ERROR;
	}

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		return PX4_ERROR;
	}

	tcflush(_fd, TCIOFLUSH);
	PX4_INFO("Opened port %s", _port);
	return PX4_OK;
}

sg_emitter_t SagetechMXS::convert_emitter_type_to_sg (int emitType) {
	switch (emitType) {
		case 0: return sg_emitter_t::aUnknown;
		case 1: return sg_emitter_t::aLight;
		case 2: return sg_emitter_t::aSmall;
		case 3: return sg_emitter_t::aLarge;
		case 4: return sg_emitter_t::aHighVortex;
		case 5: return sg_emitter_t::aHeavy;
		case 6: return sg_emitter_t::aPerformance;
		case 7: return sg_emitter_t::aRotorCraft;
		case 8: return sg_emitter_t::bUnknown;
		case 9: return sg_emitter_t::bGlider;
		case 10: return sg_emitter_t::bAir;
		case 11: return sg_emitter_t::bParachutist;
		case 12: return sg_emitter_t::bUltralight;
		case 13: return sg_emitter_t::bUnknown;
		case 14: return sg_emitter_t::bUAV;
		case 15: return sg_emitter_t::bSpace;
		case 16: return sg_emitter_t::cUnknown;
		case 17: return sg_emitter_t::cEmergency;
		case 18: return sg_emitter_t::cService;
		case 19: return sg_emitter_t::cPoint;
		default: return sg_emitter_t::dUnknown;
	}
}

/*****************************************
 * Parameter and Custom Command Handling
 * ***************************************/

void SagetechMXS::handle_params()
{
	parameter_update_s param_update;
	_parameter_update_sub.copy(&param_update);
	ModuleParams::updateParams();
}


// void SagetechMXS::handle_fid(const char* fid)
// {
// 	snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightID), "%-8s", fid);
// }

// int SagetechMXS::custom_command(int argc, char *argv[])
// {
// 	const char *verb = argv[0];

// 	if (!strcmp(verb, "flight_id")) {
// 		const char *fid = argv[1];
// 		if (strlen(fid)) {
// 			strncpy(mxs_state.fid.flightId, fid, sizeof(mxs_state.fid.flightId));
// 		}
// 		return 0;
// 	}

// 	if (!is_running()) {
// 		int ret = RCInput::task_spawn(argc, argv);

// 		if (ret) {
// 			return ret;
// 		}
// 	}
// }
