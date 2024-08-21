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

#include "SagetechMXS.hpp"

/***************************************
 * Workqueue Functions
 * *************************************/

extern "C" __EXPORT int sagetech_mxs_main(int argc, char *argv[])
{
	return SagetechMXS::main(argc, argv);
}

SagetechMXS::SagetechMXS(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';
}

SagetechMXS::~SagetechMXS()
{
	if (!(_fd < 0)) {
		close(_fd);
	}

	perf_free(_loop_elapsed_perf);
	perf_free(_loop_count_perf);
	perf_free(_loop_interval_perf);
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SagetechMXS::task_spawn(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int ch = '\0';
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			return PX4_ERROR;
			break;
		}
	}

	SagetechMXS *instance = new SagetechMXS(device_path);

	if (instance == nullptr) {
		PX4_ERR("Allocation Failed.");
	}

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}



bool SagetechMXS::init()
{
	ScheduleOnInterval(UPDATE_INTERVAL_US);	// 50Hz
	mxs_state.initialized = false;
	mxs_state.init_failed = false;

	if (vehicle_list == nullptr) {
		if (_adsb_list_max.get() > MAX_VEHICLES_LIMIT) {	// Safety Check
			_adsb_list_max.set(MAX_VEHICLES_LIMIT);
			_adsb_list_max.commit();
			list_size_allocated = MAX_VEHICLES_LIMIT;
		}

		vehicle_list = new transponder_report_s[_adsb_list_max.get()];

		if (vehicle_list == nullptr) {
			mxs_state.init_failed = true;
			PX4_ERR("Unable to initialize vehicle list.");
			return false;
		}

		list_size_allocated = _adsb_list_max.get();
	}

	return true;
}

int SagetechMXS::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!is_running()) {
		int ret = SagetechMXS::task_spawn(argc, argv);
		return ret;


	}

	if (!strcmp(verb, "flightid")) {
		const char *fid = argv[1];

		if (fid == nullptr) {
			print_usage("Missing Flight ID");
			return PX4_ERROR;
		}

		return get_instance()->handle_fid(fid);
	}

	if (!strcmp(verb, "ident")) {
		get_instance()->_adsb_ident.set(1);
		return get_instance()->_adsb_ident.commit();
	}

	if (!strcmp(verb, "opmode")) {
		const char *opmode = argv[1];

		if (opmode == nullptr) {
			print_usage("Missing Op Mode");
			return PX4_ERROR;

		} else if (!strcmp(opmode, "off") || !strcmp(opmode, "0")) {
			get_instance()->_mxs_op_mode.set(0);
			return get_instance()->_mxs_op_mode.commit();

		} else if (!strcmp(opmode, "on") || !strcmp(opmode, "1")) {
			get_instance()->_mxs_op_mode.set(1);
			return get_instance()->_mxs_op_mode.commit();

		} else if (!strcmp(opmode, "stby") || !strcmp(opmode, "2")) {
			get_instance()->_mxs_op_mode.set(2);
			return get_instance()->_mxs_op_mode.commit();

		} else if (!strcmp(opmode, "alt") || !strcmp(opmode, "3")) {
			get_instance()->_mxs_op_mode.set(3);
			return get_instance()->_mxs_op_mode.commit();

		} else {
			print_usage("Invalid Op Mode");
			return PX4_ERROR;
		}
	}

	if (!strcmp(verb, "squawk")) {
		const char *squawk = argv[1];
		int sqk = 0;

		if (squawk == nullptr) {
			print_usage("Missing Squawk Code");
			return PX4_ERROR;
		}

		sqk = atoi(squawk);

		if (!get_instance()->check_valid_squawk(sqk)) {
			print_usage("Invalid Squawk");
			return PX4_ERROR;

		} else {
			get_instance()->_adsb_squawk.set(sqk);
			return get_instance()->_adsb_squawk.commit();
		}
	}


	print_usage("Unknown Command");
	return PX4_ERROR;
}

int SagetechMXS::print_usage(const char *reason)
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description

	This driver integrates the Sagetech MXS Certified Transponder to send and receive ADSB messages and traffic.

	### Examples

	Attempt to start driver on a specified serial device.
	$ sagetech_mxs start -d /dev/ttyS1
	Stop driver
	$ sagetech_mxs stop
	Set Flight ID (8 char max)
	$ sagetech_mxs flight_id MXS12345
	Set MXS Operating Mode
	$ sagetech_mxs opmode off/on/stby/alt
	$ sagetech_mxs opmode 0/1/2/3
	Set the Squawk Code
	$ sagetech_mxs squawk 1200
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sagetech_mxs", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("transponder");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("flightid", "Set Flight ID (8 char max)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ident", "Set the IDENT bit in ADSB-Out messages");
	PRINT_MODULE_USAGE_COMMAND_DESCR("opmode",
					 "Set the MXS operating mode. ('off', 'on', 'stby', 'alt', or numerical [0-3])");
	PRINT_MODULE_USAGE_COMMAND_DESCR("squawk", "Set the Squawk Code. [0-7777] Octal (no digit larger than 7)");
	return PX4_OK;
}

int SagetechMXS::print_status()
{
	perf_print_counter(_loop_count_perf);
	perf_print_counter(_loop_elapsed_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	return 0;
}




/******************************
 * Main Run Thread
 * ****************************/

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
	 * 50 Hz Timer
	 * ***********************/
	open_serial_port();

	// Initialization
	if (!mxs_state.initialized) {
		if ((_loop_count > MXS_INIT_TIMEOUT_COUNT) && !mxs_state.init_failed) {	// Initialization timeout
			PX4_ERR("Timeout: Failed to Initialize.");
			mxs_state.init_failed = true;
		}

		if (!_mxs_ext_cfg.get()) {
			// Auto configuration
			auto_config_operating();
			auto_config_installation();
			auto_config_flightid();
			_mxs_op_mode.set(sg_op_mode_t::modeStby);
			_mxs_op_mode.commit();
			send_targetreq_msg();
			mxs_state.initialized = true;
		}
	}

	// Parse Bytes
	int bytes_available {};
	int ret = -1;
	ret = ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
	(void) ret;

	// PX4_INFO("Bytes in buffer: %d. Ret code: %d", bytes_available, ret);
	if (bytes_available > 0) {
		while (bytes_available > 0) {
			uint16_t data = 0;
			// tcflush(_fd, TCOFLUSH);
			ret = read(_fd, &data, 1);

			if (ret < 0) {
				// PX4_ERR("Read Err.");
				perf_count(_comms_errors);
				continue;
			}

			// PX4_INFO("GOT BYTE: %02x", (uint8_t)data);
			bytes_available -= ret;
			parse_byte((uint8_t)data);
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

		if (!mxs_state.initialized && _mxs_ext_cfg.get()) {
			// External Configuration
			send_data_req(dataInstall);
		}

		// If Vehicle is grounded send GPS message at 1 Hz
		if (_landed.landed) {
			send_gps_msg();
		}

		// Send 1Hz Operating Message
		send_operating_msg();

		// Update Parameters
		if (_parameter_update_sub.updated()) {
			parameter_update_s param_update{};
			_parameter_update_sub.copy(&param_update);
			ModuleParams::updateParams();

			if (mxs_state.treq.transmitPort != (sg_transmitport_t)_mxs_targ_port.get()) {
				send_targetreq_msg();
			}
		}
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


/***************************
 * ADSB Vehicle List Functions
****************************/

bool SagetechMXS::get_vehicle_by_ICAO(const uint32_t icao, transponder_report_s &vehicle) const
{
	transponder_report_s temp_vehicle{};
	temp_vehicle.icao_address = icao;

	uint16_t i = 0;

	if (find_index(temp_vehicle, &i)) {
		memcpy(&vehicle, &vehicle_list[i], sizeof(transponder_report_s));
		return true;

	} else {
		return false;
	}
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
	if (index >= list_size_allocated) {
		return; // out of range
	}

	vehicle_list[index] = vehicle;
}

void SagetechMXS::determine_furthest_aircraft()
{
	float max_distance = 0;
	uint16_t max_distance_index = 0;

	for (uint16_t index = 0; index < vehicle_count; index++) {
		if (is_special_vehicle(vehicle_list[index].icao_address)) {
			continue;
		}

		const float distance = get_distance_to_next_waypoint(_gps.latitude_deg, _gps.longitude_deg,
				       vehicle_list[index].lat,
				       vehicle_list[index].lon);

		if ((max_distance < distance) || (index == 0)) {
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

	if (index != vehicle_count - 1) {
		vehicle_list[index] = vehicle_list[vehicle_count - 1];
	}

	vehicle_count--;
}

void SagetechMXS::handle_vehicle(const transponder_report_s &vehicle)
{
	// needs to handle updating the vehicle list, keeping track of which vehicles to drop
	// and which to keep, allocating new vehicles, and publishing to the transponder_report topic
	uint16_t index = list_size_allocated + 1; // Make invalid to start with.
	const bool my_loc_is_zero = (fabs(_gps.latitude_deg) < DBL_EPSILON) && (fabs(_gps.longitude_deg) < DBL_EPSILON);
	const float my_loc_distance_to_vehicle = get_distance_to_next_waypoint(_gps.latitude_deg, _gps.longitude_deg,
			vehicle.lat, vehicle.lon);
	const bool is_tracked_in_list = find_index(vehicle, &index);
	// const bool is_special = is_special_vehicle(vehicle.icao_address);
	const uint16_t required_flags_position = transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;

	if (!(vehicle.flags & required_flags_position)) {
		if (is_tracked_in_list) {
			delete_vehicle(index);	// If the vehicle is tracked in our list but doesn't have the right flags remove it
		}

		return;

	} else if (is_tracked_in_list) {	// If the vehicle is in the list update it with the index found
		set_vehicle(index, vehicle);

	} else if (vehicle_count <
		   list_size_allocated) {	// If the vehicle is not in the list, and the vehicle count is less than the max count
		// then add it to the vehicle_count index (after the last vehicle) and increment vehicle_count
		set_vehicle(vehicle_count, vehicle);
		vehicle_count++;

	} else {	// Buffer is full. If new vehicle is closer, replace furthest with new vehicle
		if (my_loc_is_zero) {	// Invalid GPS
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
	// if ((ack.ackId != last.msg.id) || (ack.ackType != last.msg.type)) {
	// 	// The message id doesn't match the last message sent.
	// 	PX4_ERR("Message Id %d of type %d not Acked by MXS", last.msg.id, last.msg.type);
	// }

	// System health
	if (ack.failXpdr && !last.failXpdr) {
		mavlink_log_info(&_mavlink_log_pub, "Transponder Failure\t");
		events::send(events::ID("mxs_xpdr_fail"), events::Log::Critical, "Transponder Failure");
	}

	if (ack.failSystem && !last.failSystem) {
		mavlink_log_info(&_mavlink_log_pub, "Transponder System Failure\t");
		events::send(events::ID("mxs_system_fail"), events::Log::Critical, "Transponder System Failure");
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
	if (!get_vehicle_by_ICAO(svr.addr, t)) {
		memset(&t, 0, sizeof(t));
		t.icao_address = svr.addr;
	}

	t.timestamp = hrt_absolute_time();
	t.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK;
	t.flags |= transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;

	//Set data from svr message
	if (svr.validity.position) {
		t.lat = (double) svr.lat;
		t.lon = (double) svr.lon;
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	}

	if (svr.validity.geoAlt) {
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
		t.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
		t.altitude = (svr.airborne.geoAlt * SAGETECH_SCALE_FEET_TO_M); //Convert from Feet to Meters

	} else if (svr.validity.baroAlt) {
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
		t.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
		t.altitude = (svr.airborne.baroAlt * SAGETECH_SCALE_FEET_TO_M);	//Convert from Feet to Meters
	}

	if (svr.validity.baroVRate || svr.validity.geoVRate) {
		t.ver_velocity = svr.airborne.vrate * SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC; //Convert from feet/min to meters/second
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	}

	if (svr.type == svrSurface) {
		if (svr.validity.surfSpeed) {
			t.hor_velocity = svr.surface.speed * SAGETECH_SCALE_KNOTS_TO_M_PER_SEC;
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
		}

		if (svr.validity.surfHeading) {
			t.heading = matrix::wrap_pi((float)svr.surface.heading * (M_PI_F / 180.0f) + M_PI_F);
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
		}
	}

	if (svr.type == svrAirborne) {
		if (svr.validity.airSpeed) {
			t.hor_velocity = (svr.airborne.speed * SAGETECH_SCALE_KNOTS_TO_M_PER_SEC);	//Convert from knots to meters/second
			t.heading = matrix::wrap_pi((float)svr.airborne.heading * (M_PI_F / 180.0f) + M_PI_F);
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
		}
	}

	// PX4_INFO("SVR ICAO %x, SVR Air Heading %f", (int) t.icao_address, (double) t.heading);
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

	if (strlen(msr.callsign)) {
		snprintf(t.callsign, sizeof(t.callsign), "%-8s", msr.callsign);
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

	} else {
		t.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;
	}

	// PX4_INFO("Got MSR for ICAO: %x with callsign: %s", (int) t.icao_address, t.callsign);

	handle_vehicle(t);
}

/**************************************
 * Message Sending Functions
 **************************************/

int SagetechMXS::msg_write(const uint8_t *data, const uint16_t len) const
{
	int ret = 0;

	if (_fd >= 0) {
		ret = write(_fd, data, len);

	} else {
		return PX4_ERROR;
	}

	if (ret != len) {
		perf_count(_comms_errors);
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
	last.msg.type = SG_MSG_TYPE_HOST_FLIGHT;
	uint8_t txComBuffer[SG_MSG_LEN_FLIGHT] {};
	sgEncodeFlightId(txComBuffer, &mxs_state.fid, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_FLIGHT);
}

void SagetechMXS::send_operating_msg()
{

	mxs_state.op.opMode = (sg_op_mode_t)_mxs_op_mode.get();

	if (check_valid_squawk(_adsb_squawk.get())) {
		mxs_state.op.squawk = convert_base_to_decimal(BASE_OCTAL, _adsb_squawk.get());

	} else {
		mxs_state.op.squawk = convert_base_to_decimal(BASE_OCTAL, INVALID_SQUAWK);
	}

	mxs_state.op.savePowerUp = true;                                                  // Save power-up state in non-volatile
	mxs_state.op.enableSqt = true;                                                    // Enable extended squitters
	mxs_state.op.enableXBit = false;                                                  // Enable the x-bit
	mxs_state.op.milEmergency = false;                                                // Broadcast a military emergency
	mxs_state.op.emergcType = sg_emergc_t::emergcNone;                                // Enumerated civilian emergency type

	mxs_state.op.altUseIntrnl =
		true;                                                 // True = Report altitude from internal pressure sensor (will ignore other bits in the field)
	mxs_state.op.altHostAvlbl = false;
	mxs_state.op.altRes25 =
		!mxs_state.inst.altRes100;                                // Host Altitude Resolution from install

	mxs_state.op.altitude = static_cast<int32_t>(_gps.altitude_msl_m *
				SAGETECH_SCALE_M_TO_FT);   // Height above sealevel in feet

	mxs_state.op.identOn = _adsb_ident.get();

	if (mxs_state.op.identOn) {
		_adsb_ident.set(0);
		_adsb_ident.commit();
	}

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
	double heading = (double) math::degrees(matrix::wrap_2pi(_gps.cog_rad));
	mxs_state.op.airspd = speed_knots;
	mxs_state.op.heading = heading;

	last.msg.type = SG_MSG_TYPE_HOST_OPMSG;
	uint8_t txComBuffer[SG_MSG_LEN_OPMSG] {};
	sgEncodeOperating(txComBuffer, &mxs_state.op, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_OPMSG);
}


void SagetechMXS::send_gps_msg()
{
	sg_gps_t gps {};

	gps.hpl = SAGETECH_HPL_UNKNOWN;                                                     // HPL over 37,040m means unknown
	gps.hfom = _gps.eph >= 0 ? _gps.eph : 0;
	gps.vfom = _gps.epv >= 0 ? _gps.epv : 0;
	gps.nacv = sg_nacv_t::nacvUnknown;

	if (_gps.s_variance_m_s >= (float)10.0 || _gps.s_variance_m_s < 0) {
		gps.nacv = sg_nacv_t::nacvUnknown;

	} else if (_gps.s_variance_m_s >= (float)3.0) {
		gps.nacv = sg_nacv_t::nacv10dot0;

	} else if (_gps.s_variance_m_s >= (float)1.0) {
		gps.nacv = sg_nacv_t::nacv3dot0;

	} else if (_gps.s_variance_m_s >= (float)0.3) {
		gps.nacv = sg_nacv_t::nacv1dot0;

	} else { //if (_gps.s_variance_m_s >= 0.0)
		gps.nacv = sg_nacv_t::nacv0dot3;
	}

	// Get Vehicle Longitude and Latitude and Convert to string
	const int32_t longitude = static_cast<int32_t>(_gps.longitude_deg * 1e7);
	const int32_t latitude =  static_cast<int32_t>(_gps.latitude_deg * 1e7);
	const double lon_deg = longitude * 1.0E-7 * (longitude < 0 ? -1 : 1);
	const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
	snprintf((char *)&gps.longitude, 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes,
		 unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

	const double lat_deg = latitude *  1.0E-7 * (latitude < 0 ? -1 : 1);
	const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
	snprintf((char *)&gps.latitude, 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes,
		 unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

	const float speed_knots = _gps.vel_m_s * SAGETECH_SCALE_M_PER_SEC_TO_KNOTS;
	snprintf((char *)&gps.grdSpeed, 7, "%03u.%02u", (unsigned)speed_knots,
		 unsigned((speed_knots - (int)speed_knots) * (float)1.0E2));

	const float heading = matrix::wrap_2pi(_gps.cog_rad) * (180.0f / M_PI_F);

	snprintf((char *)&gps.grdTrack, 9, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * (float)1.0E4));

	gps.latNorth = latitude >= 0;
	gps.lngEast = longitude >= 0;

	gps.gpsValid = !(_gps.fix_type < 2);  // If the status is not OK, gpsValid is false.

	const time_t time_sec = _gps.time_utc_usec * 1E-6;
	struct tm *tm = gmtime(&time_sec);
	snprintf((char *)&gps.timeOfFix, 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min,
		 tm->tm_sec + (_gps.time_utc_usec % 1000000) * 1.0e-6);

	gps.height = (float)_gps.altitude_ellipsoid_m;

	// checkGPSInputs(&gps);
	last.msg.type = SG_MSG_TYPE_HOST_GPS;
	uint8_t txComBuffer[SG_MSG_LEN_GPS] {};
	sgEncodeGPS(txComBuffer, &gps, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_GPS);
}



void SagetechMXS::send_targetreq_msg()
{
	mxs_state.treq.reqType = sg_reporttype_t::reportAuto;
	mxs_state.treq.transmitPort = (sg_transmitport_t)_mxs_targ_port.get();
	mxs_state.treq.maxTargets = list_size_allocated;
	mxs_state.treq.icao = _adsb_icao_specl.get();
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
	switch (msg.type) {
	case MsgType::ACK:
		if (sgDecodeAck((uint8_t *) &msg, &mxs_state.ack)) {
			handle_ack(mxs_state.ack);
		}

		break;

	case MsgType::Installation_Response:
		if (!mxs_state.initialized && sgDecodeInstall((uint8_t *)&msg, &mxs_state.inst)) {
			store_inst_resp();
			mxs_state.initialized = true;
		}

		break;

	case MsgType::FlightID_Response: {
			sg_flightid_t fid{};

			if (sgDecodeFlightId((uint8_t *) &msg, &fid)) {
			}

			break;
		}

	case MsgType::ADSB_StateVector_Report: {
			sg_svr_t svr{};

			if (sgDecodeSVR((uint8_t *) &msg, &svr)) {
				handle_svr(svr);
			}

			break;
		}

	case MsgType::ADSB_ModeStatus_Report: {
			sg_msr_t msr{};

			if (sgDecodeMSR((uint8_t *) &msg, &msr)) {
				handle_msr(msr);
			}

			break;
		}

	case MsgType::FlightID:
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

		if (_message_in.index >=
		    _message_in.packet.payload_length) {	// Note: yeah, this is right since it will increment index after writing the last byte and it will equal the payload length
			_message_in.state = ParseState::WaitingFor_Checksum;
		}

		break;

	case ParseState::WaitingFor_Checksum:
		// PX4_INFO("Payload Bytes Got: %d", _message_in.index);
		_message_in.state = ParseState::WaitingFor_Start;

		if (_message_in.checksum == data) {
			// append the checksum to the payload and zero out the payload index
			_message_in.packet.payload[_message_in.index] = data;
			handle_packet(_message_in.packet);

		} else if (data == SG_MSG_START_BYTE) {
			// PX4_INFO("ERROR: Byte Lost. Catching new packet.");
			perf_count(_comms_errors);
			_message_in.state = ParseState::WaitingFor_MsgType;
			_message_in.checksum = data;

		} else {
			// PX4_INFO("ERROR: Checksum Mismatch. Expected %02x. Received %02x.", _message_in.checksum, data);
			perf_count(_comms_errors);
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
	if (baseIn != BASE_OCTAL && baseIn != BASE_HEX) {
		return inputNumber;
	}

	uint32_t outputNumber = 0;

	for (uint8_t i = 0; i < BASE_DEC; i++) {
		outputNumber += (inputNumber % BASE_DEC) * powf(baseIn, i);
		inputNumber /= BASE_DEC;

		if (inputNumber == 0) { break; }
	}

	return outputNumber;
}

int SagetechMXS::open_serial_port()
{

	if (_fd < 0) {	// Open port if not open
		_fd = open(_port, (O_RDWR | O_NOCTTY | O_NONBLOCK));

		if (_fd < 0) {
			PX4_ERR("Opening port %s failed %i", _port, errno);
			return PX4_ERROR;
		}

	} else {
		return PX4_OK;
	}

	// UART Configuration
	termios uart_config {};
	int termios_state = -1;

	if (tcgetattr(_fd, &uart_config)) {
		PX4_ERR("Unable to get UART Configuration");
		close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	/*****************************
	 * UART Control Options
	 * ***************************/

	// Enable Receiver and Set Local Mode
	uart_config.c_cflag |= (CREAD | CLOCAL);

	// Send 8N1 No Parity
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
	uart_config.c_cflag |= CS8;

	// Disable hardware flow control
	uart_config.c_cflag &= ~CRTSCTS;

	// Set Baud Rate
	unsigned baud = convert_to_px4_baud(_ser_mxs_baud.get());

	if ((cfsetispeed(&uart_config, baud) < 0) || (cfsetospeed(&uart_config, baud) < 0)) {
		PX4_ERR("ERR SET BAUD %s: %d\n", _port, termios_state);
		close(_fd);
		return PX4_ERROR;
	}

	/*****************************
	 * UART Local Options
	 * ***************************/

	// Set Raw Input
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN);

	/*****************************
	 * UART Output Options
	 * ***************************/

	// Set raw output and map NL to CR-LF
	uart_config.c_oflag &= ~ONLCR;

	// Flush the buffer
	tcflush(_fd, TCIOFLUSH);


	/*********************************
	 * Apply Modified Port Attributes
	 * *******************************/
	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		return PX4_ERROR;
	}

	PX4_INFO("Opened port %s", _port);
	return PX4_OK;
}

unsigned SagetechMXS::convert_to_px4_baud(int baudType)
{
	switch (baudType) {
	case 0: return B38400;

	case 1: return B600;

	case 2: return B4800;

	case 3: return B9600;

	// case 4: return B0;

	case 5: return B57600;

	case 6: return B115200;

	case 7: return B230400;

	case 8: return B19200;

	case 9: return BAUD_460800;

	case 10: return BAUD_921600;

	default: return B57600;
	}
}

sg_emitter_t SagetechMXS::convert_emitter_type_to_sg(int emitType)
{
	switch (emitType) {
	case 0: return  sg_emitter_t::aUnknown;

	case 1: return  sg_emitter_t::aLight;

	case 2: return  sg_emitter_t::aSmall;

	case 3: return  sg_emitter_t::aLarge;

	case 4: return  sg_emitter_t::aHighVortex;

	case 5: return  sg_emitter_t::aHeavy;

	case 6: return  sg_emitter_t::aPerformance;

	case 7: return  sg_emitter_t::aRotorCraft;

	case 8: return  sg_emitter_t::bUnknown;

	case 9: return  sg_emitter_t::bGlider;

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

int SagetechMXS::convert_sg_to_emitter_type(sg_emitter_t sg_emit)
{
	switch (sg_emit) {
	case sg_emitter_t::aUnknown:		return 0;

	case sg_emitter_t::aLight:		return 1;

	case sg_emitter_t::aSmall:		return 2;

	case sg_emitter_t::aLarge:		return 3;

	case sg_emitter_t::aHighVortex:		return 4;

	case sg_emitter_t::aHeavy:		return 5;

	case sg_emitter_t::aPerformance:	return 6;

	case sg_emitter_t::aRotorCraft:		return 7;

	case sg_emitter_t::bUnknown:		return 8;

	case sg_emitter_t::bGlider:		return 9;

	case sg_emitter_t::bAir:		return 10;

	case sg_emitter_t::bParachutist:	return 11;

	case sg_emitter_t::bUltralight:		return 12;

	case sg_emitter_t::bUAV:		return 14;

	case sg_emitter_t::bSpace:		return 15;

	case sg_emitter_t::cUnknown:		return 16;

	case sg_emitter_t::cEmergency:		return 17;

	case sg_emitter_t::cService:		return 18;

	case sg_emitter_t::cPoint:		return 19;

	default:				return 20;
	}
}

int SagetechMXS::handle_fid(const char *fid)
{
	if (snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightId), "%-8s", fid) != 8) {
		PX4_ERR("Failed to write Flight ID");
		return PX4_ERROR;
	}

	PX4_INFO("Changed Flight ID to %s", mxs_state.fid.flightId);
	return PX4_OK;
}

int SagetechMXS::store_inst_resp()
{
	_mxs_op_mode.set(mxs_state.ack.opMode);
	_mxs_op_mode.commit();
	_adsb_icao.set(mxs_state.inst.icao);
	_adsb_icao.commit();
	_adsb_len_width.set(mxs_state.inst.size);
	_adsb_len_width.commit();
	_adsb_emit_type.set(convert_sg_to_emitter_type(mxs_state.inst.emitter));
	_adsb_emit_type.commit();
	return PX4_OK;
}


void SagetechMXS::auto_config_operating()
{
	mxs_state.op.opMode = sg_op_mode_t::modeOff;
	_mxs_op_mode.set(sg_op_mode_t::modeOff);
	_mxs_op_mode.commit();

	if (check_valid_squawk(_adsb_squawk.get())) {
		mxs_state.op.squawk = convert_base_to_decimal(BASE_OCTAL, _adsb_squawk.get());

	} else {
		mxs_state.op.squawk = convert_base_to_decimal(BASE_OCTAL, INVALID_SQUAWK);
	}

	mxs_state.op.savePowerUp = true;                                                  // Save power-up state in non-volatile
	mxs_state.op.enableSqt = true;                                                    // Enable extended squitters
	mxs_state.op.enableXBit = false;                                                  // Enable the x-bit
	mxs_state.op.milEmergency = false;                                                // Broadcast a military emergency
	mxs_state.op.emergcType = (sg_emergc_t)
				  _adsb_emergc.get();                                // Enumerated civilian emergency type

	mxs_state.op.altUseIntrnl =
		true;                                                 // True = Report altitude from internal pressure sensor (will ignore other bits in the field)
	mxs_state.op.altHostAvlbl = false;
	mxs_state.op.altRes25 = true;                                // Host Altitude Resolution from install

	mxs_state.op.altitude = static_cast<int32_t>(_gps.altitude_msl_m *
				SAGETECH_SCALE_M_TO_FT);     // Height above sealevel in feet

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
	double heading = (double) math::degrees(matrix::wrap_2pi(_gps.cog_rad));
	mxs_state.op.airspd = speed_knots;
	mxs_state.op.heading = heading;

	last.msg.type = SG_MSG_TYPE_HOST_OPMSG;
	uint8_t txComBuffer[SG_MSG_LEN_OPMSG] {};
	sgEncodeOperating(txComBuffer, &mxs_state.op, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_OPMSG);
}

void SagetechMXS::auto_config_installation()
{
	if (mxs_state.ack.opMode != modeOff) {
		PX4_ERR("MXS not put in OFF Mode before installation.");
		return;
	}

	mxs_state.inst.icao = (uint32_t) _adsb_icao.get();
	snprintf(mxs_state.inst.reg, 8, "%-7s", "PX4TEST");

	mxs_state.inst.com0 = sg_baud_t::baud230400;
	mxs_state.inst.com1 = sg_baud_t::baud57600;

	mxs_state.inst.eth.ipAddress = 0;
	mxs_state.inst.eth.subnetMask = 0;
	mxs_state.inst.eth.portNumber = 0;

	mxs_state.inst.sil = sg_sil_t::silUnknown;
	mxs_state.inst.sda = sg_sda_t::sdaUnknown;
	mxs_state.inst.emitter = convert_emitter_type_to_sg(_adsb_emit_type.get());
	mxs_state.inst.size = (sg_size_t)_adsb_len_width.get();
	mxs_state.inst.maxSpeed = (sg_airspeed_t) _adsb_max_speed.get();
	mxs_state.inst.altOffset = 0;         // Alt encoder offset is legacy field that should always be 0.
	mxs_state.inst.antenna = sg_antenna_t::antBottom;

	mxs_state.inst.altRes100 = true;
	mxs_state.inst.hdgTrueNorth = false;
	mxs_state.inst.airspeedTrue = false;
	mxs_state.inst.heater = true;         // Heater should always be connected.
	mxs_state.inst.wowConnected = true;

	last.msg.type = SG_MSG_TYPE_HOST_INSTALL;

	uint8_t txComBuffer[SG_MSG_LEN_INSTALL] {};
	sgEncodeInstall(txComBuffer, &mxs_state.inst, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_INSTALL);
}

void SagetechMXS::auto_config_flightid()
{
	snprintf(mxs_state.fid.flightId, sizeof(mxs_state.fid.flightId), "%-8s", mxs_state.inst.reg);

	last.msg.type = SG_MSG_TYPE_HOST_FLIGHT;

	uint8_t txComBuffer[SG_MSG_LEN_FLIGHT] {};
	sgEncodeFlightId(txComBuffer, &mxs_state.fid, ++last.msg.id);
	msg_write(txComBuffer, SG_MSG_LEN_FLIGHT);
}

bool SagetechMXS::check_valid_squawk(int squawk)
{
	if (squawk > INVALID_SQUAWK) {
		return false;
	}

	for (int i = 4; i > 0; i--) {
		squawk = squawk - ((int)(squawk / powf(10, i)) * (int)powf(10, i));

		if ((int)(squawk / powf(10, i - 1)) > 7) {
			return false;
		}
	}

	return true;
}
