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

/*
 * MXS.cpp
 *
 * Sagetech MXS transponder driver
 * @author Megan McCormick megan.mccormick@sagetech.com
 * @author Check Faber chuck.faber@sagetech.com
 */

#include "MXS.hpp"


MXS::MXS(const char *serial_port, unsigned baudrate):ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
Device(MODULE_NAME),
ModuleParams(nullptr)

{
	_baudrate = baudrate;
	//Save the serial port
	if(serial_port == nullptr)
	{
		PX4_WARN("No port specified");
	}
	else
	{
		_serial_port = strdup(serial_port);
		set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);
		set_device_type(DRV_TRNS_DEVTYPE_MXS);
		char c = serial_port[strlen(serial_port) - 1]; // last digit of path (eg /dev/ttyS2)
		set_device_bus(atoi(&c));
	}



}
MXS::~MXS()
{
	stop();

	free((char *)_serial_port);
	perf_free(_loop_elapsed_perf);
	perf_free(_loop_count_perf);
	perf_free(_loop_interval_perf);
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int MXS::open_serial_port()
{
	//_baudrate = 230400;
	speed_t baud = convert_baudrate(_baudrate);
	// File descriptor already initialized?
	if (_file_descriptor > 0) {
		//PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	//PX4_INFO("Atempting to open port %s with baudrate %d",_serial_port, _baudrate);
	_file_descriptor = ::open(_serial_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	if (tcgetattr(_file_descriptor, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_file_descriptor);
		_file_descriptor = -1;
		return PX4_ERROR;
	}

	// Clear: data bit size, two stop bits, parity, hardware flow control.
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	//uart_config.c_cc[VMIN]  = 0;
	//uart_config.c_cc[VTIME] = 20;

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, baud);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, baud);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Flush the hardware buffers.
	tcflush(_file_descriptor, TCIOFLUSH);

	//PX4_INFO("opened UART port %s", _serial_port);

	return PX4_OK;
}

void MXS::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

void MXS::parse_byte(uint8_t data)
{
	//PX4_INFO("Current state: %d", _msgIn.state);
	switch(_msgIn.state)
	{
		case startByte:
			if(data == START_BYTE)
			{
				_msgIn.start = data;
				_msgIn.checksum = data;
				_msgIn.state = msgByte;
			}
			break;
		case msgByte:
			_msgIn.checksum += data;
			_msgIn.type = data;
			_msgIn.state = idByte;
			break;
		case idByte:
			_msgIn.checksum += data;
			_msgIn.id = data;
			_msgIn.state = lengthByte;
			break;
		case lengthByte:
			_msgIn.checksum += data;
			_msgIn.length = data;
			_msgIn.index = 0;
			_msgIn.state = (data ==0) ? checksumByte : payload;
			break;
		case payload:
			_msgIn.checksum += data;
			_msgIn.payload[_msgIn.index ++] = data;
			if (_msgIn.index >= _msgIn.length)
			{
				_msgIn.state = checksumByte;
			}
			break;
		case checksumByte:
			if (_msgIn.checksum == data)
			{
				//handle/build message
				handle_msg();
			}
#ifdef MXS_DEBUG
			/*else
			{
				PX4_INFO("Checksum does not match, internal: %d Read: %d",_msgIn.checksum,  data);
			}*/
#endif
			_msgIn.state = startByte;
			break;
		default:
			_msgIn.state = startByte;
			break;

	}
}

int MXS::collect()
{


	// the buffer for read chars is buflen minus null termination
	//unsigned _buffer_len = sizeof(_buffer) - 1;

	int ret = 0;

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_file_descriptor, FIONREAD, (unsigned long)&bytes_available);


	if (!bytes_available) {
		return 0;
	}
	else
	{
		//PX4_INFO("Bytes on port: %d ", bytes_available);
		perf_begin(_sample_perf);
	}

	// parse entire buffer
	//const hrt_abstime timestamp_sample = hrt_absolute_time();
	memset(_buffer,0, sizeof(_buffer));

	do {
		// read from the sensor (uart buffer)
		uint8_t data;
		tcflush(_file_descriptor, TCOFLUSH);
		ret = ::read(_file_descriptor, &data, sizeof(data));

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			break;
		}
		parse_byte(data);


		// parse buffer

		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);


	perf_end(_sample_perf);

	return PX4_OK;
}

void MXS::handle_msg()
{
	//uint8_t msgIn[255];
	memset(_buffer,0, sizeof(_buffer));
	//manual copy
	_buffer[0] = _msgIn.start;
	_buffer[1] = _msgIn.type;
	_buffer[2] = _msgIn.id;
	_buffer[3] = _msgIn.length;
	for(int i  = 0; i < _msgIn.length ; i ++)
	{
		_buffer[4 + i] = _msgIn.payload[i];
	}
	_buffer[4 + _msgIn.length] = _msgIn.checksum;
	_buffer_len = 5 + _msgIn.length;
#ifdef MXS_DEBUG
	for(int i = 0; i < _buffer_len; i ++)
	{
		PX4_INFO("Buffer at %d: %X", i , _buffer[i]);
	}
#endif
	switch(_msgIn.type)
	{
		case SG_MSG_TYPE_ADSB_MSR:
			sg_msr_t msr;
			sgDecodeMSR(_buffer, &msr);
			handle_msr(msr);
			break;
		case SG_MSG_TYPE_ADSB_SVR:
			sg_svr_t svr;
			sgDecodeSVR(_buffer, &svr);
			handle_svr(svr);
			break;

	}
}

uint8_t MXS::determine_emitter(sg_adsb_emitter_t emit)
{
	uint8_t emitCat;
	switch(emit)
	{
	case adsbUnknown:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_NO_INFO;
		break;
	case adsbLight:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_LIGHT;
		break;
	case adsbSmall:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_SMALL;
		break;
	case adsbLarge:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_LARGE;
		break;
	case adsbHighVortex:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE;
		break;
	case adsbHeavy:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_HEAVY;
		break;
	case adsbPerformance:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_HIGHLY_MANUV;
		break;
	case adsbRotorcraft:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_ROTOCRAFT;
		break;
	case adsbGlider:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_GLIDER;
		break;
	case adsbAir:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_LIGHTER_AIR;
		break;
	case adsbUnmaned:
		return transponder_report_s::ADSB_EMITTER_TYPE_UAV;
		break;
	case adsbSpace:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_SPACE;
		break;
	case adsbUltralight:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_ULTRA_LIGHT;
		break;
	case adsbParachutist:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_PARACHUTE;
		break;
	case adsbVehicle_emg:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_EMERGENCY_SURFACE;
		break;
	case adsbVehicle_serv:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_SERVICE_SURFACE;
		break;
	case adsbObsticlePoint:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_POINT_OBSTACLE;
		break;
	default:
		emitCat = transponder_report_s::ADSB_EMITTER_TYPE_NO_INFO;
		break;
	}


	return emitCat;
}

speed_t MXS::convert_baudrate(unsigned baud)
{
	speed_t ret;
	switch (baud)
	{
	case 600:
		ret = B600;
		break;
	case 4800:
		ret = B4800;
		break;
	case 9600:
		ret = B9600;
		break;
	case 19200:
		ret = B19200;
		break;
	/*case 28800:
		ret = B28800;
		break;*/
	case 38400:
		ret = B38400;
		break;
	case 57600:
		ret = B57600;
		break;
	case 115200:
		ret = B115200;
		break;
	case 230400:
		ret = B230400;
		break;
	case 460800:
		ret = B460800;
		break;
	case 921600:
		ret = B921600;
		break;
	default:
		ret = B230400;
		break;
	}
	return ret;
}

void MXS::handle_svr(sg_svr_t svr)
{
#ifdef MXS_DEBUG
	PX4_INFO("Updating SVR transponder message");
#endif

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
	t.flags |= transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;
	t.flags &= ~transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK;

	if (svr.validity.position)
	{
		t.lat = svr.lat;
		t.lon = svr.lon;
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	}

	if (svr.type == svrAirborne)
	{

		if (svr.validity.geoAlt || svr.validity.baroAlt)
		{
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
			if (svr.validity.geoAlt)
			{
				t.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
				//Convert from Feet to Meters
				t.altitude = (svr.airborne.geoAlt * SAGETECH_SCALE_FEET_TO_M);
			}
			else
			{
				t.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
				//Convert from Feet to Meters
				t.altitude = (svr.airborne.baroAlt * SAGETECH_SCALE_FEET_TO_M);
			}

		}
		if (svr.validity.airSpeed)
		{
			//Convert from knots to meters/second
			t.hor_velocity = (svr.airborne.speed * SAGETECH_SCALE_KNOTS_TO_M_PER_SEC);
			t.heading = svr.airborne.heading;
			t.flags |= transponder_report_s::transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
		}
		if (svr.validity.baroVRate || svr.validity.geoVRate)
		{
			//Convert from feet/min to meters/second
			t.ver_velocity = (svr.airborne.vrate * SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC);
			if (svr.validity.airSpeed)
			{
				t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
			}
		}
	}

	handle_vehicle(t);
}

void MXS::handle_msr(sg_msr_t msr)
{
#ifdef MXS_DEBUG
	PX4_INFO("Updating MSR transponder message");
#endif
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
	t.emitter_type = determine_emitter(msr.emitter);
	handle_vehicle(t);
}

void MXS::send_gps_msg()
{

	sg_gps_t gpsOut;
	//Grab current gps reading

	//Fill gps object for MXS
	//TODO: These are realistic values that are hard coded for now
	gpsOut.hpl = 0; //I think we can find this but it's going to take a lot of digging

	gpsOut.hfom = _gps.eph;
	gpsOut.vfom = _gps.epv;
	float velAcc = _gps.s_variance_m_s;
	gpsOut.nacv = determine_nacv(velAcc);

	//Convert Longitude and Latitude to strings
	const int32_t lon = _gps.lon;
	const double lonDeg = lon * 1.0e-7;
	const double lonMin = (lonDeg - int(lonDeg)) * 60;
	const double lonSec = (lonMin - int(lonMin)) * 1.0e5;
	snprintf(gpsOut.longitude, 12, "%03d%02u.%05u",(int)lonDeg,(uint8_t)lonMin,(uint16_t)lonSec);

	const int32_t lat = _gps.lat;
	const double latDeg = lon * 1.0e-7;
	const double latMin = (lonDeg - int(lonDeg)) * 60;
	const double latSec = (lonMin - int(lonMin)) * 1.0e5;
	snprintf(gpsOut.latitude, 11, "%02d%02u.%05u",(int)latDeg,(uint8_t)latMin,(uint16_t)latSec);

	//Convert ground speed
	const double speedKnots = _gps.vel_m_s * SAGETECH_SCALE_M_PER_SEC_TO_KNOTS;
	if (speedKnots > 1000.0)
	{
		const float speedOneDec = (speedKnots - int(speedKnots)) * 1.0e1;
		snprintf(gpsOut.grdSpeed, 7, "%04u%01u", (uint16_t)speedKnots, (uint8_t)speedOneDec);
	}
	else
	{
		const double speedTwoDec = (speedKnots - int(speedKnots)) * 1.0e1;
		snprintf(gpsOut.grdSpeed, 7, "%03u%02u", (uint16_t)speedKnots, (uint8_t)speedTwoDec);
	}

	//Convert ground track
	double grdTrackDeg = (double)_gps.heading * (180.0/SAGETECH_PI);
	if (grdTrackDeg < 0)
	{
		grdTrackDeg  = 360 + grdTrackDeg;
	}
	const double grdTrackDegDec = (grdTrackDeg - int(grdTrackDeg)) * 1.0e4;
	snprintf(gpsOut.grdTrack, 9, "%03u.%04u", (uint16_t) grdTrackDeg,(uint16_t)grdTrackDegDec);

	gpsOut.latNorth = (lat>= 0) ? true: false;
	gpsOut.lngEast = (lon>= 0) ? true: false;

	gpsOut.gpsValid = _gps.vel_ned_valid;

	//Convert Time of Fix
	uint64_t timeUsec = _gps.time_utc_usec;
	if (timeUsec)
	{
		const uint8_t hours = timeUsec % SAGETECH_USEC_PER_HOUR;
		timeUsec = timeUsec - (hours * SAGETECH_USEC_PER_HOUR);
		const uint8_t mins = timeUsec % SAGETECH_USEC_PER_MIN;
		timeUsec = timeUsec - (mins * SAGETECH_USEC_PER_MIN);
		const uint8_t secs = timeUsec % SAGETECH_USEC_PER_SEC;
		timeUsec = timeUsec - (secs * SAGETECH_USEC_PER_SEC);
		const uint16_t fracSecs = timeUsec * 1.0e-3;
		snprintf(gpsOut.timeOfFix, 11, "%02u%02u%02u.%03u",hours,mins,secs,fracSecs);
	}
	else
	{
		strncpy(gpsOut.timeOfFix, "      .   ", 11);
	}

	gpsOut.height = _gps.alt_ellipsoid * 1.0e-3; //Convert to meters

	//Encode GPS
	memset(_buffer,0, sizeof(_buffer)); 	//Make sure buffer is clear
	sgEncodeGPS(_buffer,&gpsOut, uint8_t(_msgId++));

	//Write to serial
	int ret = 0;

#ifdef DEBUG_MXS
	char out[255] = "";
	buff_to_hex(out,_buffer,int(SG_MSG_LEN_GPS +1));
	PX4_INFO("Atempting to write GPS %s",out);
#endif
	//PX4_INFO("Atempting to write GPS\n");
	//tcflush(_file_descriptor, TCIFLUSH);

	ret = ::write(_file_descriptor, _buffer, SG_MSG_LEN_GPS); //Could use SG_MSG_LEN_GPS + 1

	if(ret < 0)
	{
		PX4_INFO("Error in writing GPS");
	}
#ifdef DEBUG_MXS
	else
	{
		PX4_INFO("GPS Write was sucessful\n");

	}
#endif

}

void MXS::buff_to_hex(char*out,const uint8_t *buff, int len)
{
	for ( int i = 0; i  < len; i ++)
	{
		char str[3];
		sprintf(str,"%X",_buffer[i]);
		strcat(out, str);
	}

}

sg_nacv_t MXS::determine_nacv(float velAcc)
{
	sg_nacv_t ret;
    if (velAcc >= (float)10.0) {
    	ret = nacvUnknown;
    }
    else if (velAcc >= (float)3.0) {
    	ret =  nacv10dot0;
    }
    else if (velAcc >= (float)1.0) {
    	ret =  nacv3dot0;
    }
    else if (velAcc >= (float)0.3) {
    	ret =  nacv1dot0;
    }
    else if (velAcc >= (float)0.0) {
    	ret =  nacv0dot3;
    }
    else
    {
    	ret = nacvUnknown;
    }
    return ret;
}

void MXS::print_info()
{
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

int MXS::init()
{
	if (_baudrate == 0)
	{
		_baudrate = 230400;
	}
	_msgIn.checksum = 0;
	_msgIn.id = 0;
	_msgIn.length = 0;
	_msgIn.start = 0;
	_msgIn.state = 0;
	_msgIn.type = 0;

	return PX4_OK;
}

void MXS::Run()
{
#ifdef DEBUG_MXS
	PX4_INFO("MXS Driver running");
#endif

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

	//Subscribe to GPS uORB
	//int gps_sub_fd = orb_subscribe(ORB_ID(sensor_gps));
	//orb_set_interval(gps_sub_fd, 200);
	//orb_copy(ORB_ID(sensor_gps), gps_sub_fd, &_gps);
	// Ensure the serial port is open.
	open_serial_port();
	collect();

	/******************
	 * 5 Hz Timer
	 * ****************/

	if (!(_loop_count % FIVE_HZ_MOD)) {		// 5Hz Timer (GPS Flight)
			/* Get Subscription Data */
			if (_vehicle_land_detected_sub.updated()) {
				_vehicle_land_detected_sub.copy(&_landed);
			}
			if (_sensor_gps_sub.updated()) {
				_sensor_gps_sub.copy(&_gps);
			}

			if (!_landed.landed) {
				send_gps_msg();
			}
	}

	/***********************
	 * 1 Hz Timer
	 * *********************/

	if (!(_loop_count % ONE_HZ_MOD)) {		// 1Hz Timer (Operating Message/GPS Ground)
		if (_landed.landed) {
			send_gps_msg();
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
	}

	perf_end(_loop_elapsed_perf);


	 /*const hrt_abstime currentTime = hrt_absolute_time();

	 if (currentTime - _last_gps_send >= (1000000))
	 {
		send_gps_msg();
		::close(_file_descriptor);
		_file_descriptor = -1;

		_last_gps_send = currentTime;

	 }*/


}

/***************************
 * ADSB Vehicle List Functions
****************************/

bool MXS::get_vehicle_by_ICAO(const uint32_t icao, transponder_report_s &vehicle) const
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

bool MXS::find_index(const transponder_report_s &vehicle, uint16_t *index) const
{
	for (uint16_t i = 0; i < vehicle_count; i++) {
		if (vehicle_list[i].icao_address == vehicle.icao_address) {
			*index = i;
			return true;
		}
	}
	return false;
}

void MXS::set_vehicle(const uint16_t index, const transponder_report_s &vehicle)
{
	if (index >= MAX_VEHICLES_TRACKED) {
		return; // out of range
	}
	vehicle_list[index] = vehicle;
}

void MXS::determine_furthest_aircraft(void)
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

void MXS::delete_vehicle(const uint16_t index)
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

void MXS::handle_vehicle(const transponder_report_s &vehicle)
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
		_transponder_pub.publish(vehicle);
	}
}

void MXS::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(SAGETECH_MXS_POLL_RATE);

}

void MXS::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
	free((char *)_serial_port);
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

