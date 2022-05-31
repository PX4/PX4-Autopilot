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
 */

#include "MXS.hpp"


MXS::MXS(const char *serial_port, unsigned baudrate):ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
Device(MODULE_NAME),
_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))

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
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int MXS::open_serial_port()
{
	_baudrate = 230400;
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
	uart_config.c_cc[VMIN]  = 1;
	uart_config.c_cc[VTIME] = 0;

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
	//tcflush(_file_descriptor, TCIOFLUSH);

	//PX4_INFO("opened UART port %s", _serial_port);

	return PX4_OK;
}

void MXS::parse_byte(uint8_t data)
{
	//PX4_INFO("Current state: %d", _msgIn.state);
	PX4_INFO("Data in %x", data);
	switch(_msgIn.state)
	{
		case startByte:
			if(data == START_BYTE)
			{
				_msgIn.start = START_BYTE;
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
				PX4_INFO("Handling message");
				handle_msg(_msgIn);
			}
			else
			{
				PX4_INFO("Checksum does not match, internal: %d Read: %d",_msgIn.checksum,  data);
			}
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
		//::close(_file_descriptor);
		//_file_descriptor = -1;
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
		PX4_INFO("Current buffer: %X",data);
		//parse_byte(data);


		// parse buffer

		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);


	perf_end(_sample_perf);

	return PX4_OK;
}

void MXS::handle_msg(const sagetech_packet_t packet)
{
	PX4_INFO("In handle message");
	uint8_t msgIn[255];
	memcpy(msgIn, &packet.start, (5 + packet.index)* sizeof(uint8_t));
	char out[255] = "";
	buff_to_hex(out,msgIn, (5 + packet.index)* sizeof(uint8_t));
	PX4_INFO("Current MXS Message: %s",msgIn );
	switch(packet.type)
	{
		case SG_MSG_TYPE_ADSB_MSR:
			sg_msr_t msr;
			sgDecodeMSR(msgIn, &msr);
			handle_msr(msr);
			break;
		case SG_MSG_TYPE_ADSB_SVR:
			sg_svr_t svr;
			sgDecodeSVR(msgIn, &svr);
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
	PX4_INFO("Updating SVR transponder message");
	transponder_report_s t{};

	t.timestamp = hrt_absolute_time();
	//Set data from svr message
	//TODO: Should we always set this? It might not be an ICAO
	t.icao_address = svr.addr;

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

	//_transponder_pub.publish(t);
}

void MXS::handle_msr(sg_msr_t msr)
{
	PX4_INFO("Updating MSR transponder message");
	transponder_report_s t{};

	t.timestamp = hrt_absolute_time();
	//Set data from svr message
	//TODO: Should we always set this? It might not be an ICAO
	t.icao_address = msr.addr;

	if(msr.callsign[0] != 0)
	{
		memcpy(&t.callsign, &msr.callsign, 8);
	}
	t.emitter_type = determine_emitter(msr.emitter);
	//_transponder_pub.publish(t);
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

	ret = ::write(_file_descriptor, _buffer, sizeof(gpsOut)); //Could use SG_MSG_LEN_GPS + 1

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
	//Subscribe to GPS uORB
	int gps_sub_fd = orb_subscribe(ORB_ID(sensor_gps));
	orb_set_interval(gps_sub_fd, 200);
	orb_copy(ORB_ID(sensor_gps), gps_sub_fd, &_gps);
	// Ensure the serial port is open.
	open_serial_port();
	collect();

	 /*const hrt_abstime currentTime = hrt_absolute_time();

	 if (currentTime - _last_gps_send >= 1000000)
	 {
		send_gps_msg();

		_last_gps_send = currentTime;

	 }*/

	::close(_file_descriptor);
	_file_descriptor = -1;


}

void MXS::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(SAGETECH_MXS_POLL_RATE, SAGETECH_MXS_POLL_RATE);

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

