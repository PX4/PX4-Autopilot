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


MXS::MXS(const char *serial_port):ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
Device(MODULE_NAME),
_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))

{
	//Dave the serial port
	strncpy(_serial_port, serial_port, sizeof(_serial_port) - 1);
	/* enforce null termination */
	_serial_port[sizeof(_serial_port) - 1] = '\0';

	set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);
	set_device_type(DRV_TRNS_DEVTYPE_MXS);
	char c = serial_port[strlen(serial_port) - 1]; // last digit of path (eg /dev/ttyS2)
	set_device_bus(atoi(&c));
}

int MXS::open_serial_port()
{
	// File descriptor already initialized?
	if (_file_descriptor > 0) {
		PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
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

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, _baudrate);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, _baudrate);

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

	PX4_DEBUG("opened UART port %s", _serial_port);

	return PX4_OK;
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

void MXS::handle_svr(sg_svr_t svr)
{
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

	_transponder_pub.publish(t);
}

void MXS::handle_msr(sg_msr_t msr)
{
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
	_transponder_pub.publish(t);
}


void MXS::print_info()
{
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

void MXS::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
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

