/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
#include <termios.h>
#include <px4_log.h>
#include <px4_time.h>
#include "simulator.h"
#include "errno.h"
#include <geo/geo.h>
#include <drivers/drv_pwm_output.h>
#include <sys/socket.h>
#include <netinet/in.h>

extern "C" __EXPORT hrt_abstime hrt_reset(void);

#define SEND_INTERVAL 	20
#define UDP_PORT 	14560
#define PIXHAWK_DEVICE "/dev/ttyACM0"

#define PRESS_GROUND 101325.0f
#define DENSITY 1.2041f
#define GRAVITY 9.81f

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
static const float mg2ms2 = CONSTANTS_ONE_G / 1000.0f;

static int openUart(const char *uart_name, int baud);

static int _fd;
static unsigned char _buf[200];
sockaddr_in _srcaddr;
static socklen_t _addrlen = sizeof(_srcaddr);

using namespace simulator;

void Simulator::pack_actuator_message(mavlink_hil_controls_t &actuator_msg) {
	float out[8] = {};

	const float pwm_center = (PWM_HIGHEST_MAX + PWM_LOWEST_MIN) / 2;

	// for now we only support quadrotors
	unsigned n = 4;

	if (_vehicle_status.is_rotary_wing) {
		for (unsigned i = 0; i < 8; i++) {
			if (_actuators.output[i] > PWM_LOWEST_MIN / 2) {
				if (i < n) {
					// scale PWM out 900..2100 us to 0..1 for rotors */
					out[i] = (_actuators.output[i] - PWM_LOWEST_MIN) / (PWM_HIGHEST_MAX - PWM_LOWEST_MIN);

				} else {
					// scale PWM out 900..2100 us to -1..1 for other channels */
					out[i] = (_actuators.output[i] - pwm_center) / ((PWM_HIGHEST_MAX - PWM_LOWEST_MIN) / 2);
				}

			} else {
				// send 0 when disarmed and for disabled channels */
				out[i] = 0.0f;
			}
		}
	} else {
		// convert back to range [-1, 1]
		for (unsigned i = 0; i < 8; i++) {
			out[i] = (_actuators.output[i] - 1500) / 600.0f;
		}
	}

	// if vehicle status has not yet been updated, set actuator commands to zero
	// this is to prevent the simulation getting into a bad state
	if (_vehicle_status.timestamp == 0) {
		memset(out, 0, sizeof(out));
	}

	actuator_msg.time_usec = hrt_absolute_time();
	actuator_msg.roll_ailerons = out[0];
	actuator_msg.pitch_elevator = _vehicle_status.is_rotary_wing ? out[1] : -out[1];
	actuator_msg.yaw_rudder = out[2];
	actuator_msg.throttle = out[3];
	actuator_msg.aux1 = out[4];
	actuator_msg.aux2 = out[5];
	actuator_msg.aux3 = out[6];
	actuator_msg.aux4 = out[7];
	actuator_msg.mode = 0; // need to put something here
	actuator_msg.nav_mode = 0;
}

void Simulator::send_controls() {
	mavlink_hil_controls_t msg;
	pack_actuator_message(msg);
	//PX4_WARN("Sending HIL_CONTROLS msg");
	send_mavlink_message(MAVLINK_MSG_ID_HIL_CONTROLS, &msg, 200);
}

static void fill_rc_input_msg(struct rc_input_values *rc, mavlink_rc_channels_t *rc_channels) {
	rc->timestamp_publication = hrt_absolute_time();
	rc->timestamp_last_signal = hrt_absolute_time();
	rc->channel_count = rc_channels->chancount;
	rc->rssi = rc_channels->rssi;

/*	PX4_WARN("RC: %d, %d, %d, %d, %d, %d, %d, %d",
		 rc_channels->chan1_raw,
		 rc_channels->chan2_raw,
		 rc_channels->chan3_raw,
		 rc_channels->chan4_raw,
		 rc_channels->chan5_raw,
		 rc_channels->chan6_raw,
		 rc_channels->chan7_raw,
		 rc_channels->chan8_raw);
*/

	rc->values[0] = rc_channels->chan1_raw;
	rc->values[1] = rc_channels->chan2_raw;
	rc->values[2] = rc_channels->chan3_raw;
	rc->values[3] = rc_channels->chan4_raw;
	rc->values[4] = rc_channels->chan5_raw;
	rc->values[5] = rc_channels->chan6_raw;
	rc->values[6] = rc_channels->chan7_raw;
	rc->values[7] = rc_channels->chan8_raw;
	rc->values[8] = rc_channels->chan9_raw;
	rc->values[9] = rc_channels->chan10_raw;
	rc->values[10] = rc_channels->chan11_raw;
	rc->values[11] = rc_channels->chan12_raw;
	rc->values[12] = rc_channels->chan13_raw;
	rc->values[13] = rc_channels->chan14_raw;
	rc->values[14] = rc_channels->chan15_raw;
	rc->values[15] = rc_channels->chan16_raw;
	rc->values[16] = rc_channels->chan17_raw;
	rc->values[17] = rc_channels->chan18_raw;
}

void Simulator::update_sensors(mavlink_hil_sensor_t *imu) {
	// write sensor data to memory so that drivers can copy data from there
	RawMPUData mpu;
	mpu.accel_x = imu->xacc;
	mpu.accel_y = imu->yacc;
	mpu.accel_z = imu->zacc;
	mpu.temp = imu->temperature;
	mpu.gyro_x = imu->xgyro;
	mpu.gyro_y = imu->ygyro;
	mpu.gyro_z = imu->zgyro;

	write_MPU_data((void *)&mpu);

	RawAccelData accel;
	accel.x = imu->xacc;
	accel.y = imu->yacc;
	accel.z = imu->zacc;

	write_accel_data((void *)&accel);

	RawMagData mag;
	mag.x = imu->xmag;
	mag.y = imu->ymag;
	mag.z = imu->zmag;

	write_mag_data((void *)&mag);

	RawBaroData baro;
	// calculate air pressure from altitude (valid for low altitude)
	baro.pressure = (PRESS_GROUND - GRAVITY * DENSITY * imu->pressure_alt) / 100.0f; // convert from Pa to mbar
	baro.altitude = imu->pressure_alt;
	baro.temperature = imu->temperature;

	write_baro_data((void *)&baro);

	RawAirspeedData airspeed;
	airspeed.temperature = imu->temperature;
	airspeed.diff_pressure = imu->diff_pressure;

	write_airspeed_data((void *)&airspeed);
}

void Simulator::update_gps(mavlink_hil_gps_t *gps_sim) {
	RawGPSData gps;
	gps.lat = gps_sim->lat; 
 	gps.lon = gps_sim->lon;
 	gps.alt = gps_sim->alt;
	gps.eph = gps_sim->eph;
	gps.epv = gps_sim->epv;
	gps.vel = gps_sim->vel;
	gps.vn = gps_sim->vn;
	gps.ve = gps_sim->ve;
	gps.vd = gps_sim->vd;
	gps.cog = gps_sim->cog;
	gps.fix_type = gps_sim->fix_type;
	gps.satellites_visible = gps_sim->satellites_visible;

	write_gps_data((void *)&gps);
}

void Simulator::handle_message(mavlink_message_t *msg, bool publish) {
	switch(msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		mavlink_hil_sensor_t imu;
		mavlink_msg_hil_sensor_decode(msg, &imu);
		if (publish) {
			publish_sensor_topics(&imu);
		}
		update_sensors(&imu);
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		mavlink_hil_gps_t gps_sim;
		mavlink_msg_hil_gps_decode(msg, &gps_sim);
		if (publish) {
			//PX4_WARN("FIXME:  Need to publish GPS topic.  Not done yet.");
		}
		update_gps(&gps_sim);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:
		mavlink_rc_channels_t rc_channels;
		mavlink_msg_rc_channels_decode(msg, &rc_channels);
		fill_rc_input_msg(&_rc_input, &rc_channels);

		// publish message
		if (publish) {
			if(_rc_channels_pub == nullptr) {
				_rc_channels_pub = orb_advertise(ORB_ID(input_rc), &_rc_input);
			} else {
				orb_publish(ORB_ID(input_rc), _rc_channels_pub, &_rc_input);
			}
		}
		break;
	}
}

void Simulator::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {
	component_ID = 0;
	uint8_t payload_len = mavlink_message_lengths[msgid];
	unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	/* no idea which numbers should be here*/
	buf[2] = 100;
	buf[3] = 0;
	buf[4] = component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],msg, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	ssize_t len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, _addrlen);
	if (len <= 0) {
		PX4_WARN("Failed sending mavlink message");
	}
}

void Simulator::poll_topics() {
	// copy new actuator data if available
	bool updated;
	orb_check(_actuator_outputs_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuators);
	}

	orb_check(_vehicle_status_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void *Simulator::sending_trampoline(void *) {
	_instance->send();
	return 0;	// why do I have to put this???
}

void Simulator::send() {
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _actuator_outputs_sub;
	fds[0].events = POLLIN;

	int pret;

	while(true) {
		// wait for up to 100ms for data
		pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		//timed out
		if (pret == 0)
			continue;

		// this is undesirable but not much we can do
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			// got new data to read, update all topics
			poll_topics();
			send_controls();
		}
	}
}

void Simulator::initializeSensorData()
{
	struct baro_report baro;
	memset(&baro,0,sizeof(baro));
	baro.pressure = 120000.0f;

	// acceleration report
	struct accel_report accel;
	memset(&accel,0,sizeof(accel));
	accel.z = 9.81f;
	accel.range_m_s2 = 80.0f;

	// gyro report
	struct gyro_report gyro;
	memset(&gyro, 0 ,sizeof(gyro));

	// mag report
	struct mag_report mag;
	memset(&mag, 0 ,sizeof(mag));
}

void Simulator::pollForMAVLinkMessages(bool publish)
{
	// udp socket data
	struct sockaddr_in _myaddr;
	const int _port = UDP_PORT;

	// try to setup udp socket for communcation with simulator
	memset((char *)&_myaddr, 0, sizeof(_myaddr));
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_port);

	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed\n");
		return;
	}

	if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed\n");
		return;
	}

	// create a thread for sending data to the simulator
	pthread_t sender_thread;

	// initialize threads
	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, 1000);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);

	// setup serial connection to autopilot (used to get manual controls)
	int serial_fd = openUart(PIXHAWK_DEVICE, 115200);

	if (serial_fd < 0) {
		PX4_WARN("failed to open %s", PIXHAWK_DEVICE);
	}

	// tell the device to stream some messages
	char command[] = "\nsh /etc/init.d/rc.usb\n";
	int w = ::write(serial_fd, command, sizeof(command));

	if (w <= 0) {
		PX4_WARN("failed to send streaming command to %s", PIXHAWK_DEVICE);
	}

	char serial_buf[1024];

	struct pollfd fds[2];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	fds[1].fd = serial_fd;
	fds[1].events = POLLIN;

	int len = 0;

	// wait for first data from simulator and respond with first controls
	// this is important for the UDP communication to work
	int pret = -1;
	PX4_WARN("Waiting for initial data on UDP.. Please connect the simulator first.");
	while (pret <= 0) {
		pret = ::poll(&fds[0], (sizeof(fds[0])/sizeof(fds[0])), 100);
	}
	PX4_WARN("Found initial message, pret = %d",pret);
	_initialized = true;
	// reset system time
	(void)hrt_reset();

	if (fds[0].revents & POLLIN) {
		len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
		PX4_WARN("Sending initial controls message to jMAVSim.");
		send_controls();
	}

	// subscribe to topics
	_actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), 0);
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	// got data from simulator, now activate the sending thread
	pthread_create(&sender_thread, &sender_thread_attr, Simulator::sending_trampoline, NULL);
	pthread_attr_destroy(&sender_thread_attr);

	// wait for new mavlink messages to arrive
	while (true) {

		pret = ::poll(&fds[0], (sizeof(fds)/sizeof(fds[0])), 100);

		//timed out
		if (pret == 0)
			continue;

		// this is undesirable but not much we can do
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		// got data from simulator
		if (fds[0].revents & POLLIN) {
			len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
			if (len > 0) {
				mavlink_message_t msg;
				mavlink_status_t status;
				for (int i = 0; i < len; ++i)
				{
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
					{
						// have a message, handle it
						handle_message(&msg, publish);
					}
				}
			}
		}

		// got data from PIXHAWK
		if (fds[1].revents & POLLIN) {
			len = ::read(serial_fd, serial_buf, sizeof(serial_buf));
			if (len > 0) {
				mavlink_message_t msg;
				mavlink_status_t status;
				for (int i = 0; i < len; ++i)
				{
					if (mavlink_parse_char(MAVLINK_COMM_0, serial_buf[i], &msg, &status))
					{
						// have a message, handle it
						handle_message(&msg, publish);
					}
				}
			}
		}
	}
}

int openUart(const char *uart_name, int baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		warnx("ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n921600\n",
		      baud);
		return -EINVAL;
	}

	/* open uart */
	int uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	if (uart_fd < 0) {
		return uart_fd;
	}


	/* Try to set baud rate */
	struct termios uart_config;
	memset(&uart_config, 0, sizeof(uart_config));

	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(uart_fd);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart_fd, &uart_config);

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			::close(uart_fd);
			return -1;
		}

	}

	// Make raw
	cfmakeraw(&uart_config);

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		::close(uart_fd);
		return -1;
	}

	return uart_fd;
}

int Simulator::publish_sensor_topics(mavlink_hil_sensor_t *imu) {


        //uint64_t timestamp = imu->time_usec;
        uint64_t timestamp = hrt_absolute_time();

        if((imu->fields_updated & 0x1FFF)!=0x1FFF) {
	   	        PX4_DEBUG("All sensor fields in mavlink HIL_SENSOR packet not updated.  Got %08x",imu->fields_updated);
        }
	/*
	  static int count=0;
	  static uint64_t last_timestamp=0;
	  count++;
	  if (!(count % 200)) {
		PX4_WARN("TIME : %lu,   dt: %lu",
			 (unsigned long) timestamp,(unsigned long) timestamp - (unsigned long) last_timestamp);
		PX4_WARN("IMU  : %f %f %f",imu->xgyro,imu->ygyro,imu->zgyro);
		PX4_WARN("ACCEL: %f %f %f",imu->xacc,imu->yacc,imu->zacc);
		PX4_WARN("MAG  : %f %f %f",imu->xmag,imu->ymag,imu->zmag);
		PX4_WARN("BARO : %f %f %f",imu->abs_pressure,imu->pressure_alt,imu->temperature);
	}
	last_timestamp = timestamp;
	*/
	/* gyro */
	{
		struct gyro_report gyro;
		memset(&gyro, 0, sizeof(gyro));

		gyro.timestamp = timestamp;
		gyro.x_raw = imu->xgyro * 1000.0f;
		gyro.y_raw = imu->ygyro * 1000.0f;
		gyro.z_raw = imu->zgyro * 1000.0f;
		gyro.x = imu->xgyro;
		gyro.y = imu->ygyro;
		gyro.z = imu->zgyro;

		if (_gyro_pub == nullptr) {
			_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);

		} else {
			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &gyro);
		}
	}

	/* accelerometer */
	{
		struct accel_report accel;
		memset(&accel, 0, sizeof(accel));

		accel.timestamp = timestamp;
		accel.x_raw = imu->xacc / mg2ms2;
		accel.y_raw = imu->yacc / mg2ms2;
		accel.z_raw = imu->zacc / mg2ms2;
		accel.x = imu->xacc;
		accel.y = imu->yacc;
		accel.z = imu->zacc;

		if (_accel_pub == nullptr) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* magnetometer */
	{
		struct mag_report mag;
		memset(&mag, 0, sizeof(mag));

		mag.timestamp = timestamp;
		mag.x_raw = imu->xmag * 1000.0f;
		mag.y_raw = imu->ymag * 1000.0f;
		mag.z_raw = imu->zmag * 1000.0f;
		mag.x = imu->xmag;
		mag.y = imu->ymag;
		mag.z = imu->zmag;

		if (_mag_pub == nullptr) {
			/* publish to the first mag topic */
			_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

		} else {
			orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
		}
	}

	/* baro */
	{
		struct baro_report baro;
		memset(&baro, 0, sizeof(baro));

		baro.timestamp = timestamp;
		baro.pressure = imu->abs_pressure;
		baro.altitude = imu->pressure_alt;
		baro.temperature = imu->temperature;

		if (_baro_pub == nullptr) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		}
	}

        return OK;
}
