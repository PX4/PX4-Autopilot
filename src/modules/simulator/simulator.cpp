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

/**
 * @file simulator.cpp
 * A device simulator
 */

#include <px4_debug.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <pthread.h>
#include <poll.h>
#include <systemlib/err.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h> 
#include <drivers/drv_led.h>

#include "simulator.h"

using namespace simulator;

static px4_task_t g_sim_task = -1;

Simulator *Simulator::_instance = NULL;

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

Simulator *Simulator::getInstance()
{
	return _instance;
}

bool Simulator::getMPUReport(uint8_t *buf, int len)
{
	return _mpu.copyData(buf, len);
}

bool Simulator::getRawAccelReport(uint8_t *buf, int len)
{
	return _accel.copyData(buf, len);
}

bool Simulator::getBaroSample(uint8_t *buf, int len)
{
	return _baro.copyData(buf, len);
}

int Simulator::start(int argc, char *argv[])
{
	int ret = 0;
	_instance = new Simulator();
	if (_instance) {
		PX4_INFO("Simulator started\n");
		drv_led_start();
		if (argv[2][1] == 's') {
#ifndef __PX4_QURT
			_instance->updateSamples();
#endif
		} else {
			_instance->publishSensorsCombined();
		}
	}
	else {
		PX4_WARN("Simulator creation failed\n");
		ret = 1;
	}
	return ret;
}

void Simulator::poll_topics() {
	// copy new data if available
	bool updated;
	orb_check(_actuator_outputs_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuators);
	}

	orb_check(_vehicle_attitude_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);
	}
}

void Simulator::send_data() {
	// check if it's time to send new data
	hrt_abstime time_now = hrt_absolute_time();
	if (time_now - _time_last >= (hrt_abstime)(_interval * 1000)) {
		_time_last = time_now;
		mavlink_message_t msg;
		pack_actuator_message(&msg);
		send_mavlink_message(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, &msg, 200);
		// can add more messages here, can also setup different timings
	}
}

void Simulator::pack_actuator_message(mavlink_message_t *msg) {
	// pack message and send
	mavlink_servo_output_raw_t actuator_msg;

	actuator_msg.time_usec = hrt_absolute_time();
	actuator_msg.port = 8;	// hardcoded for now
	actuator_msg.servo1_raw = _actuators.output[0];
	actuator_msg.servo2_raw = _actuators.output[1];
	actuator_msg.servo3_raw = _actuators.output[2];
	actuator_msg.servo4_raw = _actuators.output[3];
	actuator_msg.servo5_raw = _actuators.output[4];
	actuator_msg.servo6_raw = _actuators.output[5];
	actuator_msg.servo7_raw = _actuators.output[6];
	actuator_msg.servo8_raw = _actuators.output[7];

	// encode the message
	mavlink_msg_servo_output_raw_encode(1, 100, msg, &actuator_msg);
}

void Simulator::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {
	uint8_t payload_len = mavlink_message_lengths[msgid];

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	/* no idea which numbers should be here*/
	buf[2] = 100;
	buf[3] = 1;
	buf[4] = component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],&msg, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	ssize_t len = sendto(_fd, buf, sizeof(buf), 0, (struct sockaddr *)&_srcaddr, _addrlen);
	if (len <= 0) {
		PX4_WARN("Failed sending mavlink message");
	}
}

void Simulator::fill_sensors_from_imu_msg(struct sensor_combined_s *sensor, mavlink_highres_imu_t *imu) {
	hrt_abstime timestamp = hrt_absolute_time();
	sensor->timestamp = timestamp;
	sensor->gyro_raw[0] = imu->xgyro * 1000.0f;
	sensor->gyro_raw[1] = imu->ygyro * 1000.0f;
	sensor->gyro_raw[2] = imu->zgyro * 1000.0f;
	sensor->gyro_rad_s[0] = imu->xgyro;
	sensor->gyro_rad_s[1] = imu->ygyro;
	sensor->gyro_rad_s[2] = imu->zgyro;

	sensor->accelerometer_raw[0] = imu->xacc; // mg2ms2;
	sensor->accelerometer_raw[1] = imu->yacc; // mg2ms2;
	sensor->accelerometer_raw[2] = imu->zacc; // mg2ms2;
	sensor->accelerometer_m_s2[0] = imu->xacc;
	sensor->accelerometer_m_s2[1] = imu->yacc;
	sensor->accelerometer_m_s2[2] = imu->zacc;
	sensor->accelerometer_mode = 0; // TODO what is this?
	sensor->accelerometer_range_m_s2 = 32.7f; // int16
	sensor->accelerometer_timestamp = timestamp;
	sensor->timestamp = timestamp;

	sensor->adc_voltage_v[0] = 0.0f;
	sensor->adc_voltage_v[1] = 0.0f;
	sensor->adc_voltage_v[2] = 0.0f;

	sensor->magnetometer_raw[0] = imu->xmag * 1000.0f;
	sensor->magnetometer_raw[1] = imu->ymag * 1000.0f;
	sensor->magnetometer_raw[2] = imu->zmag * 1000.0f;
	sensor->magnetometer_ga[0] = imu->xmag;
	sensor->magnetometer_ga[1] = imu->ymag;
	sensor->magnetometer_ga[2] = imu->zmag;
	sensor->magnetometer_range_ga = 32.7f; // int16
	sensor->magnetometer_mode = 0; // TODO what is this
	sensor->magnetometer_cuttoff_freq_hz = 50.0f;
	sensor->magnetometer_timestamp = timestamp;

	sensor->baro_pres_mbar = imu->abs_pressure;
	sensor->baro_alt_meter = imu->pressure_alt;
	sensor->baro_temp_celcius = imu->temperature;
	sensor->baro_timestamp = timestamp;

	sensor->differential_pressure_pa = imu->diff_pressure * 1e2f; //from hPa to Pa
	sensor->differential_pressure_timestamp = timestamp;
}

void Simulator::fill_manual_control_sp_msg(struct manual_control_setpoint_s *manual, mavlink_manual_control_t *man_msg) {
	manual->timestamp = hrt_absolute_time();
	manual->x = man_msg->x / 1000.0f;
	manual->y = man_msg->y / 1000.0f;
	manual->r = man_msg->r / 1000.0f;
	manual->z = man_msg->z / 1000.0f;
}

void Simulator::handle_message(mavlink_message_t *msg) {
	switch(msg->msgid) {
		case MAVLINK_MSG_ID_HIGHRES_IMU:
			mavlink_highres_imu_t imu;
			mavlink_msg_highres_imu_decode(msg, &imu);
			fill_sensors_from_imu_msg(&_sensor, &imu);

			// publish message
			if(_sensor_combined_pub < 0) {
				_sensor_combined_pub = orb_advertise(ORB_ID(sensor_combined), &_sensor);
			} else {
				orb_publish(ORB_ID(sensor_combined), _sensor_combined_pub, &_sensor);
			}
			break;

		case MAVLINK_MSG_ID_MANUAL_CONTROL:

			mavlink_manual_control_t man_ctrl_sp;
			mavlink_msg_manual_control_decode(msg, &man_ctrl_sp);
			fill_manual_control_sp_msg(&_manual_control_sp, &man_ctrl_sp);

			// publish message
			if(_manual_control_sp_pub < 0) {
				_manual_control_sp_pub = orb_advertise(ORB_ID(manual_control_setpoint), &_manual_control_sp);
			} else {
				orb_publish(ORB_ID(manual_control_setpoint), _manual_control_sp_pub, &_manual_control_sp);
			}
			break;
	}
}

void Simulator::publishSensorsCombined() {

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
	// init publishers
	_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);
	_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);
	_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);
	_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));
	// fill sensors with some data
	sensors.accelerometer_m_s2[2] = 9.81f;
	sensors.magnetometer_ga[0] = 0.2f;
	sensors.timestamp = hrt_absolute_time();
	sensors.accelerometer_timestamp = hrt_absolute_time();
	sensors.magnetometer_timestamp = hrt_absolute_time();
	sensors.baro_timestamp = hrt_absolute_time();
	// advertise
	_sensor_combined_pub = orb_advertise(ORB_ID(sensor_combined), &sensors);

	hrt_abstime time_last = hrt_absolute_time();
	uint64_t delta;
	for(;;) {
		delta = hrt_absolute_time() - time_last;
		if(delta > (uint64_t)1000000) {
			time_last = hrt_absolute_time();
			sensors.timestamp = time_last;
			sensors.accelerometer_timestamp = time_last;
			sensors.magnetometer_timestamp = time_last;
			sensors.baro_timestamp = time_last;
			baro.timestamp = time_last;
			accel.timestamp = time_last;
			gyro.timestamp = time_last;
			mag.timestamp = time_last;
			// publish the sensor values
			//printf("Publishing SensorsCombined\n");
			orb_publish(ORB_ID(sensor_combined), _sensor_combined_pub, &sensors);
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &baro);
			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &baro);
			orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
		}
		else {
			usleep(1000000-delta);
		}
	}
}

#ifndef __PX4_QURT

void *Simulator::sending_trampoline(void *) {
	_instance->send();
	return 0;	// why do I have to put this???
}

void Simulator::send() {
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _actuator_outputs_sub;
	fds[0].events = POLLIN;

	_time_last = hrt_absolute_time();

	while(true) {
		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

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
			send_data();
		}
	}
}

void Simulator::updateSamples()
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

	// init publishers
	_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);
	_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);
	_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);
	_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

	// subscribe to topics
	_actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

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
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 30;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);
	pthread_create(&sender_thread, &sender_thread_attr, Simulator::sending_trampoline, NULL);
	pthread_attr_destroy(&sender_thread_attr);

	struct pollfd socket_fds;
	socket_fds.fd = _fd;
	socket_fds.events = POLLIN;

	int len = 0;
	// wait for new mavlink messages to arrive
	while (true) {

		int socket_pret = ::poll(&socket_fds, (size_t)1, 100);

		//timed out
		if (socket_pret == 0)
			continue;

		// this is undesirable but not much we can do
		if (socket_pret < 0) {
			PX4_WARN("poll error %d, %d", socket_pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (socket_fds.revents & POLLIN) {
			len = recvfrom(_fd, _buf, _buflen, 0, (struct sockaddr *)&_srcaddr, &_addrlen);
			if (len > 0) {
				mavlink_message_t msg;
				mavlink_status_t status;
				for (int i = 0; i < len; ++i)
				{
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
					{
						// have a message, handle it
						handle_message(&msg);
					}
				}
			}
		}

		// publish these messages so that attitude estimator does not complain
		hrt_abstime time_last = hrt_absolute_time();
		baro.timestamp = time_last;
		accel.timestamp = time_last;
		gyro.timestamp = time_last;
		mag.timestamp = time_last;
		// publish the sensor values
		orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		orb_publish(ORB_ID(sensor_accel), _accel_pub, &baro);
		orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &baro);
		orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
	}
}
#endif

static void usage()
{
	PX4_WARN("Usage: simulator {start -[sc] |stop}");
	PX4_WARN("Simulate raw sensors:     simulator start -s");
	PX4_WARN("Publish sensors combined: simulator start -p");
}

__BEGIN_DECLS
extern int simulator_main(int argc, char *argv[]);
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

extern "C" {

int simulator_main(int argc, char *argv[])
{
	int ret = 0;
	if (argc == 3 && strcmp(argv[1], "start") == 0) {
		if (strcmp(argv[2], "-s") == 0) {
			if (g_sim_task >= 0) {
				warnx("Simulator already started");
				return 0;
			}
			g_sim_task = px4_task_spawn_cmd("Simulator",
				SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5,
				1500,
				Simulator::start,
				argv);
		}
		else if (strcmp(argv[2], "-p") == 0) {
			if (g_sim_task >= 0) {
				warnx("Simulator already started");
				return 0;
			}
			g_sim_task = px4_task_spawn_cmd("Simulator",
				SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5,
				1500,
				Simulator::start,
				argv);
		}
		else
		{
			usage();
			ret = -EINVAL;
		}
	}
	else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
		}
		else {
			px4_task_delete(g_sim_task);
			g_sim_task = -1;
		}
	}
	else {
		usage();
		ret = -EINVAL;
	}

	return ret;
}

}

bool static _led_state[2] = { false , false };

__EXPORT void led_init()
{
	PX4_DBG("LED_INIT\n");
}

__EXPORT void led_on(int led)
{
	if (led == 1 || led == 0)
	{
		PX4_DBG("LED%d_ON", led);
		_led_state[led] = true;
	}
}

__EXPORT void led_off(int led)
{
	if (led == 1 || led == 0)
	{
		PX4_DBG("LED%d_OFF", led);
		_led_state[led] = false;
	}
}

__EXPORT void led_toggle(int led)
{
	if (led == 1 || led == 0)
	{
		_led_state[led] = !_led_state[led];
		PX4_DBG("LED%d_TOGGLE: %s\n", led, _led_state[led] ? "ON" : "OFF");

	}
}

