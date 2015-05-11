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

#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <systemlib/err.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h> 
#include <drivers/drv_led.h>
#ifndef __PX4_QURT
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include "simulator.h"
#include <drivers/drv_hrt.h>

using namespace simulator;

static px4_task_t g_sim_task = -1;

Simulator *Simulator::_instance = NULL;

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

	// get samples from external provider
	struct sockaddr_in myaddr;
	struct sockaddr_in srcaddr;
	socklen_t addrlen = sizeof(srcaddr);
	int len, fd;
	const int buflen = 200;
	const int port = 14550;
	unsigned char buf[buflen];

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed\n");
		return;
	}

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(port);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		PX4_WARN("bind failed\n");
		return;
	}

	// wait for new mavlink messages to arrive
	for (;;) {
		len = 0;
		len = recvfrom(fd, buf, buflen, 0, (struct sockaddr *)&srcaddr, &addrlen);
		if (len > 0) {
			mavlink_message_t msg;
			mavlink_status_t status;
			for (int i = 0; i < len; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// have a message, handle it
					handle_message(&msg);
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
		//printf("Publishing SensorsCombined\n");
		orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		orb_publish(ORB_ID(sensor_accel), _accel_pub, &baro);
		orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &baro);
		orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
	}

	/*
	for (;;) {
		len = recvfrom(fd, buf, buflen, 0, (struct sockaddr *)&srcaddr, &addrlen);
		if (len > 0) {
			if (len == sizeof(RawMPUData)) {
				PX4_DBG("received: MPU data\n");
				_mpu.writeData(buf);
			}
			else if (len == sizeof(RawAccelData)) {
				PX4_DBG("received: accel data\n");
				_accel.writeData(buf);
			}
			else if (len == sizeof(RawBaroData)) {
				PX4_DBG("received: accel data\n");
				_baro.writeData(buf);
			}
			else {
				PX4_DBG("bad packet: len = %d\n", len);
			}
		}
	}
	*/
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

