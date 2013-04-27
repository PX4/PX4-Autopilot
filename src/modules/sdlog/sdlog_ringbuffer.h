/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file sdlog_ringbuffer.h
 * microSD logging
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef SDLOG_RINGBUFFER_H_
#define SDLOG_RINGBUFFER_H_

#pragma pack(push, 1)
struct sdlog_sysvector {
	uint64_t timestamp;		/**<  time [us] */
	float gyro[3];			/**< [rad/s] */
	float accel[3];			/**< [m/s^2] */
	float mag[3];			/**< [gauss] */
	float baro;			/**< pressure [millibar] */
	float baro_alt;			/**< altitude above MSL [meter] */
	float baro_temp;		/**< [degree celcius] */
	float control[4];		/**< roll, pitch, yaw [-1..1], thrust [0..1] */
	float actuators[8];		/**< motor 1-8, in motor units (PWM: 1000-2000,AR.Drone: 0-512) */
	float vbat;			/**< battery voltage in [volt] */
	float bat_current;		/**< battery discharge current */
	float bat_discharged;		/**< discharged energy in mAh */
	float adc[4];			/**< ADC ports [volt] */
	float local_position[3];	/**< tangent plane mapping into x,y,z [m] */
	int32_t gps_raw_position[3];	/**< latitude [degrees] north, longitude [degrees] east, altitude above MSL [millimeter] */
	float attitude[3];		/**< roll, pitch, yaw [rad] */
	float rotMatrix[9];		/**< unitvectors */
	float vicon[6];			/**< Vicon ground truth x, y, z and roll, pitch, yaw */
	float control_effective[4]; 	/**< roll, pitch, yaw [-1..1], thrust [0..1] */
	float flow[6];			/**< flow raw x, y, flow metric x, y, flow ground dist, flow quality */
	float diff_pressure;		/**< differential pressure */
	float ind_airspeed;		/**< indicated airspeed */
	float true_airspeed;		/**< true airspeed */
};
#pragma pack(pop)

struct sdlog_logbuffer {
	unsigned int start;
	// unsigned int end;
	unsigned int size;
	int count;
	struct sdlog_sysvector *elems;
};

void sdlog_logbuffer_init(struct sdlog_logbuffer *lb, int size);

int sdlog_logbuffer_is_full(struct sdlog_logbuffer *lb);

int sdlog_logbuffer_is_empty(struct sdlog_logbuffer *lb);

void sdlog_logbuffer_write(struct sdlog_logbuffer *lb, const struct sdlog_sysvector *elem);

int sdlog_logbuffer_read(struct sdlog_logbuffer *lb, struct sdlog_sysvector *elem);

#endif
