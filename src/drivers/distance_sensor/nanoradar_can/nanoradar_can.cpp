/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

#include "nanoradar_can.hpp"

#include <nuttx/can/can.h>
#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>

#define PI 3.141592653f

#define NANORADAR_RNG_FOV_RAD (50.0f * (PI) / 180.0f)
#define NANORADAR_OBS_FOV_DEG 120.0f

static NanoradarCan *instance = nullptr;

NanoradarCan::NanoradarCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_px4_rangefinder(0, distance_sensor_s::ROTATION_DOWNWARD_FACING)
{
	_device_id.devid_s.devtype = DRV_DIST_DEVTYPE_NANORADAR;
	_device_id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;

	_px4_rangefinder.set_device_id(_device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_NANORADAR);
}

NanoradarCan::~NanoradarCan()
{
	instance = nullptr;

	stop();

	if (_pcan != nullptr && _ifaces != nullptr) {
		_pcan->driver.updateEvent(_use_iface - 1).registerSignalCallback(NULL);
	}

	perf_free(_rng_sample_perf);
	perf_free(_obs_sample_perf);
	perf_free(_comms_errors);
}

int NanoradarCan::init()
{
	float   dist_min    = 0.0f;
	float   dist_max    = 0.0f;
	float   ang_offset  = 0.0f;
	int32_t rng_orien   = 0;

	// rangefinder parameter initialization
	param_get(param_find("SENS_NR_RNG_MIN"),  &dist_min);
	param_get(param_find("SENS_NR_RNG_MAX"),  &dist_max);
	param_get(param_find("SENS_NR_RNG_ORI"),  &rng_orien);
	_px4_rangefinder.set_min_distance(dist_min);
	_px4_rangefinder.set_max_distance(dist_max);
	_px4_rangefinder.set_orientation(rng_orien);
	_px4_rangefinder.set_fov(NANORADAR_RNG_FOV_RAD);

	// obstacale distance parameter initialization
	param_get(param_find("SENS_NR_OBS_MIN"),  &dist_min);
	param_get(param_find("SENS_NR_OBS_MAX"),  &dist_max);
	param_get(param_find("SENS_NR_OBS_OFT"),  &ang_offset);
	_obstacle_distance.angle_offset = ang_offset;
	_obstacle_distance.min_distance = dist_min * 100;
	_obstacle_distance.max_distance = dist_max * 100;
	_obstacle_distance.angle_offset = _wrap_360(-(NANORADAR_OBS_FOV_DEG / 2) + ang_offset);
	_obstacle_distance.increment    = NANORADAR_OBS_FOV_DEG / (NanoradarCan::BIN_COUNT - 1);
	_obstacle_distance.frame        = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_distance.sensor_type  = obstacle_distance_s::MAV_DISTANCE_SENSOR_RADAR;

	_interval = 3000;   // run period is 3ms

	start();

	return PX4_OK;
}

void NanoradarCan::busevent_signal_trampoline()
{
	if (instance) {
		// trigger the work queue (Note, this is called from IRQ context)
		instance->ScheduleNow();
	}
}

int NanoradarCan::collect()
{
	uavcan::CanFrame out_frame;
	uavcan::MonotonicTime out_ts_monotonic;
	uavcan::UtcTime out_ts_utc;
	uavcan::CanIOFlags out_flags;

	uint16_t len = 32;

	while (len--) {
		// try to read a frame from the CAN receive frame queue
		if (_ifaces->receive(out_frame, out_ts_monotonic, out_ts_utc, out_flags) == 0) {
			return 0;
		}

		// check the validity of CAN frame data
		if (out_frame.isErrorFrame() || out_frame.isRemoteTransmissionRequest() ||
		    (out_frame.id & 0x1FFFF800) || (out_frame.dlc != 8)) {
			perf_count(_comms_errors);
			continue;
		}

		// parse CAN frame and choose to publish rangfinde or obstacle avoidance distance
		_parse_pub(&out_frame);
	}

	// iif the obstacle avoidance data frame is incomplete,
	// the currently successfully parsed distance data will be published.
	// timeout is 5ms
	if (_obs_dist_rec_cnt > 0 && (hrt_absolute_time() - _obs_last_read_data_time >= 5000)) {
		_obstacle_distance.timestamp = _obs_last_read_data_time;
		_obs_dist_num = 0;
		_obs_dist_rec_cnt = 0;
		_obstacle_distance_pub.publish(_obstacle_distance);
		perf_end(_obs_sample_perf);

#if MAVLINK_SEND_OBS_DEBUG_ENABLE
		_obstacle_distance_fused_pub.publish(_obstacle_distance);
#endif
	}

	return 0;
}

void NanoradarCan::start()
{
	ScheduleOnInterval(_interval, 500000UL);
}

void NanoradarCan::stop()
{
	ScheduleClear();
}

void NanoradarCan::Run()
{
	if (_init == false) {
		_pcan = getCanInitHelper();

		// CanInitHelper interface validity check
		if (_pcan == nullptr) {
			perf_count(_comms_errors);
			return;
		}

		int32_t uavcan_en = 0;
		int32_t uavcan_if_en = 0;
		param_get(param_find("UAVCAN_ENABLE"),  &uavcan_en);
		param_get(param_find("UAVCAN_IF_EN"),   &uavcan_if_en);
		param_get(param_find("SENS_NR_CAN"),    &_use_iface);

		// CAN1 or CAN2
		if (_use_iface == 0 || _use_iface > 2) {
			perf_count(_comms_errors);
			return;
		}

		// need to avoid conflicts with uavcan functions
		// if uavcan use the CAN, not running
		if (uavcan_en && (_use_iface & uavcan_if_en) > 0) {
			perf_count(_comms_errors);
			return;
		}

		// CAN Baudrate
		if (uavcan_en) {
			int32_t bitrate = 1000000;
			param_get(param_find("UAVCAN_BITRATE"), &bitrate);
			_pcan->init(bitrate);

		} else {
			_pcan->init(1000000UL);
		}

		// get the lowwer CAN interface instance
		_ifaces = _pcan->driver.getIface(_use_iface - 1, true);

		if (_ifaces == nullptr) {
			perf_count(_comms_errors);
			return;
		}

		instance = this;

		// register the CAN receive interrupt callback function to trigger the current task to run once
		_pcan->driver.updateEvent(_use_iface - 1).registerSignalCallback(NanoradarCan::busevent_signal_trampoline);

		_init = true;

	} else {
		// perform collection
		collect();
	}
}

void NanoradarCan::print_info()
{
	perf_print_counter(_rng_sample_perf);
	perf_print_counter(_obs_sample_perf);
	perf_print_counter(_comms_errors);
}

uint8_t NanoradarCan::_check_sum(const uint8_t *data, uint8_t len)
{
	uint16_t sum = 0;

	while (len--) {
		sum += *data;
		++data;
	}

	return (uint8_t)(sum & 0xFF);
}

void NanoradarCan::_parse_pub(const uavcan::CanFrame *can_frame)
{
	perf_begin(_rng_sample_perf);

	if (can_frame->dlc == 8) {

		uint32_t msg_id = (can_frame->id & uavcan::CanFrame::MaskExtID);

		_device_id.devid_s.address = (msg_id >> 4) & 0x0F;

		switch (msg_id & 0xFFFFFF0F) {

		// read radar status
		case 0x60A: {
				// nnumber of single detection distances of obstacle avoidance radar
				// check obstacle avoidance radar id
				if (_obs_radar_id == _device_id.devid_s.address) {
					_obs_dist_num = can_frame->data[0];
					_obs_dist_rec_cnt = 0;
					perf_begin(_obs_sample_perf);
					memset(_obstacle_distance.distances, 0xFF, sizeof(_obstacle_distance.distances));
				}

				break;
			}

		// read obstacle avoidance distance
		case 0x60B: {
				_obs_radar_id = _device_id.devid_s.address;

				if (_obs_dist_num > 0) {
					// from Nanoradar MR72 datasheet，http://www.nanoradar.cn
					float x = ((((uint16_t)can_frame->data[2] & 0x07U) << 8) | can_frame->data[3]) * 0.2f - 204.6f;
					float y = (((uint16_t)can_frame->data[1] << 5) | (can_frame->data[2] >> 3)) * 0.2f - 500.0f;

					float dist = sqrtf(x * x + y * y);
					uint8_t orien = (math::degrees(atan2f(x, y)) + (NANORADAR_OBS_FOV_DEG / 2) +
							 _obstacle_distance.increment * 0.5f) / _obstacle_distance.increment;

					if (orien < NanoradarCan::BIN_COUNT) {
						uint16_t dist_cm = static_cast<uint16_t>(dist * 100);

						// if multiple distances near the angle, take the minimum value
						if (_obstacle_distance.distances[orien] > dist_cm) {
							_obstacle_distance.distances[orien] = dist_cm;
						}

					}

					++_obs_dist_rec_cnt;

					_obs_last_read_data_time = hrt_absolute_time();

					if (_obs_dist_rec_cnt == _obs_dist_num) {
						_obstacle_distance.timestamp = _obs_last_read_data_time;
						_obs_dist_num = 0;
						_obs_dist_rec_cnt = 0;
						_obstacle_distance_pub.publish(_obstacle_distance);
						perf_end(_obs_sample_perf);
#if MAVLINK_SEND_OBS_DEBUG_ENABLE
						_obstacle_distance_fused_pub.publish(_obstacle_distance);
#endif
					}
				}

				break;
			}

		// read rangefinder distance
		case 0x70C: {
				float dist = 0.0f;

				// none checksum
				if (_check_sum_pass_cnt == 0) {
					dist = (((uint16_t)can_frame->data[2] << 8) | can_frame->data[3]) / 100.0f;
				}

				// need to check the checksum
				else if (_check_sum_pass_cnt == 200) {
					if (_check_sum(can_frame->data, 7) == can_frame->data[7]) {
						dist = ((((uint32_t)can_frame->data[0] & 0xF0) << 12) |
							((uint32_t)can_frame->data[2] << 8) |
							can_frame->data[3]) / 100.0f;

					} else {
						break;
					}
				}

				// automatically identify radar communication protocols through verification algorithms
				else {
					if (_check_sum(can_frame->data, 7) == can_frame->data[7]) {
						++_check_sum_pass_cnt;

					} else {
						--_check_sum_pass_cnt;
					}

					break;
				}

				_px4_rangefinder.set_device_id(_device_id.devid);
				_px4_rangefinder.update(hrt_absolute_time(), dist, -1);

				perf_end(_rng_sample_perf);

				break;
			}

		default:
			break;
		}
	}
}

float NanoradarCan::_wrap_360(const float angle)
{
	float res = fmodf(angle, 360.0f);

	if (res < 0) {
		res += 360.0f;
	}

	return res;
}
