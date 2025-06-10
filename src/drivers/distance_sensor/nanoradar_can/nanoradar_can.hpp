/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file nanoradar_can.hpp
 * @author OuYangLei <ouyanglei@nanoradar.cn / ouyanglei92@163.com>
 *
 * Driver for the Nanoradar mmWAVE Radar(CAN Interface)(www.nanoradar.com)
 * Support the Rangfinder and Obstacle Distance
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#if defined(UAVCAN_SOCKETCAN_NUTTX)
#  include <uavcan_nuttx/uavcan_nuttx.hpp>
#elif defined(UAVCAN_KINETIS_NUTTX)
#  include <uavcan_kinetis/uavcan_kinetis.hpp>
#elif defined(UAVCAN_STM32_NUTTX)
#  include <uavcan_stm32/uavcan_stm32.hpp>
#elif defined(UAVCAN_STM32H7_NUTTX)
#  include <uavcan_stm32h7/uavcan_stm32h7.hpp>
#else
#  error "Unsupported driver"
#endif

extern UAVCAN_DRIVER::CanInitHelper<21> *getCanInitHelper(void);

/** obstacale distance send via Mavlink，When debugging use */
#define MAVLINK_SEND_OBS_DEBUG_ENABLE 1

class NanoradarCan : public px4::ScheduledWorkItem
{
public:
	NanoradarCan(void);
	~NanoradarCan() override;

	int                             init();
	void                            print_info();

	static void                     busevent_signal_trampoline();

private:
	void				start();
	void				stop();
	void				Run() override;
	int                             collect();

	uint8_t                         _check_sum(const uint8_t *data, uint8_t len);
	void                            _parse_pub(const uavcan::CanFrame *can_frame);
	float                           _wrap_360(const float angle);

	UAVCAN_DRIVER::CanInitHelper<(unsigned)21>	*_pcan{nullptr};
	uavcan::ICanIface            	*_ifaces{nullptr};

	obstacle_distance_s                    _obstacle_distance{};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{ORB_ID(obstacle_distance)};

#if MAVLINK_SEND_OBS_DEBUG_ENABLE
	uORB::Publication<obstacle_distance_s> _obstacle_distance_fused_pub {ORB_ID(obstacle_distance_fused)};
#endif

	static constexpr uint8_t        BIN_COUNT = sizeof(obstacle_distance_s::distances) / sizeof(
				obstacle_distance_s::distances[0]);

	PX4Rangefinder                  _px4_rangefinder;

	device::Device::DeviceId        _device_id;

	int         		        _interval{3000};

	uint8_t                         _check_sum_pass_cnt{100};

	uint8_t                         _obs_radar_id{255};
	uint8_t                         _obs_dist_num{0};
	uint8_t                         _obs_dist_rec_cnt{0};

	hrt_abstime                     _obs_last_read_data_time;

	int32_t                         _use_iface{2};

	bool                            _init{false};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _rng_sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read RNG")};
	perf_counter_t _obs_sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read OBS")};
};
