/****************************************************************************
 *
 *   Copyright (c) 2014-2022 PX4 Development Team. All rights reserved.
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
 * @file gnss.hpp
 *
 * UAVCAN <--> ORB bridge for GNSS messages:
 *     uavcan.equipment.gnss.Fix (deprecated, but still supported for backward compatibility)
 *     uavcan.equipment.gnss.Fix2
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/rtcm_data.h>
#include <uORB/topics/gps_dump.h>

#include <lib/failure_injection/FailureInjection.hpp>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Fix2.hpp>
#include <ardupilot/gnss/MovingBaselineData.hpp>
#include <ardupilot/gnss/RelPosHeading.hpp>
#include <uavcan/equipment/gnss/RTCMStream.hpp>

#include <lib/perf/perf_counter.h>

#include "sensor_bridge.hpp"

#include <clock_offset_estimator.hpp>

class UavcanGnssBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanGnssBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher);
	~UavcanGnssBridge();

	const char *get_name() const override { return NAME; }

	int init() override;
	void update() override;
	void print_status() const override;

private:
	/**
	 * GNSS fix message will be reported via this callback.
	 */
	void gnss_auxiliary_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary> &msg);
	void gnss_fix_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg);
	void gnss_fix2_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix2> &msg);
	void gnss_relative_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::gnss::RelPosHeading> &msg);
	void moving_baseline_data_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::gnss::MovingBaselineData> &msg);


	template <typename FixType>
	void process_fixx(const uavcan::ReceivedDataStructure<FixType> &msg,
			  uint8_t fix_type,
			  const float (&pos_cov)[9], const float (&vel_cov)[9],
			  const bool valid_pos_cov, const bool valid_vel_cov,
			  const float heading, const float heading_offset,
			  const float heading_accuracy, const int32_t noise_per_ms,
			  const int32_t jamming_indicator, const uint8_t jamming_state,
			  const uint8_t spoofing_state);

	void handleInjectDataTopic();
	void drainRtcmCorrections();
	void drainMovingBaseline();
	bool PublishRTCMStream(const uint8_t *data, size_t data_len);
	bool PublishMovingBaselineData(const uint8_t *data, size_t data_len);

	typedef uavcan::MethodBinder < UavcanGnssBridge *,
		void (UavcanGnssBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary> &) >
		AuxiliaryCbBinder;

	typedef uavcan::MethodBinder < UavcanGnssBridge *,
		void (UavcanGnssBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &) >
		FixCbBinder;

	typedef uavcan::MethodBinder < UavcanGnssBridge *,
		void (UavcanGnssBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix2> &) >
		Fix2CbBinder;

	typedef uavcan::MethodBinder<UavcanGnssBridge *,
		void (UavcanGnssBridge::*)(const uavcan::TimerEvent &)>
		TimerCbBinder;

	typedef uavcan::MethodBinder < UavcanGnssBridge *,
		void (UavcanGnssBridge::*)(const uavcan::ReceivedDataStructure<ardupilot::gnss::RelPosHeading> &) >
		RelPosHeadingCbBinder;

	typedef uavcan::MethodBinder < UavcanGnssBridge *,
		void (UavcanGnssBridge::*)(const uavcan::ReceivedDataStructure<ardupilot::gnss::MovingBaselineData> &) >
		MovingBaselineDataCbBinder;

	uavcan::INode &_node;

	uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary, AuxiliaryCbBinder> _sub_auxiliary;
	uavcan::Subscriber<uavcan::equipment::gnss::Fix, FixCbBinder> _sub_fix;
	uavcan::Subscriber<uavcan::equipment::gnss::Fix2, Fix2CbBinder> _sub_fix2;
	uavcan::Subscriber<ardupilot::gnss::RelPosHeading, RelPosHeadingCbBinder> _sub_gnss_heading;

	// Used for MSM7 logging for PPK workflows
	uavcan::Subscriber<ardupilot::gnss::MovingBaselineData, MovingBaselineDataCbBinder> _sub_moving_baseline_data;

	uavcan::Publisher<ardupilot::gnss::MovingBaselineData> _pub_moving_baseline_data;
	uavcan::Publisher<uavcan::equipment::gnss::RTCMStream> _pub_rtcm_stream;

	uint64_t	_last_gnss_auxiliary_timestamp{0};
	float		_last_gnss_auxiliary_hdop{0.0f};
	float		_last_gnss_auxiliary_vdop{0.0f};

	uORB::SubscriptionMultiArray<rtcm_data_s, rtcm_data_s::MAX_INSTANCES> _rtcm_corrections_sub{ORB_ID::rtcm_corrections};
	uORB::Subscription _rtcm_moving_baseline_sub{ORB_ID(rtcm_moving_baseline)};
	hrt_abstime		_last_rtcm_injection_time{0};	///< time of last rtcm injection
	uint8_t			_selected_rtcm_instance{0};	///< uorb instance that is being used for RTCM corrections

	uORB::Publication<gps_dump_s> _gps_dump_pub{ORB_ID(gps_dump)}; // For PPK

	bool _system_clock_set{false};  ///< Have we set the system clock at least once from GNSS data?

	failure_injection::Config _failure_config;
	failure_injection::Stuck<sensor_gps_s> _stuck[DEFAULT_MAX_CHANNELS];

	bool *_channel_using_fix2; ///< Flag for whether each channel is using Fix2 or Fix msg

	// Per-source-node clock-offset estimator. Slot count matches channel count.
	struct ClockEstimatorSlot {
		int node_id{-1};
		ClockOffsetEstimator estimator;
	};
	static constexpr unsigned kMaxClockEstimatorSlots = 4;
	ClockEstimatorSlot _clock_estimator_slots[kMaxClockEstimatorSlots];

	bool _publish_rtcm_stream{false};
	bool _publish_moving_baseline_data{false};

	float _rel_heading_accuracy{NAN};
	float _rel_heading{NAN};
	bool _rel_heading_valid{false};

	perf_counter_t _rtcm_stream_pub_perf{nullptr};
	perf_counter_t _moving_baseline_data_pub_perf{nullptr};
	perf_counter_t _moving_baseline_data_sub_perf{nullptr};

	hrt_abstime _last_rate_measurement{0};
	float _rtcm_injection_rate{0.f}; ///< fixed-base corrections injection rate (Hz); moving-baseline tracked via _moving_baseline_data_pub_perf
	unsigned _rtcm_injection_rate_message_count{0}; ///< fixed-base corrections messages since last rate calculation
};
