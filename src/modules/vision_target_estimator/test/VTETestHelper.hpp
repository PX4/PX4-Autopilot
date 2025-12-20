/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file VTETestHelper.hpp
 * @brief Helper functions for Vision Target Estimator unit tests
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/fiducial_marker_yaw_report.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/target_gnss.h>
#include <matrix/Quaternion.hpp>
#include <matrix/Vector.hpp>
#include <drivers/drv_hrt.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace vte_test
{

class ParamGuard
{
public:
	ParamGuard() = default;
	~ParamGuard() { restore(); }

	bool setFloat(const char *name, float value)
	{
		param_t handle = param_find(name);

		if (handle == PARAM_INVALID) {
			return false;
		}

		if (param_type(handle) != PARAM_TYPE_FLOAT) {
			return false;
		}

		if (!storeOriginal(handle)) {
			return false;
		}

		return param_set(handle, &value) == 0;
	}

	bool setInt(const char *name, int32_t value)
	{
		param_t handle = param_find(name);

		if (handle == PARAM_INVALID) {
			return false;
		}

		if (param_type(handle) != PARAM_TYPE_INT32) {
			return false;
		}

		if (!storeOriginal(handle)) {
			return false;
		}

		return param_set(handle, &value) == 0;
	}

	void restore()
	{
		for (auto it = _entries.rbegin(); it != _entries.rend(); ++it) {
			if (it->type == PARAM_TYPE_FLOAT) {
				(void)param_set(it->handle, &it->value.f);

			} else if (it->type == PARAM_TYPE_INT32) {
				(void)param_set(it->handle, &it->value.i);
			}
		}

		_entries.clear();
	}

private:
	struct ParamEntry {
		param_t handle{PARAM_INVALID};
		param_type_t type{PARAM_TYPE_INT32};

		union {
			float f;
			int32_t i;
		} value{};
	};

	std::vector<ParamEntry> _entries{};

	bool storeOriginal(param_t handle)
	{
		for (const auto &entry : _entries) {
			if (entry.handle == handle) {
				return true;
			}
		}

		ParamEntry entry{};
		entry.handle = handle;
		entry.type = param_type(handle);

		if (entry.type == PARAM_TYPE_FLOAT) {
			if (param_get(handle, &entry.value.f) != 0) {
				return false;
			}

		} else if (entry.type == PARAM_TYPE_INT32) {
			if (param_get(handle, &entry.value.i) != 0) {
				return false;
			}

		} else {
			return false;
		}

		_entries.push_back(entry);
		return true;
	}
};

inline matrix::Quaternionf identityQuat()
{
	matrix::Quaternionf q{1.f, 0.f, 0.f, 0.f};
	return q;
}

template<typename Subscription>
void flushSubscription(Subscription &sub)
{
	while (sub.update()) {}
}

template<typename Subscription>
void flushSubscription(std::unique_ptr<Subscription> &sub)
{
	if (!sub) {
		return;
	}

	while (sub->update()) {}
}

template<typename Msg, typename Subscription>
void flushSubscription(Subscription &sub)
{
	Msg msg{};

	while (sub.update(&msg)) {}
}

inline hrt_abstime advanceMicroseconds(hrt_abstime delta_us)
{
	if (delta_us > 0) {
		px4_usleep(static_cast<useconds_t>(delta_us));
	}

	return hrt_absolute_time();
}

inline hrt_abstime nowUs()
{
	return hrt_absolute_time();
}

inline bool publishVisionYaw(uORB::Publication<fiducial_marker_yaw_report_s> &pub,
			     float yaw, float yaw_var, hrt_abstime timestamp)
{
	fiducial_marker_yaw_report_s msg{};
	msg.timestamp = timestamp;
	msg.timestamp_sample = timestamp;
	msg.yaw_ned = yaw;
	msg.yaw_var_ned = yaw_var;
	return pub.publish(msg);
}

inline bool publishVisionPos(uORB::Publication<fiducial_marker_pos_report_s> &pub,
			     const matrix::Vector3f &rel_pos, const matrix::Quaternionf &q,
			     const matrix::Vector3f &cov, hrt_abstime timestamp)
{
	fiducial_marker_pos_report_s msg{};
	msg.timestamp = timestamp;
	msg.timestamp_sample = timestamp;
	msg.rel_pos[0] = rel_pos(0);
	msg.rel_pos[1] = rel_pos(1);
	msg.rel_pos[2] = rel_pos(2);
	msg.cov_rel_pos[0] = cov(0);
	msg.cov_rel_pos[1] = cov(1);
	msg.cov_rel_pos[2] = cov(2);
	q.copyTo(msg.q);
	return pub.publish(msg);
}

inline bool publishUavGps(uORB::Publication<sensor_gps_s> &pub, double lat, double lon, float alt,
			  float eph, float epv, const matrix::Vector3f &vel_ned, float vel_var,
			  bool vel_valid, hrt_abstime timestamp)
{
	sensor_gps_s msg{};
	msg.timestamp = timestamp;
	msg.timestamp_sample = timestamp;
	msg.latitude_deg = lat;
	msg.longitude_deg = lon;
	msg.altitude_msl_m = alt;
	msg.eph = eph;
	msg.epv = epv;
	msg.vel_n_m_s = vel_ned(0);
	msg.vel_e_m_s = vel_ned(1);
	msg.vel_d_m_s = vel_ned(2);
	msg.s_variance_m_s = vel_var;
	msg.vel_ned_valid = vel_valid;
	return pub.publish(msg);
}

inline bool publishTargetGnss(uORB::Publication<target_gnss_s> &pub, double lat, double lon, float alt,
			      float eph, float epv, hrt_abstime timestamp, bool abs_pos_updated,
			      bool vel_ned_updated = false, const matrix::Vector3f &vel_ned = matrix::Vector3f{},
			      float vel_acc = NAN)
{
	target_gnss_s msg{};
	msg.timestamp = timestamp;
	msg.timestamp_sample = timestamp;
	msg.latitude_deg = lat;
	msg.longitude_deg = lon;
	msg.altitude_msl_m = alt;
	msg.eph = eph;
	msg.epv = epv;
	msg.abs_pos_updated = abs_pos_updated;
	msg.vel_ned_updated = vel_ned_updated;
	msg.vel_n_m_s = vel_ned(0);
	msg.vel_e_m_s = vel_ned(1);
	msg.vel_d_m_s = vel_ned(2);
	msg.s_acc_m_s = vel_acc;
	return pub.publish(msg);
}

} // namespace vte_test
