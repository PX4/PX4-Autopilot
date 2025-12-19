#pragma once

#include <parameters/param.h>
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

class SimTime
{
public:
	explicit SimTime(hrt_abstime start_us = 0) : _timestamp(start_us)
	{
	}

	void reset(hrt_abstime start_us = 0)
	{
		_timestamp = start_us;
	}

	hrt_abstime now() const
	{
		return _timestamp;
	}

	hrt_abstime advanceMicroseconds(hrt_abstime delta_us)
	{
		_timestamp += delta_us;
		return _timestamp;
	}

private:
	hrt_abstime _timestamp{0};
};

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
