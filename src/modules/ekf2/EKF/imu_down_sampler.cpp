#include "imu_down_sampler.hpp"

ImuDownSampler::ImuDownSampler(float target_dt_sec) : _target_dt{target_dt_sec} { reset(); }

// integrate imu samples until target dt reached
// assumes that dt of the gyroscope is close to the dt of the accelerometer
// returns true if target dt is reached
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	if (_do_reset) {
		reset();
	}

	// accumulate time deltas
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
	_imu_down_sampled.time_us = imu_sample_new.time_us;
	_imu_down_sampled.delta_vel_clipping[0] += imu_sample_new.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] += imu_sample_new.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] += imu_sample_new.delta_vel_clipping[2];

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang));
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q;
	_delta_angle_accumulated.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	const Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f;

	// check if the target time delta between filter prediction steps has been exceeded
	if (_imu_down_sampled.delta_ang_dt >= _target_dt - _imu_collection_time_adj) {
		// accumulate the amount of time to advance the IMU collection time so that we meet the
		// average EKF update rate requirement
		_imu_collection_time_adj += 0.01f * (_imu_down_sampled.delta_ang_dt - _target_dt);
		_imu_collection_time_adj = math::constrain(_imu_collection_time_adj, -0.5f * _target_dt,
					   0.5f * _target_dt);

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated);

		return true;

	} else {

		return false;
	}
}

void ImuDownSampler::reset()
{
	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.delta_vel_clipping[0] = false;
	_imu_down_sampled.delta_vel_clipping[1] = false;
	_imu_down_sampled.delta_vel_clipping[2] = false;
	_delta_angle_accumulated.setIdentity();
	_do_reset = false;
}
