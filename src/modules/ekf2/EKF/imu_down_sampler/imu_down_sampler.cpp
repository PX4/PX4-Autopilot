#include "imu_down_sampler/imu_down_sampler.hpp"

#include <lib/mathlib/mathlib.h>

ImuDownSampler::ImuDownSampler(int32_t &target_dt_us) : _target_dt_us(target_dt_us)
{
	reset();
}

bool ImuDownSampler::update(const imuSample &imu)
{
	_delta_ang_dt_avg = 0.9f * _delta_ang_dt_avg + 0.1f * imu.delta_ang_dt;

	// accumulate time deltas
	_imu_down_sampled.time_us = imu.time_us;
	_imu_down_sampled.delta_ang += imu.delta_ang;
	_imu_down_sampled.delta_vel += imu.delta_vel;
	_imu_down_sampled.delta_ang_dt += imu.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu.delta_vel_dt;
	_imu_down_sampled.delta_vel_clipping[0] |= imu.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] |= imu.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] |= imu.delta_vel_clipping[2];

	_accumulated_samples++;

	// required number of samples accumulated and the total time is at least half of the target
	//  OR total time already exceeds the target

	// target dt in seconds safely constrained
	const float constrained_target_dt_s = math::constrain(_target_dt_us, (int32_t)1000, (int32_t)100000) * 1e-6f;
	const int required_samples = math::max((int)roundf(constrained_target_dt_s / _delta_ang_dt_avg), 1);

	float target_dt_s = required_samples * _delta_ang_dt_avg;

	// minimum delta angle dt (in addition to number of samples)
	float min_dt_s = math::max(_delta_ang_dt_avg * (required_samples - 1.f), _delta_ang_dt_avg * 0.5f);

	if ((_accumulated_samples >= required_samples && _imu_down_sampled.delta_ang_dt > min_dt_s)
	    || (_imu_down_sampled.delta_ang_dt > target_dt_s)
	) {

		return true;
	}

	return false;
}

void ImuDownSampler::reset()
{
	_imu_down_sampled = {};
	_accumulated_samples = 0;
}
