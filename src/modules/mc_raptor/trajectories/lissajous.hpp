#pragma once

#include "trajectory.hpp"
#include <math.h>

struct LissajousParameters {
	float A = 1.0f;
	float B = 0.5f;
	float C = 0.0f;
	float z_offset = 1.0f;
	float duration = 10.0f;
	float ramp_duration = 3.0f;
	float a = 1.0f;
	float b = 2.0f;
	float c = 1.0f;
};

inline Setpoint lissajous(float time, const LissajousParameters &params) {
	float time_velocity = (params.ramp_duration > 0.0f)
		? fminf(time, params.ramp_duration) / params.ramp_duration
		: 1.0f;

	float ramp_time = time_velocity * fminf(time, params.ramp_duration) / 2.0f;
	float progress = (ramp_time + fmaxf(0.0f, time - params.ramp_duration)) * 2.0f * static_cast<float>(M_PI) / params.duration;
	float d_progress = 2.0f * static_cast<float>(M_PI) * time_velocity / params.duration;

	Setpoint setpoint{};
	setpoint.position[0] = params.A * sinf(params.a * progress);
	setpoint.position[1] = params.B * sinf(params.b * progress);
	setpoint.position[2] = params.z_offset + (params.C * sinf(params.c * progress));
	setpoint.yaw = 0.0f;
	setpoint.linear_velocity[0] = params.A * cosf(params.a * progress) * params.a * d_progress;
	setpoint.linear_velocity[1] = params.B * cosf(params.b * progress) * params.b * d_progress;
	setpoint.linear_velocity[2] = params.C * cosf(params.c * progress) * params.c * d_progress;
	setpoint.yaw_rate = 0.0f;

	return setpoint;
}
