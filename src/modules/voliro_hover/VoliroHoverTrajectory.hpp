#pragma once

#include <cstdint>
#include <matrix/matrix/math.hpp>

class VoliroHoverTrajectory
{
public:
	enum class Phase : uint8_t {
		Inactive = 0,
		Ready,
		Launch,
		Takeoff,
		Hold,
		Land,
		Landed
	};

	struct Setpoint {
		matrix::Vector3f position_ned;
		matrix::Vector3f velocity_ned;
		matrix::Vector3f acceleration_ned;
		matrix::Quatf attitude_ned_frd;
		matrix::Vector3f angular_velocity_frd;
	};

	bool configure(float height, float launch_acceleration, float touchdown_acceleration,
		       float takeoff_duration, float land_duration);
	void reset(const matrix::Vector3f &ground_position_ned, float yaw_ned);
	bool beginLaunch();
	bool startTakeoff(uint64_t now_us, float current_z_ned);
	bool startLand(uint64_t now_us, float current_z_ned);
	Setpoint update(uint64_t now_us);

	Phase phase() const { return _phase; }
	bool initialized() const { return _initialized; }
	float targetHeight() const { return _height; }
	const matrix::Vector3f &groundPosition() const { return _ground_position_ned; }

private:
	struct Profile {
		float position;
		float velocity;
		float acceleration;
	};

	static Profile minimumJerk(float elapsed, float duration);

	float _height{0.5f};
	float _launch_acceleration{0.8f};
	float _touchdown_acceleration{9.81f};
	float _takeoff_duration{5.f};
	float _land_duration{5.f};
	matrix::Vector3f _ground_position_ned;
	float _yaw_ned{0.f};
	float _takeoff_start_z{0.f};
	float _land_start_z{0.f};
	uint64_t _phase_start{0};
	Phase _phase{Phase::Inactive};
	bool _initialized{false};
};
