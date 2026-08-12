#pragma once

#include <cstdint>

#include <lib/matrix/matrix/math.hpp>

class VoliroControl
{
public:
	struct Configuration {
		float mass{2.94f};
		float gravity{9.81f};
		// Provisional uniformly scaled simulation inertia. Replace it with a
		// measured/CAD value before aggressive physical all-attitude flight.
		matrix::Vector3f inertia{0.07755846f, 0.07755846f, 0.14010543f};
		matrix::Vector3f position_gain{1.5f, 1.5f, 6.f};
		matrix::Vector3f velocity_gain{2.5f, 2.5f, 3.f};
		matrix::Vector3f attitude_gain{4.f, 4.f, 3.f};
		matrix::Vector3f angular_rate_gain{1.5f, 1.5f, 0.5f};
		float max_rotor_thrust{22.67f};
		float arm_radius{0.315f};
		float kappa{0.01436f};
	};

	struct State {
		matrix::Vector3f position_ned;
		matrix::Vector3f velocity_ned;
		matrix::Quatf attitude_ned_frd;
		matrix::Vector3f angular_velocity_frd;
	};

	struct Setpoint {
		matrix::Vector3f position_ned;
		matrix::Vector3f velocity_ned;
		matrix::Vector3f acceleration_ned;
		matrix::Quatf attitude_ned_frd;
		matrix::Vector3f angular_velocity_frd;
		matrix::Vector3f angular_acceleration_frd;
	};

	struct Output {
		matrix::Vector3f force_frd;
		matrix::Vector3f moment_frd;
		matrix::Vector3f thrust_normalized;
		matrix::Vector3f torque_normalized;
		matrix::Vector3f attitude_error;
		matrix::Vector3f angular_rate_error;
		bool force_limited{false};
		uint8_t torque_limited_mask{0};
		bool valid{false};
	};

	bool configure(const Configuration &configuration);
	Output calculate(const State &state, const Setpoint &setpoint) const;
	const Configuration &configuration() const { return _configuration; }
	bool configured() const { return _configured; }

	static matrix::Vector3f attitudeError(const matrix::Quatf &attitude_ned_frd,
					     const matrix::Quatf &desired_ned_frd);

private:
	Configuration _configuration{};
	bool _configured{false};
};
