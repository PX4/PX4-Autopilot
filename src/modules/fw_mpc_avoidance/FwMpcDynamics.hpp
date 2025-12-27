#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/atmosphere/atmosphere.h>

/**
 * Minimal fixed-wing point-mass + attitude dynamics adapted from simulator_sih.
 * Intended for internal MPC rollouts; not a full fidelity replacement.
 */
class FwMpcDynamics
{
public:
	using State = struct {
		matrix::Vector3f position_N{0.f, 0.f, 0.f}; // NED position [m]
		matrix::Vector3f velocity_N{0.f, 0.f, 0.f}; // NED velocity [m/s]
		matrix::Quatf    q_nb{1.f, 0.f, 0.f, 0.f};  // rotation body->NED
		matrix::Vector3f omega_B{0.f, 0.f, 0.f};    // body rates [rad/s]
	};

	void reset(const State &state) { _state = state; }
	const State &state() const { return _state; }

	/**
	 * Propagate dynamics one step.
	 * @param thrust_body Forces in body frame [N]
	 * @param moments_body Moments in body frame [Nm]
	 * @param wind_B Wind in body frame [m/s]
	 * @param dt Time step [s]
	 */
	void propagate(const matrix::Vector3f &thrust_body, const matrix::Vector3f &moments_body,
		       const matrix::Vector3f &wind_B, float dt)
	{
		// gravity in NED (positive down)
		const matrix::Vector3f g_N{0.f, 0.f, CONSTANTS_ONE_G};
		const matrix::Quatf &q = _state.q_nb;
		const matrix::Dcmf R_nb{q};
		const matrix::Dcmf R_bn{R_nb.transpose()};

		// Simple aerodynamic drag proportional to squared body-relative airspeed.
		const matrix::Vector3f v_B = R_bn * _state.velocity_N - wind_B;
		const matrix::Vector3f v_B_abs = matrix::Vector3f(fabsf(v_B(0)), fabsf(v_B(1)), fabsf(v_B(2)));
		matrix::Vector3f drag_B = v_B_abs.emult(v_B) * -_k_drag;

		// Sum forces in body, then transform to NED.
		matrix::Vector3f force_B = thrust_body + drag_B;
		const matrix::Vector3f force_N = R_nb * force_B + g_N * _mass;

		const matrix::Vector3f acceleration_N = force_N / _mass;
		const matrix::Vector3f omega = _state.omega_B;
		matrix::Vector3f omega_dot = _I_inv * (moments_body - omega.cross(_I * omega));

		_state.velocity_N += acceleration_N * dt;
		_state.position_N += _state.velocity_N * dt;

		const matrix::Quatf dq{matrix::AxisAnglef(omega * dt)};
		_state.q_nb = (_state.q_nb * dq).normalized();
		_state.omega_B = omega + omega_dot * dt;
	}

private:
	static constexpr float _mass{1.0f}; // [kg] placeholder, MPC can override if needed
	// Diagonal inertia, using SIH-style constants as a lightweight default.
	static const matrix::Vector3f _I_diag;
	static const matrix::SquareMatrix<float, 3> _I;
	static const matrix::SquareMatrix<float, 3> _I_inv;
	static constexpr float _k_drag{0.05f};

	State _state{};
};
