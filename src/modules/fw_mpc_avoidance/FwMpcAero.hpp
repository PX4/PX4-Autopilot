#pragma once

#include <modules/simulation/simulator_sih/aero.hpp>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

/**
 * Lightweight wrapper around the SIH AeroSeg-based aerodynamics.
 * Provides total aerodynamic force/moment for a fixed-wing using wing/tail/fin/fuselage segments.
 * Frames: body FRD, inputs are normalized [-1,1] for roll/pitch/yaw deflections, and thrust [N] along +X.
 */
class FwMpcAero
{
public:
	FwMpcAero() = default;

	void set_damping(float kdv, float kdw)
	{
		_kdv = kdv;
		_kdw = kdw;
	}

	/**
	 * Compute aerodynamic force/moment in body frame.
	 * @param v_B body velocity [m/s] (FRD)
	 * @param w_B body rates [rad/s]
	 * @param altitude_m altitude above MSL [m] (used for air density)
	 * @param u normalized control inputs [roll, pitch, yaw, thrust_N]
	 * @param force_B aerodynamic force [N] in body frame
	 * @param moment_B aerodynamic moment [Nm] in body frame
	 */
	void compute(const matrix::Vector3f &v_B, const matrix::Vector3f &w_B, float altitude_m,
		     const matrix::Vector4f &u, matrix::Vector3f &force_B, matrix::Vector3f &moment_B)
	{
		const float roll_def = math::constrain(u(0), -1.f, 1.f) * FLAP_MAX;
		const float pitch_def = math::constrain(u(1), -1.f, 1.f) * FLAP_MAX;
		const float yaw_def = math::constrain(u(2), -1.f, 1.f) * FLAP_MAX;
		const float thrust = math::max(u(3), 0.f);

		_wing_l.update_aero(v_B, w_B, altitude_m, roll_def);
		_wing_r.update_aero(v_B, w_B, altitude_m, -roll_def);

		_tailplane.update_aero(v_B, w_B, altitude_m, -pitch_def, thrust);
		_fin.update_aero(v_B, w_B, altitude_m, yaw_def, thrust);
		_fuselage.update_aero(v_B, w_B, altitude_m);

		force_B = _wing_l.get_Fa() + _wing_r.get_Fa() + _tailplane.get_Fa() + _fin.get_Fa() + _fuselage.get_Fa()
			  - _kdv * v_B;
		moment_B = _wing_l.get_Ma() + _wing_r.get_Ma() + _tailplane.get_Ma() + _fin.get_Ma() + _fuselage.get_Ma()
			   - _kdw * w_B;
	}

private:
	static constexpr float SPAN = 0.86f;       // [m]
	static constexpr float MAC = 0.21f;        // [m]
	static constexpr float RP = 0.10f;         // prop radius [m]
	static constexpr float FLAP_MAX = M_PI_F / 12.f; // 15 deg

	float _kdv{1.0f};         // linear drag (N/(m/s))
	float _kdw{0.025f};       // angular damper (Nm/(rad/s))

	AeroSeg _wing_l{SPAN / 2.0f, MAC, -4.0f, matrix::Vector3f(0.0f, -SPAN / 4.0f, 0.0f), 3.0f,
			SPAN / MAC, MAC / 3.0f};
	AeroSeg _wing_r{SPAN / 2.0f, MAC, -4.0f, matrix::Vector3f(0.0f, SPAN / 4.0f, 0.0f), -3.0f,
			SPAN / MAC, MAC / 3.0f};
	AeroSeg _tailplane{0.3f, 0.1f, 0.0f, matrix::Vector3f(-0.4f, 0.0f, 0.0f), 0.0f, -1.0f, 0.05f, RP};
	AeroSeg _fin{0.25f, 0.18f, 0.0f, matrix::Vector3f(-0.45f, 0.0f, -0.1f), -90.0f, -1.0f, 0.12f, RP};
	AeroSeg _fuselage{0.2f, 0.8f, 0.0f, matrix::Vector3f(0.0f, 0.0f, 0.0f), -90.0f};
};
