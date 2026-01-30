#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include "FwMpcAero.hpp"

class FixedWingMpcModel
{
public:
	static constexpr int kStateSize = 12;
	static constexpr int kControlSize = 4;

	using State = matrix::Vector<float, kStateSize>;
	using Control = matrix::Vector4f;

	FixedWingMpcModel() = default;

	State dynamics(const State &x, const Control &u) const
	{
		const matrix::Vector3f uvw{x(0), x(1), x(2)};
		const matrix::Vector3f pqr{x(3), x(4), x(5)};
		const float phi = x(6);
		const float theta = x(7);
		const float psi = x(8);

		matrix::Vector3f Fa_body{};
		matrix::Vector3f Ma_body{};
		const float altitude_up = math::max(x(11), 0.f); // controller stores z_up
		_aero.compute(uvw, pqr, altitude_up, u, Fa_body, Ma_body);

		const matrix::Dcmf Rbi = rotationMatrix(phi, theta, psi);
		const matrix::Vector3f Fg_body = Rbi.transpose() * matrix::Vector3f{0.f, 0.f, -_mass * _g};

		const matrix::Vector3f thrust_B{u(3), 0.f, 0.f};
		const matrix::Vector3f F_body = thrust_B + Fa_body + Fg_body;

		const matrix::Vector3f uvw_dot = F_body / _mass - pqr.cross(uvw);
		const matrix::Vector3f pqrdot = _I_inv * (Ma_body - pqr.cross(_I * pqr));

		// Euler rates
		const float ct = cosf(theta);
		const float st = sinf(theta);
		const float ct_safe = (fabsf(ct) > 1e-3f) ? ct : copysignf(1e-3f, ct);
		const float tt = st / ct_safe;

		matrix::Matrix<float, 3, 3> E;
		E(0, 0) = 1.f;  E(0, 1) = sinf(phi) * tt;     E(0, 2) = cosf(phi) * tt;
		E(1, 0) = 0.f;  E(1, 1) = cosf(phi);         E(1, 2) = -sinf(phi);
		E(2, 0) = 0.f;  E(2, 1) = sinf(phi) / fabsf(ct_safe); E(2, 2) = cosf(phi) / fabsf(ct_safe);

		const matrix::Vector3f eul_dot = E * pqr;
		const matrix::Vector3f pos_dot = Rbi * uvw; // NED position rates

		State dx;
		dx(0) = uvw_dot(0);
		dx(1) = uvw_dot(1);
		dx(2) = uvw_dot(2);
		dx(3) = pqrdot(0);
		dx(4) = pqrdot(1);
		dx(5) = pqrdot(2);
		dx(6) = eul_dot(0);
		dx(7) = eul_dot(1);
		dx(8) = eul_dot(2);
		dx(9) = pos_dot(0);
		dx(10) = pos_dot(1);
		dx(11) = pos_dot(2);
		return dx;
	}

	State rk4_step(const State &x0, const Control &u, float dt) const
	{
		const State k1 = dynamics(x0, u);
		const State k2 = dynamics(x0 + 0.5f * dt * k1, u);
		const State k3 = dynamics(x0 + 0.5f * dt * k2, u);
		const State k4 = dynamics(x0 + dt * k3, u);
		return x0 + (dt / 6.f) * (k1 + 2.f * k2 + 2.f * k3 + k4);
	}

	// Basic getters used by the MPC controller for feed-forward references.
	float mass() const { return _mass; }
	float gravity() const { return _g; }
	float rho() const { return _rho; }
	float wing_area() const { return _S; }
	float wing_span() const { return _b; }
	float mean_chord() const { return _c; }
	float CL0() const { return _CL0; }
	float CL_alpha() const { return _CL_alpha; }
	float CD0() const { return _CD0; }
	float induced_drag_k() const { return _k; }
	const matrix::Matrix3f &inertia() const { return _I; }

	void set_mass(float m) { _mass = math::max(m, 0.1f); }
	void set_inertia_diag(const matrix::Vector3f &diag)
	{
		matrix::Vector3f d = diag.emult(matrix::Vector3f{1.f, 1.f, 1.f});
		d(0) = math::max(d(0), 1e-4f);
		d(1) = math::max(d(1), 1e-4f);
		d(2) = math::max(d(2), 1e-4f);
		_I = matrix::diag(d);
		_I_inv = matrix::diag(matrix::Vector3f{1.f / d(0), 1.f / d(1), 1.f / d(2)});
	}
	void set_damping(float kdv, float kdw) { _aero.set_damping(kdv, kdw); }

private:
	matrix::Dcmf rotationMatrix(float phi, float theta, float psi) const
	{
		const float cP = cosf(phi);
		const float sP = sinf(phi);
		const float cT = cosf(theta);
		const float sT = sinf(theta);
		const float cS = cosf(psi);
		const float sS = sinf(psi);
		matrix::Dcmf R;
		R(0, 0) = cT * cS;
		R(0, 1) = sP * sT * cS - cP * sS;
		R(0, 2) = cP * sT * cS + sP * sS;
		R(1, 0) = cT * sS;
		R(1, 1) = sP * sT * sS + cP * cS;
		R(1, 2) = cP * sT * sS - sP * cS;
		R(2, 0) = -sT;
		R(2, 1) = sP * cT;
		R(2, 2) = cP * cT;
		return R;
	}

	float _mass = 2.5f;
	matrix::Matrix3f _I{matrix::diag(matrix::Vector3f{0.20f, 0.30f, 1.00f})};
	matrix::Matrix3f _I_inv{matrix::diag(matrix::Vector3f{1.f / 0.20f, 1.f / 0.30f, 1.f / 1.00f})};
	const float _S = 0.50f;
	const float _b = 2.00f;
	const float _c = 0.25f;
	const float _rho = 1.225f;
	const float _g = 9.81f;
	const float _CL0 = 0.30f;
	const float _CL_alpha = 4.5f;
	const float _CD0 = 0.035f;
	const float _k = 0.040f;
	mutable FwMpcAero _aero{};
};
