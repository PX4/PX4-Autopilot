#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

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
		const float u_b = x(0);
		const float v_b = x(1);
		const float w_b = x(2);
		const float p = x(3);
		const float q = x(4);
		const float r = x(5);
		const float phi = x(6);
		const float theta = x(7);
		const float psi = x(8);

		float V = math::max(sqrtf(u_b * u_b + v_b * v_b + w_b * w_b), 1e-3f);
		float alpha = atanf2(w_b, u_b);
		float beta = asinf(math::constrain(v_b / V, -1.f, 1.f));

		alpha = math::constrain(alpha, -_alpha_max, _alpha_max);
		beta = math::constrain(beta, -_beta_max, _beta_max);

		const float q_dyn = 0.5f * _rho * V * V;

		float CL = _CL0 + _CL_alpha * alpha;
		CL = math::constrain(CL, -_CL_max, _CL_max);
		const float CD = _CD0 + _k * CL * CL;

		const float L = q_dyn * _S * CL;
		const float D = q_dyn * _S * CD;
		const float CY = _CY_beta * beta;
		const float Y = q_dyn * _S * CY;

		const float Xa = -D * cosf(alpha) + L * sinf(alpha);
		const float Za = -D * sinf(alpha) - L * cosf(alpha);

		const matrix::Dcmf Rbi = rotationMatrix(phi, theta, psi);
		const matrix::Vector3f Fg_body = Rbi.transpose() * matrix::Vector3f{0.f, 0.f, -_mass * _g};

		matrix::Vector3f F_body{Xa + u(3), Y, Za};
		F_body += Fg_body;

		const matrix::Vector3f uvw{u_b, v_b, w_b};
		const matrix::Vector3f pqr{p, q, r};

		const matrix::Vector3f uvw_dot = F_body / _mass - pqr.cross(uvw);

		// Control moments scaled with q_dyn and deflections (normalized to [-1,1])
		const matrix::Vector3f delta = matrix::Vector3f{u(0), u(1), u(2)}.emult(_defl_max);
		const matrix::Matrix3f M_ctrl_mat{
			_S * _b * _Cl_da, 0.f, 0.f,
			0.f, _S * _c * _Cm_de, 0.f,
			0.f, 0.f, _S * _b * _Cn_dr
		};
		const matrix::Vector3f M_ctrl = q_dyn * (M_ctrl_mat * delta);
		const matrix::Vector3f M_damp = _D_rot * pqr;
		const matrix::Vector3f M_body = M_ctrl - M_damp;

		const matrix::Vector3f pqrdot = _I_inv * (M_body - pqr.cross(_I * pqr));

		// Euler rates
		const float ct = cosf(theta);
		const float st = sinf(theta);
		const float tt = ct != 0.f ? st / ct : 0.f;

		matrix::Matrix<float, 3, 3> E;
		E(0, 0) = 1.f;  E(0, 1) = sinf(phi) * tt;     E(0, 2) = cosf(phi) * tt;
		E(1, 0) = 0.f;  E(1, 1) = cosf(phi);         E(1, 2) = -sinf(phi);
		E(2, 0) = 0.f;  E(2, 1) = sinf(phi) / math::max(fabsf(ct), 1e-3f); E(2, 2) = cosf(phi) / math::max(fabsf(ct), 1e-3f);

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

private:
	matrix::Dcmf rotationMatrix(float phi, float theta, float psi) const
	{
		const float cP = cosf(phi);
		const float sP = sinf(phi);
		const float cT = cosf(theta);
		const float sT = sinf(theta);
		const float cS = cosf(psi);
		const float sS = sinf(psi);
		return matrix::Dcmf{
			cT * cS, sP * sT * cS - cP * sS, cP * sT * cS + sP * sS,
			cT * sS, sP * sT * sS + cP * cS, cP * sT * sS - sP * cS,
			-sT, sP * cT, cP * cT
		};
	}

	const float _mass = 2.5f;
	const matrix::Matrix3f _I{matrix::diag(matrix::Vector3f{0.20f, 0.30f, 1.00f})};
	const matrix::Matrix3f _I_inv{matrix::diag(matrix::Vector3f{1.f / 0.20f, 1.f / 0.30f, 1.f / 1.00f})};
	const float _S = 0.50f;
	const float _b = 2.00f;
	const float _c = 0.25f;
	const float _rho = 1.225f;
	const float _g = 9.81f;
	const float _CL0 = 0.30f;
	const float _CL_alpha = 4.5f;
	const float _CD0 = 0.035f;
	const float _k = 0.040f;
	const float _CY_beta = -0.9f;
	const float _CL_max = 1.4f;
	const float _alpha_max = 25.f * M_PI_F / 180.f;
	const float _beta_max = 25.f * M_PI_F / 180.f;
	const matrix::Matrix3f _D_rot{matrix::diag(matrix::Vector3f{0.05f, 0.07f, 0.05f})};
	const matrix::Vector3f _defl_max{25.f * M_PI_F / 180.f, 25.f * M_PI_F / 180.f, 25.f * M_PI_F / 180.f};
	const float _Cl_da = 0.08f;
	const float _Cm_de = -0.50f;
	const float _Cn_dr = 0.06f;
};
