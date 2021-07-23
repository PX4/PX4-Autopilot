/****************************************************************************
*
*   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

// The aerodynamic model is from [2]
// [2] Khan, Waqas, supervised by Meyer Nahon "Dynamics modeling of agile fixed-wing unmanned aerial vehicles."
//     McGill University, PhD thesis, 2016. Sections 3.1, 3.2, and 3.3

#pragma once

#include <matrix/matrix/math.hpp>   	// matrix, vectors, dcm, quaterions
#include <conversion/rotation.h>    	// math::radians,
// #include <lib/mathlib/mathlib.h>
#include <math.h>

// class Aerodynamic Segment ------------------------------------------------------------------------
class AeroSeg
{
private:
	// Table 3.1 SEMI-EMPIRICAL COEFFICIENTS FOR RECTANGULAR FLAT PLATES
	static constexpr const int N_TAB = 12;
	// static constexpr const float AR_tab[N_TAB] = {0.1666f,0.333f,0.4f,0.5f,1.0f,1.25f, 2.0f, 3.0f, 4.0f, 6.0f};
	// static constexpr const float ale_tab[N_TAB] = {3.00f,3.64f,4.48f,7.18f,10.20f,13.38f,14.84f,14.49f,9.95f,12.93f,15.00f,15.00f};
	// static constexpr const float ate_tab[N_TAB] = {5.90f,15.51f,32.57f,39.44f,48.22f,59.29f,21.55f,7.74f,7.05f,5.26f,6.50f,6.50f};
	// static constexpr const float afle_tab[N_TAB] = {59.00f,58.60f,58.20f,50.00f,41.53f,26.70f,23.44f,21.00f,18.63f,14.28f,11.60f,10.00f};
	// static constexpr const float afte_tab[N_TAB] = {59.00f,58.60f,58.20f,51.85f,41.46f,28.09f,39.40f,35.86f,26.76f,19.76f,16.43f,14.00f};
	// static constexpr const float afs_tab[N_TAB] = {49.00f,54.00f,56.00f,48.00f,40.00f,29.00f,27.00f,25.00f,24.00f,22.00f,22.00f,20.00f};

	// deflection effectiveness curve fitted as second order
	static constexpr const float eta_poly[] = {0.0535f, -0.2688f, 0.5817f}; 	// 1/rad

	// aerodynamic and physical constants
	static constexpr const float P0 = 101325.0f;	// pressure at sea level [N/m^2]=[Pa]
	static constexpr const float R = 287.04f;		// real gas constant for air [J/kg/K]
	static constexpr const float T0_K = 288.15f;	// temperature at sea level [K]
	static constexpr const float TEMP_GRADIENT  = -6.5e-3f;    // temperature gradient in degrees per metre

	static constexpr const float KV = M_PI_F; 	// total vortex lift parameter
	static constexpr float CD0 = 0.04f; 		// no lift drag coefficient
	static constexpr float CD90 = 1.98f; 		// 90 deg angle of attack drag coefficient
	static constexpr float AF_DOT_MAX = M_PI_F / 2.0f;

	// here we make the distinction of the plate (i.e. wing, or tailplane, or fin) and the segment
	// the segment can be a portion of the wing, but the aspect ratio (AR) of the wing needs to be used
	float alpha; 		// angle of attack [rad]
	float CL, CD, CM; 	// aerodynamic coefficients
	matrix::Vector3f p_B; 	// position of the aerodynamic center of the segment from CM in body frame [m]
	matrix::Dcmf C_BS;	// dcm from segment frame to body frame
	float ar;		// aspect ratio of the plate
	float span;		// span of the segment
	float mac;		// mean aerodynamic chord of the segment
	float alpha_0;		// zero lift angle of attack [rad]
	float kp, kn;
	float ate, ale, afte, afle, afs_rad;	// semi empirical coefficients for flat plates function of AR
	float tau_te, tau_le, fte, fle; 	// leading and trailing edge functions
	float rho = 1.225f; 	// air density at current altitude [kg/m^3]
	// variables for flap model
	float eta_f;		// flap effectiveness
	float def_a;		// absolute value of the deflection angle
	float cf;		// flap chord (control surface chord length)
	float theta_f, tau_f;	// check 3.2.3 in [2]
	float deltaCL, dCLmax;	// increase in lift coefficient
	float CLmax, CLmin;	// max and min lift value
	float alpha_min, alpha_max;	// min and max aoa in the low aoa regime
	float alf0eff;		// effective zero lift angle of attack
	float alfmeff;		// effective maximum lift angle of attack
	float alpha_eff;	// effectie angle of attack
	float alpha_eff_dot;	// effectie angle of attack derivative
	float alpha_eff_old;	// angle of attack [rad]
	float pressure; 	// pressure in Pa at current altitude
	float temperature;	// temperature in K at current altitude

public:

	matrix::Vector3f Fa;	// aerodynamic force
	matrix::Vector3f Ma;	// aerodynamic moment computed at CM directly

	AeroSeg() = default;

	// public explicit constructor
	// if the aspect ratio is negative, the aspect ratio is computed from the span and MAC
	explicit AeroSeg(float span_, float mac_, float alpha_0_, matrix::Vector3f p_B_, float dihedral_deg = 0.0f,
			 float AR = -1.0f, float cf_ = 0.0f)
	{
		static const float AR_tab[N_TAB] = {0.1666f, 0.333f, 0.4f, 0.5f, 1.0f, 1.25f, 2.0f, 3.0f, 4.0f, 6.0f};
		static const float ale_tab[N_TAB] = {3.00f, 3.64f, 4.48f, 7.18f, 10.20f, 13.38f, 14.84f, 14.49f, 9.95f, 12.93f, 15.00f, 15.00f};
		static const float ate_tab[N_TAB] = {5.90f, 15.51f, 32.57f, 39.44f, 48.22f, 59.29f, 21.55f, 7.74f, 7.05f, 5.26f, 6.50f, 6.50f};
		static const float afle_tab[N_TAB] = {59.00f, 58.60f, 58.20f, 50.00f, 41.53f, 26.70f, 23.44f, 21.00f, 18.63f, 14.28f, 11.60f, 10.00f};
		static const float afte_tab[N_TAB] = {59.00f, 58.60f, 58.20f, 51.85f, 41.46f, 28.09f, 39.40f, 35.86f, 26.76f, 19.76f, 16.43f, 14.00f};
		static const float afs_tab[N_TAB] = {49.00f, 54.00f, 56.00f, 48.00f, 40.00f, 29.00f, 27.00f, 25.00f, 24.00f, 22.00f, 22.00f, 20.00f};

		span = span_;
		mac = mac_;
		alpha_0 = alpha_0_;
		p_B = matrix::Vector3f(p_B_);
		ar = (AR <= 0.0f) ? span / mac : AR; // setting AR<=0 will compute it from span and mac
		alpha_eff = 0.0f;
		alpha_eff_old = 0.0f;
		kp = 2.0f * M_PI_F / (1.0f + 2.0f * (ar + 4.0f) / (ar * (ar + 2.0f)));
		kn = 0.41f * (1.0f - expf(-17.0f / ar));
		ale = lin_interp_lkt(AR_tab, ale_tab, ar, N_TAB);
		ate = lin_interp_lkt(AR_tab, ate_tab, ar, N_TAB);
		afle = lin_interp_lkt(AR_tab, afle_tab, ar, N_TAB);
		afte = lin_interp_lkt(AR_tab, afte_tab, ar, N_TAB);
		afs_rad = math::radians(lin_interp_lkt(AR_tab, afs_tab, ar, N_TAB));
		cf = cf_;
		C_BS = matrix::Dcmf(matrix::Eulerf(math::radians(dihedral_deg), 0.0f, 0.0f));
	}

	// aerodynamic force and moments of a generic flate plate segment
	void update_aero(matrix::Vector3f v_B, matrix::Vector3f w_B, float alt = 0.0f, float def = 0.0f, float dt = -1.0f)
	{
		// ISA model taken from Mustafa Cavcar, Anadolu University, Turkey
		pressure = P0 * powf(1.0f - 0.0065f * alt / T0_K, 5.2561f);
		temperature = T0_K + TEMP_GRADIENT * alt;
		rho = pressure / R / temperature;

		matrix::Vector3f vel = C_BS.transpose() * (v_B + w_B % p_B); 	// velocity in segment frame
		float vxz2 = vel(0) * vel(0) + vel(2) * vel(2);

		if (vxz2 < 0.01f) {
			Fa = matrix::Vector3f();
			Ma = matrix::Vector3f();
			return;
		}

		alpha = atan2f(vel(2), vel(0)) - alpha_0;
		aoa_coeff(alpha, dt, sqrtf(vxz2), def);
		Fa = C_BS * (0.5f * rho * vxz2 * span * mac) * matrix::Vector3f(CL * sinf(alpha) - CD * cosf(alpha),
				0.0f,
				-CL * cosf(alpha) - CD * sinf(alpha));
		Ma = C_BS * (0.5f * rho * vxz2 * span * mac * mac) * matrix::Vector3f(0.0f, CM,
				0.0f) + p_B % Fa; 	// computed at vehicle CM
	}

	// return the air density at current altitude, must be called after update_aero()
	float get_rho() {	return rho; 	}

private:

	// low angle of attack and stalling region coefficient based on flat plate
	void aoa_coeff(float a, float dt, float vxz, float def)
	{
		alpha_eff_old = alpha_eff;
		tau_te = (vxz > 0.01f) ? 4.5f * mac / vxz : 0.0f;
		tau_le = (vxz > 0.01f) ? 0.5f * mac / vxz : 0.0f;

		// model for the control surface deflection
		if (cf / mac < 0.999f) {
			def_a = fminf(fabsf(def), math::radians(70.0f));
			eta_f = def_a * def_a * eta_poly[0] + def_a * eta_poly[1] + eta_poly[2];	// second order fit
			theta_f = acosf(2.0f * cf / mac - 1.0f);
			tau_f = 1.0f - (theta_f - sinf(theta_f)) / M_PI_F;
			deltaCL = kp * tau_f * eta_f * def;
			dCLmax = (1.0f - cf / mac) * deltaCL;
			alf0eff = solve_alpha_eff(kp, KV, deltaCL);
			alpha_eff = a - alf0eff;

			if (dt < 1e-9f) {
				alpha_eff_dot = 0.0f;

			} else {
				alpha_eff_dot = math::constrain((alpha_eff - alpha_eff_old) / dt, -AF_DOT_MAX, AF_DOT_MAX);
			}

			fte = 0.5f * (1.0f - tanhf(ate * (math::degrees(alpha_eff) - tau_te * math::degrees(
					alpha_eff_dot) - afte)));	// normalized trailing edge separation
			fle = 0.5f * (1.0f - tanhf(ale * (math::degrees(alpha_eff) - tau_le * math::degrees(
					alpha_eff_dot) - afle)));	// normalized leading edge separation
			CLmax = fCL(afs_rad - alpha_0) + dCLmax;
			alpha_max = alf0eff - solve_alpha_eff(kp, KV * fle * fle, CLmax / (0.25f * (1.0f + sqrtf(fte)) * (1.0f + sqrtf(fte))));
			CLmin = fCL(-afs_rad - alpha_0) + dCLmax;
			alpha_min = alf0eff - solve_alpha_eff(kp, KV * fle * fle, CLmin / (0.25f * (1.0f + sqrtf(fte)) * (1.0f + sqrtf(fte))));

		} else { 	// this segment is a full flap
			alpha_eff = a + def;

			if (dt < 1e-9f) {
				alpha_eff_dot = 0.0f;

			} else {
				alpha_eff_dot = math::constrain((alpha_eff - alpha_eff_old) / dt, -AF_DOT_MAX, AF_DOT_MAX);
			}

			fte = 0.5f * (1.0f - tanhf(ate * (math::degrees(alpha_eff) - tau_te * math::degrees(
					alpha_eff_dot) - afte)));	// normalized trailing edge separation
			fle = 0.5f * (1.0f - tanhf(ale * (math::degrees(alpha_eff) - tau_le * math::degrees(
					alpha_eff_dot) - afle)));	// normalized leading edge separation
			alpha_max = afs_rad;
			alpha_min = -afs_rad;
		}

		// compute the aerodynamic coefficients
		if (alpha_eff > alpha_max || alpha_eff < -alpha_min) {
			high_aoa_coeff(a, def);

		} else {
			CL = fCL(alpha_eff);
			CD = CD0 + CL * fabsf(tanf(alpha_eff));
			// CM = fCM(alpha_eff);
			CM = 0.0f; 	// debug
		}
	}

	// high angle of attack coefficient based on flat plate
	void high_aoa_coeff(float a, float def = 0.0f)
	{
		float mac_eff = sqrtf((mac - cf) * (mac - cf) + cf * cf + 2.0f * (mac - cf) * cf * cosf(fabsf(def)));
		a += asinf(cf / mac_eff * sinf(def));
		float cd90_eff = CD90 + 0.21f * def - 0.0426f * def *
				 def; // this might not be accurate for lower flap chord to chord ratio
		// normal coeff
		float CN = cd90_eff * sinf(a) * (1.0f / (0.56f + 0.44f * sinf(fabsf(a))) - kn);
		// tengential coeff
		float CT = 0.5f * CD0 * cosf(a);
		CL = CN * cosf(a) - CT * sinf(a);
		CD = CN * sinf(a) + CT * cosf(a);
		CM = -CN * (0.25f - 7.0f / 40.0f * (1.0f - 2.0f / M_PI_F * fabsf(a)));
	}

	// linear interpolation between 2 points
	static float lin_interp(const float x0, const float y0, const float x1, const float y1, const float x)
	{
		if (x < x0) {
			return y0;
		}

		if (x > x1) {
			return y1;
		}

		float slope = (y1 - y0) / (x1 - x0);
		return y0 + slope * (x - x0);
	}

	// lookup table linear interpolation
	static float lin_interp_lkt(const float x_tab [], const float y_tab [], const float x, const int length)
	{
		if (x < x_tab[0]) {
			return y_tab[0];
		}

		if (x > x_tab[length - 1]) {
			return y_tab [length - 1];
		}

		int i = length - 2;

		while (x_tab[i] > x) {
			i--;
		}

		return lin_interp(x_tab[i], y_tab[i], x_tab[i + 1], y_tab[i + 1], x);
	}

	float solve_alpha_eff(const float Kp, const float Kv, const float dCL)
	{
		// we use here the Newton method with explicit derivative to find the root of equation 3.15
		// we can remove fTE and fLE since we are at low angle
		float a = alpha_0; 	// initialized to the zero lift angle of attack

		for (int i = 0; i < 3; i++) {
			a = a - (-Kp * sinf(a) * cosf(a) * cosf(a) - Kv * fabsf(sinf(a)) * sinf(a) * cosf(a) - dCL) /
			    (Kv * fabsf(sinf(a)) * sinf(a) * sinf(a) - Kv * fabsf(sinf(a)) * cosf(a) * cosf(a) - Kp * cosf(a) * cosf(a) * cosf(
				     a) + 2 * Kp * cosf(a) * sinf(a) * sinf(a) - Kv * matrix::sign(sinf(a)) * cosf(a) * cosf(a) * sinf(a));
		}

		return a;
	}

	float fCL(float a)
	{
		return 0.25f * (1.0f + sqrtf(fte)) * (1.0f + sqrtf(fte)) * (kp * sinf(a) * cosf(a) * cosf(a)
				+ fle * fle * KV * fabsf(sinf(a)) * sinf(a) * cosf(a));
	}

	float fCM(float a)
	{
		return -0.25f * (1.0f + sqrtf(fte)) * (1.0f + sqrtf(fte)) * 0.0625f * (-1.0f + 6.0f * sqrtf(
					fte) - 5.0f * fte) * kp * sinf(a) * cosf(a)
		       + 0.17f * fle * fle * KV * fabsf(sinf(a)) * sinf(a);
	}

	// AeroSeg operator*(const float k) const {
	// 	return AeroSeg(p_I*k, v_I*k, q*k, w_B*k);
	// }

	// AeroSeg operator+(const States other) const {
	// 	return AeroSeg(p_I+other.p_I, v_I+other.v_I, q+other.q, w_B+other.w_B);
	// }

	// void unwrap_states(matrix::Vector3f &p, matrix::Vector3f &v, matrix::Quatf &q_, matrix::Vector3f &w) {
	// 	p=p_I;
	// 	v=v_I;
	// 	q_=q;
	// 	w=w_B;
	// }

}; // ---------------------------------------------------------------------------------------------------------
