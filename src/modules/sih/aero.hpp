/****************************************************************************
*
*   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file aero.hpp
 * Aerodynamic class for modeling wing, tailaplane, fin.
 * Captures the effect of partial stall, low aspect ratio, control surfaces deflection,
 * propeller slipstream.
 *
 * @author Romain Chiappinelli      <romain.chiap@gmail.com>
 *
 * Altitude R&D inc, Montreal - July 2021
 *
 * The aerodynamic model is inspired from [2]
 * [2] Khan, Waqas, supervised by Meyer Nahon "Dynamics modeling of agile fixed-wing unmanned aerial vehicles."
 *  McGill University, PhD thesis, 2016. Sections 3.1, 3.2, and 3.3
 *
 * Capabilities and limitations
 * This class can model
 * - full 360 deg angle of attack lift, drag, and pitching moment.
 * - wings with aspect ratio from 0.1666 up to 6, (gliders would perform poorly for instance).
 * - control surface (flap) deflection up to 70 degrees.
 * - unlimited flap chord, (the elevator could take the entire tailplane).
 * - stall angle function of the flap deflection.
 * - effect of the flap deflection on lift, drag, and pitching moment.
 * - any dihedral angle, the fin is modeled as a wing with dihedral angle of 90 deg.
 * - slipstream velocity (velocity from the propeller) using momentum theory.
 */



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
	static constexpr const float ETA_POLY[] = {0.0535f, -0.2688f, 0.5817f}; 	// 1/rad

	// aerodynamic and physical constants
	static constexpr const float P0 = 101325.0f;	// _pressure at sea level [N/m^2]=[Pa]
	static constexpr const float R = 287.04f;		// real gas constant for air [J/kg/K]
	static constexpr const float T0_K = 288.15f;	// _temperature at sea level [K]
	static constexpr const float TEMP_GRADIENT  = -6.5e-3f;    // _temperature gradient in degrees per metre

	static constexpr const float KV = M_PI_F; 	// total vortex lift parameter
	static constexpr float CD0 = 0.04f; 		// no lift drag coefficient
	static constexpr float CD90 = 1.98f; 		// 90 deg angle of attack drag coefficient
	static constexpr float AF_DOT_MAX = M_PI_F / 2.0f;
	static constexpr float ALPHA_BLEND = M_PI_F / 18.0f; 	// 10 degrees

	// here we make the distinction of the plate (i.e. wing, or tailplane, or fin) and the segment
	// the segment can be a portion of the wing, but the aspect ratio (AR) of the wing needs to be used
	float _alpha; 		// angle of attack [rad]
	float _CL, _CD, _CM; 	// aerodynamic coefficients
	float _CL_, _CD_, _CM_; 	// low aoa coeffs
	float _f_blend;		// blending function
	matrix::Vector3f _p_B; 	// position of the aerodynamic center of the segment from _CM in body frame [m]
	matrix::Dcmf _C_BS;	// dcm from segment frame to body frame
	float _ar;		// aspect ratio of the plate
	float _span;		// _span of the segment
	float _mac;		// mean aerodynamic chord of the segment
	float _alpha_0;		// zero lift angle of attack [rad]
	float _kp, _kn;
	float _ate, _ale, _afte, _afle;	// semi empirical coefficients for flat plates function of AR
	float _tau_te, _tau_le, _fte, _fle; 	// leading and trailing edge functions
	float _rho = 1.225f; 	// air density at current altitude [kg/m^3]
	float _kD;		// for parabolic drag model
	const float K0 = 0.87f;	// Oswald efficiency factor
	// variables for flap model
	float _eta_f;		// flap effectiveness
	float _def_a;		// absolute value of the deflection angle
	float _cf;		// flap chord (control surface chord length)
	float _theta_f, _tau_f;	// check 3.2.3 in [2]
	float _deltaCL, _dCLmax;	// increase in lift coefficient
	float _CLmax, _CLmin;	// max and min lift value
	float _alpha_eff_min; 	// min effective angle of attack
	float _alpha_eff_max;	// max effective angle of attack
	float _alpha_min; 	// min angle of attack (stall angle)
	float _alpha_max;	// min angle of attack (stall angle)
	float _alf0eff;		// effective zero lift angle of attack
	// float _alfmeff;		// effective maximum lift angle of attack
	float _alpha_eff;	// effectie angle of attack
	// float _alpha_eff_dot;	// effectie angle of attack derivative
	float _alpha_eff_old;	// angle of attack [rad]

	float _pressure; 	// pressure in Pa at current altitude
	float _temperature;	// temperature in K at current altitude
	float _prop_radius;	// propeller radius [m], used to create the slipstream
	// float _v_slipstream;	// slipstream velocity [m/s], computed from momentum theory

	matrix::Vector3f _Fa;	// aerodynamic force
	matrix::Vector3f _Ma;	// aerodynamic moment computed at _CM directly
	matrix::Vector3f _v_S;	// velocity in segment frame

public:
	/** public constructor
	 * AeroSeg(float span, float mac, float alpha_0_deg, matrix::Vector3f p_B, float dihedral_deg = 0.0f,
	 * float AR = -1.0f, float cf = 0.0f, float prop_radius=-1.0f, float cl_alpha=2.0f*M_PI_F, float alpha_max_deg=0.0f, float alpha_min_deg=0.0f)
	 *
	 * span_: span of the segment [m]
	 * mac_: mean aerodynamic chord of the segment [m]
	 * alpha_0_deg: zero lift angle of attack of the segment [deg], negative number represents a segment oriented up
	 * p_B_: position of the segment (mean aerodynamic center) in the body frame from the center of mass [m,m,m]
	 * dihedral_deg: dihedral angle of the segment [deg], set to 0 for tailplane, set to -90 for the fin. default is 0.
	 * AR: Aspect Ratio of the wing, or tailplane, or fin (not the segment).
	 *     If the aspect ratio is negative, the aspect ratio is computed from the _span and MAC
	 * cf_: flap chord [m], this is the chord length of the control surface, default is zero (no flap).
	 * prop_radius_: radius of the propeller for slipstream computation. Setting to -1 (default) will assume no slipstream.
	 * cl_alpha: 2D lift curve slope (1/rad), default it 2*pi for a flat plate. This can be computed from http://airfoiltools.com for instance.
	 * alpha_max_deg: maximum angle of attack before stall. Setting to 0 (default) will compute it from a table for flat plate.
	 * alpha_min_deg: maximum negative angle of attack before stall. Setting to 0 (default) will compute it from a table for flat plate.
	 */
	AeroSeg(float span, float mac, float alpha_0_deg, const matrix::Vector3f &p_B, float dihedral_deg = 0.0f,
		float AR = -1.0f, float cf = 0.0f, float prop_radius = -1.0f, float cl_alpha = 2.0f * M_PI_F,
		float alpha_max_deg = 0.0f, float alpha_min_deg = 0.0f)
	{
		static const float AR_tab[N_TAB] = {0.1666f, 0.333f, 0.4f, 0.5f, 1.0f, 1.25f, 2.0f, 3.0f, 4.0f, 6.0f};
		static const float ale_tab[N_TAB] = {3.00f, 3.64f, 4.48f, 7.18f, 10.20f, 13.38f, 14.84f, 14.49f, 9.95f, 12.93f, 15.00f, 15.00f};
		static const float ate_tab[N_TAB] = {5.90f, 15.51f, 32.57f, 39.44f, 48.22f, 59.29f, 21.55f, 7.74f, 7.05f, 5.26f, 6.50f, 6.50f};
		static const float afle_tab[N_TAB] = {59.00f, 58.60f, 58.20f, 50.00f, 41.53f, 26.70f, 23.44f, 21.00f, 18.63f, 14.28f, 11.60f, 10.00f};
		static const float afte_tab[N_TAB] = {59.00f, 58.60f, 58.20f, 51.85f, 41.46f, 28.09f, 39.40f, 35.86f, 26.76f, 19.76f, 16.43f, 14.00f};
		static const float afs_tab[N_TAB] = {49.00f, 54.00f, 56.00f, 48.00f, 40.00f, 29.00f, 27.00f, 25.00f, 24.00f, 22.00f, 22.00f, 20.00f};

		_span = span;
		_mac = mac;
		_alpha_0 = math::radians(alpha_0_deg);
		_p_B = p_B;
		_ar = (AR <= 0.0f) ? _span / _mac : AR; // setting AR<=0 will compute it from _span and _mac
		_alpha_eff = 0.0f;
		_alpha_eff_old = 0.0f;
		_kp = cl_alpha / (1.0f + 2.0f * (_ar + 4.0f) / (_ar * (_ar + 2.0f)));
		_kn = 0.41f * (1.0f - expf(-17.0f / _ar));
		_ale = lin_interp_lkt(AR_tab, ale_tab, _ar, N_TAB);
		_ate = lin_interp_lkt(AR_tab, ate_tab, _ar, N_TAB);
		_afle = lin_interp_lkt(AR_tab, afle_tab, _ar, N_TAB);
		_afte = lin_interp_lkt(AR_tab, afte_tab, _ar, N_TAB);
		float afs_rad = math::radians(lin_interp_lkt(AR_tab, afs_tab, _ar, N_TAB));

		if (fabsf(alpha_max_deg) < 1.0e-3f) {
			_alpha_max = afs_rad;

		} else {
			_alpha_max = math::radians(alpha_max_deg);
		}

		if (fabsf(alpha_min_deg) < 1.0e-3f) {
			_alpha_min = -afs_rad;

		} else {
			_alpha_min = math::radians(alpha_min_deg);
		}

		_cf = math::constrain(cf, 0.0f, mac);
		_C_BS = matrix::Dcmf(matrix::Eulerf(math::radians(dihedral_deg), 0.0f, 0.0f));
		_prop_radius = prop_radius;
		_kD = 1.0f / (M_PI_F * K0 * _ar);
	}


	AeroSeg() : AeroSeg(1.0f, 0.2f, 0.0f, matrix::Vector3f())
	{}

	/** aerodynamic force and moments of a generic flate plate segment
	 * void update_aero(matrix::Vector3f v_B, matrix::Vector3f w_B, float alt = 0.0f,
	 *                  float def = 0.0f, float thrust=0.0f, float dt = -1.0f)
	 *
	 * v_B: 3D velocity in body frame [m/s], (front, right, down FRD frame)
	 * w_B: 3D body rates in body frame [rad/s], FRD frame.
	 * alt: altitude above mean sea level for computing air density [m], default is 0.
	 * def: flap deflection angle [rad], default is 0.
	 * thrust: thrust force [N] from the propeller to compute the slipstream velocity, default is 0.
	 */
	void update_aero(const matrix::Vector3f &v_B, const matrix::Vector3f &w_B, float alt = 0.0f, float def = 0.0f,
			 float thrust = 0.0f)
	{
		// ISA model taken from Mustafa Cavcar, Anadolu University, Turkey
		_pressure = P0 * powf(1.0f - 0.0065f * alt / T0_K, 5.2561f);
		_temperature = T0_K + TEMP_GRADIENT * alt;
		_rho = _pressure / R / _temperature;

		_v_S = _C_BS.transpose() * (v_B + w_B % _p_B); 	// velocity in segment frame

		if (_prop_radius > 1e-4f) {
			// Add velocity generated from the propeller and thrust force.
			// Computed from momentum theory.
			// For info, the diameter of the slipstream is sqrt(2)*_prop_radius,
			// this should be the width of the segment in the slipstream.
			_v_S(0) += sqrtf(2.0f * thrust / (_rho * M_PI_F * _prop_radius * _prop_radius));
		}

		float vxz2 = _v_S(0) * _v_S(0) + _v_S(2) * _v_S(2);

		if (vxz2 < 0.01f) {
			_Fa = matrix::Vector3f();
			_Ma = matrix::Vector3f();
			_alpha = 0.0f;
			return;
		}

		_alpha = matrix::wrap_pi(atan2f(_v_S(2), _v_S(0)) - _alpha_0);
		// _alpha = atan2f(_v_S(2), _v_S(0));
		aoa_coeff(_alpha, sqrtf(vxz2), def);
		_Fa = _C_BS * (0.5f * _rho * vxz2 * _span * _mac) * matrix::Vector3f(_CL * sinf(_alpha) - _CD * cosf(_alpha),
				0.0f,
				-_CL * cosf(_alpha) - _CD * sinf(_alpha));
		_Ma = _C_BS * (0.5f * _rho * vxz2 * _span * _mac * _mac) * matrix::Vector3f(0.0f, _CM,
				0.0f) + _p_B % _Fa; 	// computed at vehicle _CM
	}

	// return the air density at current altitude, must be called after update_aero()
	float get_rho() const { return _rho; }

	// return angle of attack in radians
	float get_aoa() const {return _alpha;}

	// return the aspect ratio
	float get_ar() const {return _ar;}

	// return the sum of aerodynamic forces of the segment in the body frame, taken at the _CM,
	// must be called after update_aero()
	matrix::Vector3f get_Fa() const { return _Fa; }

	// return the sum of aerodynamic moments of the segment in the body frame, taken at the _CM,
	// must be called after update_aero()
	matrix::Vector3f get_Ma() const { return _Ma; }

	// return the velocity in segment frame
	matrix::Vector3f get_vS() const { return _v_S; }

private:

	// low angle of attack and stalling region coefficient based on flat plate
	void aoa_coeff(float a, float vxz, float def)
	{
		_alpha_eff_old = _alpha_eff;
		_tau_te = (vxz > 0.01f) ? 4.5f * _mac / vxz : 0.0f;
		_tau_le = (vxz > 0.01f) ? 0.5f * _mac / vxz : 0.0f;

		// model for the control surface deflection
		if (_cf / _mac < 0.999f) {
			_def_a = fminf(fabsf(def), math::radians(70.0f));
			_eta_f = _def_a * _def_a * ETA_POLY[0] + _def_a * ETA_POLY[1] + ETA_POLY[2];	// second order fit
			_theta_f = acosf(2.0f * _cf / _mac - 1.0f);
			_tau_f = 1.0f - (_theta_f - sinf(_theta_f)) / M_PI_F;
			_deltaCL = _kp * _tau_f * _eta_f * def;
			_dCLmax = (1.0f - _cf / _mac) * _deltaCL;
			_alf0eff = solve_alpha_eff(_kp, KV, _deltaCL, _alpha_0);
			_alpha_eff = a - _alf0eff;

			// this doesn't seem to work, so let's comment it
			// _alpha_eff_dot = math::constrain((_alpha_eff - _alpha_eff_old) / dt, -AF_DOT_MAX, AF_DOT_MAX);
			// _fte = 0.5f * (1.0f - tanhf(_ate * ((_alpha_eff) - _tau_te * (_alpha_eff_dot)
			// 	- math::radians(_afte))));	// normalized trailing edge separation
			// _fle = 0.5f * (1.0f - tanhf(_ale * ((_alpha_eff) - _tau_le * (_alpha_eff_dot)
			// 	- math::radians(_afle))));	// normalized leading edge separation

			_fte = 1.0f;
			_fle = 1.0f;
			_CLmax = fCL(_alpha_max - _alpha_0) + _dCLmax;
			_alpha_eff_max = _alf0eff - solve_alpha_eff(_kp, KV * _fle * _fle,
					 _CLmax / (0.25f * (1.0f + sqrtf(_fte)) * (1.0f + sqrtf(_fte))), _alpha_max - _alpha_0);
			_CLmin = fCL(_alpha_min - _alpha_0) + _dCLmax;
			_alpha_eff_min = _alf0eff - solve_alpha_eff(_kp, KV * _fle * _fle,
					 _CLmin / (0.25f * (1.0f + sqrtf(_fte)) * (1.0f + sqrtf(_fte))), _alpha_min - _alpha_0);

		} else { 	// this segment is a full flap
			_alpha_eff = a + def;

			// this doesn't seem to work, so let's comment it
			// _alpha_eff_dot = math::constrain((_alpha_eff - _alpha_eff_old) / dt, -AF_DOT_MAX, AF_DOT_MAX);
			// _fte = 0.5f * (1.0f - tanhf(_ate * ((_alpha_eff) - _tau_te * (_alpha_eff_dot)
			// 	- math::radians(_afte))));	// normalized trailing edge separation
			// _fle = 0.5f * (1.0f - tanhf(_ale * ((_alpha_eff) - _tau_le * (_alpha_eff_dot)
			// 	- math::radians(_afle))));	// normalized leading edge separation

			_fte = 1.0f;
			_fle = 1.0f;
			_alpha_eff_max = _alpha_max;
			_alpha_eff_min = _alpha_min;
		}

		// compute the aerodynamic coefficients
		high_aoa_coeff(_alpha_eff, def);
		_CL_ = fCL(_alpha_eff);
		_CD_ = CD0 + fabsf(_CL * tanf(_alpha_eff));
		// _CD_ = CD0 + _kD*_CL_*_CL_; 	// alternative method
		_CM_ = -fCM(_alpha_eff);

		// blending function
		if (_alpha_eff > 0.0f) {
			_f_blend = 0.5f * (1.0f - tanhf(4.0f * (_alpha_eff - _alpha_eff_max) / ALPHA_BLEND));

		} else {
			_f_blend = 0.5f * (1.0f - tanhf(-4.0f * (_alpha_eff - _alpha_eff_min) / ALPHA_BLEND));
		}

		_CL = _CL_ * _f_blend + _CL * (1.0f - _f_blend);
		_CD = _CD_ * _f_blend + _CD * (1.0f - _f_blend);
		_CM = _CM_ * _f_blend + _CM * (1.0f - _f_blend);
	}

	// high angle of attack coefficient based on flat plate
	void high_aoa_coeff(float a, float def = 0.0f)
	{
		float mac_eff = sqrtf((_mac - _cf) * (_mac - _cf) + _cf * _cf + 2.0f * (_mac - _cf) * _cf * cosf(fabsf(def)));
		a += asinf(_cf / mac_eff * sinf(def));
		float cd90_eff = CD90 + 0.21f * def - 0.0426f * def *
				 def; // this might not be accurate for lower flap chord to chord ratio
		// normal coeff
		float CN = cd90_eff * sinf(a) * (1.0f / (0.56f + 0.44f * sinf(fabsf(a))) - _kn);
		// tengential coeff
		float CT = 0.5f * CD0 * cosf(a);
		_CL = CN * cosf(a) - CT * sinf(a);
		_CD = CN * sinf(a) + CT * cosf(a);
		_CM = -CN * (0.25f - 7.0f / 40.0f * (1.0f - 2.0f / M_PI_F * fabsf(a)));
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

	float solve_alpha_eff(const float Kp, const float Kv, const float dCL, const float a0)
	{
		// we use here the Newton method with explicit derivative to find the root of equation 3.15
		float a = a0; 	// init the search

		for (int i = 0; i < 3; i++) {
			a = a - (-Kp * sinf(a) * cosf(a) * cosf(a) - Kv * fabsf(sinf(a)) * sinf(a) * cosf(a) - dCL) /
			    (Kv * fabsf(sinf(a)) * sinf(a) * sinf(a) - Kv * fabsf(sinf(a)) * cosf(a) * cosf(a) - Kp * cosf(a) * cosf(a) * cosf(
				     a) + 2 * Kp * cosf(a) * sinf(a) * sinf(a) - Kv * matrix::sign(sinf(a)) * cosf(a) * cosf(a) * sinf(a));
		}

		return a;
	}

	float fCL(float a)
	{
		return 0.25f * (1.0f + sqrtf(_fte)) * (1.0f + sqrtf(_fte)) * (_kp * sinf(a) * cosf(a) * cosf(a)
				+ _fle * _fle * KV * fabsf(sinf(a)) * sinf(a) * cosf(a));
	}

	float fCM(float a)
	{
		return -0.25f * (1.0f + sqrtf(_fte)) * (1.0f + sqrtf(_fte)) * 0.0625f * (-1.0f + 6.0f * sqrtf(
					_fte) - 5.0f * _fte) * _kp * sinf(a) * cosf(a)
		       + 0.17f * _fle * _fle * KV * fabsf(sinf(a)) * sinf(a);
	}

	// AeroSeg operator=(const AeroSeg&) const {
	// 	return this;
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
