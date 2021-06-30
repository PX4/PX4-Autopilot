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
//     McGill University, PhD thesis, 2016.

#pragma once

#include <matrix/matrix/math.hpp>   	// matrix, vectors, dcm, quaterions
#include <conversion/rotation.h>    	// math::radians,
// #include <lib/mathlib/mathlib.h>
#include <math.h>

// class Aerodynamic Segment ------------------------------------------------------------------------
class AeroSeg
{
private:
	// here we make the distinction of the plate (i.e. wing, or tailplane, or fin) and the segment
	// the segment can be a portion of the wing, but the aspect ratio (AR) of the wing needs to be used
	float alpha_old;		// angle of attack [rad]
	float CL, CD, CM; 	// aerodynamic coefficients
	matrix::Vector3f p_B; 	// position of the aerodynamic center of the segment from CM in body frame [m]
	bool horizontal;	// horizontal segment (wing) or vertical segment (fin)
	float ar;		// aspect ratio of the plate
	float span;		// span of the segment
	float mac;		// mean aerodynamic chord of the segment
	float alpha_0;		// zero lift angle of attack [rad]
	float kp, kn;
	float ate, ale, afte, afle, afs_rad;	// semi empirical coefficients for flat plates function of AR

	// Table 3.1 SEMI-EMPIRICAL COEFFICIENTS FOR RECTANGULAR FLAT PLATES
	static constexpr const int N_TAB=12;
	static constexpr const float AR_tab[N_TAB] = {0.1666f,0.333f,0.4f,0.5f,1.0f,1.25f, 2.0f, 3.0f, 4.0f, 6.0f};
	static constexpr const float ale_tab[N_TAB] = {3.00f,3.64f,4.48f,7.18f,10.20f,13.38f,14.84f,14.49f,9.95f,12.93f,15.00f,15.00f};
	static constexpr const float ate_tab[N_TAB] = {5.90f,15.51f,32.57f,39.44f,48.22f,59.29f,21.55f,7.74f,7.05f,5.26f,6.50f,6.50f};
	static constexpr const float afle_tab[N_TAB] = {59.00f,58.60f,58.20f,50.00f,41.53f,26.70f,23.44f,21.00f,18.63f,14.28f,11.60f,10.00f};
	static constexpr const float afte_tab[N_TAB] = {59.00f,58.60f,58.20f,51.85f,41.46f,28.09f,39.40f,35.86f,26.76f,19.76f,16.43f,14.00f};
	static constexpr const float afs_tab[N_TAB] = {49.00f,54.00f,56.00f,48.00f,40.00f,29.00f,27.00f,25.00f,24.00f,22.00f,22.00f,20.00f};

	// aerodynamic constants
	static constexpr const float RHO=1.225f;	// air density [kg/m^3]
	static constexpr const float KV=M_PI_F; 	// total vortex lift parameter
	static constexpr float CD0 = 0.04f; 		// no lift drag coefficient
	static constexpr float CD90 = 1.98f; 		// 90 deg angle of attack drag coefficient
	static constexpr float AF_DOT_MAX = M_PI_F/2.0f;

public:

	matrix::Vector3f Fa;	// aerodynamic force
	matrix::Vector3f Ma;	// aerodynamic moment computed at CM directly

	AeroSeg() = default;

	// public explicit constructor
	// if the aspect ratio is negative, the aspect ratio is computed from the span and MAC
	explicit AeroSeg(float span_, float mac_, float alpha_0_, matrix::Vector3f p_B_, bool horizontal_=true, float AR=-1.0f) {
		span=span_;
		mac=mac_;
		alpha_0=alpha_0_;
		p_B=matrix::Vector3f(p_B_);
		horizontal=horizontal_;
		ar=(AR<0.0f)? span/mac : AR; // setting AR<0 will compute it from span and mac
		alpha_old=0.0f;
		kp=2.0f*M_PI_F/(1.0f+2.0f*(ar+4.0f)/(ar*(ar+2.0f)));
		kn=0.41f * (1.0f - expf(-17.0f / ar));
		ale = lin_interp_lkt(AR_tab, ale_tab, ar);
		ate = lin_interp_lkt(AR_tab, ate_tab, ar);
		afle = lin_interp_lkt(AR_tab, afle_tab, ar);
		afte = lin_interp_lkt(AR_tab, afte_tab, ar);
		afs_rad = math::radians(lin_interp_lkt(AR_tab, afs_tab, ar));
	}

	// aerodynamic force and moments of a generic flate plate segment
	void update_aero(const matrix::Vector3f v_B, const matrix::Vector3f w_B, const float dt)
	{
		matrix::Vector3f vel =v_B+w_B%p_B;
		if (horizontal) { // horizontal segment? like wing and elevators
			float vxz2=vel(0)*vel(0)+vel(2)*vel(2);
			if (vxz2<0.01f) {
				Fa=matrix::Vector3f();
				Ma=matrix::Vector3f();
				return;
			}
			float alpha = atan2f(vel(2), vel(0))-alpha_0;
			float alpha_dot=math::constrain((alpha-alpha_old)/dt,-AF_DOT_MAX,AF_DOT_MAX);
			alpha_old=alpha;
			aoa_coeff(alpha, alpha_dot, sqrtf(vxz2));
			Fa=0.5f*RHO*vxz2*span*mac*matrix::Vector3f(CL*sinf(alpha)-CD*cosf(alpha),
									0.0f,
									-CL*cosf(alpha)-CD*sinf(alpha));
			Ma=0.5f*RHO*vxz2*span*mac*mac*matrix::Vector3f(0.0f,CM,0.0f) + p_B%Fa; 	// computed at vehicle CM
		} else { 	// vertical segment? like rudder and fin

			float vxy2=vel(0)*vel(0)+vel(1)*vel(1);
			if (vxy2<0.01f) {
				Fa=matrix::Vector3f();
				Ma=matrix::Vector3f();
				return;
			}
			float alpha = atan2f(vel(1), vel(0))-alpha_0;	// this is in fact beta
			float alpha_dot=math::constrain((alpha-alpha_old)/dt,-AF_DOT_MAX,AF_DOT_MAX);
			alpha_old=alpha;
			aoa_coeff(alpha, alpha_dot, sqrtf(vxy2));
			Fa=0.5f*RHO*vxy2*span*mac*matrix::Vector3f(  CL*sinf(alpha)-CD*cosf(alpha),
								-CL*cosf(alpha)-CD*sinf(alpha),
								0.0f);
			Ma=0.5f*RHO*vxy2*span*mac*mac*matrix::Vector3f(0.0f,0.0f,-CM) + p_B%Fa;	// computed at vehicle CM
		}
	}

	// low angle of attack and stalling region coefficient based on flat plate
	void aoa_coeff(float alpha, float alpha_dot, float vxz)
	{
		alpha_dot=0.0f; // DEBUG ==========================
		if (alpha>afs_rad || alpha<-afs_rad) {
			high_aoa_coeff(alpha);
			return;
		}
		float tau_te=(vxz>0.01f) ? 4.5f*mac/vxz : 0.0f;
		float tau_le=(vxz>0.01f) ? 0.5f*mac/vxz : 0.0f;
		float fte=0.5f*(1.0f-tanhf(ate*(math::degrees(alpha)-tau_te*math::degrees(alpha_dot)-afte)));	// normalized trailing edge separation
		float fle=0.5f*(1.0f-tanhf(ale*(math::degrees(alpha)-tau_le*math::degrees(alpha_dot)-afle))); 	// normalized leading edge separation

		CL = 0.25f*(1.0f+sqrtf(fte))*(1.0f+sqrtf(fte))*(kp*sinf(alpha)*cosf(alpha)*cosf(alpha)
				+fle*fle*KV*fabsf(sinf(alpha))*sinf(alpha)*cosf(alpha));
		CD = CD0+CL*fabsf(tanf(alpha));
		// float xp=0.25f; 	// quarter chord location of center of pressure
		// CM = -(0.42f-0.25f)*KV*fabsf(sinf(alpha))*sinf(alpha)-(xp-0.25f)*kp*sinf(alpha)*cosf(alpha); // low alpha pitching moment coefficient
		// CM =0.0f;
		CM = -0.25f*(1.0f+sqrtf(fte))*(1.0f+sqrtf(fte))*0.0625f*(-1.0f+6.0f*sqrtf(fte)-5.0f*fte)*kp*sinf(alpha)*cosf(alpha)
		     +0.17f*fle*fle*KV*fabsf(sinf(alpha))*sinf(alpha);
	}

	// high angle of attack coefficient based on flat plate
	void high_aoa_coeff(float alpha)
	{
		// normal coeff
		float CN = CD90 * sinf(alpha) * (1.0f / (0.56f + 0.44f * sinf(fabsf(alpha))) - kn);
		// tengential coeff
		float CT = 0.5f * CD0 * cosf(alpha);
		CL = CN * cosf(alpha) - CT * sinf(alpha);
		CD = CN * sinf(alpha) + CT * cosf(alpha);
		CM = -CN * (0.25f - 7.0f / 40.0f * (1.0f - 2.0f / M_PI_F * fabsf(alpha)));
	}

	// linear interpolation between 2 points
	static float lin_interp(const float x0, const float y0, const float x1, const float y1, const float x)
	{
		if (x<x0)
			return y0;
		if (x>x1)
			return y1;
		float slope=(y1-y0)/(x1-x0);
		return y0+slope*(x-x0);
	}

	// lookup table linear interpolation
	static float lin_interp_lkt(const float x_tab [N_TAB], const float y_tab [N_TAB], const float x) {
		if (x<x_tab[0]) {
			return y_tab[0];
		}
		if (x>x_tab[N_TAB-1]) {
			return y_tab [N_TAB-1];
		}
		int i=N_TAB-2;
		while(x_tab[i]>x) {
			i--;
		}
		return lin_interp(x_tab[i], y_tab[i], x_tab[i+1], y_tab[i+1], x);
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
