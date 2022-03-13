/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file RLSIdentification.cpp
 */

#include <RLSIdentification.hpp>


void RLSIdentification::initialize(const float (&x_init)[4], const float (&x_confidence)[4],
				   const float (&R_diag)[5], const VehicleParameters &params)
{
	_w_lpf.setAll(0.f);
	_q.setAll(0.f);
	_q(0) = 1.f;

	_mass = params.mass;
	_tilt_angle = params.tilt_angle;
	_n_rotors = params.n_rotors;
	_lpf_motor_tau = params.lpf_motor_tau;
	_km = params.torque_constant;
	_diameter = params.diameter;
	_top_motors_height = params.top_motors_height;
	_bot_motors_height = params.bot_motors_height;

	//RLS Thrust
	_force_vector.setAll(0.f);
	_K_thrust.setAll(0.f);
	_H_thrust.setAll(0.f);
	_prediction_error_thrust.setAll(0.f);

	SquareMatrix<float, 2> P_init_thrust;
	P_init_thrust.setIdentity();
	P_init_thrust(0, 0) = x_confidence[0];
	P_init_thrust(1, 1) = x_confidence[1];
	_Pp_thrust = P_init_thrust;

	SquareMatrix<float, 3> R_thrust;
	R_thrust.setIdentity();
	R_thrust(0, 0) = R_diag[0];
	R_thrust(1, 1) = R_diag[1];
	R_thrust(2, 2) = R_diag[2];
	_R_thrust = R_thrust;

	_xp_thrust(0) = x_init[0];
	_xp_thrust(1) = x_init[1];

	//RLS offset
	_moment_vector.setAll(0.f);
	_K_offset.setAll(0.f);
	_H_offset.setAll(0.f);
	_prediction_error_offset.setAll(0.f);

	SquareMatrix<float, 3> P_init_offset;
	P_init_offset.setIdentity();
	P_init_offset(0, 0) = x_confidence[2];
	P_init_offset(1, 1) = x_confidence[3];
	P_init_offset(2, 2) = 0.f;  // Results for z offset not accurate, thus setting to 0
	_Pp_offset = P_init_offset;

	SquareMatrix<float, 3> R_offset;
	R_offset.setIdentity();
	R_offset(0, 0) = R_diag[3];
	R_offset(1, 1) = R_diag[4];
	R_offset(2, 2) = R_diag[4];
	_R_offset = R_offset;

	_xp_offset(0) = x_init[2];
	_xp_offset(1) = x_init[3];
	_xp_offset(2) = 0.f;  // Results for z offset not accurate, thus setting to 0

}

void RLSIdentification::updateThrust(const Vector3f &y, const Vector<float, 8> &outputs, const float &dt,
		const bool &interaction_flag)
{
	//x[k] = x[k-1] + K[k]*(y[k]-H[k]*x[k-1])
	_dt = dt;
	_updateLpf(outputs);

	_createHThrust(_w_lpf);
	_computeKThrust(interaction_flag);

	_computePredictionErrorThrust(y * _mass);
	Vector2f x_thrust = _xp_thrust + _K_thrust * _prediction_error_thrust;

	_computePThrust();
	_xp_thrust = x_thrust;
}

void RLSIdentification::updateOffset(const Quatf &q, const bool &interaction_flag)
{
	//x[k] = x[k-1] + K[k]*(y[k]-H[k]*x[k-1])
	_q = q;
	_createHOffset();
	_computeKOffset(interaction_flag);

	_createMomentVector();
	_computePredictionErrorOffset();
	Vector3f x_offset = _xp_offset + (_K_offset * _prediction_error_offset);

	_computePOffset();
	_xp_offset = x_offset;
}

inline void RLSIdentification::_updateLpf(const Vector<float, 8> &u)
{
	//y[k] = (alpha*u[k] + (1-alpha)*y[k-1])
	// alpha = dt/(tau + dt)
	const float alpha = _dt / (_lpf_motor_tau + _dt);
	_w_lpf = alpha * u + (1.f - alpha) * _w_lpf ;
}

inline void RLSIdentification::_createHThrust(Vector<float, 8> &W)
{

	if (_n_rotors <= 4) {
		W(4) = 0.f;
		W(5) = 0.f;
		W(6) = 0.f;
		W(7) = 0.f;
	}

	Matrix<float, 3, 2> H;
	H(0, 0) = -sinf(_tilt_angle) * _kf_multiplier * ((W(0) * W(0)) + (W(1) * W(1)) - (W(2) * W(2)) - (W(3) * W(3)));
	H(0, 1) = 0.f;
	H(1, 0) = sinf(_tilt_angle) * _kf_multiplier * ((W(4) * W(4)) - (W(5) * W(5)) - (W(6) * W(6)) + (W(7) * W(7)));
	H(1, 1) = 0.f;
	H(2, 0) = -cosf(_tilt_angle) * _kf_multiplier * ((W(0) * W(0)) + (W(1) * W(1)) + (W(2) * W(2)) + (W(3) * W(3)) +
									(W(4) * W(4)) + (W(5) * W(5)) + (W(6) * W(6)) + (W(7) * W(7)));
	H(2, 1) = _kf_multiplier * ((W(0) * W(0)) + (W(1) * W(1)) + (W(2) * W(2)) + (W(3) * W(3)));
	// H(2, 1) = cosf(_tilt_angle) * _kf_multiplier * ((W(0) * W(0)) + (W(1) * W(1)) + (W(2) * W(2)) + (W(3) * W(3)));

	_H_thrust = H;
}

inline void RLSIdentification::_computeKThrust(const bool &interaction_flag)
{
	//  K[k] = P[k-1]*H[k]'*inv(H[k]*P[k-1]*H[k]'+R[k])
	if (!interaction_flag) {
		SquareMatrix<float, 3> Q;
		Q = (_H_thrust * _Pp_thrust * _H_thrust.transpose()) + _R_thrust;
		_K_thrust = _Pp_thrust * _H_thrust.transpose() * inv(Q);

	} else {
		_K_thrust.setAll(0.f);
	}
}

inline void RLSIdentification::_computePThrust()
{
	//  P[k] = (I - K[k]H[k])*P[k-1]
	SquareMatrix<float, 2> I;
	I.setIdentity();
	_Pp_thrust = (I - _K_thrust * _H_thrust) * _Pp_thrust;
}

inline void RLSIdentification::_computePredictionErrorThrust(const Vector3f &y)
{
	//e[k] = (y[k]-H[k]*x[k-1])
	_prediction_error_thrust = y - (_H_thrust * _xp_thrust);
}

inline void RLSIdentification::_createHOffset()
{
	Vector3f Rz = _q.dcm_z();
	SquareMatrix<float, 3> H;
	H.setAll(0.f);

	H(0, 0) = 0.f;
	H(0, 1) = -Rz(2);
	H(0, 2) = Rz(1);

	H(1, 0) = Rz(2);
	H(1, 1) = 0.f;
	H(1, 2) = -Rz(0);

	H(2, 0) = -Rz(1);
	H(2, 1) = Rz(0);
	H(2, 2) = 0.f;

	_H_offset = _mass * _GRAVITY * H;
}

inline void RLSIdentification::_computeKOffset(const bool &interaction_flag)
{
	//  K[k] = P[k-1]*H[k]'*inv(H[k]*P[k-1]*H[k]'+R[k])
	if (!interaction_flag) {
		SquareMatrix<float, 3> Q;
		Q = (_H_offset * _Pp_offset * _H_offset.transpose()) + _R_offset;
		_K_offset = _Pp_offset * _H_offset.transpose() * inv(Q);

	} else {
		_K_offset.setAll(0.f);
	}
}

inline void RLSIdentification::_computePredictionErrorOffset()
{
	//e[k] = (y[k]-H[k]*x[k-1])
	_prediction_error_offset = _moment_vector - (_H_offset * _xp_offset);
}

inline void RLSIdentification::_computePOffset()
{
	//  P[k] = (I - K[k]H[k])*P[k-1]
	SquareMatrix<float, 3> I;
	I.setIdentity();
	_Pp_offset = (I - _K_offset * _H_offset) * _Pp_offset;
}

inline void RLSIdentification::_createMomentVector()
{
	Vector<float, 8> w_lpf = _w_lpf;

	//Remap motors if quadcopter instead of fixed tilt octo
	if (_n_rotors <= 4) {
		w_lpf(1) = _w_lpf(2);
		w_lpf(2) = _w_lpf(1);
		w_lpf(4) = 0.f;
		w_lpf(5) = 0.f;
		w_lpf(6) = 0.f;
		w_lpf(7) = 0.f;
	}

	Vector3f xb = Vector3f(1.f, 0.f, 0.f);
	Vector3f yb = Vector3f(0.f, 1.f, 0.f);

	// Vector for each motor rotation axis and angle
	// 1 = Rotate _tilt angle about xb
	// -1 = Rotate -_tilt angle about xb
	// 2 = Rotate _tilt angle about yb
	// -2 = Rotate -_tilt angle about yb
	const int T[8] = {2, 2, -2, -2, 1, -1, -1, 1};
	Quatf q;
	Vector3f Fi(0.f, 0.f, 0.f);
	Vector3f Ft(0.f, 0.f, 0.f);
	Vector3f Qi(0.f, 0.f, 0.f);
	Vector3f Pi(0.f, 0.f, 0.f);

	// %Rotation directions (nx1)
	// % (1 - clockwise)
	// % (-1 - counterclockwise)
	const int R[8] = {-1, 1, -1, 1, -1, 1, -1, 1};

	//Rotor angular position
	const float A[8] = {M_PI / 4, 3 * M_PI / 4, 5 * M_PI / 4, 7 * M_PI / 4, 3 * M_PI / 4, M_PI / 4, 7 * M_PI / 4, 5 * M_PI / 4};

	//Motor heights
	const float H[8] = {-_top_motors_height / 1000, -_top_motors_height / 1000, -_top_motors_height / 1000, -_top_motors_height / 1000,
			_bot_motors_height / 1000, _bot_motors_height / 1000, _bot_motors_height / 1000, _bot_motors_height / 1000};

	//Bottom props reduction
	const float Fr[8] = {0, 0, 0, 0,
			     _xp_thrust(1) * _kf_multiplier *(w_lpf(1)*w_lpf(1)),
			     _xp_thrust(1) * _kf_multiplier *(w_lpf(0)*w_lpf(0)),
			     _xp_thrust(1) * _kf_multiplier *(w_lpf(3)*w_lpf(3)),
			     _xp_thrust(1) * _kf_multiplier *(w_lpf(2)*w_lpf(2))
			};

	for (int i = 0; i < 8; i++) {

		if (abs(T[i]) == 1) {
			AxisAnglef tilt_aa(xb, (sign(T[i]))*_tilt_angle);
			q = Quatf(tilt_aa);

		} else {
			AxisAnglef tilt_aa(yb, (sign(T[i]))*_tilt_angle);
			q = Quatf(tilt_aa);
		}

		Fi = q.conjugate(Vector3f(0.f, 0.f, -_xp_thrust(0) * _kf_multiplier * (w_lpf(i) * w_lpf(i))));
		Fi += Vector3f(0.f, 0.f, Fr[i]);
		Ft += Fi;
		Pi = Vector3f(.5f * _diameter * sinf(A[i]), .5f * _diameter * cosf(A[i]), H[i]);
		Qi += Pi.cross(Fi) + q.conjugate(Vector3f(0.f, 0.f, -_km * _km_multiplier * R[i] * (w_lpf(i) * w_lpf(i))));
	}

	_moment_vector = Qi;
	_force_vector = Ft;
}

