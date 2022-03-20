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
 * @file AdmittanceControl.cpp
 */

#include <AdmittanceControl.hpp>


void AdmittanceControl::initialize(BellParameters &bell_params)
{

	_y = zeros<float, 8, 1>();
	_We = zeros<float, 4, 1>();

	_params_bell = bell_params;

}

void AdmittanceControl::reset(const vehicle_local_position_setpoint_s &setpoint)
{

	_admittance_sp_ned = setpoint;
	_y(0) = setpoint.x;
	_y(2) = setpoint.y;
	_y(4) = setpoint.z;

	_y(1) = setpoint.vx;
	_y(3) = setpoint.vy;
	_y(5) = setpoint.vz;

	_y(6) = setpoint.yaw;
	_y(7) = setpoint.yawspeed;
}

void AdmittanceControl::update(const float &dt, const Vector<float, 4> &We, const Vector<float, 8> &u, const float &target_dist, const Quatf &q, const vehicle_local_position_setpoint_s &setpoint)
{

	_dt = dt;
	_We = We;

	Vector<float, 8> U = (u - 1500);
	U.abs();
	// Filter Saturation factor (1 = upper saturation limit) (-1 = lower saturation limit) (0 = 1500 pwm)
	const float alpha = _dt / (_params_bell.lpf_sat_factor + _dt);
	_sat_factor = alpha * ((U.max()) / 500) + (1.f - alpha) * _sat_factor;

	float a[4];
	float b[4];

	a[0] = _params_bell.A[0] - _sat_factor;
	a[1] = _params_bell.A[1] - _sat_factor;
	a[2] = _params_bell.A[2] - _sat_factor;
	a[3] = _params_bell.A[3] - _sat_factor;

	b[0] = _params_bell.B1[0] - (_params_bell.B2[0] * exp(-_params_bell.B3[0] * target_dist));
	b[1] = _params_bell.B1[1] - (_params_bell.B2[1] * exp(-_params_bell.B3[1] * target_dist));
	b[2] = _params_bell.B1[2] - (_params_bell.B2[2] * exp(-_params_bell.B3[2] * target_dist));
	b[3] = _params_bell.B1[3] - (_params_bell.B2[3] * exp(-_params_bell.B3[3] * target_dist));

	Vector3f pos_sp(setpoint.x,setpoint.y,setpoint.z);
	Vector3f vel_sp(setpoint.vx,setpoint.vy,setpoint.vz);
	Vector3f acc_sp(setpoint.acceleration[0],setpoint.acceleration[1],setpoint.acceleration[2]);

	// Rotate Position Setpoints
	pos_sp = q.conjugate_inversed(pos_sp);
	vel_sp = q.conjugate_inversed(vel_sp);
	acc_sp = q.conjugate_inversed(acc_sp);

	_params.M[0] = (_params_bell.M_min[0] + ((_params_bell.M_max[0] - _params_bell.M_min[0]) / (1 + pow(abs((_We(0)) / (a[0])), (2 * b[0])))));
	_params.M[1] = (_params_bell.M_min[1] + ((_params_bell.M_max[1] - _params_bell.M_min[1]) / (1 + pow(abs((_We(1)) / (a[1])), (2 * b[1])))));
	_params.M[2] = (_params_bell.M_min[2] + ((_params_bell.M_max[2] - _params_bell.M_min[2]) / (1 + pow(abs((_We(2)) / (a[2])), (2 * b[2])))));
	_params.M[3] = (_params_bell.M_min[3] + ((_params_bell.M_max[3] - _params_bell.M_min[3]) / (1 + pow(abs((_We(3)) / (a[3])), (2 * b[3])))));

	_params.K[0] = (_params_bell.K_min[0] + ((_params_bell.K_max[0] - _params_bell.K_min[0]) / (1 + pow(abs((_We(0)) / (a[0])), (2 * b[0])))));
	_params.K[1] = (_params_bell.K_min[1] + ((_params_bell.K_max[1] - _params_bell.K_min[1]) / (1 + pow(abs((_We(1)) / (a[1])), (2 * b[1])))));
	_params.K[2] = (_params_bell.K_min[2] + ((_params_bell.K_max[2] - _params_bell.K_min[2]) / (1 + pow(abs((_We(2)) / (a[2])), (2 * b[2])))));
	_params.K[3] = (_params_bell.K_min[3] + ((_params_bell.K_max[3] - _params_bell.K_min[3]) / (1 + pow(abs((_We(3)) / (a[3])), (2 * b[3])))));

	_params.C[0] = 2 * sqrt(_params.K[0] * _params.M[0]);
	_params.C[1] = 2 * sqrt(_params.K[1] * _params.M[1]);
	_params.C[2] = 2 * sqrt(_params.K[2] * _params.M[2]);
	_params.C[3] = 2 * sqrt(_params.K[3] * _params.M[3]);

	_params.Fd[0] = (_params.M[0] * acc_sp(0)) + (_params.C[0] * vel_sp(0)) + (_params.K[0] * pos_sp(0));
	_params.Fd[1] = (_params.M[1] * acc_sp(1)) + (_params.C[1] * vel_sp(1)) + (_params.K[1] * pos_sp(1));
	_params.Fd[2] = (_params.M[2] * acc_sp(2)) + (_params.C[2] * vel_sp(2)) + (_params.K[2] * pos_sp(2));
	_params.Fd[3] = (_params.C[3] * setpoint.yawspeed) + (_params.K[3] * setpoint.yaw);

	_y = _integrate();

	//Rotate back to NED
	pos_sp = q.conjugate(Vector3f(_y(0),_y(2),_y(4)));
	vel_sp = q.conjugate(Vector3f(_y(1),_y(3),_y(5)));
	acc_sp = q.conjugate(_acc_sp);

	_admittance_sp_ned.timestamp = setpoint.timestamp;
	_admittance_sp_ned.x = pos_sp(0);
	_admittance_sp_ned.y = pos_sp(1);
	_admittance_sp_ned.z = pos_sp(2);
	_admittance_sp_ned.yaw = _y(6);

	_admittance_sp_ned.vx = vel_sp(0);
	_admittance_sp_ned.vy = vel_sp(1);
	_admittance_sp_ned.vz = vel_sp(2);
	_admittance_sp_ned.yawspeed = _y(7);

	_admittance_sp_ned.acceleration[0] = acc_sp(0);
	_admittance_sp_ned.acceleration[1] = acc_sp(1);
	_admittance_sp_ned.acceleration[2] = acc_sp(2);

	// CHECK IF THIS WORKS. NOT SURE IF REQUIRED
	_admittance_sp_ned.jerk[0] = setpoint.jerk[0];
	_admittance_sp_ned.jerk[1] = setpoint.jerk[1];
	_admittance_sp_ned.jerk[2] = setpoint.jerk[2];
	_admittance_sp_ned.thrust[0] = setpoint.thrust[0];
	_admittance_sp_ned.thrust[1] = setpoint.thrust[1];
	_admittance_sp_ned.thrust[2] = setpoint.thrust[2];
}

Vector<float, 8> func(float t, const Matrix<float, 8, 1> &y, const Matrix<float, 4, 1> &u, const AdmittanceParameters &params);

Vector<float, 8> func(float t, const Matrix<float, 8, 1> &y, const Matrix<float, 4, 1> &u, const AdmittanceParameters &params)
{
	//mx_ddot + cxdot + kx = fe (for each axis)
	// Define state vector y = [x;x_dot] = [y1;y2]
	// dy/dt = [0 1;-(k/m) -(c/m)]y + [0;fe/m]
	//
	// y = [x,x_dot,y,y_dot,z,z_dot,yaw,yaw_dot]
	// u = [fx,fy,fz,m_z]

	float states[8] = {
		y(1, 0), ((-(params.C[0]*y(1, 0)) - (params.K[0]*y(0, 0)) + u(0, 0) + params.Fd[0]) / params.M[0]),
		y(3, 0), ((-(params.C[1]*y(3, 0)) - (params.K[1]*y(2, 0)) + u(1, 0) + params.Fd[1]) / params.M[1]),
		y(5, 0), ((-(params.C[2]*y(5, 0)) - (params.K[2]*y(4, 0)) + u(2, 0) + params.Fd[2]) / params.M[2]),
		y(7, 0), ((-(params.C[3]*y(7, 0)) - (params.K[3]*y(6, 0)) + u(3, 0) + params.Fd[3]) / params.M[3])
	};

	// float states[8] = {
	// 	y(1, 0), (-(params.C[0] / params.M[0])*y(1, 0) - (params.K[0] / params.M[0])*y(0, 0) + ((u(0, 0) + params.Fd[0]) / params.M[0])),
	// 	y(3, 0), (-(params.C[1] / params.M[1])*y(3, 0) - (params.K[1] / params.M[1])*y(2, 0) + ((u(1, 0) + params.Fd[1]) / params.M[1])),
	// 	y(5, 0), (-(params.C[2] / params.M[2])*y(5, 0) - (params.K[2] / params.M[2])*y(4, 0) + ((u(2, 0) + params.Fd[2]) / params.M[2])),
	// 	y(7, 0), (-(params.C[3] / params.M[3])*y(7, 0) - (params.K[3] / params.M[3])*y(6, 0) + ((u(3, 0) + params.Fd[3]) / params.M[3]))
	// };

	Vector<float, 8> v(states);

	return v;
}

Vector<float, 8> AdmittanceControl::_integrate()
{

	Vector<float, 8> y = _y;
	Vector<float, 4> u = _We;

	Vector<float, 8> dydt = func(_dt,_y,_We,_params);
	_acc_sp = Vector3f(dydt(1),dydt(3),dydt(5));

	_integrate_rk4(func, y, u, 0.f, _dt, _dt, y, _params);

	return y;
}


int AdmittanceControl::_integrate_rk4(
	Vector<float, 8> (*f)(float, const Matrix<float, 8, 1> &x, const Matrix<float, 4, 1> &u, const AdmittanceParameters &params),
	const Matrix<float, 8, 1> &y0,
	const Matrix<float, 4, 1> &u,
	float t0,
	float tf,
	float h0,
	Matrix<float, 8, 1> &y1,
	const AdmittanceParameters &params
)
{
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float t1 = t0;
	y1 = y0;
	float h = h0;
	Vector<float, 8> k1, k2, k3, k4;

	if (tf < t0) { return -1; } // make sure t1 > t0

	while (t1 < tf) {
		if (t1 + h0 < tf) {
			h = h0;

		} else {
			h = tf - t1;
		}

		k1 = f(t1, y1, u, params);
		k2 = f(t1 + h / 2, y1 + k1 * h / 2, u, params);
		k3 = f(t1 + h / 2, y1 + k2 * h / 2, u, params);
		k4 = f(t1 + h, y1 + k3 * h, u, params);
		y1 += (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);
		t1 += h;
	}

	return 0;
}
