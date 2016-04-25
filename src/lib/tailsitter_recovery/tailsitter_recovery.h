/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tailsitter_recovery.h
 *
 * Tailsitter optimal rate controller (underactuated pitch axis)
 *
 * This function can be used to compute desired body rates of a tailsitter
 * suffering from an underactuated pitch axis. Tailsitters which produce a pitching moment
 * from airflow over control surfaces mostly suffer from an underactuated pitch axis.
 * This functions captures the solution of an optimal control problem which minimises
 * the vehicle's tilt error and which penalises the desired rates of the underactuated
 * pitch axis.
 * Publication:
 * Robin Ritz and Raffaello D'Andrea. A Global Strategy for Tailsitter Hover Control.
 *
 * @author Roman Bapst <bapstroman@gmail.com>
*/
#include <lib/mathlib/mathlib.h>

#define SigmoidFunction(val) 1/(1 + expf(-val))

class TailsitterRecovery
{
public:
	TailsitterRecovery();
	~TailsitterRecovery();

	// Calculate the optimal rates:
	// If the vehicle is not in need of a recovery, this function will do normal
	// attitude control based on attitude error. If a recovery situation is detected
	// then the rates are computed in an optimal way as described above.
	void calcOptimalRates(math::Quaternion &q, math::Quaternion &q_sp, float yaw_move_rate, math::Vector<3> &rates_opt);

	// Set the gains of the controller attitude loop.
	void setAttGains(math::Vector<3> &att_p, float yaw_ff);

private:
	bool _in_recovery_mode;	// indicates that the tailsitter is performing a recovery to hover

	math::Vector<3> _att_p;	// gains for attitude loop
	float _yaw_ff;			// yaw feed forward gain
};
