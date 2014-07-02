/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file gnss_receiver.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 *
 */

#include "gnss_receiver.hpp"
#include <systemlib/err.h>

#define MM_PER_CM 			10	// Millimeters per centimeter
#define EXPECTED_COV_SIZE 	9 	// Expect a 3x3 matrix for position and velocity covariance

UavcanGnssReceiver::UavcanGnssReceiver(uavcan::INode &node) :
_node(node),
_uavcan_sub_status(node),
_report_pub(-1)
{
}

int UavcanGnssReceiver::init()
{
	int res = -1;

	// GNSS fix subscription
	res = _uavcan_sub_status.start(FixCbBinder(this, &UavcanGnssReceiver::gnss_fix_sub_cb));
	if (res < 0)
	{
		warnx("GNSS fix sub failed %i", res);
		return res;
	}

	// Clear the uORB GPS report
	memset(&_report, 0, sizeof(_report));

	return res;
}

void UavcanGnssReceiver::gnss_fix_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg)
{
	_report.timestamp_position = hrt_absolute_time();
	_report.lat = msg.lat_1e7;
	_report.lon = msg.lon_1e7;
	_report.alt = msg.alt_1e2 * MM_PER_CM;					// Convert from centimeter (1e2) to millimeters (1e3)

	_report.timestamp_variance = _report.timestamp_position;

	// Check that the covariance arrays are valid
	bool valid_position_covariance = (msg.position_covariance.size() == EXPECTED_COV_SIZE &&
			msg.position_covariance[0] > 0);
	bool valid_velocity_covariance = (msg.velocity_covariance.size() == EXPECTED_COV_SIZE &&
			msg.velocity_covariance[0] > 0);

	if (valid_position_covariance) {
		_report.p_variance_m = msg.position_covariance[0] + msg.position_covariance[4];
		_report.eph_m = sqrtf(_report.p_variance_m);
	} else {
		_report.p_variance_m = -1.0;
		_report.eph_m = -1.0;
	}

	if (valid_velocity_covariance) {
		_report.s_variance_m_s = msg.velocity_covariance[0] + msg.velocity_covariance[4] + msg.velocity_covariance[8];

		/* There is a nonlinear relationship between the velocity vector and the heading.
		 * Use Jacobian to transform velocity covariance to heading covariance
		 *
		 * Nonlinear equation:
		 * heading = atan2(vel_e_m_s, vel_n_m_s)
		 * For math, see http://en.wikipedia.org/wiki/Atan2#Derivative
		 *
		 * To calculate the variance of heading from the variance of velocity,
		 * cov(heading) = J(velocity)*cov(velocity)*J(velocity)^T
		 */
		float vel_n = msg.ned_velocity[0];
		float vel_e = msg.ned_velocity[1];
		float vel_n_sq = vel_n * vel_n;
		float vel_e_sq = vel_e * vel_e;
		_report.c_variance_rad =
				(vel_e_sq * msg.velocity_covariance[0] +
						-2 * vel_n * vel_e * msg.velocity_covariance[1] +		// Covariance matrix is symmetric
						vel_n_sq* msg.velocity_covariance[4]) / ((vel_n_sq + vel_e_sq) * (vel_n_sq + vel_e_sq));

		_report.epv_m = sqrtf(msg.position_covariance[8]);

	} else {
		_report.s_variance_m_s = -1.0;
		_report.c_variance_rad = -1.0;
		_report.epv_m = -1.0;
	}

	_report.fix_type = msg.status;

	_report.timestamp_velocity = _report.timestamp_position;
	_report.vel_n_m_s = msg.ned_velocity[0];
	_report.vel_e_m_s = msg.ned_velocity[1];
	_report.vel_d_m_s = msg.ned_velocity[2];
	_report.vel_m_s = sqrtf(_report.vel_n_m_s * _report.vel_n_m_s + _report.vel_e_m_s * _report.vel_e_m_s + _report.vel_d_m_s * _report.vel_d_m_s);
	_report.cog_rad = atan2f(_report.vel_e_m_s, _report.vel_n_m_s);
	_report.vel_ned_valid = true;

	_report.timestamp_time = _report.timestamp_position;
	_report.time_gps_usec = msg.timestamp.husec * msg.timestamp.USEC_PER_LSB;	// Convert to microseconds

	_report.timestamp_satellites = _report.timestamp_position;
	_report.satellites_visible = msg.sats_used;
	_report.satellite_info_available = 0;					// Set to 0 for no info available

	if (_report_pub > 0) {
		orb_publish(ORB_ID(vehicle_gps_position), _report_pub, &_report);

	} else {
		_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report);
	}

}
