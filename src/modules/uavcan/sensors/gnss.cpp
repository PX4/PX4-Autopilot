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
 * @file gnss.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 *
 */

#include "gnss.hpp"
#include <systemlib/err.h>
#include <mathlib/mathlib.h>

const char *const UavcanGnssBridge::NAME = "gnss";

UavcanGnssBridge::UavcanGnssBridge(uavcan::INode &node) :
_node(node),
_sub_fix(node),
_report_pub(-1)
{
}

int UavcanGnssBridge::init()
{
	int res = _sub_fix.start(FixCbBinder(this, &UavcanGnssBridge::gnss_fix_sub_cb));
	if (res < 0)
	{
		warnx("GNSS fix sub failed %i", res);
		return res;
	}
	return res;
}

unsigned UavcanGnssBridge::get_num_redundant_channels() const
{
	return (_receiver_node_id < 0) ? 0 : 1;
}

void UavcanGnssBridge::print_status() const
{
	printf("RX errors: %d, receiver node id: ", _sub_fix.getFailureCount());
	if (_receiver_node_id < 0) {
		printf("N/A\n");
	} else {
		printf("%d\n", _receiver_node_id);
	}
}

void UavcanGnssBridge::gnss_fix_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg)
{
	// This bridge does not support redundant GNSS receivers yet.
	if (_receiver_node_id < 0) {
		_receiver_node_id = msg.getSrcNodeID().get();
		warnx("GNSS receiver node ID: %d", _receiver_node_id);
	} else {
		if (_receiver_node_id != msg.getSrcNodeID().get()) {
			return;  // This GNSS receiver is the redundant one, ignore it.
		}
	}

	auto report = ::vehicle_gps_position_s();

	report.timestamp_position = msg.getMonotonicTimestamp().toUSec();
	report.lat = msg.latitude_deg_1e8 / 10;
	report.lon = msg.longitude_deg_1e8 / 10;
	report.alt = msg.height_msl_mm;

	report.timestamp_variance = report.timestamp_position;


	// Check if the msg contains valid covariance information
	const bool valid_position_covariance = !msg.position_covariance.empty();
	const bool valid_velocity_covariance = !msg.velocity_covariance.empty();

	if (valid_position_covariance) {
		float pos_cov[9];
		msg.position_covariance.unpackSquareMatrix(pos_cov);

		// Horizontal position uncertainty
		const float horizontal_pos_variance = math::max(pos_cov[0], pos_cov[4]);
		report.eph = (horizontal_pos_variance > 0) ? sqrtf(horizontal_pos_variance) : -1.0F;

		// Vertical position uncertainty
		report.epv = (pos_cov[8] > 0) ? sqrtf(pos_cov[8]) : -1.0F;
	} else {
		report.eph = -1.0F;
		report.epv = -1.0F;
	}

	if (valid_velocity_covariance) {
	    float vel_cov[9];
	    msg.velocity_covariance.unpackSquareMatrix(vel_cov);
		report.s_variance_m_s = math::max(math::max(vel_cov[0], vel_cov[4]), vel_cov[8]);

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
		report.c_variance_rad =
				(vel_e_sq * vel_cov[0] +
						-2 * vel_n * vel_e * vel_cov[1] +	// Covariance matrix is symmetric
						vel_n_sq* vel_cov[4]) / ((vel_n_sq + vel_e_sq) * (vel_n_sq + vel_e_sq));

	} else {
		report.s_variance_m_s = -1.0F;
		report.c_variance_rad = -1.0F;
	}

	report.fix_type = msg.status;

	report.timestamp_velocity = report.timestamp_position;
	report.vel_n_m_s = msg.ned_velocity[0];
	report.vel_e_m_s = msg.ned_velocity[1];
	report.vel_d_m_s = msg.ned_velocity[2];
	report.vel_m_s = sqrtf(report.vel_n_m_s * report.vel_n_m_s + report.vel_e_m_s * report.vel_e_m_s + report.vel_d_m_s * report.vel_d_m_s);
	report.cog_rad = atan2f(report.vel_e_m_s, report.vel_n_m_s);
	report.vel_ned_valid = true;

	report.timestamp_time = report.timestamp_position;
	report.time_gps_usec = uavcan::UtcTime(msg.gnss_timestamp).toUSec();	// Convert to microseconds

	report.satellites_used = msg.sats_used;

	if (_report_pub > 0) {
		orb_publish(ORB_ID(vehicle_gps_position), _report_pub, &report);

	} else {
		_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &report);
	}

}
