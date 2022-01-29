/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef MAG_CAL_REPORT_HPP
#define MAG_CAL_REPORT_HPP

#include <lib/sensor_calibration/Magnetometer.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_mag.h>

class MavlinkStreamMagCalReport : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMagCalReport(mavlink); }

	static constexpr const char *get_name_static() { return "MAG_CAL_REPORT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MAG_CAL_REPORT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _parameter_update_sub.advertised() ? MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMagCalReport(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::SubscriptionMultiArray<sensor_mag_s, MAX_SENSOR_COUNT> _sensor_mag_subs{ORB_ID::sensor_mag};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	bool send() override
	{
		bool sent = false;
		parameter_update_s parameter_update;

		if (_parameter_update_sub.update(&parameter_update)) {
			for (int mag = 0; mag < MAX_SENSOR_COUNT; mag++) {
				sensor_mag_s sensor_mag;

				if (_sensor_mag_subs[mag].update(&sensor_mag) && (sensor_mag.device_id != 0)) {
					calibration::Magnetometer calibration{sensor_mag.device_id};

					if (calibration.calibrated()) {
						mavlink_mag_cal_report_t msg{};

						msg.compass_id = mag;
						msg.cal_mask = 0;                 // TODO: Bitmask of compasses being calibrated.
						msg.cal_status = MAG_CAL_SUCCESS; // TODO: Calibration Status.
						msg.fitness = 0;                  // TODO: RMS milligauss residuals.

						msg.ofs_x = calibration.offset()(0);
						msg.ofs_y = calibration.offset()(1);
						msg.ofs_z = calibration.offset()(2);

						msg.diag_x = calibration.scale()(0, 0);
						msg.diag_y = calibration.scale()(1, 1);
						msg.diag_z = calibration.scale()(2, 2);

						msg.offdiag_x = calibration.scale()(0, 1);
						msg.offdiag_y = calibration.scale()(0, 2);
						msg.offdiag_z = calibration.scale()(1, 2);

						msg.orientation_confidence = 1.f;                  // TODO: orientation_confidence
						msg.old_orientation = calibration.rotation_enum(); // TODO: old orientation
						msg.new_orientation = calibration.rotation_enum();
						msg.scale_factor = 1.f;

						mavlink_msg_mag_cal_report_send_struct(_mavlink->get_channel(), &msg);
						sent = true;
					}
				}
			}
		}

		return sent;
	}
};

#endif // MAG_CAL_REPORT_HPP
