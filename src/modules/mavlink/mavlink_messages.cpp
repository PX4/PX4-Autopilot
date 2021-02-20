/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"
#include "mavlink_simple_analyzer.h"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_pwm_output.h>
#include <lib/conversion/rotation.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/time.h>
#include <math.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/gimbal_device_set_attitude.h>
#include <uORB/topics/gimbal_manager_information.h>
#include <uORB/topics/gimbal_manager_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>

using matrix::Vector3f;
using matrix::wrap_2pi;

#include "streams/ACTUATOR_CONTROL_TARGET.hpp"
#include "streams/ACTUATOR_OUTPUT_STATUS.hpp"
#include "streams/ALTITUDE.hpp"
#include "streams/ATTITUDE.hpp"
#include "streams/ATTITUDE_QUATERNION.hpp"
#include "streams/ATTITUDE_TARGET.hpp"
#include "streams/AUTOPILOT_VERSION.hpp"
#include "streams/COLLISION.hpp"
#include "streams/COMPONENT_INFORMATION.hpp"
#include "streams/DISTANCE_SENSOR.hpp"
#include "streams/ESC_INFO.hpp"
#include "streams/ESC_STATUS.hpp"
#include "streams/ESTIMATOR_STATUS.hpp"
#include "streams/EXTENDED_SYS_STATE.hpp"
#include "streams/FLIGHT_INFORMATION.hpp"
#include "streams/GPS_GLOBAL_ORIGIN.hpp"
#include "streams/GPS_STATUS.hpp"
#include "streams/HIGH_LATENCY2.hpp"
#include "streams/HIL_ACTUATOR_CONTROLS.hpp"
#include "streams/HIL_STATE_QUATERNION.hpp"
#include "streams/HOME_POSITION.hpp"
#include "streams/LOCAL_POSITION_NED.hpp"
#include "streams/MANUAL_CONTROL.hpp"
#include "streams/MOUNT_ORIENTATION.hpp"
#include "streams/NAV_CONTROLLER_OUTPUT.hpp"
#include "streams/OBSTACLE_DISTANCE.hpp"
#include "streams/OPTICAL_FLOW_RAD.hpp"
#include "streams/ORBIT_EXECUTION_STATUS.hpp"
#include "streams/PING.hpp"
#include "streams/POSITION_TARGET_GLOBAL_INT.hpp"
#include "streams/POSITION_TARGET_LOCAL_NED.hpp"
#include "streams/PROTOCOL_VERSION.hpp"
#include "streams/RAW_RPM.hpp"
#include "streams/RC_CHANNELS.hpp"
#include "streams/SCALED_IMU.hpp"
#include "streams/SERVO_OUTPUT_RAW.hpp"
#include "streams/STATUSTEXT.hpp"
#include "streams/STORAGE_INFORMATION.hpp"
#include "streams/TRAJECTORY_REPRESENTATION_WAYPOINTS.hpp"
#include "streams/VIBRATION.hpp"
#include "streams/WIND_COV.hpp"

#if !defined(CONSTRAINED_FLASH)
# include "streams/ATT_POS_MOCAP.hpp"
# include "streams/DEBUG.hpp"
# include "streams/DEBUG_FLOAT_ARRAY.hpp"
# include "streams/DEBUG_VECT.hpp"
# include "streams/LINK_NODE_STATUS.hpp"
# include "streams/NAMED_VALUE_FLOAT.hpp"
# include "streams/ODOMETRY.hpp"
#endif // !CONSTRAINED_FLASH

// ensure PX4 rotation enum and MAV_SENSOR_ROTATION align
static_assert(MAV_SENSOR_ROTATION_NONE == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_NONE),
	      "Roll: 0, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_45),
	      "Roll: 0, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_90),
	      "Roll: 0, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_135),
	      "Roll: 0, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_YAW_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_180),
	      "Roll: 0, Pitch: 0, Yaw: 180");
static_assert(MAV_SENSOR_ROTATION_YAW_225 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_225),
	      "Roll: 0, Pitch: 0, Yaw: 225");
static_assert(MAV_SENSOR_ROTATION_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_270),
	      "Roll: 0, Pitch: 0, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_YAW_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_315),
	      "Roll: 0, Pitch: 0, Yaw: 315");
static_assert(MAV_SENSOR_ROTATION_ROLL_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180),
	      "Roll: 180, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_45),
	      "Roll: 180, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_90),
	      "Roll: 180, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_135),
	      "Roll: 180, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_PITCH_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_180),
	      "Roll: 0, Pitch: 180, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_225 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_225),
	      "Roll: 180, Pitch: 0, Yaw: 225");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_270),
	      "Roll: 180, Pitch: 0, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_315),
	      "Roll: 180, Pitch: 0, Yaw: 315");
static_assert(MAV_SENSOR_ROTATION_ROLL_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90),
	      "Roll: 90, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_45),
	      "Roll: 90, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_90),
	      "Roll: 90, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_135),
	      "Roll: 90, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_ROLL_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270),
	      "Roll: 270, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_YAW_45),
	      "Roll: 270, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_YAW_90),
	      "Roll: 270, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_YAW_135),
	      "Roll: 270, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_90),
	      "Roll: 0, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_270),
	      "Roll: 0, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_PITCH_180_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_180_YAW_90),
	      "Roll: 0, Pitch: 180, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_PITCH_180_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_180_YAW_270),
	      "Roll: 0, Pitch: 180, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_90),
	      "Roll: 90, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_PITCH_90),
	      "Roll: 180, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_PITCH_90),
	      "Roll: 270, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_180),
	      "Roll: 90, Pitch: 180, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_270_PITCH_180), "Roll: 270, Pitch: 180, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_270),
	      "Roll: 90, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_180_PITCH_270), "Roll: 180, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_270_PITCH_270), "Roll: 270, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_90_PITCH_180_YAW_90),
	      "Roll: 90, Pitch: 180, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_270),
	      "Roll: 90, Pitch: 0, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_90_PITCH_68_YAW_293),
	      "Roll: 90, Pitch: 68, Yaw: 293");
static_assert(MAV_SENSOR_ROTATION_PITCH_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_315), "Pitch: 315");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_315),
	      "Roll: 90, Pitch: 315");

static_assert(41 == ROTATION_MAX, "Keep MAV_SENSOR_ROTATION and PX4 Rotation in sync");

static uint16_t cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_navigation_mode(const struct vehicle_status_s *const status, uint8_t *mavlink_base_mode,
				 union px4_custom_mode *custom_mode)
{
	custom_mode->data = 0;
	*mavlink_base_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
				      | MAV_MODE_FLAG_STABILIZE_ENABLED
				      | MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}
}

static void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
				   uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	union px4_custom_mode custom_mode;
	get_mavlink_navigation_mode(status, mavlink_base_mode, &custom_mode);
	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() override
	{
		return true;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &) = delete;
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &) = delete;

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() >= get_size()) {
			// always send the heartbeat, independent of the update status of the topics
			vehicle_status_s status{};
			_status_sub.copy(&status);

			uint8_t base_mode = 0;
			uint32_t custom_mode = 0;
			uint8_t system_status = 0;
			get_mavlink_mode_state(&status, &system_status, &base_mode, &custom_mode);

			mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
						   base_mode, custom_mode, system_status);

			return true;
		}

		return false;
	}
};

class MavlinkStreamCommandLong : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCommandLong::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "COMMAND_LONG";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCommandLong(mavlink);
	}

	unsigned get_size() override
	{
		return 0;	// commands stream is not regular and not predictable
	}

private:
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &) = delete;
	MavlinkStreamCommandLong &operator = (const MavlinkStreamCommandLong &) = delete;

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		bool sent = false;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _vehicle_command_sub.updated()) {

			const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_vehicle_command_sub.update(&cmd)) {
				if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("COMMAND_LONG vehicle_command lost, generation %d -> %d", last_generation,
						_vehicle_command_sub.get_last_generation());
				}

				if (!cmd.from_external && cmd.command < vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START) {
					PX4_DEBUG("sending command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);

					MavlinkCommandSender::instance().handle_vehicle_command(cmd, _mavlink->get_channel());
					sent = true;

				} else {
					PX4_DEBUG("not forwarding command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
				}
			}
		}

		MavlinkCommandSender::instance().check_timeout(_mavlink->get_channel());

		return sent;
	}
};

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send() override
	{
		if (_status_sub.updated() || _cpuload_sub.updated() || _battery_status_subs.updated()) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			cpuload_s cpuload{};
			_cpuload_sub.copy(&cpuload);

			battery_status_s battery_status[battery_status_s::MAX_INSTANCES] {};

			for (int i = 0; i < _battery_status_subs.size(); i++) {
				_battery_status_subs[i].copy(&battery_status[i]);
			}

			int lowest_battery_index = 0;

			// No battery is connected, select the first group
			// Low battery judgment is performed only when the current battery is connected
			// When the last cached battery is not connected or the current battery level is lower than the cached battery level,
			// the current battery status is replaced with the cached value
			for (int i = 0; i < _battery_status_subs.size(); i++) {
				if (battery_status[i].connected && ((!battery_status[lowest_battery_index].connected)
								    || (battery_status[i].remaining < battery_status[lowest_battery_index].remaining))) {
					lowest_battery_index = i;
				}
			}

			mavlink_sys_status_t msg{};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;

			msg.load = cpuload.load * 1000.0f;

			// TODO: Determine what data should be put here when there are multiple batteries.
			//  Right now, it uses the lowest battery. This is a safety decision, because if a client is only checking
			//  one battery using this message, it should be the lowest.
			//  In the future, this should somehow determine the "main" battery, or use the "type" field of BATTERY_STATUS
			//  to determine which battery is more important at a given time.
			const battery_status_s &lowest_battery = battery_status[lowest_battery_index];

			if (lowest_battery.connected) {
				msg.voltage_battery = lowest_battery.voltage_filtered_v * 1000.0f;
				msg.current_battery = lowest_battery.current_filtered_a * 100.0f;
				msg.battery_remaining = ceilf(lowest_battery.remaining * 100.0f);

			} else {
				msg.voltage_battery = UINT16_MAX;
				msg.current_battery = -1;
				msg.battery_remaining = -1;
			}

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamBatteryStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamBatteryStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "BATTERY_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_BATTERY_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamBatteryStatus(mavlink);
	}

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	/* do not allow top copying this class */
	MavlinkStreamBatteryStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamBatteryStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamBatteryStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send() override
	{
		bool updated = false;

		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery_status;

			if (battery_sub.update(&battery_status)) {
				/* battery status message with higher resolution */
				mavlink_battery_status_t bat_msg{};
				// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
				bat_msg.id = battery_status.id - 1;
				bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
				bat_msg.type = MAV_BATTERY_TYPE_LIPO;
				bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
				bat_msg.energy_consumed = -1;
				bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
				bat_msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;
				bat_msg.time_remaining = (battery_status.connected) ? battery_status.run_time_to_empty * 60 : 0;

				switch (battery_status.warning) {
				case (battery_status_s::BATTERY_WARNING_NONE):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
					break;

				case (battery_status_s::BATTERY_WARNING_LOW):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
					break;

				case (battery_status_s::BATTERY_WARNING_CRITICAL):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
					break;

				case (battery_status_s::BATTERY_WARNING_EMERGENCY):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
					break;

				case (battery_status_s::BATTERY_WARNING_FAILED):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
					break;

				default:
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
					break;
				}

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.0f;

				} else {
					bat_msg.temperature = INT16_MAX;
				}

				static constexpr int mavlink_cells_max = (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0]));
				static constexpr int uorb_cells_max =
					(sizeof(battery_status.voltage_cell_v) / sizeof(battery_status.voltage_cell_v[0]));

				for (int cell = 0; cell < mavlink_cells_max; cell++) {
					if (battery_status.connected && (cell < battery_status.cell_count) && (cell < uorb_cells_max)) {
						bat_msg.voltages[cell] = battery_status.voltage_cell_v[cell] * 1000.0f;

					} else {
						bat_msg.voltages[cell] = UINT16_MAX;
					}
				}

				mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

				updated = true;
			}
		}

		return updated;
	}
};


class MavlinkStreamSmartBatteryInfo : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSmartBatteryInfo::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SMART_BATTERY_INFO";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SMART_BATTERY_INFO;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSmartBatteryInfo(mavlink);
	}

	unsigned get_size() override
	{
		static constexpr unsigned size_per_battery = MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return size_per_battery * _battery_status_subs.advertised_count();
	}

private:
	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};

	/* do not allow top copying this class */
	MavlinkStreamSmartBatteryInfo(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSmartBatteryInfo &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSmartBatteryInfo(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send() override
	{
		bool updated = false;

		for (auto &battery_sub : _battery_status_subs) {
			battery_status_s battery_status;

			if (battery_sub.update(&battery_status)) {
				if (battery_status.serial_number == 0) {
					//This is not smart battery
					continue;
				}

				mavlink_smart_battery_info_t msg = {};

				msg.id = battery_status.id - 1;
				msg.capacity_full_specification = battery_status.capacity;
				msg.capacity_full = (int32_t)((float)(battery_status.state_of_health * battery_status.capacity) / 100.f);
				msg.cycle_count = battery_status.cycle_count;

				if (battery_status.manufacture_date) {
					uint16_t day = battery_status.manufacture_date % 32;
					uint16_t month = (battery_status.manufacture_date >> 5) % 16;
					uint16_t year = (80 + (battery_status.manufacture_date >> 9)) % 100;

					//Formatted as 'dd/mm/yy-123456' (maxed 15 + 1 chars)
					snprintf(msg.serial_number, sizeof(msg.serial_number), "%d/%d/%d-%d", day, month, year, battery_status.serial_number);

				} else {
					snprintf(msg.serial_number, sizeof(msg.serial_number), "%d", battery_status.serial_number);
				}

				//msg.device_name = ??
				msg.weight = -1;
				msg.discharge_minimum_voltage = -1;
				msg.charging_minimum_voltage = -1;
				msg.resting_minimum_voltage = -1;


				mavlink_msg_smart_battery_info_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}

		}

		return updated;
	}
};

class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighresIMU(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::SubscriptionMultiArray<vehicle_imu_s, 3> _vehicle_imu_subs{ORB_ID::vehicle_imu};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &) = delete;
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &) = delete;

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		bool updated = false;

		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		vehicle_imu_s imu;

		for (auto &imu_sub : _vehicle_imu_subs) {
			if (imu_sub.update(&imu)) {
				if (imu.accel_device_id == sensor_selection.accel_device_id) {
					updated = true;
					break;
				}
			}
		}

		if (updated) {
			uint16_t fields_updated = 0;

			fields_updated |= (1 << 0) | (1 << 1) | (1 << 2); // accel
			fields_updated |= (1 << 3) | (1 << 4) | (1 << 5); // gyro

			vehicle_magnetometer_s magnetometer{};

			if (_magnetometer_sub.update(&magnetometer)) {
				// mark third group dimensions as changed
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);

			} else {
				_magnetometer_sub.copy(&magnetometer);
			}

			// find corresponding estimated sensor bias
			if (_estimator_selector_status_sub.updated()) {
				estimator_selector_status_s estimator_selector_status;

				if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
					_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
				}
			}

			Vector3f accel_bias{0.f, 0.f, 0.f};
			Vector3f gyro_bias{0.f, 0.f, 0.f};
			Vector3f mag_bias{0.f, 0.f, 0.f};

			{
				estimator_sensor_bias_s bias;

				if (_estimator_sensor_bias_sub.copy(&bias)) {
					if ((bias.accel_device_id != 0) && (bias.accel_device_id == imu.accel_device_id)) {
						accel_bias = Vector3f{bias.accel_bias};
					}

					if ((bias.gyro_device_id != 0) && (bias.gyro_device_id == imu.gyro_device_id)) {
						gyro_bias = Vector3f{bias.gyro_bias};
					}

					if ((bias.mag_device_id != 0) && (bias.mag_device_id == magnetometer.device_id)) {
						mag_bias = Vector3f{bias.mag_bias};

					} else {
						// find primary mag
						uORB::SubscriptionMultiArray<vehicle_magnetometer_s> mag_subs{ORB_ID::vehicle_magnetometer};

						for (int i = 0; i < mag_subs.size(); i++) {
							if (mag_subs[i].advertised() && mag_subs[i].copy(&magnetometer)) {
								if (magnetometer.device_id == bias.mag_device_id) {
									_magnetometer_sub.ChangeInstance(i);
									break;
								}
							}

						}
					}
				}
			}

			const Vector3f mag = Vector3f{magnetometer.magnetometer_ga} - mag_bias;

			vehicle_air_data_s air_data{};

			if (_air_data_sub.update(&air_data)) {
				/* mark fourth group (baro fields) dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);

			} else {
				_air_data_sub.copy(&air_data);
			}

			differential_pressure_s differential_pressure{};

			if (_differential_pressure_sub.update(&differential_pressure)) {
				/* mark fourth group (dpres field) dimensions as changed */
				fields_updated |= (1 << 10);

			} else {
				_differential_pressure_sub.copy(&differential_pressure);
			}

			const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
			const Vector3f accel = (Vector3f{imu.delta_velocity} * accel_dt_inv) - accel_bias;

			const float gyro_dt_inv = 1.e6f / (float)imu.delta_angle_dt;
			const Vector3f gyro = (Vector3f{imu.delta_angle} * gyro_dt_inv) - gyro_bias;

			mavlink_highres_imu_t msg{};

			msg.time_usec = imu.timestamp_sample;
			msg.xacc = accel(0);
			msg.yacc = accel(1);
			msg.zacc = accel(2);
			msg.xgyro = gyro(0);
			msg.ygyro = gyro(1);
			msg.zgyro = gyro(2);
			msg.xmag = mag(0);
			msg.ymag = mag(1);
			msg.zmag = mag(2);
			msg.abs_pressure = air_data.baro_pressure_pa;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = air_data.baro_alt_meter;
			msg.temperature = air_data.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N, typename Derived>
class MavlinkStreamScaledPressureBase : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return Derived::get_name_static();
	}

	uint16_t get_id() override
	{
		return Derived::get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new Derived(mavlink);
	}

private:
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _sensor_baro_sub{ORB_ID(sensor_baro), N};

	/* do not allow top copying this class */
	MavlinkStreamScaledPressureBase(MavlinkStreamScaledPressureBase &) = delete;
	MavlinkStreamScaledPressureBase &operator = (const MavlinkStreamScaledPressureBase &) = delete;

protected:
	explicit MavlinkStreamScaledPressureBase(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		if (_sensor_baro_sub.updated() || _differential_pressure_sub.updated()) {
			sensor_baro_s sensor_baro{};
			differential_pressure_s differential_pressure{};
			_sensor_baro_sub.copy(&sensor_baro);
			_differential_pressure_sub.copy(&differential_pressure);

			typename Derived::mav_msg_type msg{};
			msg.time_boot_ms = sensor_baro.timestamp / 1000;
			msg.press_abs = sensor_baro.pressure;
			msg.press_diff = differential_pressure.differential_pressure_raw_pa;
			msg.temperature = sensor_baro.temperature;

			Derived::send(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N> class MavlinkStreamScaledPressure {};

template <>
class MavlinkStreamScaledPressure<0> : public MavlinkStreamScaledPressureBase<0, MavlinkStreamScaledPressure<0> >
{
public:
	typedef MavlinkStreamScaledPressureBase<0, MavlinkStreamScaledPressure<0> > Base;
	typedef mavlink_scaled_pressure_t mav_msg_type;

	explicit MavlinkStreamScaledPressure(Mavlink *mavlink) : Base(mavlink) {}

	static void send(mavlink_channel_t channel, const MavlinkStreamScaledPressure<0>::mav_msg_type *msg)
	{
		mavlink_msg_scaled_pressure_send_struct(channel, msg);
	}

	static const char *get_name_static()
	{
		return "SCALED_PRESSURE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE;
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
};

template <>
class MavlinkStreamScaledPressure<1> : public MavlinkStreamScaledPressureBase<1, MavlinkStreamScaledPressure<1> >
{
public:
	typedef MavlinkStreamScaledPressureBase<1, MavlinkStreamScaledPressure<1> > Base;
	typedef mavlink_scaled_pressure2_t mav_msg_type;

	explicit MavlinkStreamScaledPressure(Mavlink *mavlink) : Base(mavlink) {}

	static void send(mavlink_channel_t channel, const MavlinkStreamScaledPressure<1>::mav_msg_type *msg)
	{
		mavlink_msg_scaled_pressure2_send_struct(channel, msg);
	}

	static const char *get_name_static()
	{
		return "SCALED_PRESSURE2";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE2;
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
};

template <>
class MavlinkStreamScaledPressure<2> : public MavlinkStreamScaledPressureBase<2, MavlinkStreamScaledPressure<2> >
{
public:
	typedef MavlinkStreamScaledPressureBase<2, MavlinkStreamScaledPressure<2> > Base;
	typedef mavlink_scaled_pressure3_t mav_msg_type;

	explicit MavlinkStreamScaledPressure(Mavlink *mavlink) : Base(mavlink) {}

	static void send(mavlink_channel_t channel, const MavlinkStreamScaledPressure<2>::mav_msg_type *msg)
	{
		mavlink_msg_scaled_pressure3_send_struct(channel, msg);
	}

	static const char *get_name_static()
	{
		return "SCALED_PRESSURE3";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE3;
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
};

class MavlinkStreamVFRHUD : public MavlinkStream
{
public:

	const char *get_name() const override
	{
		return MavlinkStreamVFRHUD::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "VFR_HUD";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VFR_HUD;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVFRHUD(mavlink);
	}

	unsigned get_size() override
	{
		if (_lpos_sub.advertised() || _airspeed_validated_sub.advertised()) {
			return MAVLINK_MSG_ID_VFR_HUD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _act0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _act1_sub{ORB_ID(actuator_controls_1)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &) = delete;
	MavlinkStreamVFRHUD &operator = (const MavlinkStreamVFRHUD &) = delete;

protected:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		if (_lpos_sub.updated() || _airspeed_validated_sub.updated()) {

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			actuator_armed_s armed{};
			_armed_sub.copy(&armed);

			airspeed_validated_s airspeed_validated{};
			_airspeed_validated_sub.copy(&airspeed_validated);

			mavlink_vfr_hud_t msg{};
			msg.airspeed = airspeed_validated.calibrated_airspeed_m_s;
			msg.groundspeed = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
			msg.heading = math::degrees(wrap_2pi(lpos.heading));

			if (armed.armed) {
				actuator_controls_s act0{};
				actuator_controls_s act1{};
				_act0_sub.copy(&act0);
				_act1_sub.copy(&act1);

				// VFR_HUD throttle should only be used for operator feedback.
				// VTOLs switch between actuator_controls_0 and actuator_controls_1. During transition there isn't a
				// a single throttle value, but this should still be a useful heuristic for operator awareness.
				//
				// Use ACTUATOR_CONTROL_TARGET if accurate states are needed.
				msg.throttle = 100 * math::max(
						       act0.control[actuator_controls_s::INDEX_THROTTLE],
						       act1.control[actuator_controls_s::INDEX_THROTTLE]);

			} else {
				msg.throttle = 0.0f;
			}

			if (lpos.z_valid && lpos.z_global) {
				/* use local position estimate */
				msg.alt = -lpos.z + lpos.ref_alt;

			} else {
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				/* fall back to baro altitude */
				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter;
				}
			}

			if (lpos.v_z_valid) {
				msg.climb = -lpos.vz;
			}

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size() override
	{
		return _gps_sub.advertised() ? MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps), 0};

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &) = delete;
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &) = delete;

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		sensor_gps_s gps;

		if (_gps_sub.update(&gps)) {
			mavlink_gps_raw_int_t msg{};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.alt_ellipsoid = gps.alt_ellipsoid;
			msg.eph = gps.hdop * 100;
			msg.epv = gps.vdop * 100;
			msg.h_acc = gps.eph * 1e3f;
			msg.v_acc = gps.epv * 1e3f;
			msg.vel_acc = gps.s_variance_m_s * 1e3f;
			msg.hdg_acc = gps.c_variance_rad * 1e5f / M_DEG_TO_RAD_F;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGPS2Raw : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGPS2Raw::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS2_RAW";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS2_RAW;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPS2Raw(mavlink);
	}

	unsigned get_size() override
	{
		return _gps2_sub.advertised() ? (MAVLINK_MSG_ID_GPS2_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _gps2_sub{ORB_ID(sensor_gps), 1};

	/* do not allow top copying this class */
	MavlinkStreamGPS2Raw(MavlinkStreamGPS2Raw &) = delete;
	MavlinkStreamGPS2Raw &operator = (const MavlinkStreamGPS2Raw &) = delete;

protected:
	explicit MavlinkStreamGPS2Raw(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		sensor_gps_s gps;

		if (_gps2_sub.update(&gps)) {
			mavlink_gps2_raw_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.eph * 1e3f;
			msg.epv = gps.epv * 1e3f;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			//msg.dgps_numch = // Number of DGPS satellites
			//msg.dgps_age = // Age of DGPS info

			mavlink_msg_gps2_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#if !defined(CONSTRAINED_FLASH)
class MavlinkStreamAutopilotStateForGimbalDevice : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAutopilotStateForGimbalDevice::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAutopilotStateForGimbalDevice(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _est_sub{ORB_ID(estimator_status)};
	uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};

	/* do not allow top copying this class */
	MavlinkStreamAutopilotStateForGimbalDevice(MavlinkStreamAutopilotStateForGimbalDevice &) = delete;
	MavlinkStreamAutopilotStateForGimbalDevice &operator = (const MavlinkStreamAutopilotStateForGimbalDevice &) = delete;

protected:
	explicit MavlinkStreamAutopilotStateForGimbalDevice(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		if (_att_sub.advertised()) {

			mavlink_autopilot_state_for_gimbal_device_t msg{};

			{
				vehicle_attitude_s att{};
				_att_sub.copy(&att);
				msg.time_boot_us = att.timestamp;
				msg.q[0] = att.q[0];
				msg.q[1] = att.q[1];
				msg.q[2] = att.q[2];
				msg.q[3] = att.q[3];
				msg.q_estimated_delay_us = 0; // I don't know.
			}

			{
				vehicle_local_position_s lpos{};
				_lpos_sub.copy(&lpos);
				msg.vx = lpos.vx;
				msg.vy = lpos.vy;
				msg.vz = lpos.vz;
				msg.v_estimated_delay_us = 0; // I don't know.
			}

			{
				vehicle_attitude_setpoint_s att_sp{};
				_att_sp_sub.copy(&att_sp);
				msg.feed_forward_angular_velocity_z = att_sp.yaw_sp_move_rate;
			}

			{
				estimator_status_s est{};
				_est_sub.copy(&est);
				msg.estimator_status = est.solution_status_flags;
			}

			{
				vehicle_land_detected_s land_detected{};
				_landed_sub.copy(&land_detected);

				// Ignore take-off and landing states for now.
				msg.landed_state = land_detected.landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR;
			}

			mavlink_msg_autopilot_state_for_gimbal_device_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}

		return false;
	}
};
#endif

class MavlinkStreamSystemTime : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSystemTime::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SYSTEM_TIME";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSystemTime(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &) = delete;
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &) = delete;

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);

		mavlink_system_time_t msg{};
		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		// and we don't need to bother sending it because it's definitely wrong.
		if (msg.time_unix_usec > 978307200000000) {
			mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

class MavlinkStreamTimesync : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamTimesync::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "TIMESYNC";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TIMESYNC;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTimesync(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_TIMESYNC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamTimesync(MavlinkStreamTimesync &) = delete;
	MavlinkStreamTimesync &operator = (const MavlinkStreamTimesync &) = delete;

protected:
	explicit MavlinkStreamTimesync(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		mavlink_timesync_t msg{};

		msg.tc1 = 0;
		msg.ts1 = hrt_absolute_time() * 1000; // boot time in nanoseconds

		mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamADSBVehicle : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamADSBVehicle::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ADSB_VEHICLE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ADSB_VEHICLE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamADSBVehicle(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _pos_sub.advertised() ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_sub{ORB_ID(transponder_report)};

	/* do not allow top copying this class */
	MavlinkStreamADSBVehicle(MavlinkStreamADSBVehicle &) = delete;
	MavlinkStreamADSBVehicle &operator = (const MavlinkStreamADSBVehicle &) = delete;

protected:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		transponder_report_s pos;
		bool sent = false;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _pos_sub.update(&pos)) {

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) {
				continue;
			}

			mavlink_adsb_vehicle_t msg{};
			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamUTMGlobalPosition : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamUTMGlobalPosition::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "UTM_GLOBAL_POSITION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamUTMGlobalPosition(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _global_pos_sub.advertised() ? MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};

	/* do not allow top copying this class */
	MavlinkStreamUTMGlobalPosition(MavlinkStreamUTMGlobalPosition &) = delete;
	MavlinkStreamUTMGlobalPosition &operator = (const MavlinkStreamUTMGlobalPosition &) = delete;

protected:
	explicit MavlinkStreamUTMGlobalPosition(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		vehicle_global_position_s global_pos;

		if (_global_pos_sub.update(&global_pos)) {
			mavlink_utm_global_position_t msg{};

			// Compute Unix epoch and set time field
			timespec tv;
			px4_clock_gettime(CLOCK_REALTIME, &tv);
			uint64_t unix_epoch = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

			// If the time is before 2001-01-01, it's probably the default 2000
			if (unix_epoch > 978307200000000) {
				msg.time = unix_epoch;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_TIME_VALID;
			}

#ifndef BOARD_HAS_NO_UUID
			px4_guid_t px4_guid;
			board_get_px4_guid(px4_guid);
			static_assert(sizeof(px4_guid_t) == sizeof(msg.uas_id), "GUID byte length mismatch");
			memcpy(&msg.uas_id, &px4_guid, sizeof(msg.uas_id));
			msg.flags |= UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE;
#else
			// TODO Fill ID with something reasonable
			memset(&msg.uas_id[0], 0, sizeof(msg.uas_id));
#endif /* BOARD_HAS_NO_UUID */

			// Handle global position
			msg.lat = global_pos.lat * 1e7;
			msg.lon = global_pos.lon * 1e7;
			msg.alt = global_pos.alt_ellipsoid * 1000.0f;

			msg.h_acc = global_pos.eph * 1000.0f;
			msg.v_acc = global_pos.epv * 1000.0f;

			msg.flags |= UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE;

			// Handle local position
			vehicle_local_position_s local_pos;

			if (_local_pos_sub.copy(&local_pos)) {
				float evh = 0.0f;
				float evv = 0.0f;

				if (local_pos.v_xy_valid) {
					msg.vx = local_pos.vx * 100.0f;
					msg.vy = local_pos.vy * 100.0f;
					evh = local_pos.evh;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE;
				}

				if (local_pos.v_z_valid) {
					msg.vz = local_pos.vz * 100.0f;
					evv = local_pos.evv;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE;
				}

				msg.vel_acc = sqrtf(evh * evh + evv * evv) * 100.0f;

				if (local_pos.dist_bottom_valid) {
					msg.relative_alt = local_pos.dist_bottom * 1000.0f;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE;
				}
			}

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			bool vehicle_in_auto_mode = vehicle_status.timestamp > 0
						    && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

			// Handle next waypoint if it is valid
			position_setpoint_triplet_s position_setpoint_triplet;

			if (vehicle_in_auto_mode && _position_setpoint_triplet_sub.copy(&position_setpoint_triplet)) {
				if (position_setpoint_triplet.current.valid) {
					msg.next_lat = position_setpoint_triplet.current.lat * 1e7;
					msg.next_lon = position_setpoint_triplet.current.lon * 1e7;
					// HACK We assume that the offset between AMSL and WGS84 is constant between the current
					// vehicle position and the the target waypoint.
					msg.next_alt = (position_setpoint_triplet.current.alt + (global_pos.alt_ellipsoid - global_pos.alt)) * 1000.0f;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE;
				}
			}

			// Handle flight state
			vehicle_land_detected_s land_detected{};
			_land_detected_sub.copy(&land_detected);

			if (vehicle_status.timestamp > 0 && land_detected.timestamp > 0
			    && vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				if (land_detected.landed) {
					msg.flight_state |= UTM_FLIGHT_STATE_GROUND;

				} else {
					msg.flight_state |= UTM_FLIGHT_STATE_AIRBORNE;
				}

			} else {
				msg.flight_state |= UTM_FLIGHT_STATE_UNKNOWN;
			}

			msg.update_rate = 0; // Data driven mode

			mavlink_msg_utm_global_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#if !defined(CONSTRAINED_FLASH)
class MavlinkStreamGimbalDeviceAttitudeStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGimbalDeviceAttitudeStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GIMBAL_DEVICE_ATTITUDE_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGimbalDeviceAttitudeStatus(mavlink);
	}

	unsigned get_size() override
	{
		return _gimbal_device_attitude_status_sub.advertised() ? MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _gimbal_device_attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};

	/* do not allow top copying this class */
	MavlinkStreamGimbalDeviceAttitudeStatus(MavlinkStreamGimbalDeviceAttitudeStatus &) = delete;
	MavlinkStreamGimbalDeviceAttitudeStatus &operator = (const MavlinkStreamGimbalDeviceAttitudeStatus &) = delete;

protected:
	explicit MavlinkStreamGimbalDeviceAttitudeStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		gimbal_device_attitude_status_s gimbal_device_attitude_status{};

		if (_gimbal_device_attitude_status_sub.update(&gimbal_device_attitude_status)) {
			mavlink_gimbal_device_attitude_status_t msg{};

			msg.time_boot_ms = gimbal_device_attitude_status.timestamp / 1000;
			msg.q[0] = gimbal_device_attitude_status.q[0];
			msg.q[1] = gimbal_device_attitude_status.q[1];
			msg.q[2] = gimbal_device_attitude_status.q[2];
			msg.q[3] = gimbal_device_attitude_status.q[3];
			msg.angular_velocity_x = gimbal_device_attitude_status.angular_velocity_x;
			msg.angular_velocity_y = gimbal_device_attitude_status.angular_velocity_y;
			msg.angular_velocity_z = gimbal_device_attitude_status.angular_velocity_z;
			msg.failure_flags = gimbal_device_attitude_status.failure_flags;
			msg.flags = gimbal_device_attitude_status.device_flags;

			mavlink_msg_gimbal_device_attitude_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGimbalManagerInformation : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGimbalManagerInformation::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GIMBAL_MANAGER_INFORMATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGimbalManagerInformation(mavlink);
	}

	unsigned get_size() override
	{
		return _gimbal_manager_information_sub.advertised() ? (MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _gimbal_manager_information_sub{ORB_ID(gimbal_manager_information)};

	/* do not allow top copying this class */
	MavlinkStreamGimbalManagerInformation(MavlinkStreamGimbalManagerInformation &) = delete;
	MavlinkStreamGimbalManagerInformation &operator = (const MavlinkStreamGimbalManagerInformation &) = delete;

protected:
	explicit MavlinkStreamGimbalManagerInformation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		gimbal_manager_information_s gimbal_manager_information;

		if (_gimbal_manager_information_sub.advertised() && _gimbal_manager_information_sub.copy(&gimbal_manager_information)) {
			// send out gimbal_manager_info with info from gimbal_manager_information
			mavlink_gimbal_manager_information_t msg{};
			msg.time_boot_ms = gimbal_manager_information.timestamp / 1000;
			msg.gimbal_device_id = 0;
			msg.cap_flags = gimbal_manager_information.cap_flags;

			msg.roll_min = gimbal_manager_information.roll_min;
			msg.roll_max = gimbal_manager_information.roll_max;
			msg.pitch_min = gimbal_manager_information.pitch_min;
			msg.pitch_max = gimbal_manager_information.pitch_max;
			msg.yaw_min = gimbal_manager_information.yaw_min;
			msg.yaw_max = gimbal_manager_information.yaw_max;

			mavlink_msg_gimbal_manager_information_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}

		return false;
	}
};

class MavlinkStreamGimbalManagerStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGimbalManagerStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GIMBAL_MANAGER_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGimbalManagerStatus(mavlink);
	}

	unsigned get_size() override
	{
		return _gimbal_manager_status_sub.advertised() ? (MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _gimbal_manager_status_sub{ORB_ID(gimbal_manager_status)};

	/* do not allow top copying this class */
	MavlinkStreamGimbalManagerStatus(MavlinkStreamGimbalManagerStatus &) = delete;
	MavlinkStreamGimbalManagerStatus &operator = (const MavlinkStreamGimbalManagerStatus &) = delete;

protected:
	explicit MavlinkStreamGimbalManagerStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		gimbal_manager_status_s gimbal_manager_status;

		if (_gimbal_manager_status_sub.advertised() && _gimbal_manager_status_sub.copy(&gimbal_manager_status)) {
			mavlink_gimbal_manager_status_t msg{};
			msg.time_boot_ms = gimbal_manager_status.timestamp / 1000;
			msg.gimbal_device_id = gimbal_manager_status.gimbal_device_id;
			msg.primary_control_sysid = gimbal_manager_status.primary_control_sysid;
			msg.primary_control_compid = gimbal_manager_status.primary_control_compid;
			msg.secondary_control_sysid = gimbal_manager_status.secondary_control_sysid;
			msg.secondary_control_compid = gimbal_manager_status.secondary_control_compid;
			msg.flags = gimbal_manager_status.flags;

			mavlink_msg_gimbal_manager_status_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}

		return false;
	}
};



class MavlinkStreamGimbalDeviceSetAttitude : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGimbalDeviceSetAttitude::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GIMBAL_DEVICE_SET_ATTITUDE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGimbalDeviceSetAttitude(mavlink);
	}

	unsigned get_size() override
	{
		return _gimbal_device_set_attitude_sub.advertised() ? (MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _gimbal_device_set_attitude_sub{ORB_ID(gimbal_device_set_attitude)};

	/* do not allow top copying this class */
	MavlinkStreamGimbalDeviceSetAttitude(MavlinkStreamGimbalDeviceSetAttitude &) = delete;
	MavlinkStreamGimbalDeviceSetAttitude &operator = (const MavlinkStreamGimbalDeviceSetAttitude &) = delete;

protected:
	explicit MavlinkStreamGimbalDeviceSetAttitude(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		gimbal_device_set_attitude_s gimbal_device_set_attitude;

		if (_gimbal_device_set_attitude_sub.advertised() && _gimbal_device_set_attitude_sub.copy(&gimbal_device_set_attitude)) {
			mavlink_gimbal_device_set_attitude_t msg{};
			msg.flags = gimbal_device_set_attitude.flags;
			msg.target_system = gimbal_device_set_attitude.target_system;
			msg.target_component = gimbal_device_set_attitude.target_component;

			msg.q[0] = gimbal_device_set_attitude.q[0];
			msg.q[1] = gimbal_device_set_attitude.q[1];
			msg.q[2] = gimbal_device_set_attitude.q[2];
			msg.q[3] = gimbal_device_set_attitude.q[3];

			msg.angular_velocity_x = gimbal_device_set_attitude.angular_velocity_x;
			msg.angular_velocity_y = gimbal_device_set_attitude.angular_velocity_y;
			msg.angular_velocity_z = gimbal_device_set_attitude.angular_velocity_z;

			mavlink_msg_gimbal_device_set_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}

		return false;
	}
};
#endif

class MavlinkStreamCameraTrigger : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraTrigger::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_TRIGGER";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_TRIGGER;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraTrigger(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		if (_trigger_sub.advertised()) {
			return MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
			       + MAVLINK_MSG_ID_COMMAND_LONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; // TODO: MAV_CMD_DO_DIGICAM_CONTROL
		}

		return 0;
	}

private:
	uORB::Subscription _trigger_sub{ORB_ID(camera_trigger)};

	/* do not allow top copying this class */
	MavlinkStreamCameraTrigger(MavlinkStreamCameraTrigger &) = delete;
	MavlinkStreamCameraTrigger &operator = (const MavlinkStreamCameraTrigger &) = delete;

protected:
	explicit MavlinkStreamCameraTrigger(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		camera_trigger_s trigger;

		if ((_mavlink->get_free_tx_buf() >= get_size()) && _trigger_sub.update(&trigger)) {
			mavlink_camera_trigger_t msg{};

			msg.time_usec = trigger.timestamp;
			msg.seq = trigger.seq;

			/* ensure that only active trigger events are sent */
			if (trigger.timestamp > 0) {

				mavlink_msg_camera_trigger_send_struct(_mavlink->get_channel(), &msg);

				vehicle_command_s vcmd{};
				vcmd.timestamp = hrt_absolute_time();
				vcmd.param1 = 0.0f; // all cameras
				vcmd.param2 = 0.0f; // duration 0 because only taking one picture
				vcmd.param3 = 1.0f; // only take one
				vcmd.param4 = NAN;
				vcmd.param5 = (double)NAN;
				vcmd.param6 = (double)NAN;
				vcmd.param7 = NAN;
				vcmd.command = MAV_CMD_IMAGE_START_CAPTURE;
				vcmd.target_system = mavlink_system.sysid;
				vcmd.target_component = MAV_COMP_ID_CAMERA;

				MavlinkCommandSender::instance().handle_vehicle_command(vcmd, _mavlink->get_channel());

				// TODO: move this camera_trigger and publish as a vehicle_command
				/* send MAV_CMD_DO_DIGICAM_CONTROL*/
				mavlink_command_long_t digicam_ctrl_cmd{};

				digicam_ctrl_cmd.target_system = 0; // 0 for broadcast
				digicam_ctrl_cmd.target_component = MAV_COMP_ID_CAMERA;
				digicam_ctrl_cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
				digicam_ctrl_cmd.confirmation = 0;
				digicam_ctrl_cmd.param1 = NAN;
				digicam_ctrl_cmd.param2 = NAN;
				digicam_ctrl_cmd.param3 = NAN;
				digicam_ctrl_cmd.param4 = NAN;
				digicam_ctrl_cmd.param5 = 1;   // take 1 picture
				digicam_ctrl_cmd.param6 = NAN;
				digicam_ctrl_cmd.param7 = NAN;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &digicam_ctrl_cmd);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCameraImageCaptured : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraImageCaptured::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_IMAGE_CAPTURED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	bool const_rate() override
	{
		return true;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraImageCaptured(mavlink);
	}

	unsigned get_size() override
	{
		return _capture_sub.advertised() ? MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _capture_sub{ORB_ID(camera_capture)};

	/* do not allow top copying this class */
	MavlinkStreamCameraImageCaptured(MavlinkStreamCameraImageCaptured &) = delete;
	MavlinkStreamCameraImageCaptured &operator = (const MavlinkStreamCameraImageCaptured &) = delete;

protected:
	explicit MavlinkStreamCameraImageCaptured(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		camera_capture_s capture;

		if ((_mavlink->get_free_tx_buf() >= get_size()) && _capture_sub.update(&capture)) {
			mavlink_camera_image_captured_t msg{};

			msg.time_boot_ms = capture.timestamp / 1000;
			msg.time_utc = capture.timestamp_utc;
			msg.camera_id = 1;	// FIXME : get this from uORB
			msg.lat = capture.lat * 1e7;
			msg.lon = capture.lon * 1e7;
			msg.alt = capture.alt * 1e3f;
			msg.relative_alt = capture.ground_distance * 1e3f;
			msg.q[0] = capture.q[0];
			msg.q[1] = capture.q[1];
			msg.q[2] = capture.q[2];
			msg.q[3] = capture.q[3];
			msg.image_index = capture.seq;
			msg.capture_result = capture.result;
			msg.file_url[0] = '\0';

			mavlink_msg_camera_image_captured_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGlobalPositionInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGlobalPositionInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GLOBAL_POSITION_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGlobalPositionInt(mavlink);
	}

	unsigned get_size() override
	{
		return _gpos_sub.advertised() ? MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamGlobalPositionInt(MavlinkStreamGlobalPositionInt &) = delete;
	MavlinkStreamGlobalPositionInt &operator = (const MavlinkStreamGlobalPositionInt &) = delete;

protected:
	explicit MavlinkStreamGlobalPositionInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		vehicle_global_position_s gpos;
		vehicle_local_position_s lpos;

		if (_gpos_sub.update(&gpos) && _lpos_sub.update(&lpos)) {

			mavlink_global_position_int_t msg{};

			if (lpos.z_valid && lpos.z_global) {
				msg.alt = (-lpos.z + lpos.ref_alt) * 1000.0f;

			} else {
				// fall back to baro altitude
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter * 1000.0f;
				}
			}

			home_position_s home{};
			_home_sub.copy(&home);

			if ((home.timestamp > 0) && home.valid_alt) {
				if (lpos.z_valid) {
					msg.relative_alt = -(lpos.z - home.z) * 1000.0f;

				} else {
					msg.relative_alt = msg.alt - (home.alt * 1000.0f);
				}

			} else {
				if (lpos.z_valid) {
					msg.relative_alt = -lpos.z * 1000.0f;
				}
			}

			msg.time_boot_ms = gpos.timestamp / 1000;
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;

			msg.vx = lpos.vx * 100.0f;
			msg.vy = lpos.vy * 100.0f;
			msg.vz = lpos.vz * 100.0f;

			msg.hdg = math::degrees(wrap_2pi(lpos.heading)) * 100.0f;

			mavlink_msg_global_position_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamCameraCapture : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraCapture::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_CAPTURE";
	}

	static constexpr uint16_t get_id_static()
	{
		return 0;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraCapture(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_COMMAND_LONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamCameraCapture(MavlinkStreamCameraCapture &) = delete;
	MavlinkStreamCameraCapture &operator = (const MavlinkStreamCameraCapture &) = delete;

protected:
	explicit MavlinkStreamCameraCapture(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		vehicle_status_s status;

		if (_status_sub.update(&status)) {
			mavlink_command_long_t msg{};

			msg.target_system = 0;
			msg.target_component = MAV_COMP_ID_ALL;
			msg.command = MAV_CMD_DO_CONTROL_VIDEO;
			msg.confirmation = 0;
			msg.param1 = 0;
			msg.param2 = 0;
			msg.param3 = 0;
			/* set camera capture ON/OFF depending on arming state */
			msg.param4 = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1 : 0;
			msg.param5 = 0;
			msg.param6 = 0;
			msg.param7 = 0;

			mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
		}

		return true;
	}
};

static const StreamListItem streams_list[] = {
	create_stream_list_item<MavlinkStreamHeartbeat>(),
#if defined(STATUSTEXT_HPP)
	create_stream_list_item<MavlinkStreamStatustext>(),
#endif // STATUSTEXT_HPP
	create_stream_list_item<MavlinkStreamCommandLong>(),
	create_stream_list_item<MavlinkStreamSysStatus>(),
	create_stream_list_item<MavlinkStreamBatteryStatus>(),
	create_stream_list_item<MavlinkStreamSmartBatteryInfo>(),
	create_stream_list_item<MavlinkStreamHighresIMU>(),
#if defined(SCALED_IMU_HPP)
	create_stream_list_item<MavlinkStreamScaledIMU<0> >(),
	create_stream_list_item<MavlinkStreamScaledIMU<1> >(),
	create_stream_list_item<MavlinkStreamScaledIMU<2> >(),
#endif // SCALED_IMU_HPP
	create_stream_list_item<MavlinkStreamScaledPressure<0> >(),
	// create_stream_list_item<MavlinkStreamScaledPressure<1> >(),
	// create_stream_list_item<MavlinkStreamScaledPressure<2> >(),
#if defined(ACTUATOR_OUTPUT_STATUS_HPP)
	create_stream_list_item<MavlinkStreamActuatorOutputStatus>(),
#endif // ACTUATOR_OUTPUT_STATUS_HPP
#if defined(ATTITUDE_HPP)
	create_stream_list_item<MavlinkStreamAttitude>(),
#endif // ATTITUDE_HPP
#if defined(ATTITUDE_QUATERNION_HPP)
	create_stream_list_item<MavlinkStreamAttitudeQuaternion>(),
#endif // ATTITUDE_QUATERNION_HPP
	create_stream_list_item<MavlinkStreamVFRHUD>(),
#if defined(GPS_GLOBAL_ORIGIN_HPP)
	create_stream_list_item<MavlinkStreamGpsGlobalOrigin>(),
#endif // GPS_GLOBAL_ORIGIN_HPP
	create_stream_list_item<MavlinkStreamGPSRawInt>(),
	create_stream_list_item<MavlinkStreamGPS2Raw>(),
	create_stream_list_item<MavlinkStreamSystemTime>(),
	create_stream_list_item<MavlinkStreamTimesync>(),
	create_stream_list_item<MavlinkStreamGlobalPositionInt>(),
#if defined(LOCAL_POSITION_NED_HPP)
	create_stream_list_item<MavlinkStreamLocalPositionNED>(),
#endif // LOCAL_POSITION_NED_HPP
#if defined(ODOMETRY_HPP)
	create_stream_list_item<MavlinkStreamOdometry>(),
#endif // ODOMETRY_HPP
#if defined(ESTIMATOR_STATUS_HPP)
	create_stream_list_item<MavlinkStreamEstimatorStatus>(),
#endif // ESTIMATOR_STATUS_HPP
#if defined(VIBRATION_HPP)
	create_stream_list_item<MavlinkStreamVibration>(),
#endif // VIBRATION_HPP
#if defined(ATT_POS_MOCAP_HPP)
	create_stream_list_item<MavlinkStreamAttPosMocap>(),
#endif // ATT_POS_MOCAP_HPP
#if !defined(CONSTRAINED_FLASH)
	create_stream_list_item<MavlinkStreamGimbalDeviceAttitudeStatus>(),
	create_stream_list_item<MavlinkStreamGimbalManagerInformation>(),
	create_stream_list_item<MavlinkStreamGimbalManagerStatus>(),
	create_stream_list_item<MavlinkStreamAutopilotStateForGimbalDevice>(),
	create_stream_list_item<MavlinkStreamGimbalDeviceSetAttitude>(),
#endif
#if defined(HOME_POSITION_HPP)
	create_stream_list_item<MavlinkStreamHomePosition>(),
#endif // HOME_POSITION_HPP
#if defined(SERVO_OUTPUT_RAW_HPP)
	create_stream_list_item<MavlinkStreamServoOutputRaw<0> >(),
	create_stream_list_item<MavlinkStreamServoOutputRaw<1> >(),
#endif // SERVO_OUTPUT_RAW_HPP
#if defined(HIL_ACTUATOR_CONTROLS_HPP)
	create_stream_list_item<MavlinkStreamHILActuatorControls>(),
#endif // HIL_ACTUATOR_CONTROLS_HPP
#if defined(POSITION_TARGET_GLOBAL_INT_HPP)
	create_stream_list_item<MavlinkStreamPositionTargetGlobalInt>(),
#endif // POSITION_TARGET_GLOBAL_INT_HPP
#if defined(POSITION_TARGET_LOCAL_NED_HPP)
	create_stream_list_item<MavlinkStreamPositionTargetLocalNed>(),
#endif // POSITION_TARGET_LOCAL_NED_HPP
#if defined(ATTITUDE_TARGET_HPP)
	create_stream_list_item<MavlinkStreamAttitudeTarget>(),
#endif // ATTITUDE_TARGET_HPP
#if defined(RC_CHANNELS_HPP)
	create_stream_list_item<MavlinkStreamRCChannels>(),
#endif // RC_CHANNELS_HPP
#if defined(MANUAL_CONTROL_HPP)
	create_stream_list_item<MavlinkStreamManualControl>(),
#endif // MANUAL_CONTROL_HPP
#if defined(TRAJECTORY_REPRESENTATION_WAYPOINTS_HPP)
	create_stream_list_item<MavlinkStreamTrajectoryRepresentationWaypoints>(),
#endif // TRAJECTORY_REPRESENTATION_WAYPOINTS_HPP
#if defined(OPTICAL_FLOW_RAD_HPP)
	create_stream_list_item<MavlinkStreamOpticalFlowRad>(),
#endif // OPTICAL_FLOW_RAD_HPP
#if defined(ACTUATOR_CONTROL_TARGET_HPP)
	create_stream_list_item<MavlinkStreamActuatorControlTarget<0> >(),
	create_stream_list_item<MavlinkStreamActuatorControlTarget<1> >(),
#endif // ACTUATOR_CONTROL_TARGET_HPP
#if defined(NAMED_VALUE_FLOAT_HPP)
	create_stream_list_item<MavlinkStreamNamedValueFloat>(),
#endif // NAMED_VALUE_FLOAT_HPP
#if defined(DEBUG_HPP)
	create_stream_list_item<MavlinkStreamDebug>(),
#endif // DEBUG_HPP
#if defined(DEBUG_VECT_HPP)
	create_stream_list_item<MavlinkStreamDebugVect>(),
#endif // DEBUG_VECT_HPP
#if defined(DEBUG_FLOAT_ARRAY_HPP)
	create_stream_list_item<MavlinkStreamDebugFloatArray>(),
#endif // DEBUG_FLOAT_ARRAY_HPP
#if defined(NAV_CONTROLLER_OUTPUT_HPP)
	create_stream_list_item<MavlinkStreamNavControllerOutput>(),
#endif // NAV_CONTROLLER_OUTPUT_HPP
	create_stream_list_item<MavlinkStreamCameraCapture>(),
	create_stream_list_item<MavlinkStreamCameraTrigger>(),
	create_stream_list_item<MavlinkStreamCameraImageCaptured>(),
#if defined(DISTANCE_SENSOR_HPP)
	create_stream_list_item<MavlinkStreamDistanceSensor>(),
#endif // DISTANCE_SENSOR_HPP
#if defined(EXTENDED_SYS_STATE_HPP)
	create_stream_list_item<MavlinkStreamExtendedSysState>(),
#endif // EXTENDED_SYS_STATE_HPP
#if defined(ALTITUDE_HPP)
	create_stream_list_item<MavlinkStreamAltitude>(),
#endif // ALTITUDE_HPP
	create_stream_list_item<MavlinkStreamADSBVehicle>(),
	create_stream_list_item<MavlinkStreamUTMGlobalPosition>(),
#if defined(COLLISION_HPP)
	create_stream_list_item<MavlinkStreamCollision>(),
#endif // COLLISION_HPP
#if defined(WIND_COV_HPP)
	create_stream_list_item<MavlinkStreamWindCov>(),
#endif // WIND_COV_HPP
#if defined(MOUNT_ORIENTATION_HPP)
	create_stream_list_item<MavlinkStreamMountOrientation>(),
#endif // MOUNT_ORIENTATION_HPP
#if defined(HIGH_LATENCY2_HPP)
	create_stream_list_item<MavlinkStreamHighLatency2>(),
#endif // HIGH_LATENCY2_HPP
#if defined(HIL_STATE_QUATERNION_HPP)
	create_stream_list_item<MavlinkStreamHILStateQuaternion>(),
#endif // HIL_STATE_QUATERNION_HPP
#if defined(PING_HPP)
	create_stream_list_item<MavlinkStreamPing>(),
#endif // PING_HPP
#if defined(ORBIT_EXECUTION_STATUS_HPP)
	create_stream_list_item<MavlinkStreamOrbitStatus>(),
#endif // ORBIT_EXECUTION_STATUS_HPP
#if defined(OBSTACLE_DISTANCE_HPP)
	create_stream_list_item<MavlinkStreamObstacleDistance>(),
#endif // OBSTACLE_DISTANCE_HPP
#if defined(ESC_INFO_HPP)
	create_stream_list_item<MavlinkStreamESCInfo>(),
#endif // ESC_INFO_HPP
#if defined(ESC_STATUS_HPP)
	create_stream_list_item<MavlinkStreamESCStatus>(),
#endif // ESC_STATUS_HPP
#if defined(AUTOPILOT_VERSION_HPP)
	create_stream_list_item<MavlinkStreamAutopilotVersion>(),
#endif // AUTOPILOT_VERSION_HPP
#if defined(PROTOCOL_VERSION_HPP)
	create_stream_list_item<MavlinkStreamProtocolVersion>(),
#endif // PROTOCOL_VERSION_HPP
#if defined(FLIGHT_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamFlightInformation>(),
#endif // FLIGHT_INFORMATION_HPP
#if defined(GPS_STATUS_HPP)
	create_stream_list_item<MavlinkStreamGPSStatus>(),
#endif // GPS_STATUS_HPP
#if defined(LINK_NODE_STATUS_HPP)
	create_stream_list_item<MavlinkStreamLinkNodeStatus>(),
#endif // LINK_NODE_STATUS_HPP
#if defined(STORAGE_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamStorageInformation>(),
#endif // STORAGE_INFORMATION_HPP
#if defined(COMPONENT_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamComponentInformation>(),
#endif // COMPONENT_INFORMATION_HPP
#if defined(RAW_RPM_HPP)
	create_stream_list_item<MavlinkStreamRawRpm>()
#endif // RAW_RPM_HPP
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const uint16_t msg_id, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.new_instance(mavlink);
		}
	}

	return nullptr;
}
