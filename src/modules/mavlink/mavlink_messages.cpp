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
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/telemetry_status.h>
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
#include "streams/BATTERY_STATUS.hpp"
#include "streams/CAMERA_IMAGE_CAPTURED.hpp"
#include "streams/COLLISION.hpp"
#include "streams/COMMAND_LONG.hpp"
#include "streams/COMPONENT_INFORMATION.hpp"
#include "streams/DISTANCE_SENSOR.hpp"
#include "streams/ESC_INFO.hpp"
#include "streams/ESC_STATUS.hpp"
#include "streams/ESTIMATOR_STATUS.hpp"
#include "streams/EXTENDED_SYS_STATE.hpp"
#include "streams/FLIGHT_INFORMATION.hpp"
#include "streams/GLOBAL_POSITION_INT.hpp"
#include "streams/GPS_GLOBAL_ORIGIN.hpp"
#include "streams/GPS_RAW_INT.hpp"
#include "streams/GPS_STATUS.hpp"
#include "streams/HIGHRES_IMU.hpp"
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
#include "streams/SCALED_PRESSURE.hpp"
#include "streams/SERVO_OUTPUT_RAW.hpp"
#include "streams/STATUSTEXT.hpp"
#include "streams/STORAGE_INFORMATION.hpp"
#include "streams/SYSTEM_TIME.hpp"
#include "streams/SYS_STATUS.hpp"
#include "streams/TIMESYNC.hpp"
#include "streams/TRAJECTORY_REPRESENTATION_WAYPOINTS.hpp"
#include "streams/VFR_HUD.hpp"
#include "streams/VIBRATION.hpp"
#include "streams/WIND_COV.hpp"

#if !defined(CONSTRAINED_FLASH)
# include "streams/ADSB_VEHICLE.hpp"
# include "streams/ATT_POS_MOCAP.hpp"
# include "streams/AUTOPILOT_STATE_FOR_GIMBAL_DEVICE.hpp"
# include "streams/DEBUG.hpp"
# include "streams/DEBUG_FLOAT_ARRAY.hpp"
# include "streams/DEBUG_VECT.hpp"
# include "streams/GIMBAL_DEVICE_ATTITUDE_STATUS.hpp"
# include "streams/GIMBAL_DEVICE_SET_ATTITUDE.hpp"
# include "streams/GIMBAL_MANAGER_INFORMATION.hpp"
# include "streams/GIMBAL_MANAGER_STATUS.hpp"
# include "streams/GPS2_RAW.hpp"
# include "streams/HIGH_LATENCY2.hpp"
# include "streams/LINK_NODE_STATUS.hpp"
# include "streams/NAMED_VALUE_FLOAT.hpp"
# include "streams/ODOMETRY.hpp"
# include "streams/SCALED_PRESSURE2.hpp"
# include "streams/SCALED_PRESSURE3.hpp"
# include "streams/SMART_BATTERY_INFO.hpp"
# include "streams/UTM_GLOBAL_POSITION.hpp"
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
#if defined(COMMAND_LONG_HPP)
	create_stream_list_item<MavlinkStreamCommandLong>(),
#endif // COMMAND_LONG_HPP
#if defined(SYSTEM_TIME_HPP)
	create_stream_list_item<MavlinkStreamSysStatus>(),
#endif // SYSTEM_TIME_HPP
	create_stream_list_item<MavlinkStreamBatteryStatus>(),
#if defined(SMART_BATTERY_INFO_HPP)
	create_stream_list_item<MavlinkStreamSmartBatteryInfo>(),
#endif // SMART_BATTERY_INFO_HPP
#if defined(HIGHRES_IMU_HPP)
	create_stream_list_item<MavlinkStreamHighresIMU>(),
#endif // HIGHRES_IMU_HPP
#if defined(SCALED_IMU_HPP)
	create_stream_list_item<MavlinkStreamScaledIMU<0> >(),
	create_stream_list_item<MavlinkStreamScaledIMU<1> >(),
	create_stream_list_item<MavlinkStreamScaledIMU<2> >(),
#endif // SCALED_IMU_HPP
#if defined(SCALED_PRESSURE)
	create_stream_list_item<MavlinkStreamScaledPressure>(),
#endif // SCALED_PRESSURE
#if defined(SCALED_PRESSURE2)
	create_stream_list_item<MavlinkStreamScaledPressure2>(),
#endif // SCALED_PRESSURE2
#if defined(SCALED_PRESSURE3)
	create_stream_list_item<MavlinkStreamScaledPressure3>(),
#endif // SCALED_PRESSURE3
#if defined(ACTUATOR_OUTPUT_STATUS_HPP)
	create_stream_list_item<MavlinkStreamActuatorOutputStatus>(),
#endif // ACTUATOR_OUTPUT_STATUS_HPP
#if defined(ATTITUDE_HPP)
	create_stream_list_item<MavlinkStreamAttitude>(),
#endif // ATTITUDE_HPP
#if defined(ATTITUDE_QUATERNION_HPP)
	create_stream_list_item<MavlinkStreamAttitudeQuaternion>(),
#endif // ATTITUDE_QUATERNION_HPP
#if defined(VFR_HUD_HPP)
	create_stream_list_item<MavlinkStreamVFRHUD>(),
#endif // VFR_HUD_HPP
#if defined(GPS_GLOBAL_ORIGIN_HPP)
	create_stream_list_item<MavlinkStreamGpsGlobalOrigin>(),
#endif // GPS_GLOBAL_ORIGIN_HPP
#if defined(GPS_RAW_INT_HPP)
	create_stream_list_item<MavlinkStreamGPSRawInt>(),
#endif // GPS_RAW_INT_HPP
#if defined(GPS2_RAW_HPP)
	create_stream_list_item<MavlinkStreamGPS2Raw>(),
#endif // GPS2_RAW_HPP
#if defined(SYSTEM_TIME_HPP)
	create_stream_list_item<MavlinkStreamSystemTime>(),
#endif // SYSTEM_TIME_HPP
#if defined(TIMESYNC_HPP)
	create_stream_list_item<MavlinkStreamTimesync>(),
#endif // TIMESYNC_HPP
#if defined(GLOBAL_POSITION_INT_HPP)
	create_stream_list_item<MavlinkStreamGlobalPositionInt>(),
#endif // GLOBAL_POSITION_INT_HPP
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
#if defined(AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP)
	create_stream_list_item<MavlinkStreamAutopilotStateForGimbalDevice>(),
#endif // AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP
#if defined(GIMBAL_DEVICE_ATTITUDE_STATUS_HPP)
	create_stream_list_item<MavlinkStreamGimbalDeviceAttitudeStatus>(),
#endif // GIMBAL_DEVICE_ATTITUDE_STATUS_HPP
#if defined(GIMBAL_MANAGER_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamGimbalManagerInformation>(),
#endif // GIMBAL_MANAGER_INFORMATION_HPP
#if defined(GIMBAL_MANAGER_STATUS_HPP)
	create_stream_list_item<MavlinkStreamGimbalManagerStatus>(),
#endif // GIMBAL_MANAGER_STATUS_HPP
#if defined(GIMBAL_DEVICE_SET_ATTITUDE_HPP)
	create_stream_list_item<MavlinkStreamGimbalDeviceSetAttitude>(),
#endif // GIMBAL_DEVICE_SET_ATTITUDE_HPP
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
#if defined(CAMERA_IMAGE_CAPTURED_HPP)
	create_stream_list_item<MavlinkStreamCameraImageCaptured>(),
#endif // CAMERA_IMAGE_CAPTURED_HPP
#if defined(DISTANCE_SENSOR_HPP)
	create_stream_list_item<MavlinkStreamDistanceSensor>(),
#endif // DISTANCE_SENSOR_HPP
#if defined(EXTENDED_SYS_STATE_HPP)
	create_stream_list_item<MavlinkStreamExtendedSysState>(),
#endif // EXTENDED_SYS_STATE_HPP
#if defined(ALTITUDE_HPP)
	create_stream_list_item<MavlinkStreamAltitude>(),
#endif // ALTITUDE_HPP
#if defined(ADSB_VEHICLE_HPP)
	create_stream_list_item<MavlinkStreamADSBVehicle>(),
#endif // ADSB_VEHICLE_HPP
#if defined(UTM_GLOBAL_POSITION_HPP)
	create_stream_list_item<MavlinkStreamUTMGlobalPosition>(),
#endif // UTM_GLOBAL_POSITION_HPP
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
