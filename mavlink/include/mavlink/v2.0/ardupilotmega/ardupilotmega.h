/** @file
 *	@brief MAVLink comm protocol generated from ardupilotmega.xml
 *	@see http://mavlink.org
 */
#ifndef MAVLINK_ARDUPILOTMEGA_H
#define MAVLINK_ARDUPILOTMEGA_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ARDUPILOTMEGA.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 0, 50, 9}, {1, 0, 124, 31}, {2, 0, 137, 12}, {4, 0, 237, 14}, {5, 0, 217, 28}, {6, 0, 104, 3}, {7, 0, 119, 32}, {11, 0, 89, 6}, {20, 0, 214, 20}, {21, 0, 159, 2}, {22, 0, 220, 25}, {23, 0, 168, 23}, {24, 0, 24, 30}, {25, 0, 23, 101}, {26, 0, 170, 22}, {27, 0, 144, 26}, {28, 0, 67, 16}, {29, 0, 115, 14}, {30, 0, 39, 28}, {31, 0, 246, 32}, {32, 0, 185, 28}, {33, 0, 104, 28}, {34, 0, 237, 22}, {35, 0, 244, 22}, {36, 0, 222, 21}, {37, 0, 212, 6}, {38, 0, 9, 6}, {39, 0, 254, 37}, {40, 0, 230, 4}, {41, 0, 28, 4}, {42, 0, 28, 2}, {43, 0, 132, 2}, {44, 0, 221, 4}, {45, 0, 232, 2}, {46, 0, 11, 2}, {47, 0, 153, 3}, {48, 0, 41, 13}, {49, 0, 39, 12}, {50, 0, 78, 37}, {54, 0, 15, 27}, {55, 0, 3, 25}, {61, 0, 153, 68}, {62, 0, 183, 26}, {63, 0, 51, 185}, {64, 0, 59, 229}, {65, 0, 118, 42}, {66, 0, 148, 6}, {67, 0, 21, 4}, {69, 0, 243, 11}, {70, 0, 124, 18}, {73, 0, 38, 37}, {74, 0, 20, 20}, {75, 0, 158, 35}, {76, 0, 152, 33}, {77, 0, 143, 3}, {81, 0, 106, 22}, {82, 0, 49, 39}, {83, 0, 22, 37}, {84, 0, 143, 53}, {85, 0, 140, 51}, {86, 0, 5, 53}, {87, 0, 150, 51}, {89, 0, 231, 28}, {90, 0, 183, 56}, {91, 0, 63, 42}, {92, 0, 54, 33}, {100, 0, 175, 26}, {101, 0, 102, 32}, {102, 0, 158, 32}, {103, 0, 208, 20}, {104, 0, 56, 32}, {105, 0, 93, 62}, {106, 0, 138, 44}, {107, 0, 108, 64}, {108, 0, 32, 84}, {109, 0, 185, 9}, {110, 0, 84, 254}, {111, 0, 34, 16}, {112, 0, 174, 12}, {113, 0, 124, 36}, {114, 0, 237, 44}, {115, 0, 4, 64}, {116, 0, 76, 22}, {117, 0, 128, 6}, {118, 0, 56, 14}, {119, 0, 116, 12}, {120, 0, 134, 97}, {121, 0, 237, 2}, {122, 0, 203, 2}, {123, 0, 250, 113}, {124, 0, 87, 35}, {125, 0, 203, 6}, {126, 0, 220, 79}, {127, 0, 25, 35}, {128, 0, 226, 35}, {129, 0, 46, 22}, {130, 0, 29, 13}, {131, 0, 223, 255}, {132, 0, 85, 14}, {133, 0, 6, 18}, {134, 0, 229, 43}, {135, 0, 203, 8}, {136, 0, 1, 22}, {137, 0, 195, 14}, {138, 0, 109, 36}, {139, 0, 168, 43}, {140, 0, 181, 41}, {141, 0, 47, 32}, {142, 0, 72, 243}, {143, 0, 131, 14}, {146, 0, 103, 100}, {147, 0, 154, 36}, {148, 0, 178, 60}, {149, 0, 200, 30}, {150, 2, 134, 42}, {151, 2, 219, 8}, {152, 2, 208, 4}, {153, 2, 188, 12}, {154, 2, 84, 15}, {155, 2, 22, 13}, {156, 2, 19, 6}, {157, 2, 21, 15}, {158, 2, 134, 14}, {160, 2, 78, 12}, {161, 2, 68, 3}, {162, 2, 189, 8}, {163, 2, 127, 28}, {164, 2, 154, 44}, {165, 2, 21, 3}, {166, 2, 21, 9}, {167, 2, 144, 22}, {168, 2, 1, 12}, {169, 2, 234, 18}, {170, 2, 73, 34}, {171, 2, 181, 66}, {172, 2, 22, 98}, {173, 2, 83, 8}, {174, 2, 167, 48}, {175, 2, 138, 19}, {176, 2, 234, 3}, {177, 2, 240, 20}, {178, 2, 47, 24}, {179, 2, 189, 29}, {180, 2, 52, 45}, {181, 2, 174, 4}, {182, 2, 229, 40}, {183, 2, 85, 2}, {184, 2, 159, 206}, {185, 2, 186, 7}, {186, 2, 72, 29}, {191, 2, 92, 27}, {192, 2, 36, 44}, {193, 2, 71, 22}, {194, 2, 98, 25}, {200, 2, 134, 42}, {201, 2, 205, 14}, {214, 2, 69, 8}, {215, 2, 101, 3}, {216, 2, 50, 3}, {217, 2, 202, 6}, {218, 2, 17, 7}, {219, 2, 162, 2}, {226, 2, 207, 8}, {241, 0, 90, 32}, {242, 0, 104, 52}, {243, 0, 85, 53}, {244, 0, 95, 6}, {245, 0, 130, 2}, {246, 0, 158, 38}, {248, 0, 8, 254}, {249, 0, 204, 36}, {250, 0, 49, 30}, {251, 0, 170, 18}, {252, 0, 44, 18}, {253, 0, 83, 51}, {254, 0, 46, 9}, {256, 0, 71, 42}}
#endif

#ifndef MAVLINK_MESSAGE_INFO
#define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_SYS_STATUS, MAVLINK_MESSAGE_INFO_SYSTEM_TIME, MAVLINK_MESSAGE_INFO_PING, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL_ACK, MAVLINK_MESSAGE_INFO_AUTH_KEY, MAVLINK_MESSAGE_INFO_SET_MODE, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_VALUE, MAVLINK_MESSAGE_INFO_PARAM_SET, MAVLINK_MESSAGE_INFO_GPS_RAW_INT, MAVLINK_MESSAGE_INFO_GPS_STATUS, MAVLINK_MESSAGE_INFO_SCALED_IMU, MAVLINK_MESSAGE_INFO_RAW_IMU, MAVLINK_MESSAGE_INFO_RAW_PRESSURE, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED, MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW, MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_ITEM, MAVLINK_MESSAGE_INFO_MISSION_REQUEST, MAVLINK_MESSAGE_INFO_MISSION_SET_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_LIST, MAVLINK_MESSAGE_INFO_MISSION_COUNT, MAVLINK_MESSAGE_INFO_MISSION_CLEAR_ALL, MAVLINK_MESSAGE_INFO_MISSION_ITEM_REACHED, MAVLINK_MESSAGE_INFO_MISSION_ACK, MAVLINK_MESSAGE_INFO_SET_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_PARAM_MAP_RC, MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_SAFETY_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV, MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT_COV, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_COV, MAVLINK_MESSAGE_INFO_RC_CHANNELS, MAVLINK_MESSAGE_INFO_REQUEST_DATA_STREAM, MAVLINK_MESSAGE_INFO_DATA_STREAM, MAVLINK_MESSAGE_INFO_MANUAL_CONTROL, MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE, MAVLINK_MESSAGE_INFO_MISSION_ITEM_INT, MAVLINK_MESSAGE_INFO_VFR_HUD, MAVLINK_MESSAGE_INFO_COMMAND_INT, MAVLINK_MESSAGE_INFO_COMMAND_LONG, MAVLINK_MESSAGE_INFO_COMMAND_ACK, MAVLINK_MESSAGE_INFO_MANUAL_SETPOINT, MAVLINK_MESSAGE_INFO_SET_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, MAVLINK_MESSAGE_INFO_HIL_STATE, MAVLINK_MESSAGE_INFO_HIL_CONTROLS, MAVLINK_MESSAGE_INFO_HIL_RC_INPUTS_RAW, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_GLOBAL_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_SPEED_ESTIMATE, MAVLINK_MESSAGE_INFO_VICON_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_HIGHRES_IMU, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW_RAD, MAVLINK_MESSAGE_INFO_HIL_SENSOR, MAVLINK_MESSAGE_INFO_SIM_STATE, MAVLINK_MESSAGE_INFO_RADIO_STATUS, MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL, MAVLINK_MESSAGE_INFO_TIMESYNC, MAVLINK_MESSAGE_INFO_CAMERA_TRIGGER, MAVLINK_MESSAGE_INFO_HIL_GPS, MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION, MAVLINK_MESSAGE_INFO_SCALED_IMU2, MAVLINK_MESSAGE_INFO_LOG_REQUEST_LIST, MAVLINK_MESSAGE_INFO_LOG_ENTRY, MAVLINK_MESSAGE_INFO_LOG_REQUEST_DATA, MAVLINK_MESSAGE_INFO_LOG_DATA, MAVLINK_MESSAGE_INFO_LOG_ERASE, MAVLINK_MESSAGE_INFO_LOG_REQUEST_END, MAVLINK_MESSAGE_INFO_GPS_INJECT_DATA, MAVLINK_MESSAGE_INFO_GPS2_RAW, MAVLINK_MESSAGE_INFO_POWER_STATUS, MAVLINK_MESSAGE_INFO_SERIAL_CONTROL, MAVLINK_MESSAGE_INFO_GPS_RTK, MAVLINK_MESSAGE_INFO_GPS2_RTK, MAVLINK_MESSAGE_INFO_SCALED_IMU3, MAVLINK_MESSAGE_INFO_DATA_TRANSMISSION_HANDSHAKE, MAVLINK_MESSAGE_INFO_ENCAPSULATED_DATA, MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR, MAVLINK_MESSAGE_INFO_TERRAIN_REQUEST, MAVLINK_MESSAGE_INFO_TERRAIN_DATA, MAVLINK_MESSAGE_INFO_TERRAIN_CHECK, MAVLINK_MESSAGE_INFO_TERRAIN_REPORT, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE2, MAVLINK_MESSAGE_INFO_ATT_POS_MOCAP, MAVLINK_MESSAGE_INFO_SET_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ALTITUDE, MAVLINK_MESSAGE_INFO_RESOURCE_REQUEST, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE3, MAVLINK_MESSAGE_INFO_CONTROL_SYSTEM_STATE, MAVLINK_MESSAGE_INFO_BATTERY_STATUS, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION, MAVLINK_MESSAGE_INFO_LANDING_TARGET, MAVLINK_MESSAGE_INFO_SENSOR_OFFSETS, MAVLINK_MESSAGE_INFO_SET_MAG_OFFSETS, MAVLINK_MESSAGE_INFO_MEMINFO, MAVLINK_MESSAGE_INFO_AP_ADC, MAVLINK_MESSAGE_INFO_DIGICAM_CONFIGURE, MAVLINK_MESSAGE_INFO_DIGICAM_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_CONFIGURE, MAVLINK_MESSAGE_INFO_MOUNT_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_STATUS, MAVLINK_MESSAGE_INFO_FENCE_POINT, MAVLINK_MESSAGE_INFO_FENCE_FETCH_POINT, MAVLINK_MESSAGE_INFO_FENCE_STATUS, MAVLINK_MESSAGE_INFO_AHRS, MAVLINK_MESSAGE_INFO_SIMSTATE, MAVLINK_MESSAGE_INFO_HWSTATUS, MAVLINK_MESSAGE_INFO_RADIO, MAVLINK_MESSAGE_INFO_LIMITS_STATUS, MAVLINK_MESSAGE_INFO_WIND, MAVLINK_MESSAGE_INFO_DATA16, MAVLINK_MESSAGE_INFO_DATA32, MAVLINK_MESSAGE_INFO_DATA64, MAVLINK_MESSAGE_INFO_DATA96, MAVLINK_MESSAGE_INFO_RANGEFINDER, MAVLINK_MESSAGE_INFO_AIRSPEED_AUTOCAL, MAVLINK_MESSAGE_INFO_RALLY_POINT, MAVLINK_MESSAGE_INFO_RALLY_FETCH_POINT, MAVLINK_MESSAGE_INFO_COMPASSMOT_STATUS, MAVLINK_MESSAGE_INFO_AHRS2, MAVLINK_MESSAGE_INFO_CAMERA_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK, MAVLINK_MESSAGE_INFO_BATTERY2, MAVLINK_MESSAGE_INFO_AHRS3, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION_REQUEST, MAVLINK_MESSAGE_INFO_REMOTE_LOG_DATA_BLOCK, MAVLINK_MESSAGE_INFO_REMOTE_LOG_BLOCK_STATUS, MAVLINK_MESSAGE_INFO_LED_CONTROL, MAVLINK_MESSAGE_INFO_MAG_CAL_PROGRESS, MAVLINK_MESSAGE_INFO_MAG_CAL_REPORT, MAVLINK_MESSAGE_INFO_EKF_STATUS_REPORT, MAVLINK_MESSAGE_INFO_PID_TUNING, MAVLINK_MESSAGE_INFO_GIMBAL_REPORT, MAVLINK_MESSAGE_INFO_GIMBAL_CONTROL, MAVLINK_MESSAGE_INFO_GIMBAL_TORQUE_CMD_REPORT, MAVLINK_MESSAGE_INFO_GOPRO_HEARTBEAT, MAVLINK_MESSAGE_INFO_GOPRO_GET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_GET_RESPONSE, MAVLINK_MESSAGE_INFO_GOPRO_SET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_SET_RESPONSE, MAVLINK_MESSAGE_INFO_RPM, MAVLINK_MESSAGE_INFO_VIBRATION, MAVLINK_MESSAGE_INFO_HOME_POSITION, MAVLINK_MESSAGE_INFO_SET_HOME_POSITION, MAVLINK_MESSAGE_INFO_MESSAGE_INTERVAL, MAVLINK_MESSAGE_INFO_EXTENDED_SYS_STATE, MAVLINK_MESSAGE_INFO_ADSB_VEHICLE, MAVLINK_MESSAGE_INFO_V2_EXTENSION, MAVLINK_MESSAGE_INFO_MEMORY_VECT, MAVLINK_MESSAGE_INFO_DEBUG_VECT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_FLOAT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_DEBUG, MAVLINK_MESSAGE_INFO_SETUP_SIGNING}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ARDUPILOTMEGA

// ENUM DEFINITIONS


/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD
{
	MAV_CMD_NAV_WAYPOINT=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_UNLIM=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_TURNS=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_TIME=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_NAV_LAND=21, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  */
	MAV_CMD_NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  */
	MAV_CMD_NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
	MAV_CMD_NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Empty| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
	MAV_CMD_NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
	MAV_CMD_NAV_SPLINE_WAYPOINT=82, /* Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
	MAV_CMD_NAV_ALTITUDE_WAIT=83, /* Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |altitude (m)| descent speed (m/s)| Wiggle Time (s)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
	MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
	MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
	MAV_CMD_DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
	MAV_CMD_DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_GO_AROUND=191, /* Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
	MAV_CMD_DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
	MAV_CMD_DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
	MAV_CMD_DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
	MAV_CMD_DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|  */
	MAV_CMD_DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_PARACHUTE=208, /* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_MOTOR_TEST=209, /* Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  */
	MAV_CMD_DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_GRIPPER=211, /* Mission command to operate EPM gripper |gripper number (a number from 1 to max number of grippers on the vehicle)| gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_AUTOTUNE_ENABLE=212, /* Enable/disable autotune |enable (1: enable, 0:disable)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
	MAV_CMD_DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_GUIDED_LIMITS=222, /* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
	MAV_CMD_DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Compass/Motor interference calibration: 0: no, 1: yes| Empty|  */
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
	MAV_CMD_PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
	MAV_CMD_PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0| Reserved, send 0|  */
	MAV_CMD_OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
	MAV_CMD_MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
	MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
	MAV_CMD_GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
	MAV_CMD_START_RX_PAIR=500, /* Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  */
	MAV_CMD_GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  */
	MAV_CMD_SET_MESSAGE_INTERVAL=511, /* Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  */
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  */
	MAV_CMD_IMAGE_START_CAPTURE=2000, /* Start image capture sequence |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  */
	MAV_CMD_IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence |Reserved| Reserved|  */
	MAV_CMD_DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  */
	MAV_CMD_VIDEO_START_CAPTURE=2500, /* Starts video capture |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  */
	MAV_CMD_VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture |Reserved| Reserved|  */
	MAV_CMD_PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  */
	MAV_CMD_DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  */
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  */
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
	MAV_CMD_POWER_OFF_INITIATED=42000, /* A system wide power-off event has been initiated. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_SOLO_BTN_FLY_CLICK=42001, /* FLY button has been clicked. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_SOLO_BTN_FLY_HOLD=42002, /* FLY button has been held for 1.5 seconds. |Takeoff altitude| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_SOLO_BTN_PAUSE_CLICK=42003, /* PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_START_MAG_CAL=42424, /* Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds)| Autoreboot (0=user reboot, 1=autoreboot)| Empty| Empty|  */
	MAV_CMD_DO_ACCEPT_MAG_CAL=42425, /* Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_CANCEL_MAG_CAL=42426, /* Cancel a running magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_SET_FACTORY_TEST_MODE=42427, /* Command autopilot to get into factory test/diagnostic mode |0 means get out of test mode, 1 means get into test mode| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SEND_BANNER=42428, /* Reply with the version banner |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_GIMBAL_RESET=42501, /* Causes the gimbal to reset and boot as if it was just powered on |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS=42502, /* Reports progress and success or failure of gimbal axis calibration procedure |Gimbal axis we're reporting calibration progress for| Current calibration progress for this axis, 0x64=100%| Status of the calibration| Empty| Empty| Empty| Empty|  */
	MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION=42503, /* Starts commutation calibration on the gimbal |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_GIMBAL_FULL_RESET=42505, /* Erases gimbal application and parameters |Magic number| Magic number| Magic number| Magic number| Magic number| Magic number| Magic number|  */
	MAV_CMD_ENUM_END=42506, /*  | */
} MAV_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LIMITS_STATE
#define HAVE_ENUM_LIMITS_STATE
typedef enum LIMITS_STATE
{
	LIMITS_INIT=0, /* pre-initialization | */
	LIMITS_DISABLED=1, /* disabled | */
	LIMITS_ENABLED=2, /* checking limits | */
	LIMITS_TRIGGERED=3, /* a limit has been breached | */
	LIMITS_RECOVERING=4, /* taking action eg. RTL | */
	LIMITS_RECOVERED=5, /* we're no longer in breach of a limit | */
	LIMITS_STATE_ENUM_END=6, /*  | */
} LIMITS_STATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LIMIT_MODULE
#define HAVE_ENUM_LIMIT_MODULE
typedef enum LIMIT_MODULE
{
	LIMIT_GPSLOCK=1, /* pre-initialization | */
	LIMIT_GEOFENCE=2, /* disabled | */
	LIMIT_ALTITUDE=4, /* checking limits | */
	LIMIT_MODULE_ENUM_END=5, /*  | */
} LIMIT_MODULE;
#endif

/** @brief Flags in RALLY_POINT message */
#ifndef HAVE_ENUM_RALLY_FLAGS
#define HAVE_ENUM_RALLY_FLAGS
typedef enum RALLY_FLAGS
{
	FAVORABLE_WIND=1, /* Flag set when requiring favorable winds for landing. | */
	LAND_IMMEDIATELY=2, /* Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land. | */
	RALLY_FLAGS_ENUM_END=3, /*  | */
} RALLY_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PARACHUTE_ACTION
#define HAVE_ENUM_PARACHUTE_ACTION
typedef enum PARACHUTE_ACTION
{
	PARACHUTE_DISABLE=0, /* Disable parachute release | */
	PARACHUTE_ENABLE=1, /* Enable parachute release | */
	PARACHUTE_RELEASE=2, /* Release parachute | */
	PARACHUTE_ACTION_ENUM_END=3, /*  | */
} PARACHUTE_ACTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
#define HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
typedef enum MOTOR_TEST_THROTTLE_TYPE
{
	MOTOR_TEST_THROTTLE_PERCENT=0, /* throttle as a percentage from 0 ~ 100 | */
	MOTOR_TEST_THROTTLE_PWM=1, /* throttle as an absolute PWM value (normally in range of 1000~2000) | */
	MOTOR_TEST_THROTTLE_PILOT=2, /* throttle pass-through from pilot's transmitter | */
	MOTOR_TEST_THROTTLE_TYPE_ENUM_END=3, /*  | */
} MOTOR_TEST_THROTTLE_TYPE;
#endif

/** @brief Gripper actions. */
#ifndef HAVE_ENUM_GRIPPER_ACTIONS
#define HAVE_ENUM_GRIPPER_ACTIONS
typedef enum GRIPPER_ACTIONS
{
	GRIPPER_ACTION_RELEASE=0, /* gripper release of cargo | */
	GRIPPER_ACTION_GRAB=1, /* gripper grabs onto cargo | */
	GRIPPER_ACTIONS_ENUM_END=2, /*  | */
} GRIPPER_ACTIONS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CAMERA_STATUS_TYPES
#define HAVE_ENUM_CAMERA_STATUS_TYPES
typedef enum CAMERA_STATUS_TYPES
{
	CAMERA_STATUS_TYPE_HEARTBEAT=0, /* Camera heartbeat, announce camera component ID at 1hz | */
	CAMERA_STATUS_TYPE_TRIGGER=1, /* Camera image triggered | */
	CAMERA_STATUS_TYPE_DISCONNECT=2, /* Camera connection lost | */
	CAMERA_STATUS_TYPE_ERROR=3, /* Camera unknown error | */
	CAMERA_STATUS_TYPE_LOWBATT=4, /* Camera battery low. Parameter p1 shows reported voltage | */
	CAMERA_STATUS_TYPE_LOWSTORE=5, /* Camera storage low. Parameter p1 shows reported shots remaining | */
	CAMERA_STATUS_TYPE_LOWSTOREV=6, /* Camera storage low. Parameter p1 shows reported video minutes remaining | */
	CAMERA_STATUS_TYPES_ENUM_END=7, /*  | */
} CAMERA_STATUS_TYPES;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CAMERA_FEEDBACK_FLAGS
#define HAVE_ENUM_CAMERA_FEEDBACK_FLAGS
typedef enum CAMERA_FEEDBACK_FLAGS
{
	CAMERA_FEEDBACK_PHOTO=0, /* Shooting photos, not video | */
	CAMERA_FEEDBACK_VIDEO=1, /* Shooting video, not stills | */
	CAMERA_FEEDBACK_BADEXPOSURE=2, /* Unable to achieve requested exposure (e.g. shutter speed too low) | */
	CAMERA_FEEDBACK_CLOSEDLOOP=3, /* Closed loop feedback from camera, we know for sure it has successfully taken a picture | */
	CAMERA_FEEDBACK_OPENLOOP=4, /* Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture | */
	CAMERA_FEEDBACK_FLAGS_ENUM_END=5, /*  | */
} CAMERA_FEEDBACK_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_MODE_GIMBAL
#define HAVE_ENUM_MAV_MODE_GIMBAL
typedef enum MAV_MODE_GIMBAL
{
	MAV_MODE_GIMBAL_UNINITIALIZED=0, /* Gimbal is powered on but has not started initializing yet | */
	MAV_MODE_GIMBAL_CALIBRATING_PITCH=1, /* Gimbal is currently running calibration on the pitch axis | */
	MAV_MODE_GIMBAL_CALIBRATING_ROLL=2, /* Gimbal is currently running calibration on the roll axis | */
	MAV_MODE_GIMBAL_CALIBRATING_YAW=3, /* Gimbal is currently running calibration on the yaw axis | */
	MAV_MODE_GIMBAL_INITIALIZED=4, /* Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter | */
	MAV_MODE_GIMBAL_ACTIVE=5, /* Gimbal is actively stabilizing | */
	MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT=6, /* Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command | */
	MAV_MODE_GIMBAL_ENUM_END=7, /*  | */
} MAV_MODE_GIMBAL;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GIMBAL_AXIS
#define HAVE_ENUM_GIMBAL_AXIS
typedef enum GIMBAL_AXIS
{
	GIMBAL_AXIS_YAW=0, /* Gimbal yaw axis | */
	GIMBAL_AXIS_PITCH=1, /* Gimbal pitch axis | */
	GIMBAL_AXIS_ROLL=2, /* Gimbal roll axis | */
	GIMBAL_AXIS_ENUM_END=3, /*  | */
} GIMBAL_AXIS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_STATUS
#define HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_STATUS
typedef enum GIMBAL_AXIS_CALIBRATION_STATUS
{
	GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS=0, /* Axis calibration is in progress | */
	GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED=1, /* Axis calibration succeeded | */
	GIMBAL_AXIS_CALIBRATION_STATUS_FAILED=2, /* Axis calibration failed | */
	GIMBAL_AXIS_CALIBRATION_STATUS_ENUM_END=3, /*  | */
} GIMBAL_AXIS_CALIBRATION_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_REQUIRED
#define HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_REQUIRED
typedef enum GIMBAL_AXIS_CALIBRATION_REQUIRED
{
	GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN=0, /* Whether or not this axis requires calibration is unknown at this time | */
	GIMBAL_AXIS_CALIBRATION_REQUIRED_TRUE=1, /* This axis requires calibration | */
	GIMBAL_AXIS_CALIBRATION_REQUIRED_FALSE=2, /* This axis does not require calibration | */
	GIMBAL_AXIS_CALIBRATION_REQUIRED_ENUM_END=3, /*  | */
} GIMBAL_AXIS_CALIBRATION_REQUIRED;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_HEARTBEAT_STATUS
#define HAVE_ENUM_GOPRO_HEARTBEAT_STATUS
typedef enum GOPRO_HEARTBEAT_STATUS
{
	GOPRO_HEARTBEAT_STATUS_DISCONNECTED=0, /* No GoPro connected | */
	GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE=1, /* The detected GoPro is not HeroBus compatible | */
	GOPRO_HEARTBEAT_STATUS_CONNECTED=2, /* A HeroBus compatible GoPro is connected | */
	GOPRO_HEARTBEAT_STATUS_ERROR=3, /* An unrecoverable error was encountered with the connected GoPro, it may require a power cycle | */
	GOPRO_HEARTBEAT_STATUS_ENUM_END=4, /*  | */
} GOPRO_HEARTBEAT_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_HEARTBEAT_FLAGS
#define HAVE_ENUM_GOPRO_HEARTBEAT_FLAGS
typedef enum GOPRO_HEARTBEAT_FLAGS
{
	GOPRO_FLAG_RECORDING=1, /* GoPro is currently recording | */
	GOPRO_HEARTBEAT_FLAGS_ENUM_END=2, /*  | */
} GOPRO_HEARTBEAT_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_REQUEST_STATUS
#define HAVE_ENUM_GOPRO_REQUEST_STATUS
typedef enum GOPRO_REQUEST_STATUS
{
	GOPRO_REQUEST_SUCCESS=0, /* The write message with ID indicated succeeded | */
	GOPRO_REQUEST_FAILED=1, /* The write message with ID indicated failed | */
	GOPRO_REQUEST_STATUS_ENUM_END=2, /*  | */
} GOPRO_REQUEST_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_COMMAND
#define HAVE_ENUM_GOPRO_COMMAND
typedef enum GOPRO_COMMAND
{
	GOPRO_COMMAND_POWER=0, /* (Get/Set) | */
	GOPRO_COMMAND_CAPTURE_MODE=1, /* (Get/Set) | */
	GOPRO_COMMAND_SHUTTER=2, /* (___/Set) | */
	GOPRO_COMMAND_BATTERY=3, /* (Get/___) | */
	GOPRO_COMMAND_MODEL=4, /* (Get/___) | */
	GOPRO_COMMAND_VIDEO_SETTINGS=5, /* (Get/Set) | */
	GOPRO_COMMAND_LOW_LIGHT=6, /* (Get/Set) | */
	GOPRO_COMMAND_PHOTO_RESOLUTION=7, /* (Get/Set) | */
	GOPRO_COMMAND_PHOTO_BURST_RATE=8, /* (Get/Set) | */
	GOPRO_COMMAND_PROTUNE=9, /* (Get/Set) | */
	GOPRO_COMMAND_PROTUNE_WHITE_BALANCE=10, /* (Get/Set) Hero 3+ Only | */
	GOPRO_COMMAND_PROTUNE_COLOUR=11, /* (Get/Set) Hero 3+ Only | */
	GOPRO_COMMAND_PROTUNE_GAIN=12, /* (Get/Set) Hero 3+ Only | */
	GOPRO_COMMAND_PROTUNE_SHARPNESS=13, /* (Get/Set) Hero 3+ Only | */
	GOPRO_COMMAND_PROTUNE_EXPOSURE=14, /* (Get/Set) Hero 3+ Only | */
	GOPRO_COMMAND_TIME=15, /* (Get/Set) | */
	GOPRO_COMMAND_CHARGING=16, /* (Get/Set) | */
	GOPRO_COMMAND_ENUM_END=17, /*  | */
} GOPRO_COMMAND;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_CAPTURE_MODE
#define HAVE_ENUM_GOPRO_CAPTURE_MODE
typedef enum GOPRO_CAPTURE_MODE
{
	GOPRO_CAPTURE_MODE_VIDEO=0, /* Video mode | */
	GOPRO_CAPTURE_MODE_PHOTO=1, /* Photo mode | */
	GOPRO_CAPTURE_MODE_BURST=2, /* Burst mode, hero 3+ only | */
	GOPRO_CAPTURE_MODE_TIME_LAPSE=3, /* Time lapse mode, hero 3+ only | */
	GOPRO_CAPTURE_MODE_MULTI_SHOT=4, /* Multi shot mode, hero 4 only | */
	GOPRO_CAPTURE_MODE_PLAYBACK=5, /* Playback mode, hero 4 only, silver only except when LCD or HDMI is connected to black | */
	GOPRO_CAPTURE_MODE_SETUP=6, /* Playback mode, hero 4 only | */
	GOPRO_CAPTURE_MODE_UNKNOWN=255, /* Mode not yet known | */
	GOPRO_CAPTURE_MODE_ENUM_END=256, /*  | */
} GOPRO_CAPTURE_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_RESOLUTION
#define HAVE_ENUM_GOPRO_RESOLUTION
typedef enum GOPRO_RESOLUTION
{
	GOPRO_RESOLUTION_480p=0, /* 848 x 480 (480p) | */
	GOPRO_RESOLUTION_720p=1, /* 1280 x 720 (720p) | */
	GOPRO_RESOLUTION_960p=2, /* 1280 x 960 (960p) | */
	GOPRO_RESOLUTION_1080p=3, /* 1920 x 1080 (1080p) | */
	GOPRO_RESOLUTION_1440p=4, /* 1920 x 1440 (1440p) | */
	GOPRO_RESOLUTION_2_7k_17_9=5, /* 2704 x 1440 (2.7k-17:9) | */
	GOPRO_RESOLUTION_2_7k_16_9=6, /* 2704 x 1524 (2.7k-16:9) | */
	GOPRO_RESOLUTION_2_7k_4_3=7, /* 2704 x 2028 (2.7k-4:3) | */
	GOPRO_RESOLUTION_4k_16_9=8, /* 3840 x 2160 (4k-16:9) | */
	GOPRO_RESOLUTION_4k_17_9=9, /* 4096 x 2160 (4k-17:9) | */
	GOPRO_RESOLUTION_720p_SUPERVIEW=10, /* 1280 x 720 (720p-SuperView) | */
	GOPRO_RESOLUTION_1080p_SUPERVIEW=11, /* 1920 x 1080 (1080p-SuperView) | */
	GOPRO_RESOLUTION_2_7k_SUPERVIEW=12, /* 2704 x 1520 (2.7k-SuperView) | */
	GOPRO_RESOLUTION_4k_SUPERVIEW=13, /* 3840 x 2160 (4k-SuperView) | */
	GOPRO_RESOLUTION_ENUM_END=14, /*  | */
} GOPRO_RESOLUTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_FRAME_RATE
#define HAVE_ENUM_GOPRO_FRAME_RATE
typedef enum GOPRO_FRAME_RATE
{
	GOPRO_FRAME_RATE_12=0, /* 12 FPS | */
	GOPRO_FRAME_RATE_15=1, /* 15 FPS | */
	GOPRO_FRAME_RATE_24=2, /* 24 FPS | */
	GOPRO_FRAME_RATE_25=3, /* 25 FPS | */
	GOPRO_FRAME_RATE_30=4, /* 30 FPS | */
	GOPRO_FRAME_RATE_48=5, /* 48 FPS | */
	GOPRO_FRAME_RATE_50=6, /* 50 FPS | */
	GOPRO_FRAME_RATE_60=7, /* 60 FPS | */
	GOPRO_FRAME_RATE_80=8, /* 80 FPS | */
	GOPRO_FRAME_RATE_90=9, /* 90 FPS | */
	GOPRO_FRAME_RATE_100=10, /* 100 FPS | */
	GOPRO_FRAME_RATE_120=11, /* 120 FPS | */
	GOPRO_FRAME_RATE_240=12, /* 240 FPS | */
	GOPRO_FRAME_RATE_12_5=13, /* 12.5 FPS | */
	GOPRO_FRAME_RATE_ENUM_END=14, /*  | */
} GOPRO_FRAME_RATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_FIELD_OF_VIEW
#define HAVE_ENUM_GOPRO_FIELD_OF_VIEW
typedef enum GOPRO_FIELD_OF_VIEW
{
	GOPRO_FIELD_OF_VIEW_WIDE=0, /* 0x00: Wide | */
	GOPRO_FIELD_OF_VIEW_MEDIUM=1, /* 0x01: Medium | */
	GOPRO_FIELD_OF_VIEW_NARROW=2, /* 0x02: Narrow | */
	GOPRO_FIELD_OF_VIEW_ENUM_END=3, /*  | */
} GOPRO_FIELD_OF_VIEW;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_VIDEO_SETTINGS_FLAGS
#define HAVE_ENUM_GOPRO_VIDEO_SETTINGS_FLAGS
typedef enum GOPRO_VIDEO_SETTINGS_FLAGS
{
	GOPRO_VIDEO_SETTINGS_TV_MODE=1, /* 0=NTSC, 1=PAL | */
	GOPRO_VIDEO_SETTINGS_FLAGS_ENUM_END=2, /*  | */
} GOPRO_VIDEO_SETTINGS_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PHOTO_RESOLUTION
#define HAVE_ENUM_GOPRO_PHOTO_RESOLUTION
typedef enum GOPRO_PHOTO_RESOLUTION
{
	GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM=0, /* 5MP Medium | */
	GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM=1, /* 7MP Medium | */
	GOPRO_PHOTO_RESOLUTION_7MP_WIDE=2, /* 7MP Wide | */
	GOPRO_PHOTO_RESOLUTION_10MP_WIDE=3, /* 10MP Wide | */
	GOPRO_PHOTO_RESOLUTION_12MP_WIDE=4, /* 12MP Wide | */
	GOPRO_PHOTO_RESOLUTION_ENUM_END=5, /*  | */
} GOPRO_PHOTO_RESOLUTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_WHITE_BALANCE
#define HAVE_ENUM_GOPRO_PROTUNE_WHITE_BALANCE
typedef enum GOPRO_PROTUNE_WHITE_BALANCE
{
	GOPRO_PROTUNE_WHITE_BALANCE_AUTO=0, /* Auto | */
	GOPRO_PROTUNE_WHITE_BALANCE_3000K=1, /* 3000K | */
	GOPRO_PROTUNE_WHITE_BALANCE_5500K=2, /* 5500K | */
	GOPRO_PROTUNE_WHITE_BALANCE_6500K=3, /* 6500K | */
	GOPRO_PROTUNE_WHITE_BALANCE_RAW=4, /* Camera Raw | */
	GOPRO_PROTUNE_WHITE_BALANCE_ENUM_END=5, /*  | */
} GOPRO_PROTUNE_WHITE_BALANCE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_COLOUR
#define HAVE_ENUM_GOPRO_PROTUNE_COLOUR
typedef enum GOPRO_PROTUNE_COLOUR
{
	GOPRO_PROTUNE_COLOUR_STANDARD=0, /* Auto | */
	GOPRO_PROTUNE_COLOUR_NEUTRAL=1, /* Neutral | */
	GOPRO_PROTUNE_COLOUR_ENUM_END=2, /*  | */
} GOPRO_PROTUNE_COLOUR;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_GAIN
#define HAVE_ENUM_GOPRO_PROTUNE_GAIN
typedef enum GOPRO_PROTUNE_GAIN
{
	GOPRO_PROTUNE_GAIN_400=0, /* ISO 400 | */
	GOPRO_PROTUNE_GAIN_800=1, /* ISO 800 (Only Hero 4) | */
	GOPRO_PROTUNE_GAIN_1600=2, /* ISO 1600 | */
	GOPRO_PROTUNE_GAIN_3200=3, /* ISO 3200 (Only Hero 4) | */
	GOPRO_PROTUNE_GAIN_6400=4, /* ISO 6400 | */
	GOPRO_PROTUNE_GAIN_ENUM_END=5, /*  | */
} GOPRO_PROTUNE_GAIN;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_SHARPNESS
#define HAVE_ENUM_GOPRO_PROTUNE_SHARPNESS
typedef enum GOPRO_PROTUNE_SHARPNESS
{
	GOPRO_PROTUNE_SHARPNESS_LOW=0, /* Low Sharpness | */
	GOPRO_PROTUNE_SHARPNESS_MEDIUM=1, /* Medium Sharpness | */
	GOPRO_PROTUNE_SHARPNESS_HIGH=2, /* High Sharpness | */
	GOPRO_PROTUNE_SHARPNESS_ENUM_END=3, /*  | */
} GOPRO_PROTUNE_SHARPNESS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_EXPOSURE
#define HAVE_ENUM_GOPRO_PROTUNE_EXPOSURE
typedef enum GOPRO_PROTUNE_EXPOSURE
{
	GOPRO_PROTUNE_EXPOSURE_NEG_5_0=0, /* -5.0 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_NEG_4_5=1, /* -4.5 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_NEG_4_0=2, /* -4.0 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_NEG_3_5=3, /* -3.5 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_NEG_3_0=4, /* -3.0 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_NEG_2_5=5, /* -2.5 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_NEG_2_0=6, /* -2.0 EV | */
	GOPRO_PROTUNE_EXPOSURE_NEG_1_5=7, /* -1.5 EV | */
	GOPRO_PROTUNE_EXPOSURE_NEG_1_0=8, /* -1.0 EV | */
	GOPRO_PROTUNE_EXPOSURE_NEG_0_5=9, /* -0.5 EV | */
	GOPRO_PROTUNE_EXPOSURE_ZERO=10, /* 0.0 EV | */
	GOPRO_PROTUNE_EXPOSURE_POS_0_5=11, /* +0.5 EV | */
	GOPRO_PROTUNE_EXPOSURE_POS_1_0=12, /* +1.0 EV | */
	GOPRO_PROTUNE_EXPOSURE_POS_1_5=13, /* +1.5 EV | */
	GOPRO_PROTUNE_EXPOSURE_POS_2_0=14, /* +2.0 EV | */
	GOPRO_PROTUNE_EXPOSURE_POS_2_5=15, /* +2.5 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_POS_3_0=16, /* +3.0 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_POS_3_5=17, /* +3.5 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_POS_4_0=18, /* +4.0 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_POS_4_5=19, /* +4.5 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_POS_5_0=20, /* +5.0 EV (Hero 3+ Only) | */
	GOPRO_PROTUNE_EXPOSURE_ENUM_END=21, /*  | */
} GOPRO_PROTUNE_EXPOSURE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_CHARGING
#define HAVE_ENUM_GOPRO_CHARGING
typedef enum GOPRO_CHARGING
{
	GOPRO_CHARGING_DISABLED=0, /* Charging disabled | */
	GOPRO_CHARGING_ENABLED=1, /* Charging enabled | */
	GOPRO_CHARGING_ENUM_END=2, /*  | */
} GOPRO_CHARGING;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_MODEL
#define HAVE_ENUM_GOPRO_MODEL
typedef enum GOPRO_MODEL
{
	GOPRO_MODEL_UNKNOWN=0, /* Unknown gopro model | */
	GOPRO_MODEL_HERO_3_PLUS_SILVER=1, /* Hero 3+ Silver (HeroBus not supported by GoPro) | */
	GOPRO_MODEL_HERO_3_PLUS_BLACK=2, /* Hero 3+ Black | */
	GOPRO_MODEL_HERO_4_SILVER=3, /* Hero 4 Silver | */
	GOPRO_MODEL_HERO_4_BLACK=4, /* Hero 4 Black | */
	GOPRO_MODEL_ENUM_END=5, /*  | */
} GOPRO_MODEL;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_BURST_RATE
#define HAVE_ENUM_GOPRO_BURST_RATE
typedef enum GOPRO_BURST_RATE
{
	GOPRO_BURST_RATE_3_IN_1_SECOND=0, /* 3 Shots / 1 Second | */
	GOPRO_BURST_RATE_5_IN_1_SECOND=1, /* 5 Shots / 1 Second | */
	GOPRO_BURST_RATE_10_IN_1_SECOND=2, /* 10 Shots / 1 Second | */
	GOPRO_BURST_RATE_10_IN_2_SECOND=3, /* 10 Shots / 2 Second | */
	GOPRO_BURST_RATE_10_IN_3_SECOND=4, /* 10 Shots / 3 Second (Hero 4 Only) | */
	GOPRO_BURST_RATE_30_IN_1_SECOND=5, /* 30 Shots / 1 Second | */
	GOPRO_BURST_RATE_30_IN_2_SECOND=6, /* 30 Shots / 2 Second | */
	GOPRO_BURST_RATE_30_IN_3_SECOND=7, /* 30 Shots / 3 Second | */
	GOPRO_BURST_RATE_30_IN_6_SECOND=8, /* 30 Shots / 6 Second | */
	GOPRO_BURST_RATE_ENUM_END=9, /*  | */
} GOPRO_BURST_RATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LED_CONTROL_PATTERN
#define HAVE_ENUM_LED_CONTROL_PATTERN
typedef enum LED_CONTROL_PATTERN
{
	LED_CONTROL_PATTERN_OFF=0, /* LED patterns off (return control to regular vehicle control) | */
	LED_CONTROL_PATTERN_FIRMWAREUPDATE=1, /* LEDs show pattern during firmware update | */
	LED_CONTROL_PATTERN_CUSTOM=255, /* Custom Pattern using custom bytes fields | */
	LED_CONTROL_PATTERN_ENUM_END=256, /*  | */
} LED_CONTROL_PATTERN;
#endif

/** @brief Flags in EKF_STATUS message */
#ifndef HAVE_ENUM_EKF_STATUS_FLAGS
#define HAVE_ENUM_EKF_STATUS_FLAGS
typedef enum EKF_STATUS_FLAGS
{
	EKF_ATTITUDE=1, /* set if EKF's attitude estimate is good | */
	EKF_VELOCITY_HORIZ=2, /* set if EKF's horizontal velocity estimate is good | */
	EKF_VELOCITY_VERT=4, /* set if EKF's vertical velocity estimate is good | */
	EKF_POS_HORIZ_REL=8, /* set if EKF's horizontal position (relative) estimate is good | */
	EKF_POS_HORIZ_ABS=16, /* set if EKF's horizontal position (absolute) estimate is good | */
	EKF_POS_VERT_ABS=32, /* set if EKF's vertical position (absolute) estimate is good | */
	EKF_POS_VERT_AGL=64, /* set if EKF's vertical position (above ground) estimate is good | */
	EKF_CONST_POS_MODE=128, /* EKF is in constant position mode and does not know it's absolute or relative position | */
	EKF_PRED_POS_HORIZ_REL=256, /* set if EKF's predicted horizontal position (relative) estimate is good | */
	EKF_PRED_POS_HORIZ_ABS=512, /* set if EKF's predicted horizontal position (absolute) estimate is good | */
	EKF_STATUS_FLAGS_ENUM_END=513, /*  | */
} EKF_STATUS_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PID_TUNING_AXIS
#define HAVE_ENUM_PID_TUNING_AXIS
typedef enum PID_TUNING_AXIS
{
	PID_TUNING_ROLL=1, /*  | */
	PID_TUNING_PITCH=2, /*  | */
	PID_TUNING_YAW=3, /*  | */
	PID_TUNING_ACCZ=4, /*  | */
	PID_TUNING_STEER=5, /*  | */
	PID_TUNING_AXIS_ENUM_END=6, /*  | */
} PID_TUNING_AXIS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAG_CAL_STATUS
#define HAVE_ENUM_MAG_CAL_STATUS
typedef enum MAG_CAL_STATUS
{
	MAG_CAL_NOT_STARTED=0, /*  | */
	MAG_CAL_WAITING_TO_START=1, /*  | */
	MAG_CAL_RUNNING_STEP_ONE=2, /*  | */
	MAG_CAL_RUNNING_STEP_TWO=3, /*  | */
	MAG_CAL_SUCCESS=4, /*  | */
	MAG_CAL_FAILED=5, /*  | */
	MAG_CAL_STATUS_ENUM_END=6, /*  | */
} MAG_CAL_STATUS;
#endif

/** @brief Special ACK block numbers control activation of dataflash log streaming */
#ifndef HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
#define HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
typedef enum MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
{
	MAV_REMOTE_LOG_DATA_BLOCK_STOP=2147483645, /* UAV to stop sending DataFlash blocks | */
	MAV_REMOTE_LOG_DATA_BLOCK_START=2147483646, /* UAV to start sending DataFlash blocks | */
	MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_ENUM_END=2147483647, /*  | */
} MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS;
#endif

/** @brief Possible remote log data block statuses */
#ifndef HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
#define HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
typedef enum MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
{
	MAV_REMOTE_LOG_DATA_BLOCK_NACK=0, /* This block has NOT been received | */
	MAV_REMOTE_LOG_DATA_BLOCK_ACK=1, /* This block has been received | */
	MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_ENUM_END=2, /*  | */
} MAV_REMOTE_LOG_DATA_BLOCK_STATUSES;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_sensor_offsets.h"
#include "./mavlink_msg_set_mag_offsets.h"
#include "./mavlink_msg_meminfo.h"
#include "./mavlink_msg_ap_adc.h"
#include "./mavlink_msg_digicam_configure.h"
#include "./mavlink_msg_digicam_control.h"
#include "./mavlink_msg_mount_configure.h"
#include "./mavlink_msg_mount_control.h"
#include "./mavlink_msg_mount_status.h"
#include "./mavlink_msg_fence_point.h"
#include "./mavlink_msg_fence_fetch_point.h"
#include "./mavlink_msg_fence_status.h"
#include "./mavlink_msg_ahrs.h"
#include "./mavlink_msg_simstate.h"
#include "./mavlink_msg_hwstatus.h"
#include "./mavlink_msg_radio.h"
#include "./mavlink_msg_limits_status.h"
#include "./mavlink_msg_wind.h"
#include "./mavlink_msg_data16.h"
#include "./mavlink_msg_data32.h"
#include "./mavlink_msg_data64.h"
#include "./mavlink_msg_data96.h"
#include "./mavlink_msg_rangefinder.h"
#include "./mavlink_msg_airspeed_autocal.h"
#include "./mavlink_msg_rally_point.h"
#include "./mavlink_msg_rally_fetch_point.h"
#include "./mavlink_msg_compassmot_status.h"
#include "./mavlink_msg_ahrs2.h"
#include "./mavlink_msg_camera_status.h"
#include "./mavlink_msg_camera_feedback.h"
#include "./mavlink_msg_battery2.h"
#include "./mavlink_msg_ahrs3.h"
#include "./mavlink_msg_autopilot_version_request.h"
#include "./mavlink_msg_remote_log_data_block.h"
#include "./mavlink_msg_remote_log_block_status.h"
#include "./mavlink_msg_led_control.h"
#include "./mavlink_msg_mag_cal_progress.h"
#include "./mavlink_msg_mag_cal_report.h"
#include "./mavlink_msg_ekf_status_report.h"
#include "./mavlink_msg_pid_tuning.h"
#include "./mavlink_msg_gimbal_report.h"
#include "./mavlink_msg_gimbal_control.h"
#include "./mavlink_msg_gimbal_torque_cmd_report.h"
#include "./mavlink_msg_gopro_heartbeat.h"
#include "./mavlink_msg_gopro_get_request.h"
#include "./mavlink_msg_gopro_get_response.h"
#include "./mavlink_msg_gopro_set_request.h"
#include "./mavlink_msg_gopro_set_response.h"
#include "./mavlink_msg_rpm.h"

// base include
#include "../common/common.h"

#if MAVLINK_MULTI_DIALECT
#include "../mavlink_get_info.h"
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_ARDUPILOTMEGA_H
