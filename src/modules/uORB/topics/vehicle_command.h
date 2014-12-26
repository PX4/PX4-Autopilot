/****************************************************************************
 *
 *   Copyright (C) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vehicle_command.h
 * Definition of the vehicle command uORB topic.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef TOPIC_VEHICLE_COMMAND_H_
#define TOPIC_VEHICLE_COMMAND_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * Commands for commander app.
 *
 * Should contain all commands from MAVLink's VEHICLE_CMD ENUM,
 * but can contain additional ones.
 */
enum VEHICLE_CMD {
	VEHICLE_CMD_CUSTOM_0 = 0, /* test command */
	VEHICLE_CMD_CUSTOM_1 = 1, /* test command */
	VEHICLE_CMD_CUSTOM_2 = 2, /* test command */
	VEHICLE_CMD_NAV_WAYPOINT = 16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_NAV_LOITER_UNLIM = 17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_NAV_LOITER_TURNS = 18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_NAV_LOITER_TIME = 19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_NAV_LAND = 21, /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_NAV_TAKEOFF = 22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_NAV_ROI = 80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
	VEHICLE_CMD_NAV_PATHPLANNING = 81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
	VEHICLE_CMD_NAV_GUIDED_LIMITS=90, /* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
	VEHICLE_CMD_NAV_GUIDED_MASTER=91, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_NAV_LAST = 95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_CONDITION_DELAY = 112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_CONDITION_CHANGE_ALT = 113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
	VEHICLE_CMD_CONDITION_DISTANCE = 114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_CONDITION_YAW = 115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
	VEHICLE_CMD_CONDITION_LAST = 159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_SET_MODE = 176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_JUMP = 177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_CHANGE_SPEED = 178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_SET_HOME = 179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
	VEHICLE_CMD_DO_SET_PARAMETER = 180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_SET_RELAY = 181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_REPEAT_RELAY = 182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_SET_SERVO = 183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_REPEAT_SERVO = 184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_CONTROL_VIDEO = 200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
	VEHICLE_CMD_DO_LAST = 240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Empty| Empty|  */
	VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
	VEHICLE_CMD_PREFLIGHT_STORAGE = 245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
	VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
	VEHICLE_CMD_OVERRIDE_GOTO = 252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
	VEHICLE_CMD_MISSION_START = 300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
	VEHICLE_CMD_COMPONENT_ARM_DISARM = 400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
	VEHICLE_CMD_START_RX_PAIR = 500, /* Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  */
	VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001, /**< Prepare a payload deployment in the flight plan */
	VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002 /**< Control a pre-programmed payload deployment */
};

/**
 * Commands for commander app.
 *
 * Should contain all of MAVLink's VEHICLE_CMD_RESULT values
 * but can contain additional ones.
 */
enum VEHICLE_CMD_RESULT {
	VEHICLE_CMD_RESULT_ACCEPTED = 0, /* Command ACCEPTED and EXECUTED | */
	VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1, /* Command TEMPORARY REJECTED/DENIED | */
	VEHICLE_CMD_RESULT_DENIED = 2, /* Command PERMANENTLY DENIED | */
	VEHICLE_CMD_RESULT_UNSUPPORTED = 3, /* Command UNKNOWN/UNSUPPORTED | */
	VEHICLE_CMD_RESULT_FAILED = 4, /* Command executed, but failed | */
	VEHICLE_CMD_RESULT_ENUM_END = 5, /*  | */
};

/**
 * @addtogroup topics
 * @{
 */

struct vehicle_command_s {
	float param1;			/**< Parameter 1, as defined by MAVLink VEHICLE_CMD enum.   */
	float param2;			/**< Parameter 2, as defined by MAVLink VEHICLE_CMD enum.   */
	float param3;			/**< Parameter 3, as defined by MAVLink VEHICLE_CMD enum.   */
	float param4;			/**< Parameter 4, as defined by MAVLink VEHICLE_CMD enum.   */
	double param5;			/**< Parameter 5, as defined by MAVLink VEHICLE_CMD enum.   */
	double param6;			/**< Parameter 6, as defined by MAVLink VEHICLE_CMD enum.   */
	float param7;			/**< Parameter 7, as defined by MAVLink VEHICLE_CMD enum.   */
	enum VEHICLE_CMD command;		/**< Command ID, as defined MAVLink by VEHICLE_CMD enum.   */
	uint8_t target_system;		/**< System which should execute the command  */
	uint8_t target_component;	/**< Component which should execute the command, 0 for all components  */
	uint8_t source_system;		/**< System sending the command  */
	uint8_t source_component;	/**< Component sending the command  */
	uint8_t confirmation;		/**< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) */
}; /**< command sent to vehicle */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_command);



#endif
