/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mission_block.cpp
 *
 * Helper class to use mission items
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <float.h>

#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>

#include "navigator.h"
#include "mission_block.h"

actuator_controls_s actuators;
orb_advert_t actuator_pub_fd;


MissionBlock::MissionBlock(Navigator *navigator, const char *name) :
	NavigatorMode(navigator, name),
	_mission_item({0}),
	_waypoint_position_reached(false),
	_waypoint_yaw_reached(false),
	_time_first_inside_orbit(0),
	_action_start(0),
	_time_wp_reached(0),
	_actuators{},
	_actuator_pub(nullptr),
	_cmd_pub(nullptr),
	_param_yaw_timeout(this, "MIS_YAW_TMT", false),
	_param_yaw_err(this, "MIS_YAW_ERR", false),
	_param_vtol_wv_land(this, "VT_WV_LND_EN", false),
	_param_vtol_wv_loiter(this, "VT_WV_LTR_EN", false)
{
}

MissionBlock::~MissionBlock()
{
}

bool
MissionBlock::is_mission_item_reached()
{
	/* handle non-navigation or indefinite waypoints */
	switch (_mission_item.nav_cmd) {
		case NAV_CMD_DO_SET_SERVO:
			return true;

		case NAV_CMD_LAND: /* fall through */
		case NAV_CMD_VTOL_LAND:
			return _navigator->get_vstatus()->condition_landed;

		/* TODO: count turns */
		/*_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||*/
		case NAV_CMD_IDLE: /* fall through */
		case NAV_CMD_LOITER_UNLIMITED:
			return false;

		case NAV_CMD_DO_DIGICAM_CONTROL:
		case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
			return true;

		case NAV_CMD_DO_VTOL_TRANSITION:
			/*
			 * We wait half a second to give the transition command time to propagate.
			 * Then monitor the transition status for completion.
			 */
			if (hrt_absolute_time() - _action_start > 500000 &&
					!_navigator->get_vstatus()->in_transition_mode) {
				_action_start = 0;
				return true;
			} else {
				return false;
			}

		case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
			// XXX not differentiating ground and airspeed yet
			if (_mission_item.params[1] > 0.0f) {
				_navigator->set_cruising_speed(_mission_item.params[1]);
			} else {
				_navigator->set_cruising_speed();
			}
			return true;

		default:
			/* do nothing, this is a 3D waypoint */
			break;
	}

	hrt_abstime now = hrt_absolute_time();

	if ((_navigator->get_vstatus()->condition_landed == false)
		&& !_waypoint_position_reached) {

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		float altitude_amsl = _mission_item.altitude_is_relative
				      ? _mission_item.altitude + _navigator->get_home_position()->alt
			              : _mission_item.altitude;

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, altitude_amsl,
				                          _navigator->get_global_position()->lat,
							  _navigator->get_global_position()->lon,
							  _navigator->get_global_position()->alt,
				&dist_xy, &dist_z);

		if ((_mission_item.nav_cmd == NAV_CMD_TAKEOFF || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)
			&& _navigator->get_vstatus()->is_rotary_wing) {
			/* require only altitude for takeoff for multicopter, do not use waypoint acceptance radius */
			if (_navigator->get_global_position()->alt >
				altitude_amsl - _navigator->get_acceptance_radius()) {
				_waypoint_position_reached = true;
			}
		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
			/* for takeoff mission items use the parameter for the takeoff acceptance radius */
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius()) {
				_waypoint_position_reached = true;
			}
		} else if (!_navigator->get_vstatus()->is_rotary_wing &&
			(_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
			_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT)) {
			/* Loiter mission item on a non rotary wing: the aircraft is going to circle the
			 * coordinates with a radius equal to the loiter_radius field. It is not flying
			 * through the waypoint center.
			 * Therefore the item is marked as reached once the system reaches the loiter
			 * radius (+ some margin). Time inside and turn count is handled elsewhere.
			 */
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius(_mission_item.loiter_radius * 1.2f)) {
				_waypoint_position_reached = true;
			}
		} else {
			/* for normal mission items used their acceptance radius */
			float mission_acceptance_radius = _navigator->get_acceptance_radius(_mission_item.acceptance_radius);

			/* if set to zero use the default instead */
			if (mission_acceptance_radius < NAV_EPSILON_POSITION) {
				mission_acceptance_radius = _navigator->get_acceptance_radius();
			}

			if (dist >= 0.0f && dist <= mission_acceptance_radius) {
				_waypoint_position_reached = true;
			}
		}

		if (_waypoint_position_reached) {
			// reached just now
			_time_wp_reached = now;
		}
	}

	/* Check if the waypoint and the requested yaw setpoint. */

	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		/* TODO: removed takeoff, why? */
		if (_navigator->get_vstatus()->is_rotary_wing && PX4_ISFINITE(_mission_item.yaw)) {

			/* check yaw if defined only for rotary wing except takeoff */
			float yaw_err = _wrap_pi(_mission_item.yaw - _navigator->get_global_position()->yaw);

			/* accept yaw if reached or if timeout is set in which case we ignore not forced headings */
			if (fabsf(yaw_err) < math::radians(_param_yaw_err.get())
					|| (_param_yaw_timeout.get() >= FLT_EPSILON && !_mission_item.force_heading)) {
				_waypoint_yaw_reached = true;
			}

			/* if heading needs to be reached, the timeout is enabled and we don't make it, abort mission */
			if (!_waypoint_yaw_reached && _mission_item.force_heading &&
						_param_yaw_timeout.get() >= FLT_EPSILON &&
						now - _time_wp_reached >= (hrt_abstime)_param_yaw_timeout.get() * 1e6f) {
				_navigator->set_mission_failure("unable to reach heading within timeout");
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	/* Once the waypoint and yaw setpoint have been reached we can start the loiter time countdown */
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		if (_time_first_inside_orbit == 0) {
			_time_first_inside_orbit = now;

			// if (_mission_item.time_inside > 0.01f) {
			// 	mavlink_log_critical(_mavlink_log_pub, "waypoint reached, wait for %.1fs",
			// 		(double)_mission_item.time_inside);
			// }
		}

		/* check if the MAV was long enough inside the waypoint orbit */
		if (now - _time_first_inside_orbit >= (hrt_abstime)_mission_item.time_inside * 1e6f) {
			return true;
		}
	}
	return false;
}

void
MissionBlock::reset_mission_item_reached()
{
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_first_inside_orbit = 0;
	_time_wp_reached = 0;
}

void
MissionBlock::mission_item_to_vehicle_command(const struct mission_item_s *item, struct vehicle_command_s *cmd)
{
	// we're expecting a mission command item here so assign the "raw" inputs to the command
	// (MAV_FRAME_MISSION mission item)
	cmd->param1 = item->params[0];
	cmd->param2 = item->params[1];
	cmd->param3 = item->params[2];
	cmd->param4 = item->params[3];
	cmd->param5 = item->params[4];
	cmd->param6 = item->params[5];
	cmd->param7 = item->params[6];
	cmd->command = item->nav_cmd;

	cmd->target_system = _navigator->get_vstatus()->system_id;
	cmd->target_component = _navigator->get_vstatus()->component_id;
	cmd->source_system = _navigator->get_vstatus()->system_id;
	cmd->source_component = _navigator->get_vstatus()->component_id;
	cmd->confirmation = false;
}

void
MissionBlock::issue_command(const struct mission_item_s *item)
{
	if (item_contains_position(item)) {
		return;
	}

	if (item->nav_cmd == NAV_CMD_DO_SET_SERVO) {
		PX4_WARN("do_set_servo command");
		// XXX: we should issue a vehicle command and handle this somewhere else
		memset(&actuators, 0, sizeof(actuators));
		// params[0] actuator number to be set 0..5 ( corresponds to AUX outputs 1..6
		// params[1] new value for selected actuator in ms 900...2000
		actuators.control[(int)item->params[0]] = 1.0f / 2000 * -item->params[1];
		actuators.timestamp = hrt_absolute_time();

		if (_actuator_pub != nullptr) {
			orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &actuators);

		} else {
			_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuators);
		}

	} else {
		PX4_WARN("forwarding command %d\n", item->nav_cmd);
		struct vehicle_command_s cmd = {};
		mission_item_to_vehicle_command(item, &cmd);
		_action_start = hrt_absolute_time();

		if (_cmd_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_command), _cmd_pub, &cmd);

		} else {
			_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
		}
	}
}

bool
MissionBlock::item_contains_position(const struct mission_item_s *item)
{
	// XXX: maybe extend that check onto item properties
	if (item->nav_cmd == NAV_CMD_DO_JUMP ||
		item->nav_cmd == NAV_CMD_DO_CHANGE_SPEED ||
		item->nav_cmd == NAV_CMD_DO_SET_SERVO ||
		item->nav_cmd == NAV_CMD_DO_REPEAT_SERVO ||
		item->nav_cmd == NAV_CMD_DO_DIGICAM_CONTROL ||
		item->nav_cmd == NAV_CMD_DO_SET_CAM_TRIGG_DIST ||
		item->nav_cmd == NAV_CMD_DO_VTOL_TRANSITION) {
		return false;
	}

	return true;
}

void
MissionBlock::mission_item_to_position_setpoint(const struct mission_item_s *item, struct position_setpoint_s *sp)
{
	/* set the correct setpoint for vtol transition */

	if(item->nav_cmd == NAV_CMD_DO_VTOL_TRANSITION && PX4_ISFINITE(item->yaw)
			&& item->params[0] >= vehicle_status_s::VEHICLE_VTOL_STATE_FW - 0.5f) {
		sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		waypoint_from_heading_and_distance(_navigator->get_global_position()->lat,
										   _navigator->get_global_position()->lon,
										   item->yaw,
										   1000000.0f,
										   &sp->lat,
										   &sp->lon);
		sp->alt = _navigator->get_global_position()->alt;
	}


	/* don't change the setpoint for non-position items */
	if (!item_contains_position(item)) {
		return;
	}

	sp->valid = true;
	sp->lat = item->lat;
	sp->lon = item->lon;
	sp->alt = item->altitude_is_relative ? item->altitude + _navigator->get_home_position()->alt : item->altitude;
	sp->yaw = item->yaw;
	sp->loiter_radius = (item->loiter_radius > NAV_EPSILON_POSITION) ? item->loiter_radius :
				_navigator->get_loiter_radius();
	sp->loiter_direction = item->loiter_direction;
	sp->pitch_min = item->pitch_min;
	sp->acceptance_radius = item->acceptance_radius;
	sp->disable_mc_yaw_control = false;
	sp->cruising_speed = _navigator->get_cruising_speed();

	switch (item->nav_cmd) {
	case NAV_CMD_IDLE:
		sp->type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		break;

	case NAV_CMD_TAKEOFF:
	case NAV_CMD_VTOL_TAKEOFF:
		sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		break;

	case NAV_CMD_LAND:
	case NAV_CMD_VTOL_LAND:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LAND;
		if(_navigator->get_vstatus()->is_vtol && _param_vtol_wv_land.get()){
			sp->disable_mc_yaw_control = true;
		}
		break;

	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_TURN_COUNT:
	case NAV_CMD_LOITER_UNLIMITED:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		if(_navigator->get_vstatus()->is_vtol && _param_vtol_wv_loiter.get()){
			sp->disable_mc_yaw_control = true;
		}
		break;

	default:
		sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;
	}
}

void
MissionBlock::set_previous_pos_setpoint()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    if (pos_sp_triplet->current.valid) {
        memcpy(&pos_sp_triplet->previous, &pos_sp_triplet->current, sizeof(struct position_setpoint_s));
    }
}

void
MissionBlock::set_loiter_item(struct mission_item_s *item, float min_clearance)
{
	if (_navigator->get_vstatus()->condition_landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {
		item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		if (_navigator->get_can_loiter_at_sp() && pos_sp_triplet->current.valid) {
			/* use current position setpoint */
			item->lat = pos_sp_triplet->current.lat;
			item->lon = pos_sp_triplet->current.lon;
			item->altitude = pos_sp_triplet->current.alt;

		} else {
			/* use current position and use return altitude as clearance */
			item->lat = _navigator->get_global_position()->lat;
			item->lon = _navigator->get_global_position()->lon;
			item->altitude = _navigator->get_global_position()->alt;

			if (min_clearance > 0.0f && item->altitude < _navigator->get_home_position()->alt + min_clearance) {
				item->altitude = _navigator->get_home_position()->alt + min_clearance;
			}
		}

		item->altitude_is_relative = false;
		item->yaw = NAN;
		item->loiter_radius = _navigator->get_loiter_radius();
		item->loiter_direction = 1;
		item->acceptance_radius = _navigator->get_acceptance_radius();
		item->time_inside = 0.0f;
		item->pitch_min = 0.0f;
		item->autocontinue = false;
		item->origin = ORIGIN_ONBOARD;
	}
}

void
MissionBlock::set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s & target, float yaw)
{
	if (_navigator->get_vstatus()->condition_landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {

		item->nav_cmd = NAV_CMD_FOLLOW_TARGET;

		/* use current target position */

		item->lat = target.lat;
		item->lon = target.lon;
		item->altitude = _navigator->get_home_position()->alt;

		if (min_clearance > 0.0f) {
			item->altitude += min_clearance;
		} else {
			item->altitude += 8.0f; // if min clearance is bad set it to 8.0 meters (well above the average height of a person)
		}
	}

	item->altitude_is_relative = false;
	item->yaw = yaw;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->loiter_direction = 1;
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->pitch_min = 0.0f;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_takeoff_item(struct mission_item_s *item, float min_clearance, float min_pitch)
{
	item->nav_cmd = NAV_CMD_TAKEOFF;

	/* use current position and use return altitude as clearance */
	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;
	item->altitude = _navigator->get_global_position()->alt;

	if (min_clearance > 0.0f) {
		item->altitude += min_clearance;

		/* we must takeoff to a point further above ground than the acceptance radius */
		if (_navigator->get_acceptance_radius() > min_clearance) {
			item->altitude += _navigator->get_acceptance_radius();
		}
	}

	item->altitude_is_relative = false;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->loiter_direction = 1;
	item->time_inside = 0.0f;
	item->pitch_min = min_pitch;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_land_item(struct mission_item_s *item, bool at_current_location)
{

	/* VTOL transition to RW before landing */
	if(_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing){
		struct vehicle_command_s cmd = {};
		cmd.command = NAV_CMD_DO_VTOL_TRANSITION;
		cmd.param1 = vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		if (_cmd_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_command), _cmd_pub, &cmd);
		} else {
			_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
		}
	}

	/* set the land item */
	item->nav_cmd = NAV_CMD_LAND;

	/* use current position */
	if (at_current_location) {
		item->lat = _navigator->get_global_position()->lat;
		item->lon = _navigator->get_global_position()->lon;

	/* use home position */
	} else {
		item->lat = _navigator->get_home_position()->lat;
		item->lon = _navigator->get_home_position()->lon;
	}

	item->altitude = 0;
	item->altitude_is_relative = false;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->loiter_direction = 1;
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->pitch_min = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_current_position_item(struct mission_item_s *item)
{
	item->nav_cmd = NAV_CMD_WAYPOINT;
	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;
	item->altitude_is_relative = false;
	item->altitude = _navigator->get_global_position()->alt;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->loiter_direction = 1;
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->pitch_min = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_idle_item(struct mission_item_s *item)
{
	item->nav_cmd = NAV_CMD_IDLE;
	item->lat = _navigator->get_home_position()->lat;
	item->lon = _navigator->get_home_position()->lon;
	item->altitude_is_relative = false;
	item->altitude = _navigator->get_home_position()->alt;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->loiter_direction = 1;
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->pitch_min = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}


