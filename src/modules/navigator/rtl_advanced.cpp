/**
 * @file rcrecover.cpp
 * RC recovery navigation mode
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <algorithm>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "rtl_advanced.h"

#define DELAY_SIGMA	0.01f

RTLAdvanced::RTLAdvanced(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_fallback_delay(this, "RTLA_FALLBCK_DLY", false),
	_param_land_delay(this, "RTL_LAND_DELAY", false)
{
	// load initial params
	updateParams();
	
	// initial reset
	on_inactive();
}

RTLAdvanced::~RTLAdvanced()
{
}

void RTLAdvanced::on_inactive() {
	_tracker = NULL;
	deadline = HRT_ABSTIME_MAX;
}

void RTLAdvanced::on_activation() {
	// For safety reasons don't go into RTL if landed
	if (_navigator->get_land_detected()->landed) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: not available when landed");
		return;
	}

	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: return along shortest path");

	_tracker = _navigator->get_tracker();
	_tracker->dump_graph();
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Init return path and setpoint triplet
	float x, y, z;
	if ((pos_sp_triplet->current.valid = _tracker->get_current_pos(current_pos_handle, x, y, z)))
		setpoint_from_xyz(pos_sp_triplet->current, x, y, z);

	pos_sp_triplet->next.valid = false;
	advance_setpoint_triplet(pos_sp_triplet);
	update_deadline();
}

void RTLAdvanced::on_active() {
	if (deadline <= hrt_absolute_time()) {
		
		_tracker = NULL;
		deadline = HRT_ABSTIME_MAX;

		if (land_after_deadline) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: landing");
			TRACKER_DBG("proceed to land");

			// Perform landing
			set_land_item(&_mission_item, true);
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
			pos_sp_triplet->next.valid = false;
			_navigator->set_position_setpoint_triplet_updated();

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: fallback to legacy mode");
			TRACKER_DBG("fall back to legacy RTL");
			_navigator->set_rtl_variant(false);
		}

	} else if (_tracker && _tracker->is_close_to_pos(current_pos_handle)) {
		if (advance_setpoint_triplet(_navigator->get_position_setpoint_triplet()))
			update_deadline(); // We made progress, update the deadline.
		else
			_tracker = NULL; // If the return path is empty, discard the tracker so we don't make further unneccessary checks.
	}
}

void RTLAdvanced::update_deadline() {
	updateParams();

	land_after_deadline = _tracker->is_close_to_home(current_pos_handle);
	float delay = land_after_deadline ? _param_land_delay.get() : _param_fallback_delay.get();
	
	if (delay < 0)
		deadline = HRT_ABSTIME_MAX;
	else
		deadline = hrt_absolute_time() + (hrt_abstime)delay * 1e6f;
}

void RTLAdvanced::setpoint_from_xyz(position_setpoint_s &sp, float x, float y, float z) {
	double lat;
	double lon;
	float alt;

	if (globallocalconverter_toglobal(x, y, z, &lat, &lon, &alt))
		return; // todo: proper error handling

	sp = {
		.valid = true,
		//.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD, // this seems wrong but we want to define a local (instead of global) position for this setpoint 
		.position_valid = true,
		//.x = x,
		//.y = y,
		//.z = z,
		.lat = lat,
		.lon = lon,
		.alt = alt,
		.acceptance_radius = 1,
		.disable_mc_yaw_control = true,
		.cruising_speed = _navigator->get_cruising_speed(),
		.cruising_throttle = _navigator->get_cruising_throttle(),
		.loiter_radius = _navigator->get_loiter_radius(),
		.loiter_direction = 1
	};
}


void RTLAdvanced::dump_setpoint(const char *name, position_setpoint_s &sp, bool local) {
	if (local)
		TRACKER_DBG("%s setpoint is (%f, %f, %f), %s", name, sp.x, sp.y, sp.z, sp.valid ? "valid" : "invalid");
	else
		TRACKER_DBG("%s setpoint is (%lf, %lf, %f), %s", name, sp.lat, sp.lon, sp.alt, sp.valid ? "valid" : "invalid");
}


bool RTLAdvanced::advance_setpoint_triplet(position_setpoint_triplet_s *pos_sp_triplet) {
	// Shift setpoints if possible
	if (pos_sp_triplet->next.valid) {
		if (pos_sp_triplet->current.valid)
			pos_sp_triplet->previous = pos_sp_triplet->current;
		pos_sp_triplet->current = pos_sp_triplet->next;
		current_pos_handle = next_pos_handle;
	} else {
		next_pos_handle = current_pos_handle;
	}

	// Load next setpoint
	float x, y, z;
	if ((pos_sp_triplet->next.valid = _tracker->get_path_to_home(next_pos_handle, x, y, z)))
		setpoint_from_xyz(pos_sp_triplet->next, x, y, z);
	
	// Apply updated setpoint triplet
	_navigator->set_position_setpoint_triplet_updated();

	//dump_setpoint("previous", pos_sp_triplet->previous, false);
	//dump_setpoint("current", pos_sp_triplet->current, false);
	//dump_setpoint("next", pos_sp_triplet->next, false);

	return pos_sp_triplet->next.valid && !_tracker->is_same_pos(current_pos_handle, next_pos_handle);
}
