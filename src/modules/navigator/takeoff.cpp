/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file Takeoff.cpp
 *
 * Helper class to Takeoff
 *
 * @author Lorenz Meier <lorenz@px4.io
 */
/*******************************
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "takeoff.h"
#include "navigator.h"

Takeoff::Takeoff(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false)
{
	// load initial params
	updateParams();
}

Takeoff::~Takeoff()
{
}

void
Takeoff::on_inactive()
{
}

void
Takeoff::on_activation()
{
	set_takeoff_position();
}

void
Takeoff::on_active()
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();
	if (rep->current.valid) {
		// reset the position
		set_takeoff_position();

	} else if (is_mission_item_reached() && !_navigator->get_mission_result()->finished) {
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		// set loiter item so position controllers stop doing takeoff logic
		set_loiter_item(&_mission_item);
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
Takeoff::set_takeoff_position()
{
	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, _param_min_alt.get());
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.yaw = _navigator->get_home_position()->yaw;
	pos_sp_triplet->current.yaw_valid = true;
	pos_sp_triplet->next.valid = false;

	// check if a specific target altitude has been set
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();
	if (rep->current.valid) {
		if (PX4_ISFINITE(rep->current.alt)) {
			pos_sp_triplet->current.alt = rep->current.alt;
		}

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		if (PX4_ISFINITE(rep->current.lat) && PX4_ISFINITE(rep->current.lon)) {
			pos_sp_triplet->current.lat = rep->current.lat;
			pos_sp_triplet->current.lon = rep->current.lon;
		}

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}

	_navigator->set_can_loiter_at_sp(true);

	_navigator->set_position_setpoint_triplet_updated();
}
***************************/


//更改后的程序


#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>
#include <float.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <uORB/uORB.h>  
#include <uORB/topics/position_setpoint_triplet.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <lib/geo/geo.h>

#include "takeoff.h"
#include "navigator.h"
#include "rtl.h"

//构造函数
Takeoff::Takeoff(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	takeoff_state{TAKEOFF_STATE_TAKEOFF_NONE},
	take_off_lock{false},
	takeoff_ed_flag{true},
	takeoff_timestamp_takeoff_ed{1000000},
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false),//起飞高度设定值
	_param_rtl_min_dist(this, "RTL_MIN_DIST", false)//最小距离设定值
{
	// load initial params
	updateParams();
}

Takeoff::~Takeoff()//析构函数
{
}

void
Takeoff::on_inactive()
{
}

void
Takeoff::on_activation()//第一次运行
{
	////设置自动起飞的坐标，根据home position进行计算得出，高度由para_min_alt调节,MIS_TAKEOFF_ALT参数决定
	set_takeoff_position();
}

void
Takeoff::on_active()
{
	//未着陆且未到达目标航点
	if (is_mission_item_reached()) {
			advance_takeoff();
			set_takeoff_item_add();
		}

}
void Takeoff::advance_takeoff()
{
	switch(takeoff_state){
	case TAKEOFF_STATE_TAKEOFF:
		takeoff_state=TAKEOFF_STATE_TAKEOFF_ED_LOITER;
		break;
	case TAKEOFF_STATE_TAKEOFF_ED_LOITER:
		takeoff_state=TAKEOFF_STATE_NAVIGATION;
		//takeoff_state=TAKEOFF_STATE_LAND;
		break;
	case TAKEOFF_STATE_NAVIGATION:
		takeoff_state=TAKEOFF_STATE_NAVIGATION_ED_LOITER;
		break;
	case TAKEOFF_STATE_NAVIGATION_ED_LOITER:
		takeoff_state=TAKEOFF_STATE_LAND;
		break;
	case TAKEOFF_STATE_LAND:
		takeoff_state=TAKEOFF_STATE_LAND_ED;
		break;
	default:
		break;
	}
}

void Takeoff::set_takeoff_item_add()//##############################
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	switch(takeoff_state){

	case TAKEOFF_STATE_TAKEOFF:{//起飞过程中
		struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();
		if (rep->current.valid) {
			// reset the position
			set_takeoff_position();

		}
		break;
	}

	case TAKEOFF_STATE_TAKEOFF_ED_LOITER:{//达到起飞高度后游荡一定时间
		_mission_item.lat=_navigator->get_home_position()->lat;
		_mission_item.lon=_navigator->get_home_position()->lon;
		_mission_item.yaw = _navigator->get_home_position()->yaw;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.nav_cmd =NAV_CMD_LOITER_TIME_LIMIT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside =10;//游荡15秒
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;
		_navigator->set_can_loiter_at_sp(true);
		break;
	}

	case TAKEOFF_STATE_NAVIGATION:{//航点飞行
		struct map_projection_reference_s takeoff_ref;
		float distance_x,distance_y;
		distance_x=15;
		distance_y=0;
		double distance_position_lat,distance_position_lon;
		map_projection_init(&takeoff_ref,float(_navigator->get_home_position()->lat),float(_navigator->get_home_position()->lon));
		map_projection_reproject(&takeoff_ref,distance_x,distance_y,&distance_position_lat,&distance_position_lon);
		_mission_item.lat=distance_position_lat;//0.0005度
		_mission_item.lon=distance_position_lon;
		//_mission_item.yaw = _navigator->get_home_position()->yaw;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.nav_cmd =NAV_CMD_WAYPOINT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside=0;
		_mission_item.autocontinue=false;
		_mission_item.origin=ORIGIN_ONBOARD;
		_navigator->set_can_loiter_at_sp(false);
		break;
	}

	case TAKEOFF_STATE_NAVIGATION_ED_LOITER:{//航点飞行结束，游荡一定时间
		//_mission_item.lat=_navigator->get_home_position()->lat+distance_lat;//0.0005度
		//_mission_item.lon=_navigator->get_home_position()->lon+distance_lon;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.nav_cmd =NAV_CMD_LOITER_TIME_LIMIT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside =10;//游荡15秒
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;
		_navigator->set_can_loiter_at_sp(true);
		break;
		break;
	}
	case TAKEOFF_STATE_LAND:{//下落过程中
		_mission_item.yaw = _navigator->get_home_position()->yaw;
		set_land_item(&_mission_item, false);
		break;
	}

	case TAKEOFF_STATE_LAND_ED:{//着陆
		set_idle_item(&_mission_item);
		break;
	}

	default:break;

}
	reset_mission_item_reached();
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();

}


//设置自动起飞的坐标，根据home position进行计算得出，高度由para_min_alt调节,MIS_TAKEOFF_ALT参数决定
void
Takeoff::set_takeoff_position()
{
	/************************position_setpoint_triplet_s结构体内容*************************************/
//	uint64_t timestamp; // required for logger
//	uint8_t nav_state;
//	uint8_t _padding0[7]; // required for logger
//	struct position_setpoint_s previous;
//	struct position_setpoint_s current;
//	struct position_setpoint_s next;
	/*********************************************************************************************/
	takeoff_state=TAKEOFF_STATE_TAKEOFF;
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();//返回_takeoff_triplet结构体地址
/////////////////////////////////////////////////////////////////////////计算绝对高度
	float abs_altitude = 0.0f;

	//get_home_position()获得home坐标，navigator_main中由主题发布
	const float min_abs_altitude = _navigator->get_home_position()->alt + _param_min_alt.get();

	// Use altitude if it has been set.
	if (rep->current.valid && PX4_ISFINITE(rep->current.alt)) {
		abs_altitude = rep->current.alt;

		// If the altitude suggestion is lower than home + minimum clearance, raise it and complain.
		if (abs_altitude < min_abs_altitude) {
			abs_altitude = min_abs_altitude;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
					     "Using minimum takeoff altitude: %.2f m", (double)_param_min_alt.get());
		}

	} else {
		// Use home + minimum clearance but only notify.
		abs_altitude = min_abs_altitude;//使用最小起飞高度作为绝对高度
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
				 "Using minimum takeoff altitude: %.2f m", (double)_param_min_alt.get());
	}

//get_global_position为获取当前的球面坐标值，在navigator_main中订阅了
	if (abs_altitude < _navigator->get_global_position()->alt) {
		// If the suggestion is lower than our current alt, let's not go down.
		abs_altitude = _navigator->get_global_position()->alt;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(),
				     "Already higher than takeoff altitude");
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, abs_altitude);//填充_mission_item结构体
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	//转变_mission_item为当前设定值
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);//转变为位置设定值


	if (rep->current.valid) {

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		if (PX4_ISFINITE(rep->current.lat) && PX4_ISFINITE(rep->current.lon)) {
			pos_sp_triplet->current.lat = rep->current.lat;
			pos_sp_triplet->current.lon = rep->current.lon;
		}

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}

	_navigator->set_can_loiter_at_sp(true);

	_navigator->set_position_setpoint_triplet_updated();
	mavlink_log_info(_navigator->get_mavlink_log_pub(),
					 "takeoff_message:takeoff successfully!!");
}
