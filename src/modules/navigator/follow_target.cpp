/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>

#include "navigator.h"

FollowTarget::FollowTarget(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_navigator(navigator),
	_param_min_alt(this, "NAV_MIN_FT_HT", false),//飞机相对于homeposition的最小高度
	_param_tracking_dist(this, "NAV_FT_DST", false),//飞机跟随目标的距离，单位为米
	_param_tracking_side(this, "NAV_FT_FS", false),//飞机从哪个方向跟随目标
	_param_tracking_resp(this, "NAV_FT_RS", false),
	_param_yaw_auto_max(this, "MC_YAWRAUTO_MAX", false),//最大的偏航角速度，推荐值30,默认值为45度/秒
	_follow_target_state(SET_WAIT_FOR_TARGET_POSITION),//初始模式设定
	_follow_target_position(FOLLOW_FROM_BEHIND),//跟随方向确定
	_follow_target_sub(-1),//初始化话题订阅句柄
	_step_time_in_ms(0.0f),//初始化步距时间
	_follow_offset(OFFSET_M),//初始化跟随距离
	_target_updates(0),//标志位，用于判断位置跟随和速度跟随的可用性
	_last_update_time(0),
	_current_target_motion(),
	_previous_target_motion(),
	_yaw_rate(0.0F),
	_responsiveness(0.0F),
	_yaw_auto_max(0.0F),
	_yaw_angle(0.0F)
{
	updateParams();
	_current_target_motion = {};
	_previous_target_motion =  {};
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	_target_position_delta.zero();
}

FollowTarget::~FollowTarget()
{
}

void FollowTarget::on_inactive()
{
	reset_target_validity();
}

//对参数处理并计算出旋转矩阵
void FollowTarget::on_activation()//第一次运行
{
	updateParams();

	_follow_offset = _param_tracking_dist.get() < 1.0F ? 1.0F : _param_tracking_dist.get();//限制跟随距离大于1米

	_responsiveness = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);//限定范围

	_yaw_auto_max = math::radians(_param_yaw_auto_max.get());//将速度值从角度转化为弧度

	_follow_target_position = _param_tracking_side.get();//获取跟随方向

	if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
		_follow_target_position = FOLLOW_FROM_BEHIND;
	}
	_rot_matrix = (_follow_position_matricies[_follow_target_position]);//由跟随方向计算跟随矩阵

	if (_follow_target_sub < 0) {//第一次运行时执行
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}
}

void FollowTarget::on_active()
{
	struct map_projection_reference_s target_ref;//参考点结构体
	math::Vector<3> target_reported_velocity(0, 0, 0);
	follow_target_s target_motion_with_offset = {};//跟随坐标点
	uint64_t current_time = hrt_absolute_time();
	bool _radius_entered = false;
	bool _radius_exited = false;
	bool updated = false;
	float dt_ms = 0;

	orb_check(_follow_target_sub, &updated);//更新目标坐标值判断

	if (updated) {
		follow_target_s target_motion;//当前传回来的未处理过的目标坐标值

		_target_updates++;

		// save last known motion topic

		_previous_target_motion = _current_target_motion;

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);//由mavlink_receiver.cpp发布

		if (_current_target_motion.timestamp == 0) {
			_current_target_motion = target_motion;
		}

		//根据上一次的坐标值结合当前目标坐标值计算目标纬度和经度
		_current_target_motion.timestamp = target_motion.timestamp;
		_current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
						     1 - _responsiveness);
		_current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
						     1 - _responsiveness);

		//目标移动速度
		target_reported_velocity(0) = _current_target_motion.vx;
		target_reported_velocity(1) = _current_target_motion.vy;

		//当目标坐标超过2.5秒未更新时，复位
	} else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}

	// update distance to target

	if (target_position_valid()) { // 目标点位置有效，则获取到目标点的距离

		// get distance to target

		map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0),
				       &_target_distance(1)); // 将球体坐标系投影到XoY平面上

	}

	// update target velocity

	if (target_velocity_valid() && updated) { // 目标点速度有效，则获取到目标点的速度

		dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);//两次坐标更新的时间差

		// ignore a small dt

		if (dt_ms > 10.0F) {

			math::Vector<3> prev_position_delta = _target_position_delta;

			// get last gps known reference for target
			//将上一次的目标坐标点作为参考点，并计算当前目标坐标值相对上一次目标坐标点的位置

			map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

			// calculate distance the target has moved

			map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
					       &(_target_position_delta(0)), &(_target_position_delta(1)));

			// update the average velocity of the target based on the position

			_est_target_vel = _target_position_delta / (dt_ms / 1000.0f);//计算目标的速度，单位为米/秒：m/s

			// if the target is moving add an offset and rotation

			//_follow_offset为跟随的距离
			//_rot_matrix等于于以下矩阵
			// {
			//	-1.0F,  0.0F, 0.0F,
			//   0.0F, -1.0F, 0.0F,
			//	 0.0F,  0.0F, 1.0F
			//	}
			if (_est_target_vel.length() > .5F) {
				_target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
			}

			// are we within the target acceptance radius?
			// give a buffer to exit/enter the radius to give the velocity controller
			// a chance to catch up

			//_target_distance为目标点相对于飞机的位置
			//判断飞机与目标点的距离是否处于半径内
			_radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f); // 出口半径
			_radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M); // 入口半径

			// to keep the velocity increase/decrease smooth 为了保持速度增加或减小的平滑性
			// calculate how many velocity increments/decrements 计算速度的增减量
			// it will take to reach the targets velocity 增减量能改变当前速度到目标速度
			// with the given amount of steps also add a feed forward input that adjusts the 加入前馈输入，用于适应速度
			// velocity as the position gap increases since 当位置间隙增加时能更好的跟踪目标，不至于太近或太远
			// just traveling at the exact velocity of the target will not
			// get any closer or farther from the target

			//INTERPOLATION_PNTS表示插值点的数目，通常为20个点
			//计算每一步的速度递增值，该计算只有在坐标更新的时候会执行一次
			_step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
			_step_vel /= (dt_ms / 1000.0F * (float) INTERPOLATION_PNTS);
			_step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

			// if we are less than 1 meter from the target don't worry about trying to yaw
			// lock the yaw until we are at a distance that makes sense

			if ((_target_distance).length() > 1.0F) { 
				//计算航向角大小和速度

				// yaw rate smoothing

				// this really needs to control the yaw rate directly in the attitude pid controller
				// but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode
				// 需要在姿态PID控制器中直接控制偏航速度
				// 但是在自动控制模式下偏航速度不可控。

				_yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_current_target_motion.lat,
						_current_target_motion.lon); // 返回theta角， bearing->方位

				_yaw_rate = (_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0F);

				_yaw_rate = _wrap_pi(_yaw_rate); //周期为2pi的转化

				_yaw_rate = math::constrain(_yaw_rate, -1.0F * _yaw_auto_max, _yaw_auto_max);

			} else {
				_yaw_angle = _yaw_rate = NAN;
			}
		}

//		warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d con ratio = %3.6f yaw rate = %3.6f",
//				(double) _step_vel(0),
//				(double) _step_vel(1),
//				(double) _current_vel(0),
//				(double) _current_vel(1),
//				(double) _est_target_vel(0),
//				(double) _est_target_vel(1),
//				(double) (_target_distance).length(),
//				(double) (_target_position_offset + _target_distance).length(),
//				_follow_target_state,
//				(double)_avg_cos_ratio, (double) _yaw_rate);
	}

	if (target_position_valid()) {//如果当前目标位置可用

		// get the target position using the calculated offset

		map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
		map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
					 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
	}

	// clamp yaw rate smoothing if we are with in
	// 3 degrees of facing target

	//判断航向角差值是否小于3度
	if (PX4_ISFINITE(_yaw_rate)) {
		if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
			_yaw_rate = NAN;
		}
	}

	// update state machine

	switch (_follow_target_state) {

	case TRACK_POSITION: {//跟踪坐标

			if (_radius_entered == true) { //当距离目标点距离小于5米时进入
				_follow_target_state = TRACK_VELOCITY; //切换状态

			} else if (target_velocity_valid()) {//速度值可用，且不在半径内时
				set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset,_navigator->get_global_position()->yaw);
				// keep the current velocity updated with the target velocity for when it's needed
				_current_vel = _est_target_vel; //当前目标点的速度

				update_position_sp(true, true, 0.0f);// 使用速度(true)，使用位置(true)，偏航速度(0)

			} else {
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	case TRACK_VELOCITY: {

			if (_radius_exited == true) {
				_follow_target_state = TRACK_POSITION;

			} else if (target_velocity_valid()) {

				//对时间差进行调整
				//按照时间步距进行速度累加
				//初始速度值为TRACK_POSITION所赋值的_est_target_vel
				//最终结果为距离目标点小于5米后进行TARCH_VELOCITY控制，速度值根据目标的速度值为最终值逐渐变化至相等
				if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
					_current_vel += _step_vel;
					_last_update_time = current_time;
				}

				set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _navigator->get_global_position()->yaw);

				update_position_sp(true, false, 0.0f); // 使用速度(true)，使用位置(false)，偏航速度(0)

			} else {
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	case SET_WAIT_FOR_TARGET_POSITION: {//第一次进入时运行该段，将飞机当前坐标作为target坐标点，且高度设置为最小高度

			// Climb to the minimum altitude
			// and wait until a position is received

			follow_target_s target = {};

			// for now set the target at the minimum height above the uav

			target.lat = _navigator->get_global_position()->lat;
			target.lon = _navigator->get_global_position()->lon;
			target.alt = 0.0F;

			//将target的经度和纬度直接赋值给mission_item，高度取home点加上最小高度值，航向角直接取home点的偏航角。
			set_follow_target_item(&_mission_item, _param_min_alt.get(), target,_navigator->get_global_position()->yaw);//###

			update_position_sp(false, false, 0.0f);

			_follow_target_state = WAIT_FOR_TARGET_POSITION; //跳转到下一个状态
		}

	case WAIT_FOR_TARGET_POSITION: {//等待目标点可用

			if (is_mission_item_reached() && target_velocity_valid()) {//当至少出现两个可用的点时为速度可用状态
				_target_position_offset(0) = _follow_offset; // 确定跟随距离
				_follow_target_state = TRACK_POSITION; //切换状态
			}

			break;
		}
	}
}

void FollowTarget::update_position_sp(bool use_velocity, bool use_position, float yaw_rate)
{
	// convert mission item to current setpoint

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// activate line following in pos control if position is valid

	pos_sp_triplet->previous.valid = use_position;
	pos_sp_triplet->previous = pos_sp_triplet->current;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = use_position;
	pos_sp_triplet->current.velocity_valid = use_velocity;
	pos_sp_triplet->current.vx = _current_vel(0);
	pos_sp_triplet->current.vy = _current_vel(1);
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = PX4_ISFINITE(yaw_rate);
	pos_sp_triplet->current.yawspeed = yaw_rate;
	_navigator->set_position_setpoint_triplet_updated();
}

void FollowTarget::reset_target_validity()
{
	_yaw_rate = NAN;
	_previous_target_motion = {};
	_current_target_motion = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	reset_mission_item_reached();
	_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
}

bool FollowTarget::target_velocity_valid()
{
	// need at least 2 continuous data points for velocity estimate
	return (_target_updates >= 2);
}

bool FollowTarget::target_position_valid()
{
	// need at least 1 continuous data points for position estimate
	return (_target_updates >= 1);
}
