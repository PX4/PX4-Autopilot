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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <math.h>
#include <fcntl.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/formation_followers.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/chen_sd_formation.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>
#include "follow_target.h"
#include "navigator.h"
#include <pthread.h>
int ischanging=0;
bool formation_exit;
chen_sd_formation_s sd_formation;
char file_path[256];
FollowTarget::FollowTarget(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_navigator(navigator),
	_param_min_alt(this, "NAV_MIN_FT_HT", false),
	_param_tracking_dist(this, "NAV_FT_DST", false),
	_param_tracking_side(this, "NAV_FT_FS", false),
	_param_tracking_resp(this, "NAV_FT_RS", false),
	_param_yaw_auto_max(this, "MC_YAWRAUTO_MAX", false),
	_follow_target_state(SET_WAIT_FOR_TARGET_POSITION),
	_follow_target_position(0),
	_follow_target_sub(-1),
	_step_time_in_ms(0.0f),
	_follow_offset(OFFSET_M),
	_target_updates(0),
	_last_update_time(0),
	_chen_last_time(0),
	_current_target_motion(),
	_previous_target_motion(),
	_yaw_rate(0.0F),
	_responsiveness(0.0F),
	_yaw_auto_max(0.0F),
	_yaw_angle(0.0F),
	follower(),
	_param_x_offset(this, "INI_X_OFFSET", false),
	_param_y_offset(this, "INI_Y_OFFSET", false),
	_param_z_offset(this, "INI_Z_OFFSET", false),
	collision_x(0),
	collision_y(0),
	hdg_offset(0),
	log_pthread(0),
	log_opened(false)
{
	formation_exit=false;
	updateParams();
	_current_target_motion = {};
	_previous_target_motion =  {};
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	_target_position_delta.zero();
        _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_chen_sd_formation_pub = 0;
}

FollowTarget::~FollowTarget()
{
	formation_exit=true;
	pthread_join(log_pthread,NULL);
}

void FollowTarget::on_inactive()
{
	reset_target_validity();
}

void FollowTarget::on_activation()
{
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	follower.x_offset = _param_x_offset.get();
	follower.y_offset = _param_y_offset.get();
	follower.z_offset = _param_z_offset.get();
	limit_z_offset();
	follower.offset_to_leader = sqrt(
			follower.x_offset * follower.x_offset
					+ follower.y_offset * follower.y_offset);
	if (follower.x_offset >0||follower.x_offset <0)

		follower.relative_angle = M_PI/2 -atan(follower.y_offset / follower.x_offset);
	else

		follower.relative_angle = 0;

	_follow_offset = _param_tracking_dist.get() < 1.0F ? 1.0F : _param_tracking_dist.get();

	_responsiveness = 0.7;

	_yaw_auto_max = math::radians(_param_yaw_auto_max.get());

	_follow_target_position = _param_tracking_side.get();

	_rot_matrix = (_follow_position_matricies[_follow_target_position]);
	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}
}

void FollowTarget::on_active()
{
	struct map_projection_reference_s target_ref;
	follow_target_s target_motion_with_offset = {};
	uint64_t current_time = hrt_absolute_time();
	//double x_velocity_offset=0;
	//double y_velocity_offset=0;
	//double angle_to_leader;
	bool _radius_entered = false;
	bool _radius_exited = false;
	bool updated = false;
	//float dt = 0;

	struct manual_control_setpoint_s	_manual;
	static follow_target_s target_motion;

	orb_check(_manual_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
		if (_manual.x > 0.9 && fabs(_manual.y) < 0.1) {
			follower.x_offset += 0.05;
			calcu_relative_angle_distance();
		} else if (_manual.x < -0.9 && fabs(_manual.y) < 0.1) {
			follower.x_offset -= 0.05;
			calcu_relative_angle_distance();
		} else if (_manual.y > 0.9 && fabs(_manual.x) < 0.1) {
			follower.y_offset += 0.05;
			calcu_relative_angle_distance();
		} else if (_manual.y < -0.9 && fabs(_manual.x) < 0.1) {
			follower.y_offset -= 0.05;
			calcu_relative_angle_distance();
		} else if (_manual.y < -0.9 && _manual.x < -0.9) {
			if (_current_target_motion.plane_id % 4 == 1) {
				follower.x_offset = 3;
				follower.y_offset = 3;
				ischanging = 1;
				calcu_relative_angle_distance();
			} else if (_current_target_motion.plane_id % 4 == 2) {
				follower.x_offset = -3;
				follower.y_offset = 3;
				ischanging = 1;
				calcu_relative_angle_distance();
			} else if (_current_target_motion.plane_id % 4 == 3) {
				follower.x_offset = -3;
				follower.y_offset = -3;
				ischanging = 1;
				calcu_relative_angle_distance();
			} else if (_current_target_motion.plane_id % 4 == 0) {
				follower.x_offset = 3;
				follower.y_offset = -3;
				calcu_relative_angle_distance();
				ischanging = 1;
			}
		} else if (_manual.y > 0.9 && _manual.x > 0.9) {
			follower.x_offset = (_current_target_motion.plane_id - 1) * 2;
			follower.y_offset = (_current_target_motion.plane_id - 1) * 2;
			calcu_relative_angle_distance();
			ischanging = 1;
		}
		if (_manual.r > 0.9) {
			follower.relative_angle += 0.06;
			calcu_xy_offset();
		} else if (_manual.r < -0.9) {
			follower.relative_angle -= 0.06;
			calcu_xy_offset();
		}
		if (_manual.z > 0.98) {
			follower.z_offset += 0.015;
			limit_z_offset();
		} else if (_manual.z < 0.02) {
			follower.z_offset -= 0.015;
			limit_z_offset();
		}
	}
	orb_check(_follow_target_sub, &updated);
	if (updated) {

		_target_updates++;
		_chen_last_time = current_time;
		// save last known motion topic
		_previous_target_motion = _current_target_motion;

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);

		if (_current_target_motion.timestamp == 0) {
			_current_target_motion = target_motion;
		}
		if(!log_opened)
		{
			open_log(log_pthread);
			log_opened = true;
		}
		_current_target_motion.plane_id = target_motion.plane_id;
		_current_target_motion.timestamp = target_motion.timestamp;
		_current_target_motion.alt = (double)target_motion.alt+follower.z_offset;
		_current_target_motion.vz = target_motion.vz;
		_current_target_motion.vx = target_motion.vx;
		_current_target_motion.vy = target_motion.vy;
		hdg_offset = ((int)(target_motion.leader_hdg/0.08)) *0.08;
	} else if (((current_time - _chen_last_time) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}
	if (follower.offset_to_leader < 2) {
		follower.offset_to_leader = 2;
		calcu_xy_offset();
	}
	if (follower.offset_to_leader > MAX_DISTANCE) {
		follower.offset_to_leader = MAX_DISTANCE;
		calcu_xy_offset();
	}

	if(target_position_valid())
	{
		map_projection_init(&target_ref, target_motion.lat, target_motion.lon);
		/*map_projection_project(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon, &_target_distance(0),
						       &_target_distance(1));
		follower.distance_to_leader = (double)sqrtf(_target_distance(1)*_target_distance(1)
				+_target_distance(0)*_target_distance(0));
		if(follower.distance_to_leader<MIN_DISTANCE)
		{
			if (_target_distance(0) > 0 || _target_distance(0) < 0)
				angle_to_leader
			= M_PI / 2-atan( _target_distance(1) /  _target_distance(0));
			else
			angle_to_leader = 0;
			collision_x = 3*sin(angle_to_leader+M_PI/2);
			collision_y = 3*cos(angle_to_leader+M_PI/2);
		}
		else
		{
			collision_x= 0;
			collision_y= 0;
		}*/

		map_projection_reproject(&target_ref,
				follower.x_offset * cos(hdg_offset)
						- sin(hdg_offset) * follower.y_offset,
				follower.y_offset * cos(hdg_offset)
						+ follower.x_offset * sin(hdg_offset),
				&_current_target_motion.lat, &_current_target_motion.lon);
		sd_formation.tar_lat = _current_target_motion.lat;
		sd_formation.tar_lon = _current_target_motion.lon;
		sd_formation.tar_alt = _current_target_motion.alt;
		sd_formation.tar_vx = _current_target_motion.vx;
		sd_formation.tar_vy = _current_target_motion.vy;
		sd_formation.tar_vz = _current_target_motion.vz;
		sd_formation.vx = _navigator->get_global_position()->vel_n;
		sd_formation.vy = _navigator->get_global_position()->vel_e;
		sd_formation.vz = -_navigator->get_global_position()->vel_d;
		sd_formation.lat = _navigator->get_global_position()->lat;
		sd_formation.lon = _navigator->get_global_position()->lon;
		sd_formation.alt = _navigator->get_global_position()->alt;

		map_projection_init(&target_ref, _current_target_motion.lat,_current_target_motion.lon );
		map_projection_project(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon, &_target_distance(0),
							   &_target_distance(1));
		follower.distance_to_target = (double)sqrtf(_target_distance(1)*_target_distance(1)
				+_target_distance(0)*_target_distance(0));
	}

	if (target_velocity_valid() && updated) {

		double dt_ms = ((_current_target_motion.timestamp
				- _previous_target_motion.timestamp) / 1000);
		if (dt_ms > 10.0F) {
			//map_projection_init(&target_ref, _previous_target_motion.lat,
					//_previous_target_motion.lon);
			//map_projection_project(&target_ref, _current_target_motion.lat,
					//_current_target_motion.lon, &(_target_position_delta(0)),
					//&(_target_position_delta(1)));
			//_est_target_vel = _target_position_delta / (dt_ms / 1000.0f);
			_est_target_vel(0) = _current_target_motion.vx;
			_est_target_vel(1) = _current_target_motion.vy;
			/*if (_est_target_vel.length() > .5F) {
				_target_position_offset = _est_target_vel.normalized() * 3;
			}*/
			// if the target is moving add an offset and rotation
	}
	}

	_radius_exited = (follower.distance_to_target +_target_position_offset.length()
			> TARGET_ACCEPTANCE_RADIUS_M * 1.5);
	_radius_entered = (follower.distance_to_target + _target_position_offset.length()
			< TARGET_ACCEPTANCE_RADIUS_M);
	if (target_velocity_valid()){
		double x_vel = _navigator->get_global_position()->vel_n +_target_distance(0);
		double y_vel = _navigator->get_global_position()->vel_e +_target_distance(1);
		if ((_est_target_vel(0) - x_vel) > 1.0F) {
			_target_position_offset(0) = 6;
		} else if ((_est_target_vel(0) - x_vel) < -1.0F) {
			_target_position_offset(0) = -6;
		} else if (fabs(_est_target_vel(0) - x_vel) > 0.3F) {
			_target_position_offset(0) = (_est_target_vel(0) - x_vel) * 6;
		} else {
			_target_position_offset(0) = 0;
		}
		if ((_est_target_vel(1) - y_vel) > 1.0F) {
			_target_position_offset(1) = 6;
		} else if ((_est_target_vel(1) - y_vel) < -1.0F) {
			_target_position_offset(1) = -6;
		} else if (fabs(_est_target_vel(1) - y_vel) > 0.3F) {
			_target_position_offset(1) = (_est_target_vel(1) - y_vel) * 6;
		} else {
			_target_position_offset(1) = 0;
		}
	}
	if (_follow_target_state == TRACK_POSITION) {
		//dt = ((_current_target_motion.timestamp
				//- _previous_target_motion.timestamp) / 1000000);
				// ignore a small dt
			// get last gps known reference for target
			/*map_projection_init(&target_ref, _previous_target_motion.lat,
					_previous_target_motion.lon);
			// calculate distance the target has moved
			map_projection_project(&target_ref, _current_target_motion.lat,
					_current_target_motion.lon, &(_target_position_delta(0)),
					&(_target_position_delta(1)));*/
			// update the average velocity of the target based on the position
		//x_velocity_offset = _target_position_offset(0);//_current_target_motion.vx * dt;
		//y_velocity_offset = _target_position_offset(1);//_current_target_motion.vy * dt;
		//target_motion_with_offset = _current_target_motion;

		target_motion_with_offset.vz = _current_target_motion.vz;
		map_projection_init(&target_ref, _current_target_motion.lat,
				_current_target_motion.lon);
		map_projection_reproject(&target_ref, (float) _target_position_offset(0),
				(float)_target_position_offset(1), &target_motion_with_offset.lat,
				&target_motion_with_offset.lon);
	}
	if(_follow_target_state ==TRACK_VELOCITY)
	{
		//dt = ((_current_target_motion.timestamp
						//- _previous_target_motion.timestamp) / 1000000);
	/*	if (target_velocity_valid() && updated) {
			_est_target_vel(0) = _current_target_motion.vx;
			_est_target_vel(1) = _current_target_motion.vy;
		}*/
		//target_motion_with_offset = _current_target_motion;
		/*if ((_current_target_motion.vx > 0.6)||_current_target_motion.vx <-0.6) {
			x_velocity_offset = _current_target_motion.vx;
	} else {
				x_velocity_offset = 0;
	}
		if ((_current_target_motion.vy > 0.6)||_current_target_motion.vy <-0.6) {
			y_velocity_offset = _current_target_motion.vy ;
		} else {
			y_velocity_offset = 0;
		}*/
		target_motion_with_offset.vz = _current_target_motion.vz;
		map_projection_init(&target_ref, _current_target_motion.lat,
				_current_target_motion.lon);
		map_projection_reproject(&target_ref,
				(float) _target_position_offset(0),
				(float) _target_position_offset(1),
				&target_motion_with_offset.lat, &target_motion_with_offset.lon);
	}

	switch (_follow_target_state) {

		case TRACK_POSITION: {

				if (_radius_entered == true) {
					_follow_target_state = TRACK_VELOCITY;
					ischanging =0;
				} else if (target_velocity_valid()) {
					set_follow_target_item(&_mission_item, (double)(_current_target_motion.alt+_current_target_motion.plane_id*ischanging), target_motion_with_offset, _yaw_angle);
					// keep the current velocity updated with the target velocity for when it's needed
					//_current_vel = _est_target_vel;

					_current_vel = _est_target_vel;
					update_position_sp(true, true, 0);
				} else {
					_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
				}

				break;
			}

		case TRACK_VELOCITY: {

				if (_radius_exited == true) {
					_follow_target_state = TRACK_POSITION;

				} else if (target_velocity_valid()) {
					/*if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
										_current_vel += _step_vel;

									_last_update_time = current_time;
									}*/
					_current_vel = _est_target_vel;
					set_follow_target_item(&_mission_item,(double)_current_target_motion.alt, target_motion_with_offset, _yaw_angle);
					update_position_sp(true, true, 0);

				} else {
					_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
				}

				break;
			}

		case SET_WAIT_FOR_TARGET_POSITION: {

				// Climb to the minimum altitude
				// and wait until a position is received

				follow_target_s target = {};

				// for now set the target at the minimum height above the uav

				target.lat = _navigator->get_global_position()->lat;
				target.lon = _navigator->get_global_position()->lon;

				set_follow_target_item(&_mission_item,(double)2.0f, target, _yaw_angle);

				update_position_sp(false, false,0);

				_follow_target_state = WAIT_FOR_TARGET_POSITION;
			}

		/* FALLTHROUGH */

		case WAIT_FOR_TARGET_POSITION: {

				if (is_mission_item_reached() && target_velocity_valid()) {
					//_target_position_offset(0) = follower.x_offset;
					_follow_target_state = TRACK_POSITION;
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
	pos_sp_triplet->current.vx = _est_target_vel(0);//_current_target_motion.vx;
	pos_sp_triplet->current.vy = _est_target_vel(1);//_current_target_motion.vy;
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
static void *logwriter_thread(void *arg) {
	FILE *fd;
	static uint64_t last_log_time;
	char path[256];
	char file[256];
	time_t timeSec = time(0);				//1970.01.01
	struct tm *curTime = localtime(&timeSec);
	sprintf(path,PX4_ROOTFSDIR"/fs/microsd/log/%04d-%02d-%02d",curTime->tm_year + 2100, curTime->tm_mon + 1, curTime->tm_mday);
	mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
	sprintf(file, "%s/main%02d-%02d-%02d.ulg", path, curTime->tm_hour,
			curTime->tm_min, curTime->tm_sec);
	fd = fopen(file, "w");
	if (fd == 0) {
		return NULL;
	}
	fprintf(fd,
			"timefollow\ttar_lat\ttar_lon\ttar_alt\ttar_vx\ttar_vy\ttar_vz\tlat\tlon\talt\tvx\tvy\tvz\n");
	fclose(fd);
	while (!formation_exit) {
		if ((hrt_absolute_time() - last_log_time) >= 100000) {
			fd = fopen(file, "a+");
			last_log_time = hrt_absolute_time();
			fprintf(fd, "%d\t", (int) (last_log_time / 100000));
			fprintf(fd, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					sd_formation.tar_lat*1e7, sd_formation.tar_lon*1e7,
					sd_formation.tar_alt*1e7, sd_formation.tar_vx*1e7,
					sd_formation.tar_vy*1e7, sd_formation.tar_vz*1e7, sd_formation.lat*1e7,
					sd_formation.lon*1e7, sd_formation.alt*1e7, sd_formation.vx*1e7,
					sd_formation.vy*1e7, sd_formation.vz*1e7);
			fclose(fd);
		}
		usleep(50000);
	}
		return 0;
}
bool open_log(pthread_t &log_pthread) {
	pthread_attr_t logwriter_attr;
	pthread_attr_init(&logwriter_attr);
	pthread_attr_setstacksize(&logwriter_attr, PX4_STACK_ADJUSTED(3000));
#if !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)
	struct sched_param param;
	(void) pthread_attr_getschedparam(&logwriter_attr, &param);
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 5;
	(void) pthread_attr_setschedparam(&logwriter_attr, &param);
#endif
	pthread_create(&log_pthread, &logwriter_attr, logwriter_thread,
			nullptr);
	pthread_attr_destroy(&logwriter_attr);
	return true;
}
