/*
 * target_track.cpp
 *
 *  Created on: Mar 5, 2017
 *      Author: kzf
 */
//#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/target_basic.h>
#include <uORB/topics/follow_target.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>
#include <poll.h>
#include "target_track.h"
#include <navigator/navigator.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/distance_sensor.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <navigator/navigation.h>
#include <navigator/navigator.h>
#include <uORB/topics/home_position.h>
#include <navigator/navigator_mode.h>
#include <systemlib/mavlink_log.h>

extern "C" __EXPORT int target_track_main(int argc, char *argv[]);
extern orb_advert_t mavlink_log_pub;

Navigator *_navigator;

using math::Vector;//向量
using math::Matrix;//矩阵
using math::Quaternion;//四元数

static bool thread_running=false;
static bool thread_should_exit=false;
static int target_track_task=-1;
int distance_delta=15;//最短距离触发阀值

Vector<3> target_vector_basic;
Vector<3> target_vector_ned;
math::Matrix<3,3> rotation;
//Quaternion q;

target_basic_s target_basic;
follow_target_s target_track;
vehicle_global_position_s global_position;

int target_track_thread_main(int argc, char *argv[]){
	memset(&target_basic,0,sizeof(target_basic));
	memset(&target_track,0,sizeof(target_track));
	memset(&global_position,0,sizeof(global_position));
	int global_position_s=-1;
	int target_basic_s=-1;
	int sensor_distance_s=-1;
	struct map_projection_reference_s target_ref;
	struct distance_sensor_s target_distance;
	orb_advert_t target_track_pub;


	target_vector_basic.zero();
	target_vector_ned.zero();

	//订阅相关主题
	sensor_distance_s = orb_subscribe(ORB_ID(distance_sensor));
	global_position_s=orb_subscribe(ORB_ID(vehicle_global_position));
	target_basic_s = orb_subscribe(ORB_ID(target_basic));
	target_track_pub=orb_advertise(ORB_ID(follow_target),&target_track);

	thread_running=true;

	//mavlink_log_info(_navigator->get_mavlink_log_pub(),"qgc test successful!");

	px4_pollfd_struct_t fds[1];
	fds[0].fd = target_basic_s;
	fds[0].events = POLLIN;

	hrt_abstime t;
	t=hrt_absolute_time();
	target_basic.degree=0;
	while(!thread_should_exit)
	{

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 250);//阻塞100ms，10HZ

		//PX4_WARN("pret=%d",pret);
		//mavlink_log_info(&mavlink_log_pub,"target_track test successful!");//调试用

	if(pret==0)
	{
//			//continue;
		}
		//warn("cetc3:target_track test successful!");


		//if(pret<0){
		//	warn("target_track: poll error %d, %d", pret, errno);
			//usleep(100000);
			//continue;
		//}

		//if(pret>0){//轮询成功
			//orb_copy(ORB_ID(target_basic),target_basic_s,&target_basic);
			double rad;
			if(hrt_absolute_time()-t>=1000){
				t=hrt_absolute_time();
				target_basic.degree++;
			}

			rad=double(target_basic.degree)*M_DEG_TO_RAD;
			target_basic.distance=20;
			target_basic.get_lidar_data=0;

			bool updated;
			orb_check(global_position_s,&updated);
			if(updated){
				orb_copy(ORB_ID(vehicle_global_position),global_position_s,&global_position);
						}


			orb_check(sensor_distance_s,&updated);
			if(updated){
				orb_copy(ORB_ID(distance_sensor),sensor_distance_s,&target_distance);
						}

			if(target_basic.get_lidar_data&&target_basic.distance<=distance_delta){
				//将极坐标值转化为机体坐标系下的坐标值
				target_vector_basic(0)=target_basic.distance * float(cos(target_basic.degree));
				target_vector_basic(1)=target_basic.distance * float(sin(target_basic.degree));
				target_vector_basic(2)=0.0f;

			}

			else{
				target_vector_basic(0)=distance_delta * cos(rad);
				target_vector_basic(1)=distance_delta * sin(rad);
				target_vector_basic(2)=0.0f;
			}




			float cos_yaw,sin_yaw;
			cos_yaw=float(cos(double(global_position.yaw)));
			sin_yaw=float(sin(double(global_position.yaw)));

			//warn("yaw=%.2f!",double(global_position.yaw));

			//mavlink_log_info(_navigator->get_mavlink_log_pub(),"yaw= %f",double(global_position.yaw));

			rotation.data[0][0]=cos_yaw;
			rotation.data[0][1]=sin_yaw;
			rotation.data[0][2]=0.0f;
			rotation.data[1][0]= -sin_yaw;
			rotation.data[1][1]=cos_yaw;
			rotation.data[1][2]=0.0f;
			rotation.data[2][0]= 0.0f;
			rotation.data[2][1]=0.0f;
			rotation.data[2][2]=1.0f;

			//将计算出来的坐标值转化为NED下的值
			target_vector_ned = rotation.transposed() * target_vector_basic;

			//将坐标映射到经纬度上
			map_projection_init(&target_ref,global_position.lat,global_position.lon);
			map_projection_reproject(&target_ref,target_vector_ned(0),target_vector_ned(1),&target_track.lat,&target_track.lon);

		//}

			//PX4_WARN("pret=%d",pret);
			//PX4_WARN("target_track.lon=%.2f",target_track.lon);
		target_track.timestamp=hrt_absolute_time();
		//target_track.alt=_navigator->get_home_position()->alt;
		//warn("cetc3:target_track test successful!");

		if (target_track_pub == nullptr) {
			target_track_pub = orb_advertise(ORB_ID(follow_target), &target_track);

		} else {
			orb_publish(ORB_ID(follow_target), target_track_pub, &target_track);
		}
				}

	thread_running=false;
	return 0;

}
static void usage(const char *reason)
{
	if (reason && *reason) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage: target_track {start|stop|status}\n");
	return;
}


int target_track_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("target_track already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		target_track_task = px4_task_spawn_cmd("target_track_task",
							SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 1036,
							target_track_thread_main,
							(argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		if (target_track_task < 0) {
			warn("cetc3:target_track task start failed");
			return -errno;
		}
		else{
			warn("cetc3:target_track auto_task start successfully!!");
		}
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("target_track stoped successfully");
			thread_should_exit = true;

		} else {
			warnx("target_track not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("target_track is running");

		} else {
			warnx("target_track not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}
