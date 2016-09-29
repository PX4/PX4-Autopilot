/**
 * @file example_lz_main.cpp
 * For UAVGP2016 with new mavlink messages--- LiuZhong
 * 2016/9/29
 */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <uORB/uORB.h>
#include <nuttx/config.h>
#include <stdlib.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h> //获得时间戳

#include <uORB/topics/vehicle_local_position.h>
// 8个自动定义的Mavlink消息
#include <uORB/topics/fixed_target_position_g2p.h>
#include <uORB/topics/fixed_target_return_p2g.h>
#include <uORB/topics/yaw_sp_calculated_p2g.h>
#include <uORB/topics/task_status_change_g2p.h>
#include <uORB/topics/task_status_monitor_p2g.h>
#include <uORB/topics/vision_num_scan_p2g.h>
#include <uORB/topics/vision_one_num_get_p2g.h>
#include <uORB/topics/obstacle_position_p2g.h>

static bool thread_should_exit = false;		/**< example_lz exit flag */
static bool thread_running = false;			/**< example_lz status flag */
static int  example_lz_task;				/**< Handle of example_lz task / thread */

// 线程管理程序
extern "C" __EXPORT int example_lz_main(int argc, char *argv[]);
// 用户线程, 执行用户代码
int example_lz_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
static void
usage(const char *reason)
{

}

// 线程管理程序
int example_lz_main(int argc, char *argv[])
{
	if (argc < 2) {
			warnx("usage: example_lz {start|stop|status}");
			return 1;
		}

	if (!strcmp(argv[1], "start")) {   //shell启动命令
		if (thread_running) {		   // 如果线程已经启动了
			warnx("example_lz already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;		// 将线程状态位设置为false
		example_lz_task = px4_task_spawn_cmd("example_lz",				    // 线程名
										SCHED_DEFAULT,					// 调度模式
										SCHED_PRIORITY_DEFAULT,			// 优先级
										1200,							// 堆栈大小
										example_lz_thread_main,			// 线程入口
										nullptr);
		if (example_lz_task < 0) {
				warn("task start failed");
				return -errno;
			}
		exit(0);						// 正常退出
	}
	if (!strcmp(argv[1], "stop")) {		// shell停止命令
		thread_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {	// shell查询命令, 用于查询线程的状态.
		if (thread_running) {
			warnx("\t running\n");
		} else {
			warnx("\t not started\n");
		}
		exit(0);
	}
	usage("unrecognized command");
	exit(1);
}
// 线程主体
int example_lz_thread_main(int argc, char *argv[])
{
	thread_running=true;

	// 订阅位置信息
	int _fixed_target_position_sub;
	_fixed_target_position_sub = orb_subscribe(ORB_ID(fixed_target_position_g2p));
	struct fixed_target_position_g2p_s position_sub;
	bool updated_pos;
	// 发布位置信息
	orb_advert_t _fixed_target_return_pub;
	struct fixed_target_return_p2g_s position_pub;
	memset(&position_pub, 0, sizeof(position_pub));
	// 发布航向信息
	orb_advert_t _yaw_sp_calculated_pub;
	struct yaw_sp_calculated_p2g_s yaw_sp_pub;
	memset(&yaw_sp_pub, 0, sizeof(yaw_sp_pub));

	// 订阅任务状态修改信息
	int _task_status_change_sub;
	_task_status_change_sub  = orb_subscribe(ORB_ID(task_status_change_g2p));
	struct task_status_change_g2p_s task_status_sub;
	bool updated_task;
	// 发布实时任务状态信息
	orb_advert_t _task_status_monitor_pub;
	struct task_status_monitor_p2g_s task_status_pub;
	memset(&task_status_pub, 0, sizeof(task_status_pub));

	// 发布数字扫描信息
	orb_advert_t _vision_num_scan_pub;
	struct vision_num_scan_p2g_s num_scan_pub;
	memset(&num_scan_pub, 0, sizeof(num_scan_pub));
	// 发布数据采集信息
	orb_advert_t _vision_one_num_get_pub;
	struct vision_one_num_get_p2g_s one_num_pub;
	memset(&one_num_pub, 0, sizeof(one_num_pub));

	// 订阅飞机NED位置信息 vehicle_local_position
	int _vehicle_local_position_sub;
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s local_position_sub;
	bool updated_local_pos;
	// 发布障碍物信息
	orb_advert_t _obstacle_position_pub;
	struct obstacle_position_p2g_s obstacle_position_pub;
	memset(&obstacle_position_pub, 0, sizeof(obstacle_position_pub));

	// 如果线程没有被停止
	while(!thread_should_exit)
	{
		// 位置信息——H、O、SL、SR
		orb_check(_fixed_target_position_sub, &updated_pos);
		if (updated_pos)
		{
			// 读取主题
			orb_copy(ORB_ID(fixed_target_position_g2p), _fixed_target_position_sub, &position_sub);
			// 位置变量赋值
			position_pub.timestamp=hrt_absolute_time();
			position_pub.home_lon=position_sub.home_lon;
			position_pub.home_lat=position_sub.home_lat;
			position_pub.home_alt=position_sub.home_alt;
			position_pub.observe_lon=position_sub.observe_lon;
			position_pub.observe_lat=position_sub.observe_lat;
			position_pub.observe_alt=position_sub.observe_alt;
			position_pub.spray_left_lon=position_sub.spray_left_lon;
			position_pub.spray_left_lat=position_sub.spray_left_lat;
			position_pub.spray_left_alt=position_sub.spray_left_alt;
			position_pub.spray_right_lon=position_sub.spray_right_lon;
			position_pub.spray_right_lat=position_sub.spray_right_lat;
			position_pub.spray_right_alt=position_sub.spray_right_alt;
			// 发布主题，返回上述位置信息
			_fixed_target_return_pub = orb_advertise(ORB_ID(fixed_target_return_p2g), &position_pub);
			orb_publish(ORB_ID(fixed_target_return_p2g), _fixed_target_return_pub, &position_pub);

			// 航向期望赋值
			yaw_sp_pub.timestamp=hrt_absolute_time();
			yaw_sp_pub.yaw_sp=3.1415926/2; //假设为pi/2
			// 发布主题，返回上述航向期望
			_yaw_sp_calculated_pub = orb_advertise(ORB_ID(yaw_sp_calculated_p2g), &yaw_sp_pub);
			orb_publish(ORB_ID(yaw_sp_calculated_p2g), _yaw_sp_calculated_pub, &yaw_sp_pub);
		}

		// 任务状态修改信息
		orb_check(_task_status_change_sub, &updated_task);
		if (updated_task)
		{
			// 读取主题
			orb_copy(ORB_ID(task_status_change_g2p), _task_status_change_sub, &task_status_sub);
			// 变量赋值
			task_status_pub.timestamp=hrt_absolute_time();
			task_status_pub.num_odd_even=task_status_sub.num_odd_even;
			task_status_pub.task_status=task_status_sub.task_status; //实际任务状态
			task_status_pub.loop_value=task_status_sub.loop_value; //数字编号
			task_status_pub.target_lon=position_sub.home_lon;
			task_status_pub.target_lat=position_sub.home_lat+0.000001;
			task_status_pub.target_alt=position_sub.home_alt+1;
			// 发布主题，返回任务状态监视信息
			_task_status_monitor_pub = orb_advertise(ORB_ID(task_status_monitor_p2g), &task_status_pub);
			orb_publish(ORB_ID(task_status_monitor_p2g), _task_status_monitor_pub, &task_status_pub);

			if(task_status_sub.task_status==3) //悬停
			{
				// 仅仅进行悬停
			}
			if(task_status_sub.task_status==4) //预扫
			{
				num_scan_pub.timestamp=hrt_absolute_time();
				// 扫描信息--1\5\9
				usleep(1000000);
				num_scan_pub.timestamp=hrt_absolute_time();
				num_scan_pub.board_num=1;
				num_scan_pub.board_x=11;
				num_scan_pub.board_y=12;
				num_scan_pub.board_z=13;
				num_scan_pub.board_valid=1;
				_vision_num_scan_pub = orb_advertise(ORB_ID(vision_num_scan_p2g), &num_scan_pub);
				orb_publish(ORB_ID(vision_num_scan_p2g), _vision_num_scan_pub, &num_scan_pub);
				usleep(1000000);
				num_scan_pub.timestamp=hrt_absolute_time();
				num_scan_pub.board_num=5;
				num_scan_pub.board_x=51;
				num_scan_pub.board_y=52;
				num_scan_pub.board_z=53;
				num_scan_pub.board_valid=1;
				_vision_num_scan_pub = orb_advertise(ORB_ID(vision_num_scan_p2g), &num_scan_pub);
				orb_publish(ORB_ID(vision_num_scan_p2g), _vision_num_scan_pub, &num_scan_pub);
				usleep(1000000);
				num_scan_pub.timestamp=hrt_absolute_time();
				num_scan_pub.board_num=9;
				num_scan_pub.board_x=91;
				num_scan_pub.board_y=92;
				num_scan_pub.board_z=93;
				num_scan_pub.board_valid=1;
				_vision_num_scan_pub = orb_advertise(ORB_ID(vision_num_scan_p2g), &num_scan_pub);
				orb_publish(ORB_ID(vision_num_scan_p2g), _vision_num_scan_pub, &num_scan_pub);
			}
			if(task_status_sub.task_status==5) //飞往“点O”
			{
				usleep(1000000);
				// 进行数字识别
				task_status_pub.task_status=7;
				_task_status_monitor_pub = orb_advertise(ORB_ID(task_status_monitor_p2g), &task_status_pub);
				orb_publish(ORB_ID(task_status_monitor_p2g), _task_status_monitor_pub, &task_status_pub);
				usleep(1000000);
				// 返回扫描结果
				one_num_pub.timestamp=hrt_absolute_time();
				one_num_pub.loop_value=task_status_sub.loop_value; //数字编号
				one_num_pub.num=task_status_sub.loop_value+10; //识别数字
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get_p2g), &one_num_pub);
				orb_publish(ORB_ID(vision_one_num_get_p2g), _vision_one_num_get_pub, &one_num_pub);
				usleep(1000000);
				// 进行旋喷
				task_status_pub.task_status=12;
				_task_status_monitor_pub = orb_advertise(ORB_ID(task_status_monitor_p2g), &task_status_pub);
				orb_publish(ORB_ID(task_status_monitor_p2g), _task_status_monitor_pub, &task_status_pub);
				usleep(1000000);
				// 完成
				task_status_pub.task_status=13;
				_task_status_monitor_pub = orb_advertise(ORB_ID(task_status_monitor_p2g), &task_status_pub);
				orb_publish(ORB_ID(task_status_monitor_p2g), _task_status_monitor_pub, &task_status_pub);
			}
			if(task_status_sub.task_status==13) //喷绘“完成”
			{
				// 作为喷绘完成的标志
			}
			if(task_status_sub.task_status==14) //“返航”
			{
				// 任务完成后，返航
			}
			if(task_status_sub.task_status==15) //“降落”
			{
				// 降落在当前经纬位置下
			}
		}

		// 飞机NED位置信息
		orb_check(_vehicle_local_position_sub, &updated_local_pos);
		if(updated_local_pos)
		{
			// 读取主题
			orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &local_position_sub);
			// 障碍物位置赋值
			obstacle_position_pub.timestamp=hrt_absolute_time();
			obstacle_position_pub.obstacle_x=local_position_sub.x+3;
			obstacle_position_pub.obstacle_y=local_position_sub.y+2;
			obstacle_position_pub.obstacle_z=local_position_sub.z-1;
			obstacle_position_pub.obstacle_valid=1; //障碍物可见
			// 发布主题，返回上述障碍物虚拟位置
			_obstacle_position_pub=orb_advertise(ORB_ID(obstacle_position_p2g), &obstacle_position_pub);
			orb_publish(ORB_ID(obstacle_position_p2g), _obstacle_position_pub, &obstacle_position_pub);
		}

		usleep(100000); // 100msv
	}

	thread_running=false;
	return 0;
}
