/**
 * @file uavgp_commucation_main.cpp
 * For UAVGP2016 commucation
 * GCS->Pix->Mavros
 * or
 * Mavros->Pix->GCS
 *
 * 2016/9/30
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

// 16个自动定义的Mavlink消息
#include <uORB/topics/fixed_target_position_g2p.h>
#include <uORB/topics/fixed_target_position_p2m.h>
#include <uORB/topics/fixed_target_return_m2p.h>
#include <uORB/topics/fixed_target_return_p2g.h>
#include <uORB/topics/yaw_sp_calculated_m2p.h>
#include <uORB/topics/yaw_sp_calculated_p2g.h>
#include <uORB/topics/task_status_change_g2p.h>
#include <uORB/topics/task_status_change_p2m.h>
#include <uORB/topics/task_status_monitor_m2p.h>
#include <uORB/topics/task_status_monitor_p2g.h>
#include <uORB/topics/vision_num_scan_m2p.h>
#include <uORB/topics/vision_num_scan_p2g.h>
#include <uORB/topics/vision_one_num_get_m2p.h>
#include <uORB/topics/vision_one_num_get_p2g.h>
#include <uORB/topics/obstacle_position_m2p.h>
#include <uORB/topics/obstacle_position_p2g.h>

static bool thread_should_exit = false;		/**< uavgp_commucation exit flag */
static bool thread_running = false;			/**< uavgp_commucation status flag */
static int  uavgp_commucation_task;				/**< Handle of uavgp_commucation task / thread */

// 线程管理程序
extern "C" __EXPORT int uavgp_commucation_main(int argc, char *argv[]);
// 用户线程, 执行用户代码
int uavgp_commucation_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
static void
usage(const char *reason)
{

}

// 线程管理程序
int uavgp_commucation_main(int argc, char *argv[])
{
	if (argc < 2) {
			warnx("usage: uavgp_commucation {start|stop|status}");
			return 1;
		}

	if (!strcmp(argv[1], "start")) {   //shell启动命令
		if (thread_running) {		   // 如果线程已经启动了
			warnx("uavgp_commucation already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;		// 将线程状态位设置为false
		uavgp_commucation_task = px4_task_spawn_cmd("uavgp_commucation",  // 线程名
										SCHED_DEFAULT,					  // 调度模式
										SCHED_PRIORITY_DEFAULT,			  // 优先级
										1200,							  // 堆栈大小
										uavgp_commucation_thread_main,	  // 线程入口
										nullptr);
		if (uavgp_commucation_task < 0) {
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
int uavgp_commucation_thread_main(int argc, char *argv[])
{
	thread_running=true;

	// fixed_target_position
	// 订阅
	int _fixed_target_position_sub;
	_fixed_target_position_sub = orb_subscribe(ORB_ID(fixed_target_position_g2p));
	struct fixed_target_position_g2p_s fixed_target_position_sub;
	bool updated_fixed_target_position;
	// 发布
	orb_advert_t _fixed_target_position_pub;
	_fixed_target_position_pub=nullptr;
	struct fixed_target_position_p2m_s fixed_target_position_pub;
	memset(&fixed_target_position_pub, 0, sizeof(fixed_target_position_pub));

	// fixed_target_return
	// 订阅
	int _fixed_target_return_sub;
	_fixed_target_return_sub = orb_subscribe(ORB_ID(fixed_target_return_m2p));
	struct fixed_target_return_m2p_s fixed_target_return_sub;
	bool updated_fixed_target_return;
	// 发布
	orb_advert_t _fixed_target_return_pub;
	_fixed_target_return_pub=nullptr;
	struct fixed_target_return_p2g_s fixed_target_return_pub;
	memset(&fixed_target_return_pub, 0, sizeof(fixed_target_return_pub));

	// yaw_sp_calculated
	// 订阅
	int _yaw_sp_calculated_sub;
	_yaw_sp_calculated_sub = orb_subscribe(ORB_ID(yaw_sp_calculated_m2p));
	struct yaw_sp_calculated_m2p_s yaw_sp_calculated_sub;
	bool updated_yaw_sp_calculated;
	// 发布
	orb_advert_t _yaw_sp_calculated_pub;
	_yaw_sp_calculated_pub=nullptr;
	struct yaw_sp_calculated_p2g_s yaw_sp_calculated_pub;
	memset(&yaw_sp_calculated_pub, 0, sizeof(yaw_sp_calculated_pub));

	// task_status_change
	int _task_status_change_sub;
	_task_status_change_sub = orb_subscribe(ORB_ID(task_status_change_g2p));
	struct task_status_change_g2p_s task_status_change_sub;
	bool updated_task_status_change;
	// 发布
	orb_advert_t _task_status_change_pub;
	_task_status_change_pub=nullptr;
	struct task_status_change_p2m_s task_status_change_pub;
	memset(&task_status_change_pub, 0, sizeof(task_status_change_pub));

	// task_status_monitor
	// 订阅
	int _task_status_monitor_sub;
	_task_status_monitor_sub = orb_subscribe(ORB_ID(task_status_monitor_m2p));
	struct task_status_monitor_m2p_s task_status_monitor_sub;
	bool updated_task_status_monitor;
	// 发布
	orb_advert_t _task_status_monitor_pub;
	_task_status_monitor_pub=nullptr;
	struct task_status_monitor_p2g_s task_status_monitor_pub;
	memset(&task_status_monitor_pub, 0, sizeof(task_status_monitor_pub));

	// vision_num_scan
	// 订阅
	int _vision_num_scan_sub;
	_vision_num_scan_sub = orb_subscribe(ORB_ID(vision_num_scan_m2p));
	struct vision_num_scan_m2p_s vision_num_scan_sub;
	bool updated_vision_num_scan;
	// 发布
	orb_advert_t _vision_num_scan_pub;
	_vision_num_scan_pub=nullptr;
	struct vision_num_scan_p2g_s vision_num_scan_pub;
	memset(&vision_num_scan_pub, 0, sizeof(vision_num_scan_pub));

	// vision_one_num_get
	// 订阅
	int _vision_one_num_get_sub;
	_vision_one_num_get_sub = orb_subscribe(ORB_ID(vision_one_num_get_m2p));
	struct vision_one_num_get_m2p_s vision_one_num_get_sub;
	bool updated_vision_one_num_get;
	// 发布
	orb_advert_t _vision_one_num_get_pub;
	_vision_one_num_get_pub=nullptr;
	struct vision_one_num_get_p2g_s vision_one_num_get_pub;
	memset(&vision_one_num_get_pub, 0, sizeof(vision_one_num_get_pub));

	// obstacle_position
	// 订阅
	int _obstacle_position_sub;
	_obstacle_position_sub = orb_subscribe(ORB_ID(obstacle_position_m2p));
	struct obstacle_position_m2p_s obstacle_position_sub;
	bool updated_obstacle_position;
	// 发布
	orb_advert_t _obstacle_position_pub;
	_obstacle_position_pub=nullptr;
	struct obstacle_position_p2g_s obstacle_position_pub;
	memset(&obstacle_position_pub, 0, sizeof(obstacle_position_pub));

	// 如果线程没有被停止
	while(!thread_should_exit)
	{
		// fixed_target_position
		orb_check(_fixed_target_position_sub, &updated_fixed_target_position);
		if (updated_fixed_target_position)
		{
			orb_copy(ORB_ID(fixed_target_position_g2p), _fixed_target_position_sub, &fixed_target_position_sub);
			fixed_target_position_pub.home_lon=fixed_target_position_sub.home_lon;
			fixed_target_position_pub.home_lat=fixed_target_position_sub.home_lat;
			fixed_target_position_pub.home_alt=fixed_target_position_sub.home_alt;
			fixed_target_position_pub.observe_lon=fixed_target_position_sub.observe_lon;
			fixed_target_position_pub.observe_lat=fixed_target_position_sub.observe_lat;
			fixed_target_position_pub.observe_alt=fixed_target_position_sub.observe_alt;
			fixed_target_position_pub.spray_left_lon=fixed_target_position_sub.spray_left_lon;
			fixed_target_position_pub.spray_left_lat=fixed_target_position_sub.spray_left_lat;
			fixed_target_position_pub.spray_left_alt=fixed_target_position_sub.spray_left_alt;
			fixed_target_position_pub.spray_right_lon=fixed_target_position_sub.spray_right_lon;
			fixed_target_position_pub.spray_right_lat=fixed_target_position_sub.spray_right_lat;
			fixed_target_position_pub.spray_right_alt=fixed_target_position_sub.spray_right_alt;
			if (_fixed_target_position_pub == nullptr) {
				_fixed_target_position_pub = orb_advertise(ORB_ID(fixed_target_position_p2m), &fixed_target_position_pub);
			} else {
				orb_publish(ORB_ID(fixed_target_position_p2m), _fixed_target_position_pub, &fixed_target_position_pub);
			}
		}
		// fixed_target_return
		orb_check(_fixed_target_return_sub, &updated_fixed_target_return);
		if (updated_fixed_target_return)
		{
			orb_copy(ORB_ID(fixed_target_return_m2p), _fixed_target_return_sub, &fixed_target_return_sub);
			fixed_target_return_pub.timestamp=hrt_absolute_time();
			fixed_target_return_pub.home_lon=fixed_target_return_sub.home_lon;
			fixed_target_return_pub.home_lat=fixed_target_return_sub.home_lat;
			fixed_target_return_pub.home_alt=fixed_target_return_sub.home_alt;
			fixed_target_return_pub.observe_lon=fixed_target_return_sub.observe_lon;
			fixed_target_return_pub.observe_lat=fixed_target_return_sub.observe_lat;
			fixed_target_return_pub.observe_alt=fixed_target_return_sub.observe_alt;
			fixed_target_return_pub.spray_left_lon=fixed_target_return_sub.spray_left_lon;
			fixed_target_return_pub.spray_left_lat=fixed_target_return_sub.spray_left_lat;
			fixed_target_return_pub.spray_left_alt=fixed_target_return_sub.spray_left_alt;
			fixed_target_return_pub.spray_right_lon=fixed_target_return_sub.spray_right_lon;
			fixed_target_return_pub.spray_right_lat=fixed_target_return_sub.spray_right_lat;
			fixed_target_return_pub.spray_right_alt=fixed_target_return_sub.spray_right_alt;
			if (_fixed_target_return_pub == nullptr) {
				_fixed_target_return_pub = orb_advertise(ORB_ID(fixed_target_return_p2g), &fixed_target_return_pub);
			} else {
				orb_publish(ORB_ID(fixed_target_return_p2g), _fixed_target_return_pub, &fixed_target_return_pub);
			}
		}
		// yaw_sp_calculated
		orb_check(_yaw_sp_calculated_sub, &updated_yaw_sp_calculated);
		if (updated_yaw_sp_calculated)
		{
			orb_copy(ORB_ID(yaw_sp_calculated_m2p), _yaw_sp_calculated_sub, &yaw_sp_calculated_sub);
			yaw_sp_calculated_pub.timestamp=hrt_absolute_time();
			yaw_sp_calculated_pub.yaw_sp=yaw_sp_calculated_sub.yaw_sp;
			if (_yaw_sp_calculated_pub == nullptr) {
				_yaw_sp_calculated_pub = orb_advertise(ORB_ID(yaw_sp_calculated_p2g), &yaw_sp_calculated_pub);
			} else {
				orb_publish(ORB_ID(yaw_sp_calculated_p2g), _yaw_sp_calculated_pub, &yaw_sp_calculated_pub);
			}
		}
		// task_status_change
		orb_check(_task_status_change_sub, &updated_task_status_change);
		if (updated_task_status_change)
		{
			orb_copy(ORB_ID(task_status_change_g2p), _task_status_change_sub, &task_status_change_sub);
			task_status_change_pub.spray_duration=task_status_change_sub.spray_duration;
			task_status_change_pub.task_status=task_status_change_sub.task_status;
			task_status_change_pub.loop_value=task_status_change_sub.loop_value;
			if (_task_status_change_pub == nullptr) {
				_task_status_change_pub = orb_advertise(ORB_ID(task_status_change_p2m), &task_status_change_pub);
			} else {
				orb_publish(ORB_ID(task_status_change_p2m), _task_status_change_pub, &task_status_change_pub);
			}
		}
		// task_status_monitor
		orb_check(_task_status_monitor_sub, &updated_task_status_monitor);
		if (updated_task_status_monitor)
		{
			orb_copy(ORB_ID(task_status_monitor_m2p), _task_status_monitor_sub, &task_status_monitor_sub);
			task_status_monitor_pub.timestamp=hrt_absolute_time();
			task_status_monitor_pub.spray_duration=task_status_monitor_sub.spray_duration;
			task_status_monitor_pub.task_status=task_status_monitor_sub.task_status;
			task_status_monitor_pub.loop_value=task_status_monitor_sub.loop_value;
			task_status_monitor_pub.target_lon=task_status_monitor_sub.target_lon;
			task_status_monitor_pub.target_lat=task_status_monitor_sub.target_lat;
			task_status_monitor_pub.target_alt=task_status_monitor_sub.target_alt;
			if (_task_status_monitor_pub == nullptr) {
				_task_status_monitor_pub = orb_advertise(ORB_ID(task_status_monitor_p2g), &task_status_monitor_pub);
			} else {
				orb_publish(ORB_ID(task_status_monitor_p2g), _task_status_monitor_pub, &task_status_monitor_pub);
			}
		}
		// vision_num_scan
		orb_check(_vision_num_scan_sub, &updated_vision_num_scan);
		if (updated_vision_num_scan)
		{
			orb_copy(ORB_ID(vision_num_scan_m2p), _vision_num_scan_sub, &vision_num_scan_sub);
			vision_num_scan_pub.timestamp=hrt_absolute_time();
			vision_num_scan_pub.board_num=vision_num_scan_sub.board_num;
			vision_num_scan_pub.board_x=vision_num_scan_sub.board_x;
			vision_num_scan_pub.board_y=vision_num_scan_sub.board_y;
			vision_num_scan_pub.board_z=vision_num_scan_sub.board_z;
			vision_num_scan_pub.board_valid=vision_num_scan_sub.board_valid;
			if (_vision_num_scan_pub == nullptr) {
				_vision_num_scan_pub = orb_advertise(ORB_ID(vision_num_scan_p2g), &vision_num_scan_pub);
			} else {
				orb_publish(ORB_ID(vision_num_scan_p2g), _vision_num_scan_pub, &vision_num_scan_pub);
			}
		}
		// vision_one_num_get
		orb_check(_vision_one_num_get_sub, &updated_vision_one_num_get);
		if (updated_vision_one_num_get)
		{
			orb_copy(ORB_ID(vision_one_num_get_m2p), _vision_one_num_get_sub, &vision_one_num_get_sub);
			vision_one_num_get_pub.timestamp=hrt_absolute_time();
			vision_one_num_get_pub.loop_value=vision_one_num_get_sub.loop_value;
			vision_one_num_get_pub.num=vision_one_num_get_sub.num;
			if (_vision_one_num_get_pub == nullptr) {
				_vision_one_num_get_pub = orb_advertise(ORB_ID(vision_one_num_get_p2g), &vision_one_num_get_pub);
			} else {
				orb_publish(ORB_ID(vision_one_num_get_p2g), _vision_one_num_get_pub, &vision_one_num_get_pub);
			}
		}
		// obstacle_position
		orb_check(_obstacle_position_sub, &updated_obstacle_position);
		if (updated_obstacle_position)
		{
			orb_copy(ORB_ID(obstacle_position_m2p), _obstacle_position_sub, &obstacle_position_sub);
			obstacle_position_pub.timestamp=hrt_absolute_time();
			obstacle_position_pub.obstacle_x=obstacle_position_sub.obstacle_x;
			obstacle_position_pub.obstacle_y=obstacle_position_sub.obstacle_y;
			obstacle_position_pub.obstacle_z=obstacle_position_sub.obstacle_z;
			obstacle_position_pub.obstacle_valid=obstacle_position_sub.obstacle_valid;
			if (_obstacle_position_pub == nullptr) {
				_obstacle_position_pub = orb_advertise(ORB_ID(obstacle_position_p2g), &obstacle_position_pub);
			} else {
				orb_publish(ORB_ID(obstacle_position_p2g), _obstacle_position_pub, &obstacle_position_pub);
			}
		}
	}

	thread_running=false;
	return 0;
}
