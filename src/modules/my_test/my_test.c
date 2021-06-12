//#include <px4_config.h>
#include <px4_platform_common/px4_config.h>
//#include <px4_tasks.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/my_test_public.h>
#include <termios.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <px4_defines.h>
#include <px4_time.h>

#include <float.h>
#include "my_test.h"
 //#include <px4_posix.h>
 #include <px4_platform_common/posix.h>
//#include <lib/ecl/geo/geo.h>
 #include <mathlib/mathlib.h>
// #include <matrix/math.hpp>
 #include <uORB/topics/sensor_combined.h>
// #include <drivers/drv_tone_alarm.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/distance_sensor.h>
#include <lib/systemlib/mavlink_log.h>

#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/mavlink_log.h>
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int px4_test_public;				/**< Handle of daemon task / thread */


__EXPORT int my_test_main(int argc, char *argv[]);

int my_test_thread_main(int argc, char *argv[]);
//bool detect_hold_still(float x , float y , float z, bool lenient_still_position);
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }
    warnx("usage: my_test {start|stop|status} [-p <additional params>]\n\n");
}

int my_test_main(int argc, char *argv[])
{
    if (argc < 2) {
            usage("missing command");
            return 1;
        }

    if (!strcmp(argv[1], "start")) {

            if (thread_running) {
                warnx("daemon already running\n");
                /* this is not an error */
                return 0;
            }

            thread_should_exit = false;//定义一个守护进程
                   px4_test_public = px4_task_spawn_cmd(
                                    "my_test_app",
                                     SCHED_DEFAULT,
                                     SCHED_PRIORITY_DEFAULT,//调度优先级
                                     2000,//堆栈分配大小
                                     my_test_thread_main,
                                     (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                    return 0;
                }

             if (!strcmp(argv[1], "stop")) {
                 thread_should_exit = true;
                 return 0;
                 }

             if (!strcmp(argv[1], "status")) {
                    if (thread_running) {
                      warnx("\trunning\n");

                  } else {
                      warnx("\tnot started\n");
                  }
                    return 0;
                 }

             usage("unrecognized command");
             return 1;

   }

int my_test_thread_main(int argc, char *argv[])
{
 // struct   input_rc_s input;
    // for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
	// 	//mavlink_log_critical(&_mavlink_log_pub, " subscribe input_rc %d",i);
	// 	_inputs_rc_sub[i] = orb_subscribe_multi(ORB_ID(input_rc), i);
    //      memset(&input, 0, sizeof(input));
	// }
    // orb_advert_t _mavlink_log_pub;
    // memset(&_mavlink_log_pub, 0, sizeof(_mavlink_log_pub));
    //int rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    //int rc_channel_fd = orb_subscribe(ORB_ID(rc_channels));
    //int man_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
   // struct manual_control_setpoint_s manual;
   // memset(&manual, 0, sizeof(manual));
  //  struct rc_channels_s channel;
  //  memset(&channel, 0, sizeof(channel));
   // orb_set_interval(rc_sub_fd, 20);
  //  struct input_rc_s rc_in;
   // memset(&rc_in, 0, sizeof(rc_in));

   int ds_sub = orb_subscribe(ORB_ID(distance_sensor));
   struct distance_sensor_s ds_sen;
    memset(&ds_sen, 0, sizeof(ds_sen));


PX4_INFO(" begin!");



/********************************参数begin**********************************/

/*********************************参数end*************************************/

thread_running = true;
while (!thread_should_exit) {
/******************************************main loop begin*******************************************************/

PX4_INFO("Loop begin!");
px4_pollfd_struct_t fds[] = {
            { .fd = ds_sub, .events = POLLIN },

            };
             int error_counter = 0;
             int poll_ret = px4_poll(fds, 1, 1000);
             if (poll_ret == 0) {
                 PX4_ERR("Got no data within a second");
             } else if (poll_ret < 0) {
                 if (error_counter < 10 || error_counter % 50 == 0) {
                     PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                 }
                 error_counter++;
             } else {
                    if (fds[0].revents & POLLIN)
                    {
        //                          bool rc_updated = 0;
        // for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		// // check if subscriber has updated
		// orb_check(_inputs_rc_sub[i], &rc_updated);

		// // copy message into struct
		// if (rc_updated) {
		// 	orb_copy(ORB_ID(input_rc), _inputs_rc_sub[i], &input);
        //     int aa,bb,cc;
        //  aa = input.values[0];
        //  bb = input.values[1];
        //  cc = input.values[2];

		// 	// get priority
		// // 	int32_t priority;
		// // 	orb_priority(_inputs_rc_sub[i], &priority);
        // // dd = priority;
        //   PX4_INFO("\nRC:\t%d\t%d\t%d\n",aa,bb,cc);

		// }
        // }
                                //  bool updated1 = 0;
                                //  orb_check(rc_channel_fd,&updated);
                                //  orb_check(man_fd,&updated1);
                                //  if (updated)
                                //  {
                            //    int  fff,ggg,hhh,iii;
                            //    int aa,bb,cc,dd;
                                //_mavlink_log_pub{nullptr};
                                // orb_copy(ORB_ID(input_rc), rc_sub_fd, &rc_in);
                                // orb_copy(ORB_ID(rc_channels), rc_channel_fd, &channel);
                                // orb_copy(ORB_ID(manual_control_setpoint),man_fd,&manual);
                                // aa = manual.posctl_switch;
                                // bb = manual.return_switch;
                                // cc = manual.loiter_switch;
                                // dd = manual.mode_switch;
                              //  px4_usleep(200000);
                                // aaa = rc_in.values[0];
                                // bbb = rc_in.values[1];
                                // ccc = rc_in.values[2];
                                // ddd = rc_in.values[3];
                                // eee = rc_in.values[4];
                                // fff =  rc_in.values[0];
                                // ggg= rc_in.values[1];
                                // hhh= rc_in.values[2];
                                // iii= rc_in.values[3];
                                // jjj= rc_in.values[9];
                                // kkk= rc_in.values[10];
                                // lll= rc_in.values[11];
                                // mmm= rc_in.values[12];
                                //  int aaa, bbb, ccc, ddd ;
                                // aaa = channel.channels[0]*1000;
                                // bbb =channel.channels[1]*1000;
                                // ccc = channel.channels[2]*1000;
                                // ddd = channel.channels[3]*1000;
                                //eee = channel.channels[4]*1000;
                                //mavlink_log_emergency(&_mavlink_log_pub, "%f %f %f %f %f", (double)aaa, (double)bbb, (double)ccc, (double)ddd, (double)eee);
                                // fff =  channel.channels[5];
                                // ggg= channel.channels[6];
                                // hhh= channel.channels[7];
                                // iii= channel.channels[8];
                                // jjj= channel.channels[9];
                                // kkk= channel.channels[10];
                                // lll= channel.channels[11];
                                // mmm= channel.channels[12];
                           //mavlink_log_critical(&_mavlink_log_pub, "%d %d %d %d  OR %d %d %d %d RC %d %d %d %d", aaa, bbb, ccc, ddd,fff,ggg,hhh,iii,aa,bb,cc,dd);
                                //mavlink_log_emergency(&_mavlink_log_pub, "%d %d %d %d %d", aaa, bbb, ccc, ddd, eee);
                                //PX4_INFO("\nupdate:\t%d\n", test.attu);
                                //PX4_INFO("\nRC:\t%d\t%d\t%d\t%d\n",rc_in.values[0],rc_in.values[1],rc_in.values[2],rc_in.values[3]);
                                //PX4_INFO("\nRC:\t%d\t%d\t%d\t%d\n",rc_in.values[0],rc_in.values[1],rc_in.values[2],rc_in.values[3]);
                               // }

                    orb_copy(ORB_ID(distance_sensor), ds_sub, &ds_sen);
                    PX4_INFO("\nupdate:\t%f\n", (double)ds_sen.current_distance);






                   }

             }




 /*********************************************main loop end*******************************************************/
}

   // orb_unsubscribe(rc_sub_fd);
  //  orb_unsubscribe(rc_channel_fd);
    warnx("exiting.\n");
    thread_running = false;
    return 0;
}



