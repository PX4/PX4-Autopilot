
#include "rw_uart.h"
#include "rw_uart_define.h"

/**
* daemon management function.
 */
__EXPORT int rw_uart_main(int argc, char *argv[]);

static bool rw_thread_should_exit = false;		/**< px4_uart exit flag */
static bool rw_uart_thread_running = false;		/**< px4_uart status flag */
static uint8_t param_saved[62];

static int rw_uart_task;				/**< Handle of px4_uart task / thread */
static int rw_uart_init(void);
static int set_rw_uart_baudrate(const int fd, unsigned int baud);

void msg_orb_sub (MSG_orb_sub *msg_fd);
void msg_orb_data(MSG_orb_data *msg_data, MSG_orb_sub msg_fd);
void msg_orb_unsub (MSG_orb_sub *msg_fd);
void msg_param_hd_cache (MSG_param_hd *msg_hd);

/**
 * Mainloop of daemon.
 */
int rw_uart_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
        if (reason) {
                printf("%s\n", reason);
        }

       printf("usage: px4_uart {start|stop|status} [-p <additional params>]\n\n");
}

int set_rw_uart_baudrate(const int fd, unsigned int baud)
{
        int speed;

        switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
                printf("ERR: baudrate: %d\n", baud);
                return -EINVAL;
        }

        struct termios uart_config;

        int termios_state;

        tcgetattr(fd, &uart_config); // 获取终端参数

        /* clear ONLCR flag (which appends a CR for every LF) */
        uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

        /* 无偶校验，一个停止位 */
        uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

         /* 设置波特率 */
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
                //warnx("ERR: %d (cfsetispeed)\n", termios_state);
                 printf("ERR: %d (cfsetispeed)\n", termios_state);
                return false;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
                //warnx("ERR: %d (cfsetospeed)\n", termios_state);
                  printf("ERR: %d (cfsetospeed)\n", termios_state);
                return false;
        }
        // 设置与终端相关的参数，TCSANOW 立即改变参数
        if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
               //warnx("ERR: %d (tcsetattr)\n", termios_state);
                printf("ERR: %d (tcsetattr)\n", termios_state);
                return false;
        }

        return true;
}


int rw_uart_init (void)
{
       char *uart_name = "/dev/ttyS3";
       //char uart_name[] = "/dev/ttyS3";
        int serial_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
        // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
        if (serial_fd < 0) {
                printf("failed to open port: %s\n", uart_name);
                return false;
        }
        printf("Open the %s\n",uart_name);
        return serial_fd;
}


void msg_orb_sub (MSG_orb_sub *msg_fd)
{
    msg_fd->arm_fd = orb_subscribe(ORB_ID(actuator_armed));
    msg_fd->gps_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
    msg_fd->command_fd = orb_subscribe(ORB_ID(vehicle_command));
    msg_fd->mission_fd = orb_subscribe(ORB_ID(mission));
    msg_fd->manual_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
    msg_fd->status_fd = orb_subscribe(ORB_ID(vehicle_status));
    msg_fd->local_position_sp_fd = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    msg_fd->local_position_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    msg_fd->air_data_fd = orb_subscribe(ORB_ID(vehicle_air_data));
    msg_fd->attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    msg_fd->battery_fd = orb_subscribe(ORB_ID(battery_status));
    msg_fd->geofence_fd = orb_subscribe(ORB_ID(geofence_result));
    //msg_fd->cpu_fd = orb_subscribe(ORB_ID(cpuload));
}


void msg_orb_data(MSG_orb_data *msg_data, MSG_orb_sub msg_fd)
{
   orb_copy(ORB_ID(actuator_armed), msg_fd.arm_fd,&msg_data->arm_data);
   orb_copy(ORB_ID(vehicle_gps_position), msg_fd.gps_fd,&msg_data->gps_data);
   orb_copy(ORB_ID(vehicle_command), msg_fd.command_fd,&msg_data->command_data);
   orb_copy(ORB_ID(mission), msg_fd.mission_fd, &msg_data->mission_data);
   orb_copy(ORB_ID(manual_control_setpoint), msg_fd.manual_fd, &msg_data->manual_data);
   orb_copy(ORB_ID(vehicle_status), msg_fd.status_fd, &msg_data->status_data);
   orb_copy(ORB_ID(vehicle_local_position_setpoint), msg_fd.local_position_sp_fd, &msg_data->local_position_sp_data);
   orb_copy(ORB_ID(vehicle_local_position), msg_fd.local_position_fd, &msg_data->local_position_data);
   orb_copy(ORB_ID(vehicle_air_data), msg_fd.air_data_fd, &msg_data->air_data);
   orb_copy(ORB_ID(vehicle_attitude_setpoint), msg_fd.attitude_fd, &msg_data->attitude_data);
   orb_copy(ORB_ID(battery_status), msg_fd.battery_fd, &msg_data->battery_data);
   orb_copy(ORB_ID(geofence_result), msg_fd.geofence_fd, &msg_data->geofence_data);
   //orb_copy(ORB_ID(cpuload), msg_fd.cpu_fd, &msg_data->cpu_data);
}

void msg_orb_unsub (MSG_orb_sub *msg_fd)
{
    orb_unsubscribe(msg_fd->arm_fd);
    orb_unsubscribe(msg_fd->gps_fd);
    orb_unsubscribe(msg_fd->command_fd);
    orb_unsubscribe(msg_fd->mission_fd);
    orb_unsubscribe(msg_fd->manual_fd);
    orb_unsubscribe(msg_fd->status_fd);
    orb_unsubscribe(msg_fd->local_position_sp_fd);
    orb_unsubscribe(msg_fd->local_position_fd);
    orb_unsubscribe(msg_fd->air_data_fd);
    orb_unsubscribe(msg_fd->attitude_fd);
    orb_unsubscribe(msg_fd->battery_fd);
    orb_unsubscribe(msg_fd->geofence_fd);
    //orb_unsubscribe(msg_fd->cpu_fd);
}

void msg_param_hd_cache (MSG_param_hd *msg_hd)
{
    msg_hd->roll_p_hd = param_find("MC_ROLL_P");
    msg_hd->pitch_p_hd = param_find("MC_PITCH_P");
    msg_hd->yaw_p_hd = param_find("MC_YAW_P");
    msg_hd->z_p_hd = param_find("MPC_Z_P");
    msg_hd->up_vel_max_hd = param_find("MPC_Z_VEL_MAX_UP");
    msg_hd->xy_vel_max_hd = param_find("MPC_VEL_MANUAL");
    msg_hd->roll_rate_hd = param_find("MC_ROLLRATE_MAX");
    msg_hd->pitch_rate_hd = param_find("MC_PITCHRATE_MAX");
    msg_hd->yaw_rate_hd = param_find("MC_YAWRATE_MAX");
    msg_hd->acc_up_max_hd = param_find("MPC_ACC_UP_MAX");
    msg_hd->yaw_max_hd = param_find("MPC_MAN_Y_MAX");
    msg_hd->roll_max_hd = param_find("MPC_MAN_TILT_MAX");
    msg_hd->pitch_max_hd =param_find("MPC_TILTMAX_AIR");
    msg_hd->higt_max_hd = param_find("GF_MAX_VER_DIST");
    msg_hd->acc_hor_max_hd = param_find("MPC_ACC_HOR_MAX");
    msg_hd->dist_max_hd = param_find("GF_MAX_HOR_DIST");
    msg_hd->mav_type_hd = param_find("MAV_TYPE");
    msg_hd->battery_n_cells_hd = param_find("BAT_N_CELLS");
    msg_hd->battery_warn_hd = param_find("BAT_LOW_THR");
    msg_hd->battery_fail_hd = param_find("COM_LOW_BAT_ACT");
    msg_hd->rc_lost_act_hd = param_find("NAV_RCL_ACT");
    msg_hd->dn_vel_max_hd = param_find("MPC_Z_VEL_MAX_DN");
}

int rw_uart_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage("missing command");
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (rw_uart_thread_running) {
                       printf("px4_uart already running\n");
                        /* this is not an error */
                        return 0;
                }

                rw_thread_should_exit = false;//定义一个守护进程
                rw_uart_task = px4_task_spawn_cmd("rw_uart",
                        SCHED_DEFAULT,
                        SCHED_PRIORITY_DEFAULT,//调度优先级
                        4000,//堆栈分配大小
                        rw_uart_thread_main,
                        (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                rw_thread_should_exit = true;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (rw_uart_thread_running) {
                        printf("\trunning\n");

                }
                else {
                        printf("\tnot started\n");
                }

                return 0;
        }

        usage("unrecognized command");
        return 1;
}

int rw_uart_thread_main(int argc, char *argv[])
{
        //char *data = "\n";
        //char sample_test_uart[]="rw_uart TX-test:running!\n";

        /*
                GPS1:/dev/ttyS0
                TEL1:/dev/ttyS1
                TEL2:/dev/ttyS2
                TEL4:/dev/ttyS3
         */

         int uart_read = rw_uart_init();

         if (false == set_rw_uart_baudrate(uart_read, 115200)) {
                 printf("set_rw_uart_baudrate is failed\n");
                 return -1;
         }
         printf("uart init is successful\n");

        MSG_orb_sub msg_fd;
        memset(&msg_fd, 0, sizeof(msg_fd));
        msg_orb_sub(&msg_fd);

        MSG_orb_pub msg_pd;
        memset(&msg_pd, 0, sizeof(msg_pd));

        MSG_param_hd msg_hd;
        memset(&msg_hd, 0, sizeof(msg_hd));
        msg_param_hd_cache(&msg_hd);

        px4_pollfd_struct_t fds[] = {
               { .fd = uart_read,   .events = POLLIN },
           };

        int error_counter = 0;

        memset(param_saved, 0, sizeof(param_saved));
        msg_param_saved_get(msg_hd, uart_read, param_saved);

        rw_uart_thread_running = true;

        while (!rw_thread_should_exit)
        {
            uint8_t data;

            MSG_orb_data msg_data;
            memset(&msg_data, 0, sizeof(msg_data));

            uint8_t buffer[65];
            memset(buffer, 0, sizeof(buffer));

            msg_orb_data(&msg_data, msg_fd);
            msg_pack_send(msg_data, uart_read);

            int poll_ret = poll(fds,1,10);//阻塞等待10ms
            if (poll_ret == 0)
            {
                    /* this means none of our providers is giving us data */
                  //printf("No receive data for 10ms\n");
            } else if (poll_ret < 0)
            {
               /* this is seriously bad - should be an emergency */
               if (error_counter < 10 || error_counter % 50 == 0)
               {
                       /* use a counter to prevent flooding (and slowing us down) */
                       printf("ERROR return value from poll(): %d\n", poll_ret);
               }
                   error_counter++;
            }
            else
            {
               if (fds[0].revents & POLLIN)
               {
                       /*接收服务系统发过来的消息*/
                       read(uart_read,&data,1);//读取串口数据
                       if(data == '$')
                       {//找到帧头$
                               buffer[0] = '$';
                               for(int i = 1; i  < 5; i++)
                               {
                                       read(uart_read,&data,1);//读取后面的数据
                                       buffer[i] = data;
                           }
                       }
                       printf("data=%s\n", buffer);
                       find_r_type(buffer, &msg_data, &msg_pd, msg_hd, uart_read, param_saved);
               }
            }

          usleep(1000000);

        }

        msg_orb_unsub(&msg_fd);
        //printf("[rw_uart] exiting\n");
        rw_uart_thread_running = false;
        close(uart_read);
        printf("uart close\n");

        fflush(stdout);
        return 0;
}
