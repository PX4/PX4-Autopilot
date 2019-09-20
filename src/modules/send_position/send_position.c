
#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <systemlib/err.h>
#include <string.h>
#include <poll.h>

#include <arch/board/board.h>

#include <math.h>
#include <float.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>

/**
* daemon management function.
 */
__EXPORT int send_position_main(int argc, char *argv[]);

static bool send_position_thread_should_exit = false;		/**< px4_uart exit flag */
static bool send_position_thread_running = false;		/**< px4_uart status flag */
int uart_send;

static int send_position_task;				/**< Handle of px4_uart task / thread */
static int send_position_init(void);
static int set_send_position_baudrate(const int fd, unsigned int baud);

/**
 * Mainloop of daemon.
 */
int send_position_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

#pragma  pack(1)
typedef struct
{
    char head[5];
    uint16_t buflen;
    uint8_t command;
    uint8_t command_re;
    float64 lat;
    float64 lon;
    float32 alt;
    float32 vy;
    float32 vx;
    float32 vz;
    uint8_t follow_pos;
    float32 follow_dis;
    uint16_t crc_check;
}EXYF_FOLLOW;
#pragma  pack()

static void usage(const char *reason)
{
        if (reason) {
                printf("%s\n", reason);
        }

       printf("usage: px4_uart {start|stop|status} [-p <additional params>]\n\n");
}

int set_send_position_baudrate(const int fd, unsigned int baud)
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


int send_position_init (void)
{
       char *uart_name = "/dev/ttyS6";
       //uart_name = "/dev/ttyS3";
       //char uart_name[] = "/dev/ttyS6";
        int serial_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
        // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
        if (serial_fd < 0) {
                printf("failed to open port: %s\n", uart_name);
                return false;
        }
        printf("Open the %s\n",uart_name);
        return serial_fd;
}

int send_position_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage("missing command");
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (send_position_thread_running) {
                       printf("px4_uart already running\n");
                        /* this is not an error */
                        return 0;
                }

                send_position_thread_should_exit = false;//定义一个守护进程
                send_position_task = px4_task_spawn_cmd("send_position",
                        SCHED_DEFAULT,
                        SCHED_PRIORITY_DEFAULT + 50,//调度优先级
                        3000,//堆栈分配大小
                        send_position_thread_main,
                        (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                send_position_thread_should_exit = true;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (send_position_thread_running) {
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

int send_position_thread_main(int argc, char *argv[])
{

        /*
                GPS1:/dev/ttyS0
                TEL1:/dev/ttyS1
                TEL2:/dev/ttyS2
                TEL4:/dev/ttyS3
         */

         uart_send = send_position_init();

         if (false == set_send_position_baudrate(uart_send, 115200)) {
                 printf("set_send_position_baudrate is failed\n");
                 return -1;
         }
         printf("uart init is successful\n");

         int vehicle_global_position_fd;
         vehicle_global_position_fd = orb_subscribe(ORB_ID(vehicle_global_position));

         bool updated = false;

        send_position_thread_running = true;

        while (!send_position_thread_should_exit)
        {
            printf("main thread start\n");
            orb_check(vehicle_global_position_fd, &updated);
            if(updated)
            {
                printf("global_position_data updated\n");
                struct vehicle_global_position_s global_position_data;
                orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_fd, &global_position_data);
                EXYF_FOLLOW follow_data;
                follow_data.head[0] = '$';
                follow_data.head[1] = 'E';
                follow_data.head[2] = 'X';
                follow_data.head[3] = 'Y';
                follow_data.head[4] = 'F';
                follow_data.buflen = 48;
                follow_data.command = 20;
                follow_data.command_re = 20;
                follow_data.lat = global_position_data.lat;
                follow_data.lon = global_position_data.lon;
                follow_data.alt = global_position_data.alt;
                follow_data.vy = global_position_data.vel_e;
                follow_data.vx = global_position_data.vel_n;
                follow_data.vz = global_position_data.vel_d;
                printf("lat is %.4f\n", global_position_data.lat);
                printf("lon is %.4f\n",  global_position_data.lon);
                printf("alt is %.4f\n", global_position_data.alt);
                follow_data.follow_pos = 1;
                follow_data.follow_dis = 5.0;
                follow_data.crc_check = 0;
                uint8_t send_message[48];
                memcpy(send_message, &follow_data, sizeof(EXYF_FOLLOW));
                send_message[47] = 0x3f;
                write(uart_send, send_message, sizeof(send_message));
                printf("message sended size is %d\n", sizeof(send_message));
                for (int i =0; i< sizeof(send_message); i++){
                    printf("send_message[%d] is %x\n", i, send_message[i]);
                }
            }
            usleep(10000);
        }

        send_position_thread_running = false;
        orb_unsubscribe(vehicle_global_position_fd);
        close(uart_send);
        printf("uart close\n");

        fflush(stdout);
        return 0;
}
