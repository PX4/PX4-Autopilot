/* 
 * 串口读取函数
 * rw_uart.c 
 */
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
//#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <uORB/uORB.h>
#include <uORB/topics/pm3901_with_tof.h>
#include <systemlib/mavlink_log.h>

// ORB_DEFINE(rw_uart_topic, struct rw_uart_topic_s);

__EXPORT int rw_uart_main(int argc, char *argv[]);

static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static orb_advert_t mavlink_log_pub = (void *) 0;

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* 以新的配置填充结构体 */
    /* 设置某个选项，那么就使用"|="运算，
     * 如果关闭某个选项就使用"&="和"~"运算
     * */
    tcgetattr(fd, &uart_config); // 获取终端参数

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

    /* 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

     /* 设置波特率 */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    /*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
//    printf("Open the %s\n",serial_fd);
    return serial_fd;
}

int rw_uart_main(int argc, char *argv[])
{
    // define and initialize data structure*/
    struct pm3901_with_tof_s pm3901_tof_data;
    memset(&pm3901_tof_data, 0, sizeof(pm3901_tof_data));

    // initialize the orb topic
    orb_advert_t pub_fd = orb_advertise(ORB_ID(pm3901_with_tof), &pm3901_tof_data);
    int    pm3901_and_tof_sub = orb_subscribe(ORB_ID(pm3901_with_tof));
    struct pm3901_with_tof_s test;

    char data = '0';
    char buffer[6] = "";
    /*
        GPS1:/dev/ttyS0
        TEL1:/dev/ttyS1
        TEL2:/dev/ttyS2
        TEL4:/dev/ttyS3
     */
    int uart_read = uart_init("/dev/ttyS3");
    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,38400)){
        printf("[JXF]set_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_critical(&mavlink_log_pub, "pm3901_with_tof start successful");
    //printf("[JXF]uart init is successful\n");

    while(true){
        read(uart_read,&data,1);
        if(data == 0xaa){
            for(int i = 0; i < 6; ++ i){
                read(uart_read, &data, 1);
                buffer[i] = data;
                data = '0';
            }

            float xvel    = buffer[1];
            float yvel    = - buffer[2];
            float quality = buffer[3];
            float height  = (buffer[4] + (buffer[5]<<8))/1000.f;

            pm3901_tof_data.xvel       = xvel;
            pm3901_tof_data.yvel       = yvel;
            pm3901_tof_data.quality    = quality;
            pm3901_tof_data.tof_height = height;

            orb_publish(ORB_ID(pm3901_with_tof), pub_fd, &pm3901_tof_data);

            static int count = 0; count ++;
            if(count % 100 == 0)
            {
                printf("xvel: %d yvel: %d quality: %d height: %.4f\n", buffer[1], -buffer[2], buffer[3], (double)(height));

                orb_copy(ORB_ID(pm3901_with_tof), pm3901_and_tof_sub, &test);

                printf("height:%.4f\n", (double)(test.tof_height));
            }
        }
    }

    return 0;
}
