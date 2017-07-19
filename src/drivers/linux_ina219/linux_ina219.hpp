#ifndef LINUX_INA219_H
#define LINUX_INA219_H
#include "INA219.hpp"
#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
namespace linux_ina219 {
struct work_s _work;
uint8_t __current_status = 0; // 0 stoped,1 running,
struct battery_status_s __battery_status_data;
orb_advert_t __battery_status_pub;
bool __should_exit = false;
bool __is_running = false;
bool __ina219_bus = 1;
INA219 *ina219 = nullptr;
void start();
void stop();
void running(int argc, char**argv);
void status();
void usage();
}
extern "C" __EXPORT int linux_ina219_main(int argc, char **argv);
#endif
