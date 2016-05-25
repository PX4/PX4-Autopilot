
#ifndef __MAVSTATION_FIRMWARE_GPIO_H__
#define __MAVSTATION_FIRMWARE_GPIO_H__

#include <stdbool.h>

void gpio_interface_init(void);
void gpio_interface_tick(void);
void gpio_interface_setled(int, bool);
bool gpio_interface_getbtn(int);

void gpio_interface_setusart2mux(bool conn_to_rpi);

#endif // __MAVSTATION_FIRMWARE_GPIO_H__

