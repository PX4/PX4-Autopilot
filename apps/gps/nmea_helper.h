/*
 * nmea_helper.h
 *
 *  Created on: Mar 15, 2012
 *      Author: thomasgubler
 */

#ifndef NMEA_H_
#define NMEA_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "nmealib/nmea/nmea.h"


//definitions for watchdog
#define NMEA_WATCHDOG_CRITICAL_TIME_MICROSECONDS 2000000
#define NMEA_WATCHDOG_WAIT_TIME_MICROSECONDS 800000

typedef struct {
	uint64_t last_message_timestamp;
}  __attribute__((__packed__)) type_gps_bin_nmea_state;

typedef type_gps_bin_nmea_state gps_bin_nmea_state_t;

extern gps_bin_nmea_state_t *nmea_state;
extern pthread_mutex_t *nmea_mutex;



int read_gps_nmea(int *fd, char *gps_rx_buffer, int buffer_size, nmeaINFO *info, nmeaPARSER *parser);

void *nmea_loop(void *arg);

void *nmea_watchdog_loop(void *arg);

/**
 * \brief Convert NDEG (NMEA degree) to fractional degree
 */
float ndeg2degree(float val);

void nmea_init(void);


#endif /* NMEA_H_ */
