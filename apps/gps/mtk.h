/*
 * mtk.h
 *
 *  Created on: Mar 6, 2012
 *      Author: thomasgubler
 */

#ifndef MTK_H_
#define MTK_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>

//Definition for mtk custom mode
#define MEDIATEK_REFRESH_RATE_4HZ "$PMTK220,250*29\r\n" //refresh rate - 4Hz - 250 milliseconds
#define MEDIATEK_REFRESH_RATE_5HZ "$PMTK220,200*2C\r\n"
#define MEDIATEK_REFRESH_RATE_10HZ "$PMTK220,100*2F\r\n" //refresh rate - 10Hz - 100 milliseconds
#define MEDIATEK_FACTORY_RESET "$PMTK104*37\r\n" //clear current settings
#define MEDIATEK_CUSTOM_BINARY_MODE "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MEDIATEK_FULL_COLD_RESTART "$PMTK104*37\r\n"
//#define NMEA_GGA_ENABLE "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*27\r\n" //Set GGA messages

//definitions for watchdog
#define MTK_WATCHDOG_CRITICAL_TIME_MICROSECONDS 2000000
#define MTK_WATCHDOG_WAIT_TIME_MICROSECONDS 800000




// ************
// the structure of the binary packet

typedef struct {
	uint8_t payload; ///< Number of payload bytes
	int32_t latitude;  ///< Latitude in degrees * 10^7
	int32_t longitude; ///< Longitude in degrees * 10^7
	uint32_t msl_altitude;  ///< MSL altitude in meters * 10^2
	uint32_t ground_speed; ///< FIXME SPEC UNCLEAR
	int32_t heading;
	uint8_t satellites;
	uint8_t fix_type;
	uint32_t date;
	uint32_t utc_time;
	uint16_t hdop;
	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_mtk_packet;

typedef type_gps_bin_mtk_packet gps_bin_mtk_packet_t;

enum MTK_DECODE_STATES {
	MTK_DECODE_UNINIT = 0,
	MTK_DECODE_GOT_CK_A = 1,
	MTK_DECODE_GOT_CK_B = 2
};

typedef struct {
	union {
		uint16_t ck;
		struct {
			uint8_t ck_a;
			uint8_t ck_b;
		};
	};
	uint8_t decode_state;
//    bool new_data;
//    uint8_t fix;
	bool print_errors;
	int16_t rx_count;

	uint64_t last_message_timestamp;
}  __attribute__((__packed__)) type_gps_bin_mtk_state;

typedef type_gps_bin_mtk_state gps_bin_mtk_state_t;

extern pthread_mutex_t *mtk_mutex;
extern gps_bin_mtk_state_t *mtk_state;

void mtk_decode_init(void);

void mtk_checksum(uint8_t b, uint8_t *ck_a, uint8_t *ck_b);

int mtk_parse(uint8_t b,  char *gps_rx_buffer);

int read_gps_mtk(int *fd, char *gps_rx_buffer, int buffer_size);

int configure_gps_mtk(int *fd);

void *mtk_loop(void *args);

void *mtk_watchdog_loop(void *args);

#endif /* MTK_H_ */
