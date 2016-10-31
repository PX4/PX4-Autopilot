/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <stdint.h>


extern const char *syslink_magic;

#define SYSLINK_GROUP	0xF0

#define SYSLINK_RADIO	0x00
#define SYSLINK_RADIO_RAW	0x00
#define SYSLINK_RADIO_CHANNEL	0x01
#define SYSLINK_RADIO_DATARATE	0x02
#define SYSLINK_RADIO_CONTWAVE	0x03
#define SYSLINK_RADIO_RSSI	0x04
#define SYSLINK_RADIO_ADDRESS	0x05

#define SYSLINK_PM	0x10
#define SYSLINK_PM_SOURCE	0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF	0x11
#define SYSLINK_PM_BATTERY_VOLTAGE	0x12
#define SYSLINK_PM_BATTERY_STATE	0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE	0x14

#define SYSLINK_OW	0x20
#define SYSLINK_OW_SCAN	0x20
#define SYSLINK_OW_GETINFO	0x21
#define SYSLINK_OW_READ	0x22
#define SYSLINK_OW_WRITE	0x23

// Limited by the CRTP packet which is limited by the ESB protocol used by the NRF
#define SYSLINK_MAX_DATA_LEN	32

#define SYSLINK_RADIO_RATE_250K 0
#define SYSLINK_RADIO_RATE_1M 1
#define SYSLINK_RADIO_RATE_2M 2


typedef struct {
	uint8_t type;
	uint8_t length;
	uint8_t data[SYSLINK_MAX_DATA_LEN];
	uint8_t cksum[2];
} __attribute__((packed)) syslink_message_t;


#define OW_SIZE 112
#define OW_READ_BLOCK 29
#define OW_WRITE_BLOCK 26 // TODO: Use even, but can be up to 27

typedef struct {
	uint8_t nmems;
} __attribute__((packed)) syslink_ow_scan_t;

typedef struct {
	uint8_t family; // Should by 0x0D for most chips
	uint8_t sn[6];
	uint8_t crc;
} __attribute__((packed)) syslink_ow_id_t;

typedef struct {
	uint8_t idx;
	uint8_t id[8];
} __attribute__((packed)) syslink_ow_getinfo_t;

typedef struct {
	uint8_t idx;
	uint16_t addr;
	uint8_t data[OW_READ_BLOCK];
} __attribute__((packed)) syslink_ow_read_t;

typedef struct {
	uint8_t idx;
	uint16_t addr;
	uint16_t length;
	uint8_t data[OW_WRITE_BLOCK];
} __attribute__((packed)) syslink_ow_write_t;


typedef enum {
	SYSLINK_STATE_START = 0,
	SYSLINK_STATE_TYPE,
	SYSLINK_STATE_LENGTH,
	SYSLINK_STATE_DATA,
	SYSLINK_STATE_CKSUM
} SYSLINK_STATE;

typedef struct  {
	SYSLINK_STATE state;
	int index;

} syslink_parse_state;


#ifdef __cplusplus
extern "C" {
#endif

extern void syslink_parse_init(syslink_parse_state *state);

// Returns true if a full message was parsed
extern int syslink_parse_char(syslink_parse_state *state, char c, syslink_message_t *msg);

extern void syslink_compute_cksum(syslink_message_t *msg);

#ifdef __cplusplus
}
#endif
