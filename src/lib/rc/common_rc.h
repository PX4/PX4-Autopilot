
#pragma once

#include <stdint.h>

#include "crsf.h"
#include "dsm.h"
#include "ghst.hpp"
#include "sbus.h"
#include "st24.h"
#include "sumd.h"

#pragma pack(push, 1)
typedef  struct rc_decode_buf_ {
	union {
		crsf_frame_t crsf_frame;
		ghst_frame_t ghst_frame;
		dsm_decode_t dsm;
		sbus_frame_t sbus_frame;
		ReceiverFcPacket _strxpacket;
		ReceiverFcPacketHoTT _hottrxpacket;
	};
} rc_decode_buf_t;
#pragma pack(pop)

extern rc_decode_buf_t rc_decode_buf;

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t crc8_dvb_s2_buf(uint8_t *buf, int len);

#define RC_INPUT_RSSI_MAX		100		//should be same as  input_rc_s::RC_RSSI_MAX
#define RC_INPUT_RSSI_NO_SIGNAL		0	//should be same as  input_rc_s::RC_RSSI_NO_SIGNAL
#define RC_INPUT_RSSI_UNDEFINED	255		//should be same as  input_rc_s::RC_RSSI_UNDEFINED
