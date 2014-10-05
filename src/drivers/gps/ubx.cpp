/****************************************************************************
 *
 *   Copyright (c) 2012, 2013, 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file ubx.cpp
 *
 * U-Blox protocol implementation. Following u-blox 6/7/8 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @author Hannes Delago
 *   (rework, add ubx7+ compatibility)
 *
 * @see http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
 * @see http://www.u-blox.com/images/downloads/Product_Docs/u-bloxM8-V15_ReceiverDescriptionProtocolSpec_Public_%28UBX-13003221%29.pdf
 */

#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>

#include "ubx.h"

#define UBX_CONFIG_TIMEOUT	200		// ms, timeout for waiting ACK
#define UBX_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full update received
#define UBX_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define DISABLE_MSG_INTERVAL	1000000		// us, try to disable message with this interval

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

#define FNV1_32_INIT	((uint32_t)0x811c9dc5)	// init value for FNV1 hash algorithm
#define FNV1_32_PRIME	((uint32_t)0x01000193)	// magic prime for FNV1 hash algorithm


/**** Trace macros, disable for production builds */
#define UBX_TRACE_PARSER(s, ...)	{/*printf(s, ## __VA_ARGS__);*/}	/* decoding progress in parse_char() */
#define UBX_TRACE_RXMSG(s, ...)		{/*printf(s, ## __VA_ARGS__);*/}	/* Rx msgs in payload_rx_done() */
#define UBX_TRACE_SVINFO(s, ...)	{/*printf(s, ## __VA_ARGS__);*/}	/* NAV-SVINFO processing (debug use only, will cause rx buffer overflows) */

/**** Warning macros, disable to save memory */
#define UBX_WARN(s, ...)		{warnx(s, ## __VA_ARGS__);}


UBX::UBX(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info) :
	_fd(fd),
	_gps_position(gps_position),
	_satellite_info(satellite_info),
	_configured(false),
	_ack_state(UBX_ACK_IDLE),
	_got_posllh(false),
	_got_velned(false),
	_disable_cmd_last(0),
	_ack_waiting_msg(0),
	_ubx_version(0),
	_use_nav_pvt(false)
{
	decode_init();
}

UBX::~UBX()
{
}

int
UBX::configure(unsigned &baudrate)
{
	_configured = false;
	/* try different baudrates */
	const unsigned baudrates[] = {9600, 38400, 19200, 57600, 115200};

	unsigned baud_i;

	for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++) {
		baudrate = baudrates[baud_i];
		set_baudrate(_fd, baudrate);

		/* flush input and wait for at least 20 ms silence */
		decode_init();
		receive(20);
		decode_init();

		/* Send a CFG-PRT message to set the UBX protocol for in and out
		 * and leave the baudrate as it is, we just want an ACK-ACK for this */
		memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
		_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
		_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
		_buf.payload_tx_cfg_prt.baudRate	= baudrate;
		_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
		_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;

		send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));

		if (wait_for_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false) < 0) {
			/* try next baudrate */
			continue;
		}

		/* Send a CFG-PRT message again, this time change the baudrate */
		memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
		_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
		_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
		_buf.payload_tx_cfg_prt.baudRate	= UBX_TX_CFG_PRT_BAUDRATE;
		_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
		_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;

		send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));

		/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
		wait_for_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);

		if (UBX_TX_CFG_PRT_BAUDRATE != baudrate) {
			set_baudrate(_fd, UBX_TX_CFG_PRT_BAUDRATE);
			baudrate = UBX_TX_CFG_PRT_BAUDRATE;
		}

		/* at this point we have correct baudrate on both ends */
		break;
	}

	if (baud_i >= sizeof(baudrates) / sizeof(baudrates[0])) {
		return 1;	// connection and/or baudrate detection failed
	}

	/* Send a CFG-RATE message to define update rate */
	memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
	_buf.payload_tx_cfg_rate.measRate	= UBX_TX_CFG_RATE_MEASINTERVAL;
	_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
	_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

	send_message(UBX_MSG_CFG_RATE, _buf.raw, sizeof(_buf.payload_tx_cfg_rate));

	if (wait_for_ack(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, true) < 0) {
		return 1;
	}

	/* send a NAV5 message to set the options for the internal filter */
	memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
	_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	_buf.payload_tx_cfg_nav5.dynModel	= UBX_TX_CFG_NAV5_DYNMODEL;
	_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;

	send_message(UBX_MSG_CFG_NAV5, _buf.raw, sizeof(_buf.payload_tx_cfg_nav5));

	if (wait_for_ack(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, true) < 0) {
		return 1;
	}

#ifdef UBX_CONFIGURE_SBAS
	/* send a SBAS message to set the SBAS options */
	memset(&_buf.payload_tx_cfg_sbas, 0, sizeof(_buf.payload_tx_cfg_sbas));
	_buf.payload_tx_cfg_sbas.mode		= UBX_TX_CFG_SBAS_MODE;

	send_message(UBX_MSG_CFG_SBAS, _buf.raw, sizeof(_buf.payload_tx_cfg_sbas));

	if (wait_for_ack(UBX_MSG_CFG_SBAS, UBX_CONFIG_TIMEOUT, true) < 0) {
		return 1;
	}
#endif

	/* configure message rates */
	/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */

	/* try to set rate for NAV-PVT */
	/* (implemented for ubx7+ modules only, use NAV-SOL, NAV-POSLLH, NAV-VELNED and NAV-TIMEUTC for ubx6) */
	configure_message_rate(UBX_MSG_NAV_PVT, 1);
	if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
		_use_nav_pvt = false;
	} else {
		_use_nav_pvt = true;
	}
	UBX_WARN("%susing NAV-PVT", _use_nav_pvt ? "" : "not ");

	if (!_use_nav_pvt) {
		configure_message_rate(UBX_MSG_NAV_TIMEUTC, 5);
		if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
			return 1;
		}

		configure_message_rate(UBX_MSG_NAV_POSLLH, 1);
		if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
			return 1;
		}

		configure_message_rate(UBX_MSG_NAV_SOL, 1);
		if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
			return 1;
		}

		configure_message_rate(UBX_MSG_NAV_VELNED, 1);
		if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
			return 1;
		}
	}

	configure_message_rate(UBX_MSG_NAV_SVINFO, (_satellite_info != nullptr) ? 5 : 0);
	if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
		return 1;
	}

	configure_message_rate(UBX_MSG_MON_HW, 1);
	if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
		return 1;
	}

	/* request module version information by sending an empty MON-VER message */
	send_message(UBX_MSG_MON_VER, nullptr, 0);

	_configured = true;
	return 0;
}

int	// -1 = NAK, error or timeout, 0 = ACK
UBX::wait_for_ack(const uint16_t msg, const unsigned timeout, const bool report)
{
	int ret = -1;

	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check

	hrt_abstime time_started = hrt_absolute_time();

	while ((_ack_state == UBX_ACK_WAITING) && (hrt_absolute_time() < time_started + timeout * 1000)) {
		receive(timeout);
	}

	if (_ack_state == UBX_ACK_GOT_ACK) {
		ret = 0;	// ACK received ok
	} else if (report) {
		if (_ack_state == UBX_ACK_GOT_NAK) {
			UBX_WARN("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));
		} else {
			UBX_WARN("ubx msg 0x%04x ACK timeout", SWAP16((unsigned)msg));
		}
	}

	_ack_state = UBX_ACK_IDLE;
	return ret;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
UBX::receive(const unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[128];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	ssize_t count = 0;

	int handled = 0;

	while (true) {
		bool ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;

		/* poll for new data, wait for only UBX_PACKET_TIMEOUT (2ms) if something already received */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), ready_to_return ? UBX_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			UBX_WARN("ubx poll() err");
			return -1;

		} else if (ret == 0) {
			/* return success after short delay after receiving a packet or timeout after long delay */
			if (ready_to_return) {
				_got_posllh = false;
				_got_velned = false;
				return handled;

			} else {
				return -1;
			}

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device. But don't read immediately
				 * by 1-2 bytes, wait for some more data to save expensive read() calls.
				 * If more bytes are available, we'll go back to poll() again.
				 */
				usleep(UBX_WAIT_BEFORE_READ * 1000);
				count = read(_fd, buf, sizeof(buf));

				/* pass received bytes to the packet decoder */
				for (int i = 0; i < count; i++) {
					handled |= parse_char(buf[i]);
				}
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			return -1;
		}
	}
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
UBX::parse_char(const uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {

	/* Expecting Sync1 */
	case UBX_DECODE_SYNC1:
		if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
			UBX_TRACE_PARSER("\nA");
			_decode_state = UBX_DECODE_SYNC2;
		}
		break;

	/* Expecting Sync2 */
	case UBX_DECODE_SYNC2:
		if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
			UBX_TRACE_PARSER("B");
			_decode_state = UBX_DECODE_CLASS;

		} else {		// Sync1 not followed by Sync2: reset parser
			decode_init();
		}
		break;

	/* Expecting Class */
	case UBX_DECODE_CLASS:
		UBX_TRACE_PARSER("C");
		add_byte_to_checksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
		_rx_msg = b;
		_decode_state = UBX_DECODE_ID;
		break;

	/* Expecting ID */
	case UBX_DECODE_ID:
		UBX_TRACE_PARSER("D");
		add_byte_to_checksum(b);
		_rx_msg |= b << 8;
		_decode_state = UBX_DECODE_LENGTH1;
		break;

	/* Expecting first length byte */
	case UBX_DECODE_LENGTH1:
		UBX_TRACE_PARSER("E");
		add_byte_to_checksum(b);
		_rx_payload_length = b;
		_decode_state = UBX_DECODE_LENGTH2;
		break;

	/* Expecting second length byte */
	case UBX_DECODE_LENGTH2:
		UBX_TRACE_PARSER("F");
		add_byte_to_checksum(b);
		_rx_payload_length |= b << 8;	// calculate payload size
		if (payload_rx_init() != 0) {	// start payload reception
			// payload will not be handled, discard message
			decode_init();
		} else {
			_decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
		}
		break;

	/* Expecting payload */
	case UBX_DECODE_PAYLOAD:
		UBX_TRACE_PARSER(".");
		add_byte_to_checksum(b);
		switch (_rx_msg) {
		case UBX_MSG_NAV_SVINFO:
			ret = payload_rx_add_nav_svinfo(b);	// add a NAV-SVINFO payload byte
			break;
		case UBX_MSG_MON_VER:
			ret = payload_rx_add_mon_ver(b);	// add a MON-VER payload byte
			break;
		default:
			ret = payload_rx_add(b);		// add a payload byte
			break;
		}
		if (ret < 0) {
			// payload not handled, discard message
			decode_init();
		} else if (ret > 0) {
			// payload complete, expecting checksum
			_decode_state = UBX_DECODE_CHKSUM1;
		} else {
			// expecting more payload, stay in state UBX_DECODE_PAYLOAD
		}
		ret = 0;
		break;

	/* Expecting first checksum byte */
	case UBX_DECODE_CHKSUM1:
		if (_rx_ck_a != b) {
			UBX_WARN("ubx checksum err");
			decode_init();
		} else {
			_decode_state = UBX_DECODE_CHKSUM2;
		}
		break;

	/* Expecting second checksum byte */
	case UBX_DECODE_CHKSUM2:
		if (_rx_ck_b != b) {
			UBX_WARN("ubx checksum err");
		} else {
			ret = payload_rx_done();	// finish payload processing
		}
		decode_init();
		break;

	default:
		break;
	}

	return ret;
}

/**
 * Start payload rx
 */
int	// -1 = abort, 0 = continue
UBX::payload_rx_init()
{
	int ret = 0;

	_rx_state = UBX_RXMSG_HANDLE;	// handle by default

	switch (_rx_msg) {
	case UBX_MSG_NAV_PVT:
		if (   (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)		/* u-blox 7 msg format */
		    && (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8))	/* u-blox 8+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (!_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if not using NAV-PVT
		break;

	case UBX_MSG_NAV_POSLLH:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_SOL:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_SVINFO:
		if (_satellite_info == nullptr)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if sat info not requested
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else
			memset(_satellite_info, 0, sizeof(*_satellite_info));	// initialize sat info
		break;

	case UBX_MSG_NAV_VELNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_MON_VER:
		break;		// unconditionally handle this message

	case UBX_MSG_MON_HW:
		if (   (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))	/* u-blox 6 msg format */
		    && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t)))	/* u-blox 7+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		break;

	case UBX_MSG_ACK_ACK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if _configured
		break;

	case UBX_MSG_ACK_NAK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if _configured
		break;

	default:
		_rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
		break;
	}

	switch (_rx_state) {
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
		UBX_WARN("ubx msg 0x%04x len %u unexpected", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);

		{
			hrt_abstime t = hrt_absolute_time();

			if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL) {
				/* don't attempt for every message to disable, some might not be disabled */
				_disable_cmd_last = t;
				UBX_WARN("ubx disabling msg 0x%04x", SWAP16((unsigned)_rx_msg));
				configure_message_rate(_rx_msg, 0);
			}
		}

		ret = -1;	// return error, abort handling this message
		break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		UBX_WARN("ubx msg 0x%04x invalid len %u", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		UBX_WARN("ubx internal err1");
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}

/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
UBX::payload_rx_add(const uint8_t b)
{
	int ret = 0;

	_buf.raw[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add NAV-SVINFO payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
UBX::payload_rx_add_nav_svinfo(const uint8_t b)
{
	int ret = 0;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
		// Fill Part 1 buffer
		_buf.raw[_rx_payload_index] = b;
	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			_satellite_info->count = MIN(_buf.payload_rx_nav_svinfo_part1.numCh, SAT_INFO_MAX_SATELLITES);
			UBX_TRACE_SVINFO("SVINFO len %u  numCh %u\n", (unsigned)_rx_payload_length, (unsigned)_buf.payload_rx_nav_svinfo_part1.numCh);
		}
		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t) + _satellite_info->count * sizeof(ubx_payload_rx_nav_svinfo_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(ubx_payload_rx_nav_svinfo_part2_t);
			_buf.raw[buf_index] = b;
			if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) / sizeof(ubx_payload_rx_nav_svinfo_part2_t);
				_satellite_info->used[sat_index]	= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.flags & 0x01);
				_satellite_info->snr[sat_index]		= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.cno);
				_satellite_info->elevation[sat_index]	= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.elev);
				_satellite_info->azimuth[sat_index]	= (uint8_t)((float)_buf.payload_rx_nav_svinfo_part2.azim * 255.0f / 360.0f);
				_satellite_info->svid[sat_index]	= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.svid);
				UBX_TRACE_SVINFO("SVINFO #%02u  used %u  snr %3u  elevation %3u  azimuth %3u  svid %3u\n",
						(unsigned)sat_index + 1,
						(unsigned)_satellite_info->used[sat_index],
						(unsigned)_satellite_info->snr[sat_index],
						(unsigned)_satellite_info->elevation[sat_index],
						(unsigned)_satellite_info->azimuth[sat_index],
						(unsigned)_satellite_info->svid[sat_index]
				);
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add MON-VER payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
UBX::payload_rx_add_mon_ver(const uint8_t b)
{
	int ret = 0;

	if (_rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
		// Fill Part 1 buffer
		_buf.raw[_rx_payload_index] = b;
	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
			// Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.hwVersion, _ubx_version);
			UBX_WARN("VER hash 0x%08x", _ubx_version);
			UBX_WARN("VER hw  \"%10s\"", _buf.payload_rx_mon_ver_part1.hwVersion);
			UBX_WARN("VER sw  \"%30s\"", _buf.payload_rx_mon_ver_part1.swVersion);
		}
		// fill Part 2 buffer
		unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
		_buf.raw[buf_index] = b;
		if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
			// Part 2 complete: decode Part 2 buffer
			UBX_WARN("VER ext \" %30s\"", _buf.payload_rx_mon_ver_part2.extension);
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Finish payload rx
 */
int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
UBX::payload_rx_done(void)
{
	int ret = 0;

	// return if no message handled
	if (_rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

	// handle message
	switch (_rx_msg) {

	case UBX_MSG_NAV_PVT:
		UBX_TRACE_RXMSG("Rx NAV-PVT\n");

		_gps_position->fix_type		= _buf.payload_rx_nav_pvt.fixType;
		_gps_position->satellites_used	= _buf.payload_rx_nav_pvt.numSV;

		_gps_position->lat		= _buf.payload_rx_nav_pvt.lat;
		_gps_position->lon		= _buf.payload_rx_nav_pvt.lon;
		_gps_position->alt		= _buf.payload_rx_nav_pvt.hMSL;

		_gps_position->eph		= (float)_buf.payload_rx_nav_pvt.hAcc * 1e-3f;
		_gps_position->epv		= (float)_buf.payload_rx_nav_pvt.vAcc * 1e-3f;
		_gps_position->s_variance_m_s	= (float)_buf.payload_rx_nav_pvt.sAcc * 1e-3f;

		_gps_position->vel_m_s		= (float)_buf.payload_rx_nav_pvt.gSpeed * 1e-3f;

		_gps_position->vel_n_m_s	= (float)_buf.payload_rx_nav_pvt.velN * 1e-3f;
		_gps_position->vel_e_m_s	= (float)_buf.payload_rx_nav_pvt.velE * 1e-3f;
		_gps_position->vel_d_m_s	= (float)_buf.payload_rx_nav_pvt.velD * 1e-3f;
		_gps_position->vel_ned_valid	= true;

		_gps_position->cog_rad		= (float)_buf.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad	= (float)_buf.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;

		{
			/* convert to unix timestamp */
			struct tm timeinfo;
			timeinfo.tm_year	= _buf.payload_rx_nav_pvt.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_pvt.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_pvt.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_pvt.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_pvt.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_pvt.sec;
			time_t epoch = mktime(&timeinfo);

#ifndef CONFIG_RTC
			//Since we lack a hardware RTC, set the system time clock based on GPS UTC
			//TODO generalize this by moving into gps.cpp?
			timespec ts;
			ts.tv_sec = epoch;
			ts.tv_nsec = _buf.payload_rx_nav_pvt.nano;
			clock_settime(CLOCK_REALTIME, &ts);
#endif

			_gps_position->time_gps_usec = (uint64_t)epoch * 1000000; //TODO: test this
			_gps_position->time_gps_usec += (uint64_t)(_buf.payload_rx_nav_pvt.nano * 1e-3f);
		}

		_gps_position->timestamp_time		= hrt_absolute_time();
		_gps_position->timestamp_velocity 	= hrt_absolute_time();
		_gps_position->timestamp_variance 	= hrt_absolute_time();
		_gps_position->timestamp_position	= hrt_absolute_time();

		_rate_count_vel++;
		_rate_count_lat_lon++;

		_got_posllh = true;
		_got_velned = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_POSLLH:
		UBX_TRACE_RXMSG("Rx NAV-POSLLH\n");

		_gps_position->lat	= _buf.payload_rx_nav_posllh.lat;
		_gps_position->lon	= _buf.payload_rx_nav_posllh.lon;
		_gps_position->alt	= _buf.payload_rx_nav_posllh.hMSL;
		_gps_position->eph	= (float)_buf.payload_rx_nav_posllh.hAcc * 1e-3f; // from mm to m
		_gps_position->epv	= (float)_buf.payload_rx_nav_posllh.vAcc * 1e-3f; // from mm to m

		_gps_position->timestamp_position = hrt_absolute_time();

		_rate_count_lat_lon++;
		_got_posllh = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_SOL:
		UBX_TRACE_RXMSG("Rx NAV-SOL\n");

		_gps_position->fix_type		= _buf.payload_rx_nav_sol.gpsFix;
		_gps_position->s_variance_m_s	= (float)_buf.payload_rx_nav_sol.sAcc * 1e-2f;	// from cm to m
		_gps_position->satellites_used	= _buf.payload_rx_nav_sol.numSV;

		_gps_position->timestamp_variance = hrt_absolute_time();

		ret = 1;
		break;

	case UBX_MSG_NAV_TIMEUTC:
		UBX_TRACE_RXMSG("Rx NAV-TIMEUTC\n");

		{
			/* convert to unix timestamp */
			struct tm timeinfo;
			timeinfo.tm_year	= _buf.payload_rx_nav_timeutc.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_timeutc.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_timeutc.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_timeutc.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_timeutc.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_timeutc.sec;
			time_t epoch = mktime(&timeinfo);

#ifndef CONFIG_RTC
			//Since we lack a hardware RTC, set the system time clock based on GPS UTC
			//TODO generalize this by moving into gps.cpp?
			timespec ts;
			ts.tv_sec = epoch;
			ts.tv_nsec = _buf.payload_rx_nav_timeutc.nano;
			clock_settime(CLOCK_REALTIME, &ts);
#endif

			_gps_position->time_gps_usec = (uint64_t)epoch * 1000000; //TODO: test this
			_gps_position->time_gps_usec += (uint64_t)(_buf.payload_rx_nav_timeutc.nano * 1e-3f);
		}

		_gps_position->timestamp_time = hrt_absolute_time();

		ret = 1;
		break;

	case UBX_MSG_NAV_SVINFO:
		UBX_TRACE_RXMSG("Rx NAV-SVINFO\n");

		// _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
		_satellite_info->timestamp = hrt_absolute_time();

		ret = 2;
		break;

	case UBX_MSG_NAV_VELNED:
		UBX_TRACE_RXMSG("Rx NAV-VELNED\n");

		_gps_position->vel_m_s		= (float)_buf.payload_rx_nav_velned.speed * 1e-2f;
		_gps_position->vel_n_m_s	= (float)_buf.payload_rx_nav_velned.velN * 1e-2f; /* NED NORTH velocity */
		_gps_position->vel_e_m_s	= (float)_buf.payload_rx_nav_velned.velE * 1e-2f; /* NED EAST velocity */
		_gps_position->vel_d_m_s	= (float)_buf.payload_rx_nav_velned.velD * 1e-2f; /* NED DOWN velocity */
		_gps_position->cog_rad		= (float)_buf.payload_rx_nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad	= (float)_buf.payload_rx_nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->vel_ned_valid	= true;

		_gps_position->timestamp_velocity = hrt_absolute_time();

		_rate_count_vel++;
		_got_velned = true;

		ret = 1;
		break;

	case UBX_MSG_MON_VER:
		UBX_TRACE_RXMSG("Rx MON-VER\n");

		ret = 1;
		break;

	case UBX_MSG_MON_HW:
		UBX_TRACE_RXMSG("Rx MON-HW\n");

		switch (_rx_payload_length) {

		case sizeof(ubx_payload_rx_mon_hw_ubx6_t):	/* u-blox 6 msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx6.noisePerMS;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx6.jamInd;

			ret = 1;
			break;

		case sizeof(ubx_payload_rx_mon_hw_ubx7_t):	/* u-blox 7+ msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx7.noisePerMS;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx7.jamInd;

			ret = 1;
			break;

		default:		// unexpected payload size:
			ret = 0;	// don't handle message
			break;
		}
		break;

	case UBX_MSG_ACK_ACK:
		UBX_TRACE_RXMSG("Rx ACK-ACK\n");

		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_ACK_NAK:
		UBX_TRACE_RXMSG("Rx ACK-NAK\n");

		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_NAK;
		}

		ret = 1;
		break;

	default:
		break;
	}

	return ret;
}

void
UBX::decode_init(void)
{
	_decode_state = UBX_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
}

void
UBX::add_byte_to_checksum(const uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
UBX::calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}

void
UBX::configure_message_rate(const uint16_t msg, const uint8_t rate)
{
	ubx_payload_tx_cfg_msg_t cfg_msg;	// don't use _buf (allow interleaved operation)

	cfg_msg.msg	= msg;
	cfg_msg.rate	= rate;

	send_message(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
}

void
UBX::send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	calc_checksum(((uint8_t*)&header) + 2, sizeof(header) - 2, &checksum);  // skip 2 sync bytes
	if (payload != nullptr)
		calc_checksum(payload, length, &checksum);

	// Send message
	write(_fd, (const void *)&header, sizeof(header));
	if (payload != nullptr)
		write(_fd, (const void *)payload, length);
	write(_fd, (const void *)&checksum, sizeof(checksum));
}

uint32_t
UBX::fnv1_32_str(uint8_t *str, uint32_t hval)
{
    uint8_t *s = str;

    /*
     * FNV-1 hash each octet in the buffer
     */
    while (*s) {

	/* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
	hval *= FNV1_32_PRIME;
#else
	hval += (hval<<1) + (hval<<4) + (hval<<7) + (hval<<8) + (hval<<24);
#endif

	/* xor the bottom with the current octet */
	hval ^= (uint32_t)*s++;
    }

    /* return our new hash value */
    return hval;
}

