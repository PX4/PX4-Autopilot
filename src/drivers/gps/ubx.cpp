/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * U-Blox protocol implementation. Following u-blox 6/7 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @see http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
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
#include <drivers/drv_hrt.h>

#include "ubx.h"

#define UBX_CONFIG_TIMEOUT		200		// ms, timeout for waiting ACK
#define UBX_PACKET_TIMEOUT		2		// ms, if now data during this delay assume that full update received
#define UBX_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define DISABLE_MSG_INTERVAL	1000000	// us, try to disable message with this interval

UBX::UBX(const int &fd, struct vehicle_gps_position_s *gps_position) :
	_fd(fd),
	_gps_position(gps_position),
	_configured(false),
	_waiting_for_ack(false),
	_got_posllh(false),
	_got_velned(false),
	_got_timeutc(false),
	_disable_cmd_last(0)
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
	const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};

	int baud_i;

	for (baud_i = 0; baud_i < 5; baud_i++) {
		baudrate = baudrates_to_try[baud_i];
		set_baudrate(_fd, baudrate);

		/* Send a CFG-PRT message to set the UBX protocol for in and out
		 * and leave the baudrate as it is, we just want an ACK-ACK from this
		 */
		type_gps_bin_cfg_prt_packet_t cfg_prt_packet;
		/* Set everything else of the packet to 0, otherwise the module wont accept it */
		memset(&cfg_prt_packet, 0, sizeof(cfg_prt_packet));

		_message_class_needed = UBX_CLASS_CFG;
		_message_id_needed = UBX_MESSAGE_CFG_PRT;

		/* Define the package contents, don't change the baudrate */
		cfg_prt_packet.clsID		= UBX_CLASS_CFG;
		cfg_prt_packet.msgID		= UBX_MESSAGE_CFG_PRT;
		cfg_prt_packet.length		= UBX_CFG_PRT_LENGTH;
		cfg_prt_packet.portID		= UBX_CFG_PRT_PAYLOAD_PORTID;
		cfg_prt_packet.mode			= UBX_CFG_PRT_PAYLOAD_MODE;
		cfg_prt_packet.baudRate		= baudrate;
		cfg_prt_packet.inProtoMask	= UBX_CFG_PRT_PAYLOAD_INPROTOMASK;
		cfg_prt_packet.outProtoMask	= UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK;

		send_config_packet(_fd, (uint8_t *)&cfg_prt_packet, sizeof(cfg_prt_packet));

		if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		/* Send a CFG-PRT message again, this time change the baudrate */

		cfg_prt_packet.clsID		= UBX_CLASS_CFG;
		cfg_prt_packet.msgID		= UBX_MESSAGE_CFG_PRT;
		cfg_prt_packet.length		= UBX_CFG_PRT_LENGTH;
		cfg_prt_packet.portID		= UBX_CFG_PRT_PAYLOAD_PORTID;
		cfg_prt_packet.mode			= UBX_CFG_PRT_PAYLOAD_MODE;
		cfg_prt_packet.baudRate		= UBX_CFG_PRT_PAYLOAD_BAUDRATE;
		cfg_prt_packet.inProtoMask	= UBX_CFG_PRT_PAYLOAD_INPROTOMASK;
		cfg_prt_packet.outProtoMask	= UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK;

		send_config_packet(_fd, (uint8_t *)&cfg_prt_packet, sizeof(cfg_prt_packet));

		/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
		wait_for_ack(UBX_CONFIG_TIMEOUT);

		if (UBX_CFG_PRT_PAYLOAD_BAUDRATE != baudrate) {
			set_baudrate(_fd, UBX_CFG_PRT_PAYLOAD_BAUDRATE);
			baudrate = UBX_CFG_PRT_PAYLOAD_BAUDRATE;
		}

		/* at this point we have correct baudrate on both ends */
		break;
	}

	if (baud_i >= 5) {
		return 1;
	}

	/* send a CFG-RATE message to define update rate */
	type_gps_bin_cfg_rate_packet_t cfg_rate_packet;
	memset(&cfg_rate_packet, 0, sizeof(cfg_rate_packet));

	_message_class_needed = UBX_CLASS_CFG;
	_message_id_needed = UBX_MESSAGE_CFG_RATE;

	cfg_rate_packet.clsID		= UBX_CLASS_CFG;
	cfg_rate_packet.msgID		= UBX_MESSAGE_CFG_RATE;
	cfg_rate_packet.length		= UBX_CFG_RATE_LENGTH;
	cfg_rate_packet.measRate	= UBX_CFG_RATE_PAYLOAD_MEASINTERVAL;
	cfg_rate_packet.navRate		= UBX_CFG_RATE_PAYLOAD_NAVRATE;
	cfg_rate_packet.timeRef		= UBX_CFG_RATE_PAYLOAD_TIMEREF;

	send_config_packet(_fd, (uint8_t *)&cfg_rate_packet, sizeof(cfg_rate_packet));

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("CFG FAIL: RATE");
		return 1;
	}

	/* send a NAV5 message to set the options for the internal filter */
	type_gps_bin_cfg_nav5_packet_t cfg_nav5_packet;
	memset(&cfg_nav5_packet, 0, sizeof(cfg_nav5_packet));

	_message_class_needed = UBX_CLASS_CFG;
	_message_id_needed = UBX_MESSAGE_CFG_NAV5;

	cfg_nav5_packet.clsID        = UBX_CLASS_CFG;
	cfg_nav5_packet.msgID        = UBX_MESSAGE_CFG_NAV5;
	cfg_nav5_packet.length       = UBX_CFG_NAV5_LENGTH;
	cfg_nav5_packet.mask         = UBX_CFG_NAV5_PAYLOAD_MASK;
	cfg_nav5_packet.dynModel     = UBX_CFG_NAV5_PAYLOAD_DYNMODEL;
	cfg_nav5_packet.fixMode      = UBX_CFG_NAV5_PAYLOAD_FIXMODE;

	send_config_packet(_fd, (uint8_t *)&cfg_nav5_packet, sizeof(cfg_nav5_packet));

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("CFG FAIL: NAV5");
		return 1;
	}

	/* configure message rates */
	/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */
	configure_message_rate(UBX_CLASS_NAV, UBX_MESSAGE_NAV_POSLLH, 1);

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("MSG CFG FAIL: NAV POSLLH");
		return 1;
	}

	configure_message_rate(UBX_CLASS_NAV, UBX_MESSAGE_NAV_TIMEUTC, 1);

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("MSG CFG FAIL: NAV TIMEUTC");
		return 1;
	}

	configure_message_rate(UBX_CLASS_NAV, UBX_MESSAGE_NAV_SOL, 1);

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("MSG CFG FAIL: NAV SOL");
		return 1;
	}

	configure_message_rate(UBX_CLASS_NAV, UBX_MESSAGE_NAV_VELNED, 1);

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("MSG CFG FAIL: NAV VELNED");
		return 1;
	}

	configure_message_rate(UBX_CLASS_NAV, UBX_MESSAGE_NAV_SVINFO, 5);

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("MSG CFG FAIL: NAV SVINFO");
		return 1;
	}

	configure_message_rate(UBX_CLASS_MON, UBX_MESSAGE_MON_HW, 1);

	if (wait_for_ack(UBX_CONFIG_TIMEOUT) < 0) {
		warnx("MSG CFG FAIL: MON HW");
		return 1;
	}

	_configured = true;
	return 0;
}

int
UBX::wait_for_ack(unsigned timeout)
{
	_waiting_for_ack = true;
	uint64_t time_started = hrt_absolute_time();

	while (hrt_absolute_time() < time_started + timeout * 1000) {
		if (receive(timeout) > 0) {
			if (!_waiting_for_ack) {
				return 1;
			}

		} else {
			return -1;	// timeout or error receiving, or NAK
		}
	}

	return -1;	// timeout
}

int
UBX::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[128];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	ssize_t count = 0;

	bool handled = false;

	while (true) {
		bool ready_to_return = _configured ? (_got_posllh && _got_velned && _got_timeutc) : handled;

		/* poll for new data, wait for only UBX_PACKET_TIMEOUT (2ms) if something already received */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), ready_to_return ? UBX_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			warnx("poll error");
			return -1;

		} else if (ret == 0) {
			/* return success after short delay after receiving a packet or timeout after long delay */
			if (ready_to_return) {
				_got_posllh = false;
				_got_velned = false;
				_got_timeutc = false;
				return 1;

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
					if (parse_char(buf[i]) > 0) {
						if (handle_message() > 0)
							handled = true;
					}
				}
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			warnx("timeout - no useful messages");
			return -1;
		}
	}
}

int
UBX::parse_char(uint8_t b)
{
	switch (_decode_state) {
		/* First, look for sync1 */
	case UBX_DECODE_UNINIT:
		if (b == UBX_SYNC1) {
			_decode_state = UBX_DECODE_GOT_SYNC1;
		}

		break;

		/* Second, look for sync2 */
	case UBX_DECODE_GOT_SYNC1:
		if (b == UBX_SYNC2) {
			_decode_state = UBX_DECODE_GOT_SYNC2;

		} else {
			/* Second start symbol was wrong, reset state machine */
			decode_init();
			/* don't return error, it can be just false sync1 */
		}

		break;

		/* Now look for class */
	case UBX_DECODE_GOT_SYNC2:
		/* everything except sync1 and sync2 needs to be added to the checksum */
		add_byte_to_checksum(b);
		_message_class = b;
		_decode_state = UBX_DECODE_GOT_CLASS;
		break;

	case UBX_DECODE_GOT_CLASS:
		add_byte_to_checksum(b);
		_message_id = b;
		_decode_state = UBX_DECODE_GOT_MESSAGEID;
		break;

	case UBX_DECODE_GOT_MESSAGEID:
		add_byte_to_checksum(b);
		_payload_size = b; //this is the first length byte
		_decode_state = UBX_DECODE_GOT_LENGTH1;
		break;

	case UBX_DECODE_GOT_LENGTH1:
		add_byte_to_checksum(b);
		_payload_size += b << 8; // here comes the second byte of length
		_decode_state = UBX_DECODE_GOT_LENGTH2;
		break;

	case UBX_DECODE_GOT_LENGTH2:

		/* Add to checksum if not yet at checksum byte */
		if (_rx_count < _payload_size)
			add_byte_to_checksum(b);

		_rx_buffer[_rx_count] = b;

		/* once the payload has arrived, we can process the information */
		if (_rx_count >= _payload_size + 1) { //+1 because of 2 checksum bytes
			/* compare checksum */
			if (_rx_ck_a == _rx_buffer[_rx_count - 1] && _rx_ck_b == _rx_buffer[_rx_count]) {
				decode_init();
				return 1;	// message received successfully

			} else {
				warnx("checksum wrong");
				decode_init();
				return -1;
			}

		} else if (_rx_count < RECV_BUFFER_SIZE) {
			_rx_count++;

		} else {
			warnx("buffer full");
			decode_init();
			return -1;
		}

		break;

	default:
		break;
	}

	return 0;	// message decoding in progress
}



int
UBX::handle_message()
{
	int ret = 0;

	if (_configured) {
		/* handle only info messages when configured */
		switch (_message_class) {
		case UBX_CLASS_NAV:
			switch (_message_id) {
			case UBX_MESSAGE_NAV_POSLLH: {
					// printf("GOT NAV_POSLLH\n");
					gps_bin_nav_posllh_packet_t *packet = (gps_bin_nav_posllh_packet_t *) _rx_buffer;

					_gps_position->lat = packet->lat;
					_gps_position->lon = packet->lon;
					_gps_position->alt = packet->height_msl;
					_gps_position->eph_m = (float)packet->hAcc * 1e-3f; // from mm to m
					_gps_position->epv_m = (float)packet->vAcc * 1e-3f; // from mm to m
					_gps_position->timestamp_position = hrt_absolute_time();

					_rate_count_lat_lon++;

					_got_posllh = true;
					ret = 1;
					break;
				}

			case UBX_MESSAGE_NAV_SOL: {
					// printf("GOT NAV_SOL\n");
					gps_bin_nav_sol_packet_t *packet = (gps_bin_nav_sol_packet_t *) _rx_buffer;

					_gps_position->fix_type = packet->gpsFix;
					_gps_position->s_variance_m_s = (float)packet->sAcc * 1e-2f; // from cm/s to m/s
					_gps_position->p_variance_m = (float)packet->pAcc * 1e-2f; // from cm to m
					_gps_position->timestamp_variance = hrt_absolute_time();

					ret = 1;
					break;
				}

			case UBX_MESSAGE_NAV_TIMEUTC: {
					// printf("GOT NAV_TIMEUTC\n");
					gps_bin_nav_timeutc_packet_t *packet = (gps_bin_nav_timeutc_packet_t *) _rx_buffer;

					/* convert to unix timestamp */
					struct tm timeinfo;
					timeinfo.tm_year = packet->year - 1900;
					timeinfo.tm_mon = packet->month - 1;
					timeinfo.tm_mday = packet->day;
					timeinfo.tm_hour = packet->hour;
					timeinfo.tm_min = packet->min;
					timeinfo.tm_sec = packet->sec;
					time_t epoch = mktime(&timeinfo);

#ifndef CONFIG_RTC
					//Since we lack a hardware RTC, set the system time clock based on GPS UTC
					//TODO generalize this by moving into gps.cpp?
					timespec ts;
					ts.tv_sec = epoch;
					ts.tv_nsec = packet->time_nanoseconds;
					clock_settime(CLOCK_REALTIME, &ts);
#endif

					_gps_position->time_gps_usec = (uint64_t)epoch * 1000000; //TODO: test this
					_gps_position->time_gps_usec += (uint64_t)(packet->time_nanoseconds * 1e-3f);
					_gps_position->timestamp_time = hrt_absolute_time();

					_got_timeutc = true;
					ret = 1;
					break;
				}

			case UBX_MESSAGE_NAV_SVINFO: {
					//printf("GOT NAV_SVINFO\n");
					const int length_part1 = 8;
					gps_bin_nav_svinfo_part1_packet_t *packet_part1 = (gps_bin_nav_svinfo_part1_packet_t *) _rx_buffer;
					const int length_part2 = 12;
					gps_bin_nav_svinfo_part2_packet_t *packet_part2;

					uint8_t satellites_used = 0;
					int i;

					//printf("Number of Channels: %d\n", packet_part1->numCh);
					for (i = 0; i < packet_part1->numCh; i++) {
						/* set pointer to sattelite_i information */
						packet_part2 = (gps_bin_nav_svinfo_part2_packet_t *) & (_rx_buffer[length_part1 + i * length_part2]);

						/* write satellite information to global storage */
						uint8_t sv_used = packet_part2->flags & 0x01;

						if (sv_used) {
							/* count SVs used for NAV */
							satellites_used++;
						}

						/* record info for all channels, whether or not the SV is used for NAV */
						_gps_position->satellite_used[i] = sv_used;
						_gps_position->satellite_snr[i] = packet_part2->cno;
						_gps_position->satellite_elevation[i] = (uint8_t)(packet_part2->elev);
						_gps_position->satellite_azimuth[i] = (uint8_t)((float)packet_part2->azim * 255.0f / 360.0f);
						_gps_position->satellite_prn[i] = packet_part2->svid;
						//printf("SAT %d: %d %d %d %d\n", i, (int)sv_used, (int)packet_part2->cno, (int)(uint8_t)(packet_part2->elev), (int)packet_part2->svid);
					}

					for (i = packet_part1->numCh; i < 20; i++) {
						/* unused channels have to be set to zero for e.g. MAVLink */
						_gps_position->satellite_prn[i] = 0;
						_gps_position->satellite_used[i] = 0;
						_gps_position->satellite_snr[i] = 0;
						_gps_position->satellite_elevation[i] = 0;
						_gps_position->satellite_azimuth[i] = 0;
					}

					_gps_position->satellites_visible = satellites_used; // visible ~= used but we are interested in the used ones

					if (packet_part1->numCh > 0) {
						_gps_position->satellite_info_available = true;

					} else {
						_gps_position->satellite_info_available = false;
					}

					_gps_position->timestamp_satellites = hrt_absolute_time();

					ret = 1;
					break;
				}

			case UBX_MESSAGE_NAV_VELNED: {
					// printf("GOT NAV_VELNED\n");
					gps_bin_nav_velned_packet_t *packet = (gps_bin_nav_velned_packet_t *) _rx_buffer;

					_gps_position->vel_m_s   = (float)packet->speed * 1e-2f;
					_gps_position->vel_n_m_s = (float)packet->velN * 1e-2f; /* NED NORTH velocity */
					_gps_position->vel_e_m_s = (float)packet->velE * 1e-2f; /* NED EAST velocity */
					_gps_position->vel_d_m_s = (float)packet->velD * 1e-2f; /* NED DOWN velocity */
					_gps_position->cog_rad   = (float)packet->heading * M_DEG_TO_RAD_F * 1e-5f;
					_gps_position->c_variance_rad = (float)packet->cAcc * M_DEG_TO_RAD_F * 1e-5f;
					_gps_position->vel_ned_valid = true;
					_gps_position->timestamp_velocity = hrt_absolute_time();

					_rate_count_vel++;

					_got_velned = true;
					ret = 1;
					break;
				}

			default:
				break;
			}

			break;

		case UBX_CLASS_ACK: {
				/* ignore ACK when already configured */
				ret = 1;
				break;
			}

		case UBX_CLASS_MON: {
			switch (_message_id) {
			case UBX_MESSAGE_MON_HW: {

					struct gps_bin_mon_hw_packet *p = (struct gps_bin_mon_hw_packet*) _rx_buffer;

					_gps_position->noise_per_ms = p->noisePerMS;
					_gps_position->jamming_indicator = p->jamInd;

					ret = 1;
					break;
				}

			default:
				break;
			}
		}

		default:
			break;
		}

		if (ret == 0) {
			/* message not handled */
			warnx("ubx: unknown message received: 0x%02x-0x%02x", (unsigned)_message_class, (unsigned)_message_id);

			hrt_abstime t = hrt_absolute_time();

			if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL) {
				/* don't attempt for every message to disable, some might not be disabled */
				_disable_cmd_last = t;
				warnx("ubx: disabling message 0x%02x-0x%02x", (unsigned)_message_class, (unsigned)_message_id);
				configure_message_rate(_message_class, _message_id, 0);
			}
		}

	} else {
		/* handle only ACK while configuring */
		if (_message_class == UBX_CLASS_ACK) {
			switch (_message_id) {
			case UBX_MESSAGE_ACK_ACK: {
					// printf("GOT ACK_ACK\n");
					gps_bin_ack_ack_packet_t *packet = (gps_bin_ack_ack_packet_t *) _rx_buffer;

					if (_waiting_for_ack) {
						if (packet->clsID == _message_class_needed && packet->msgID == _message_id_needed) {
							_waiting_for_ack = false;
							ret = 1;
						}
					}

					break;
				}

			case UBX_MESSAGE_ACK_NAK: {
					// printf("GOT ACK_NAK\n");
					warnx("ubx: not acknowledged");
					/* configuration obviously not successful */
					_waiting_for_ack = false;
					ret = -1;
					break;
				}

			default:
				break;
			}
		}
	}

	decode_init();
	return ret;
}

void
UBX::decode_init(void)
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_decode_state = UBX_DECODE_UNINIT;
	_payload_size = 0;
	/* don't reset _message_class, _message_id, _rx_buffer leave it for message handler */
}

void
UBX::add_byte_to_checksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
UBX::add_checksum_to_message(uint8_t *message, const unsigned length)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	unsigned i;

	for (i = 0; i < length - 2; i++) {
		ck_a = ck_a + message[i];
		ck_b = ck_b + ck_a;
	}

	/* the checksum is written to the last to bytes of a message */
	message[length - 2] = ck_a;
	message[length - 1] = ck_b;
}

void
UBX::add_checksum(uint8_t *message, const unsigned length, uint8_t &ck_a, uint8_t &ck_b)
{
	for (unsigned i = 0; i < length; i++) {
		ck_a = ck_a + message[i];
		ck_b = ck_b + ck_a;
	}
}

void
UBX::configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
	struct ubx_cfg_msg_rate msg;
	msg.msg_class	= msg_class;
	msg.msg_id		= msg_id;
	msg.rate		= rate;
	send_message(UBX_CLASS_CFG, UBX_MESSAGE_CFG_MSG, &msg, sizeof(msg));
}

void
UBX::send_config_packet(const int &fd, uint8_t *packet, const unsigned length)
{
	ssize_t ret = 0;

	/* calculate the checksum now */
	add_checksum_to_message(packet, length);

	const uint8_t sync_bytes[] = {UBX_SYNC1, UBX_SYNC2};

	/* start with the two sync bytes */
	ret += write(fd, sync_bytes, sizeof(sync_bytes));
	ret += write(fd, packet, length);

	if (ret != (int)length + (int)sizeof(sync_bytes)) // XXX is there a neater way to get rid of the unsigned signed warning?
		warnx("ubx: configuration write fail");
}

void
UBX::send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
	struct ubx_header header;
	uint8_t ck_a = 0, ck_b = 0;
	header.sync1 = UBX_SYNC1;
	header.sync2 = UBX_SYNC2;
	header.msg_class = msg_class;
	header.msg_id    = msg_id;
	header.length    = size;

	add_checksum((uint8_t *)&header.msg_class, sizeof(header) - 2, ck_a, ck_b);
	add_checksum((uint8_t *)msg, size, ck_a, ck_b);

	/* configure ACK check */
	_message_class_needed = msg_class;
	_message_id_needed = msg_id;

	write(_fd, (const char *)&header, sizeof(header));
	write(_fd, (const char *)msg, size);
	write(_fd, (const char *)&ck_a, 1);
	write(_fd, (const char *)&ck_b, 1);
}
