/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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

/* @file U-Blox protocol implementation */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

#include "ubx.h"

#define UBX_CONFIG_TIMEOUT 100

UBX::UBX(const int &fd, struct vehicle_gps_position_s *gps_position) :
_fd(fd),
_gps_position(gps_position),
_waiting_for_ack(false)
{
	decode_init();
}

UBX::~UBX()
{
}

int
UBX::configure(unsigned &baudrate)
{
	_waiting_for_ack = true;

	/* try different baudrates */
	const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};

	for (int baud_i = 0; baud_i < 5; baud_i++) {
		baudrate = baudrates_to_try[baud_i];
		set_baudrate(_fd, baudrate);

		/* Send a CFG-PRT message to set the UBX protocol for in and out
		 * and leave the baudrate as it is, we just want an ACK-ACK from this
		 */
		type_gps_bin_cfg_prt_packet_t cfg_prt_packet;
		/* Set everything else of the packet to 0, otherwise the module wont accept it */
		memset(&cfg_prt_packet, 0, sizeof(cfg_prt_packet));

		_clsID_needed = UBX_CLASS_CFG;
		_msgID_needed = UBX_MESSAGE_CFG_PRT;

		/* Define the package contents, don't change the baudrate */
		cfg_prt_packet.clsID		= UBX_CLASS_CFG;
		cfg_prt_packet.msgID		= UBX_MESSAGE_CFG_PRT;
		cfg_prt_packet.length		= UBX_CFG_PRT_LENGTH;
		cfg_prt_packet.portID		= UBX_CFG_PRT_PAYLOAD_PORTID;
		cfg_prt_packet.mode			= UBX_CFG_PRT_PAYLOAD_MODE;
		cfg_prt_packet.baudRate		= baudrate;
		cfg_prt_packet.inProtoMask	= UBX_CFG_PRT_PAYLOAD_INPROTOMASK;
		cfg_prt_packet.outProtoMask	= UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK;

		send_config_packet(_fd, (uint8_t*)&cfg_prt_packet, sizeof(cfg_prt_packet));

		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
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

		send_config_packet(_fd, (uint8_t*)&cfg_prt_packet, sizeof(cfg_prt_packet));
		if (UBX_CFG_PRT_PAYLOAD_BAUDRATE != baudrate) {
			set_baudrate(_fd, UBX_CFG_PRT_PAYLOAD_BAUDRATE);
			baudrate = UBX_CFG_PRT_PAYLOAD_BAUDRATE;
		}

		/* no ack is ecpected here, keep going configuring */

		/* send a CFT-RATE message to define update rate */
		type_gps_bin_cfg_rate_packet_t cfg_rate_packet;
		memset(&cfg_rate_packet, 0, sizeof(cfg_rate_packet));

		_clsID_needed = UBX_CLASS_CFG;
		_msgID_needed = UBX_MESSAGE_CFG_RATE;

		cfg_rate_packet.clsID		= UBX_CLASS_CFG;
		cfg_rate_packet.msgID		= UBX_MESSAGE_CFG_RATE;
		cfg_rate_packet.length		= UBX_CFG_RATE_LENGTH;
		cfg_rate_packet.measRate	= UBX_CFG_RATE_PAYLOAD_MEASRATE;
		cfg_rate_packet.navRate		= UBX_CFG_RATE_PAYLOAD_NAVRATE;
		cfg_rate_packet.timeRef		= UBX_CFG_RATE_PAYLOAD_TIMEREF;

		send_config_packet(_fd, (uint8_t*)&cfg_rate_packet, sizeof(cfg_rate_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		/* send a NAV5 message to set the options for the internal filter */
		type_gps_bin_cfg_nav5_packet_t cfg_nav5_packet;
		memset(&cfg_nav5_packet, 0, sizeof(cfg_nav5_packet));

		_clsID_needed = UBX_CLASS_CFG;
		_msgID_needed = UBX_MESSAGE_CFG_NAV5;

		cfg_nav5_packet.clsID        = UBX_CLASS_CFG;
		cfg_nav5_packet.msgID        = UBX_MESSAGE_CFG_NAV5;
		cfg_nav5_packet.length       = UBX_CFG_NAV5_LENGTH;
		cfg_nav5_packet.mask         = UBX_CFG_NAV5_PAYLOAD_MASK;
		cfg_nav5_packet.dynModel     = UBX_CFG_NAV5_PAYLOAD_DYNMODEL;
		cfg_nav5_packet.fixMode      = UBX_CFG_NAV5_PAYLOAD_FIXMODE;

		send_config_packet(_fd, (uint8_t*)&cfg_nav5_packet, sizeof(cfg_nav5_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		type_gps_bin_cfg_msg_packet_t cfg_msg_packet;
		memset(&cfg_msg_packet, 0, sizeof(cfg_msg_packet));

		_clsID_needed = UBX_CLASS_CFG;
		_msgID_needed = UBX_MESSAGE_CFG_MSG;

		cfg_msg_packet.clsID        = UBX_CLASS_CFG;
		cfg_msg_packet.msgID        = UBX_MESSAGE_CFG_MSG;
		cfg_msg_packet.length       = UBX_CFG_MSG_LENGTH;
		/* Choose fast 5Hz rate for all messages except SVINFO which is big and not important */
		cfg_msg_packet.rate[1]      = UBX_CFG_MSG_PAYLOAD_RATE1_5HZ;

		cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
		cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_POSLLH;

		send_config_packet(_fd, (uint8_t*)&cfg_msg_packet, sizeof(cfg_msg_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
		cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_TIMEUTC;

		send_config_packet(_fd, (uint8_t*)&cfg_msg_packet, sizeof(cfg_msg_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
		cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_SVINFO;
		/* For satelites info 1Hz is enough */
		cfg_msg_packet.rate[1] = UBX_CFG_MSG_PAYLOAD_RATE1_1HZ;

		send_config_packet(_fd, (uint8_t*)&cfg_msg_packet, sizeof(cfg_msg_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
		cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_SOL;

		send_config_packet(_fd, (uint8_t*)&cfg_msg_packet, sizeof(cfg_msg_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}

		cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
		cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_VELNED;

		send_config_packet(_fd, (uint8_t*)&cfg_msg_packet, sizeof(cfg_msg_packet));
		if (receive(UBX_CONFIG_TIMEOUT) < 0) {
			/* try next baudrate */
			continue;
		}
//		cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
//		cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_DOP;

//		cfg_msg_packet.msgClass_payload = UBX_CLASS_RXM;
//		cfg_msg_packet.msgID_payload = UBX_MESSAGE_RXM_SVSI;

		_waiting_for_ack = false;
		return 0;
	}
	return -1;
}

int
UBX::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[32];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	int j = 0;
	ssize_t count = 0;

	while (true) {

		/* pass received bytes to the packet decoder */
		while (j < count) {
			if (parse_char(buf[j]) > 0) {
				/* return to configure during configuration or to the gps driver during normal work
				 * if a packet has arrived */
				 if (handle_message() > 0)
					return 1;
			}
			/* in case we keep trying but only get crap from GPS */
			if (time_started + timeout*1000 < hrt_absolute_time() ) {
				return -1;
			}
			j++;
		}

		/* everything is read */
		j = count = 0;

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_fd, buf, sizeof(buf));
			}
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
			}
			break;
		/* Now look for class */
		case UBX_DECODE_GOT_SYNC2:
			/* everything except sync1 and sync2 needs to be added to the checksum */
			add_byte_to_checksum(b);
			/* check for known class */
			switch (b) {
				case UBX_CLASS_ACK:
					_decode_state = UBX_DECODE_GOT_CLASS;
					_message_class = ACK;
					break;

				case UBX_CLASS_NAV:
					_decode_state = UBX_DECODE_GOT_CLASS;
					_message_class = NAV;
					break;

//				case UBX_CLASS_RXM:
//					_decode_state = UBX_DECODE_GOT_CLASS;
//					_message_class = RXM;
//					break;

				case UBX_CLASS_CFG:
					_decode_state = UBX_DECODE_GOT_CLASS;
					_message_class = CFG;
					break;
				default: //unknown class: reset state machine
					decode_init();
					break;
			}
			break;
		case UBX_DECODE_GOT_CLASS:
			add_byte_to_checksum(b);
			switch (_message_class) {
				case NAV:
					switch (b) {
					case UBX_MESSAGE_NAV_POSLLH:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_POSLLH;
						break;

					case UBX_MESSAGE_NAV_SOL:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_SOL;
						break;

					case UBX_MESSAGE_NAV_TIMEUTC:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_TIMEUTC;
						break;

//					case UBX_MESSAGE_NAV_DOP:
//						_decode_state = UBX_DECODE_GOT_MESSAGEID;
//						_message_id = NAV_DOP;
//						break;

					case UBX_MESSAGE_NAV_SVINFO:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_SVINFO;
						break;

					case UBX_MESSAGE_NAV_VELNED:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_VELNED;
						break;

					default: //unknown class: reset state machine, should not happen
						decode_init();
						break;
					}
					break;
//				case RXM:
//					switch (b) {
//					case UBX_MESSAGE_RXM_SVSI:
//						_decode_state = UBX_DECODE_GOT_MESSAGEID;
//						_message_id = RXM_SVSI;
//						break;
//
//					default: //unknown class: reset state machine, should not happen
//						decode_init();
//						break;
//					}
//					break;

				case CFG:
					switch (b) {
					case UBX_MESSAGE_CFG_NAV5:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = CFG_NAV5;
						break;

					default: //unknown class: reset state machine, should not happen
						decode_init();
						break;
					}
					break;

				case ACK:
					switch (b) {
					case UBX_MESSAGE_ACK_ACK:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = ACK_ACK;
						break;
					case UBX_MESSAGE_ACK_NAK:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = ACK_NAK;
						break;
					default: //unknown class: reset state machine, should not happen
						decode_init();
						break;
					}
					break;
				default: //should not happen because we set the class
					warnx("UBX Error, we set a class that we don't know");
					decode_init();
//					config_needed = true;
					break;
				}
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
						if (_rx_ck_a == _rx_buffer[_rx_count-1] && _rx_ck_b == _rx_buffer[_rx_count]) {
							return 1;
						} else {
							decode_init();
							return -1;
							warnx("ubx: Checksum wrong");
						}

						return 1;
					} else if (_rx_count < RECV_BUFFER_SIZE) {
						_rx_count++;
					} else {
						warnx("ubx: buffer full");
						decode_init();
						return -1;
					}
				break;
		default:
			break;
	}
	return 0; //XXX ?
}


int
UBX::handle_message()
{
	int ret = 0;

	switch (_message_id) { //this enum is unique for all ids --> no need to check the class
		case NAV_POSLLH: {
//			printf("GOT NAV_POSLLH MESSAGE\n");
			if (!_waiting_for_ack) {
				gps_bin_nav_posllh_packet_t *packet = (gps_bin_nav_posllh_packet_t *) _rx_buffer;

				_gps_position->lat = packet->lat;
				_gps_position->lon = packet->lon;
				_gps_position->alt = packet->height_msl;

				_gps_position->eph_m = (float)packet->hAcc * 1e-3f; // from mm to m
				_gps_position->epv_m = (float)packet->vAcc * 1e-3f; // from mm to m

				/* Add timestamp to finish the report */
				_gps_position->timestamp_position = hrt_absolute_time();
				/* only return 1 when new position is available */
				ret = 1;
			}
			break;
		}

		case NAV_SOL: {
//			printf("GOT NAV_SOL MESSAGE\n");
			if (!_waiting_for_ack) {
				gps_bin_nav_sol_packet_t *packet = (gps_bin_nav_sol_packet_t *) _rx_buffer;

				_gps_position->fix_type = packet->gpsFix;
				_gps_position->s_variance_m_s = packet->sAcc;
				_gps_position->p_variance_m = packet->pAcc;

				_gps_position->timestamp_variance = hrt_absolute_time();
			}
			break;
		}

//		case NAV_DOP: {
////		printf("GOT NAV_DOP MESSAGE\n");
//			gps_bin_nav_dop_packet_t *packet = (gps_bin_nav_dop_packet_t *) _rx_buffer;
//
//			_gps_position->eph_m =  packet->hDOP;
//			_gps_position->epv =  packet->vDOP;
//
//			_gps_position->timestamp_posdilution = hrt_absolute_time();
//
//			_new_nav_dop = true;
//
//			break;
//		}

		case NAV_TIMEUTC: {
//			printf("GOT NAV_TIMEUTC MESSAGE\n");

			if (!_waiting_for_ack) {
				gps_bin_nav_timeutc_packet_t *packet = (gps_bin_nav_timeutc_packet_t *) _rx_buffer;

				//convert to unix timestamp
				struct tm timeinfo;
				timeinfo.tm_year = packet->year - 1900;
				timeinfo.tm_mon = packet->month - 1;
				timeinfo.tm_mday = packet->day;
				timeinfo.tm_hour = packet->hour;
				timeinfo.tm_min = packet->min;
				timeinfo.tm_sec = packet->sec;

				time_t epoch = mktime(&timeinfo);

				_gps_position->time_gps_usec = (uint64_t)epoch * 1000000; //TODO: test this
				_gps_position->time_gps_usec += (uint64_t)(packet->time_nanoseconds * 1e-3f);

				_gps_position->timestamp_time = hrt_absolute_time();
			}
			break;
		}

		case NAV_SVINFO: {
//			printf("GOT NAV_SVINFO MESSAGE\n");

			if (!_waiting_for_ack) {
				//this is a more complicated message: the length depends on the number of satellites. This number is extracted from the first part of the message
				const int length_part1 = 8;
				char _rx_buffer_part1[length_part1];
				memcpy(_rx_buffer_part1, _rx_buffer, length_part1);
				gps_bin_nav_svinfo_part1_packet_t *packet_part1 = (gps_bin_nav_svinfo_part1_packet_t *) _rx_buffer_part1;

				//read checksum
				const int length_part3 = 2;
				char _rx_buffer_part3[length_part3];
				memcpy(_rx_buffer_part3, &(_rx_buffer[_rx_count - 1]), length_part3);

				//definitions needed to read numCh elements from the buffer:
				const int length_part2 = 12;
				gps_bin_nav_svinfo_part2_packet_t *packet_part2;
				char _rx_buffer_part2[length_part2]; //for temporal storage

				uint8_t satellites_used = 0;
				int i;

				for (i = 0; i < packet_part1->numCh; i++) { //for each channel

					/* Get satellite information from the buffer */
					memcpy(_rx_buffer_part2, &(_rx_buffer[length_part1 + i * length_part2]), length_part2);
					packet_part2 = (gps_bin_nav_svinfo_part2_packet_t *) _rx_buffer_part2;


					/* Write satellite information in the global storage */
					_gps_position->satellite_prn[i] = packet_part2->svid;

					//if satellite information is healthy store the data
					uint8_t unhealthy = packet_part2->flags & 1 << 4; //flags is a bitfield

					if (!unhealthy) {
						if ((packet_part2->flags) & 1) { //flags is a bitfield
							_gps_position->satellite_used[i] = 1;
							satellites_used++;

						} else {
							_gps_position->satellite_used[i] = 0;
						}

						_gps_position->satellite_snr[i] = packet_part2->cno;
						_gps_position->satellite_elevation[i] = (uint8_t)(packet_part2->elev);
						_gps_position->satellite_azimuth[i] = (uint8_t)((float)packet_part2->azim * 255.0f / 360.0f);

					} else {
						_gps_position->satellite_used[i] = 0;
						_gps_position->satellite_snr[i] = 0;
						_gps_position->satellite_elevation[i] = 0;
						_gps_position->satellite_azimuth[i] = 0;
					}

				}

				for (i = packet_part1->numCh; i < 20; i++) { //these channels are unused
					/* Unused channels have to be set to zero for e.g. MAVLink */
					_gps_position->satellite_prn[i] = 0;
					_gps_position->satellite_used[i] = 0;
					_gps_position->satellite_snr[i] = 0;
					_gps_position->satellite_elevation[i] = 0;
					_gps_position->satellite_azimuth[i] = 0;
				}
				_gps_position->satellites_visible = satellites_used; // visible ~= used but we are interested in the used ones

				/* set timestamp if any sat info is available */
				if (packet_part1->numCh > 0) {
					_gps_position->satellite_info_available = true;
				} else {
					_gps_position->satellite_info_available = false;
				}
				_gps_position->timestamp_satellites = hrt_absolute_time();
			}

			break;
		}

		case NAV_VELNED: {
//			printf("GOT NAV_VELNED MESSAGE\n");

			if (!_waiting_for_ack) {
				gps_bin_nav_velned_packet_t *packet = (gps_bin_nav_velned_packet_t *) _rx_buffer;

				_gps_position->vel_m_s   = (float)packet->speed * 1e-2f;
				_gps_position->vel_n_m_s = (float)packet->velN * 1e-2f;
				_gps_position->vel_e_m_s = (float)packet->velE * 1e-2f;
				_gps_position->vel_d_m_s = (float)packet->velD * 1e-2f;
				_gps_position->cog_rad   = (float)packet->heading * M_DEG_TO_RAD_F * 1e-5f;
				_gps_position->vel_ned_valid = true;
				_gps_position->timestamp_velocity = hrt_absolute_time();
			}

			break;
		}

//		case RXM_SVSI: {
//				printf("GOT RXM_SVSI MESSAGE\n");
//				const int length_part1 = 7;
//				char _rx_buffer_part1[length_part1];
//				memcpy(_rx_buffer_part1, _rx_buffer, length_part1);
//				gps_bin_rxm_svsi_packet_t *packet = (gps_bin_rxm_svsi_packet_t *) _rx_buffer_part1;
//
//				_gps_position->satellites_visible = packet->numVis;
//				_gps_position->counter++;
//				_last_message_timestamps[RXM_SVSI - 1] = hrt_absolute_time();
//
//			break;
//		}
		case ACK_ACK: {
//			printf("GOT ACK_ACK\n");
			gps_bin_ack_ack_packet_t *packet = (gps_bin_ack_ack_packet_t *) _rx_buffer;

			if (_waiting_for_ack) {
				if (packet->clsID == _clsID_needed && packet->msgID == _msgID_needed) {
					ret = 1;
				}
			}
		}
		break;

		case ACK_NAK: {
//			printf("GOT ACK_NAK\n");
			warnx("UBX: Received: Not Acknowledged");
			/* configuration obviously not successful */
			ret = -1;
			break;
		}

		default: //we don't know the message
			warnx("UBX: Unknown message received: %d-%d\n",_message_class,_message_id);
			ret = -1;
			break;
		}
	// end if _rx_count high enough
	decode_init();
	return ret; //XXX?
}

void
UBX::decode_init(void)
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_decode_state = UBX_DECODE_UNINIT;
	_message_class = CLASS_UNKNOWN;
	_message_id = ID_UNKNOWN;
	_payload_size = 0;
}

void
UBX::add_byte_to_checksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
UBX::add_checksum_to_message(uint8_t* message, const unsigned length)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	unsigned i;

	for (i = 0; i < length-2; i++) {
		ck_a = ck_a + message[i];
		ck_b = ck_b + ck_a;
	}
	/* The checksum is written to the last to bytes of a message */
	message[length-2] = ck_a;
	message[length-1] = ck_b;
}

void
UBX::send_config_packet(const int &fd, uint8_t *packet, const unsigned length)
{
	ssize_t ret = 0;

	/* Calculate the checksum now */
	add_checksum_to_message(packet, length);

	const uint8_t sync_bytes[] = {UBX_SYNC1, UBX_SYNC2};

	/* Start with the two sync bytes */
	ret += write(fd, sync_bytes, sizeof(sync_bytes));
	ret += write(fd, packet, length);

	if (ret != (int)length + (int)sizeof(sync_bytes)) // XXX is there a neater way to get rid of the unsigned signed warning?
		warnx("ubx: config write fail");
}
