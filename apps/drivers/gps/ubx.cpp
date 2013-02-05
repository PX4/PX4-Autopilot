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
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

#include "ubx.h"


UBX::UBX() :
_config_state(UBX_CONFIG_STATE_PRT),
_waiting_for_ack(false),
_new_nav_posllh(false),
_new_nav_timeutc(false),
_new_nav_dop(false),
_new_nav_sol(false),
_new_nav_velned(false)
{
	reset();
}

UBX::~UBX()
{
}

void
UBX::reset()
{
	decodeInit();
	_config_state = UBX_CONFIG_STATE_PRT;
	_waiting_for_ack = false;
}

void
UBX::configure(bool &config_needed, bool &baudrate_changed, unsigned &baudrate, uint8_t *buffer, int &length, const unsigned max_length)
{
	/* make sure the buffer, where the message is written to, is long enough */
	assert(sizeof(type_gps_bin_cfg_prt_packet_t)+2 <= max_length);

	/* Only send a new config message when we got the ACK of the last one,
	 * otherwise we might not configure all the messages because the ACK comes from an older/previos CFD command
	 * reason being that the ACK only includes CFG-MSG but not to which NAV MSG it belongs.
	 */
	if (!_waiting_for_ack) {
		_waiting_for_ack = true;
		if (_config_state == UBX_CONFIG_STATE_CONFIGURED) {
			config_needed = false;
			length = 0;
			_config_state = UBX_CONFIG_STATE_PRT; /* set the state for next time */
			_waiting_for_ack = false;
			return;
		} else if (_config_state == UBX_CONFIG_STATE_PRT) {

			/* Send a CFG-PRT message to set the UBX protocol for in and out
			 * and leave the baudrate as it is, we just want an ACK-ACK from this
			 */
			type_gps_bin_cfg_prt_packet_t cfg_prt_packet;
			/* Set everything else of the packet to 0, otherwise the module wont accept it */
			memset(&cfg_prt_packet, 0, sizeof(cfg_prt_packet));

			/* Define the package contents, don't change the baudrate */
			cfg_prt_packet.clsID		= UBX_CLASS_CFG;
			cfg_prt_packet.msgID		= UBX_MESSAGE_CFG_PRT;
			cfg_prt_packet.length		= UBX_CFG_PRT_LENGTH;
			cfg_prt_packet.portID		= UBX_CFG_PRT_PAYLOAD_PORTID;
			cfg_prt_packet.mode			= UBX_CFG_PRT_PAYLOAD_MODE;
			cfg_prt_packet.baudRate		= baudrate;
			cfg_prt_packet.inProtoMask	= UBX_CFG_PRT_PAYLOAD_INPROTOMASK;
			cfg_prt_packet.outProtoMask	= UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK;

			/* Calculate the checksum now */
			addChecksumToMessage((uint8_t*)&cfg_prt_packet, sizeof(cfg_prt_packet));

			/* Start with the two sync bytes */
			buffer[0] = UBX_SYNC1;
			buffer[1] = UBX_SYNC2;
			/* Copy it to the buffer that will be written back in the main gps driver */
			memcpy(&(buffer[2]), &cfg_prt_packet, sizeof(cfg_prt_packet));
			/* Set the length of the packet (plus the 2 sync bytes) */
			length = sizeof(cfg_prt_packet)+2;

		} else if (_config_state == UBX_CONFIG_STATE_PRT_NEW_BAUDRATE) {

			/* Send a CFG-PRT message again, this time change the baudrate */
			type_gps_bin_cfg_prt_packet_t cfg_prt_packet;
			memset(&cfg_prt_packet, 0, sizeof(cfg_prt_packet));

			cfg_prt_packet.clsID		= UBX_CLASS_CFG;
			cfg_prt_packet.msgID		= UBX_MESSAGE_CFG_PRT;
			cfg_prt_packet.length		= UBX_CFG_PRT_LENGTH;
			cfg_prt_packet.portID		= UBX_CFG_PRT_PAYLOAD_PORTID;
			cfg_prt_packet.mode			= UBX_CFG_PRT_PAYLOAD_MODE;
			cfg_prt_packet.baudRate		= UBX_CFG_PRT_PAYLOAD_BAUDRATE;
			cfg_prt_packet.inProtoMask	= UBX_CFG_PRT_PAYLOAD_INPROTOMASK;
			cfg_prt_packet.outProtoMask	= UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK;

			addChecksumToMessage((uint8_t*)&cfg_prt_packet, sizeof(cfg_prt_packet));

			buffer[0] = UBX_SYNC1;
			buffer[1] = UBX_SYNC2;
			memcpy(&(buffer[2]), &cfg_prt_packet, sizeof(cfg_prt_packet));
			length = sizeof(cfg_prt_packet)+2;

			/* If the new baudrate will be different from the current one, we should report that back to the driver */
			if (UBX_CFG_PRT_PAYLOAD_BAUDRATE != baudrate) {
				baudrate=UBX_CFG_PRT_PAYLOAD_BAUDRATE;
				baudrate_changed = true;
				/* Don't wait for an ACK, we're switching baudrate and we might never get,
				 * after that, start fresh */
				reset();
			}

		} else if (_config_state == UBX_CONFIG_STATE_RATE) {

			/* send a CFT-RATE message to define update rate */
			type_gps_bin_cfg_rate_packet_t cfg_rate_packet;
			memset(&cfg_rate_packet, 0, sizeof(cfg_rate_packet));

			cfg_rate_packet.clsID		= UBX_CLASS_CFG;
			cfg_rate_packet.msgID		= UBX_MESSAGE_CFG_RATE;
			cfg_rate_packet.length		= UBX_CFG_RATE_LENGTH;
			cfg_rate_packet.measRate	= UBX_CFG_RATE_PAYLOAD_MEASRATE;
			cfg_rate_packet.navRate		= UBX_CFG_RATE_PAYLOAD_NAVRATE;
			cfg_rate_packet.timeRef		= UBX_CFG_RATE_PAYLOAD_TIMEREF;

			addChecksumToMessage((uint8_t*)&cfg_rate_packet, sizeof(cfg_rate_packet));

			buffer[0] = UBX_SYNC1;
			buffer[1] = UBX_SYNC2;
			memcpy(&(buffer[2]), &cfg_rate_packet, sizeof(cfg_rate_packet));
			length = sizeof(cfg_rate_packet)+2;

		} else if (_config_state == UBX_CONFIG_STATE_NAV5) {
			/* send a NAV5 message to set the options for the internal filter */
			type_gps_bin_cfg_nav5_packet_t cfg_nav5_packet;
			memset(&cfg_nav5_packet, 0, sizeof(cfg_nav5_packet));

			cfg_nav5_packet.clsID        = UBX_CLASS_CFG;
			cfg_nav5_packet.msgID        = UBX_MESSAGE_CFG_NAV5;
			cfg_nav5_packet.length       = UBX_CFG_NAV5_LENGTH;
			cfg_nav5_packet.mask         = UBX_CFG_NAV5_PAYLOAD_MASK;
			cfg_nav5_packet.dynModel     = UBX_CFG_NAV5_PAYLOAD_DYNMODEL;
			cfg_nav5_packet.fixMode      = UBX_CFG_NAV5_PAYLOAD_FIXMODE;

			addChecksumToMessage((uint8_t*)&cfg_nav5_packet, sizeof(cfg_nav5_packet));

			buffer[0] = UBX_SYNC1;
			buffer[1] = UBX_SYNC2;
			memcpy(&(buffer[2]), &cfg_nav5_packet, sizeof(cfg_nav5_packet));
			length = sizeof(cfg_nav5_packet)+2;

		} else {
			/* Catch the remaining config states here, they all need the same packet type */

			type_gps_bin_cfg_msg_packet_t cfg_msg_packet;
			memset(&cfg_msg_packet, 0, sizeof(cfg_msg_packet));

			cfg_msg_packet.clsID        = UBX_CLASS_CFG;
			cfg_msg_packet.msgID        = UBX_MESSAGE_CFG_MSG;
			cfg_msg_packet.length       = UBX_CFG_MSG_LENGTH;
			/* Choose fast 5Hz rate for all messages except SVINFO which is big and not important */
			cfg_msg_packet.rate[1]      = UBX_CFG_MSG_PAYLOAD_RATE1_5HZ;

			switch (_config_state) {
				case UBX_CONFIG_STATE_MSG_NAV_POSLLH:
					cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
					cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_POSLLH;
					break;
				case UBX_CONFIG_STATE_MSG_NAV_TIMEUTC:
					cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
					cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_TIMEUTC;
					break;
				case UBX_CONFIG_STATE_MSG_NAV_DOP:
					cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
					cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_DOP;
					break;
				case UBX_CONFIG_STATE_MSG_NAV_SVINFO:
					cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
					cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_SVINFO;
					/* For satelites info 1Hz is enough */
					cfg_msg_packet.rate[1] = UBX_CFG_MSG_PAYLOAD_RATE1_1HZ;
					break;
				case UBX_CONFIG_STATE_MSG_NAV_SOL:
					cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
					cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_SOL;
					break;
				case UBX_CONFIG_STATE_MSG_NAV_VELNED:
					cfg_msg_packet.msgClass_payload = UBX_CLASS_NAV;
					cfg_msg_packet.msgID_payload = UBX_MESSAGE_NAV_VELNED;
					break;
//				case UBX_CONFIG_STATE_MSG_RXM_SVSI:
//					cfg_msg_packet.msgClass_payload = UBX_CLASS_RXM;
//					cfg_msg_packet.msgID_payload = UBX_MESSAGE_RXM_SVSI;
//					break;
				default:
					break;
			}

			addChecksumToMessage((uint8_t*)&cfg_msg_packet, sizeof(cfg_msg_packet));

			buffer[0] = UBX_SYNC1;
			buffer[1] = UBX_SYNC2;
			memcpy(&(buffer[2]), &cfg_msg_packet, sizeof(cfg_msg_packet));
			length = sizeof(cfg_msg_packet)+2;
		}
	}
}

int
UBX::parse(uint8_t b, struct vehicle_gps_position_s *gps_position)
{
	/* if no error happens and no new report is ready yet, ret will stay 0 */
	int ret = 0;

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
				decodeInit();
			}
			break;
		/* Now look for class */
		case UBX_DECODE_GOT_SYNC2:
			/* everything except sync1 and sync2 needs to be added to the checksum */
			addByteToChecksum(b);
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
					decodeInit();
					break;
			}
			break;
		case UBX_DECODE_GOT_CLASS:
			addByteToChecksum(b);
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

					case UBX_MESSAGE_NAV_DOP:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_DOP;
						break;

					case UBX_MESSAGE_NAV_SVINFO:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_SVINFO;
						break;

					case UBX_MESSAGE_NAV_VELNED:
						_decode_state = UBX_DECODE_GOT_MESSAGEID;
						_message_id = NAV_VELNED;
						break;

					default: //unknown class: reset state machine, should not happen
						decodeInit();
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
//						decodeInit();
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
						decodeInit();
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
						decodeInit();
						break;
					}
					break;
				default: //should not happen because we set the class
					warnx("UBX Error, we set a class that we don't know");
					decodeInit();
					ret = -1;
					break;
				}
				break;
			case UBX_DECODE_GOT_MESSAGEID:
				addByteToChecksum(b);
				_payload_size = b; //this is the first length byte
				_decode_state = UBX_DECODE_GOT_LENGTH1;
				break;
			case UBX_DECODE_GOT_LENGTH1:
				addByteToChecksum(b);
				_payload_size += b << 8; // here comes the second byte of length
				_decode_state = UBX_DECODE_GOT_LENGTH2;
				break;
			case UBX_DECODE_GOT_LENGTH2:
				/* Add to checksum if not yet at checksum byte */
				if (_rx_count < _payload_size)
					addByteToChecksum(b);
					_rx_buffer[_rx_count] = b;
					/* once the payload has arrived, we can process the information */
					if (_rx_count >= _payload_size + 1) { //+1 because of 2 checksum bytes
						switch (_message_id) { //this enum is unique for all ids --> no need to check the class
							case NAV_POSLLH: {
//								printf("GOT NAV_POSLLH MESSAGE\n");
								gps_bin_nav_posllh_packet_t *packet = (gps_bin_nav_posllh_packet_t *) _rx_buffer;

								//Check if checksum is valid and the store the gps information
								if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {
									gps_position->lat = packet->lat;
									gps_position->lon = packet->lon;
									gps_position->alt = packet->height_msl;

									gps_position->counter_pos_valid++;
									gps_position->counter++;

									_new_nav_posllh = true;

								} else {
									warnx("NAV_POSLLH: checksum invalid");
								}

								// Reset state machine to decode next packet
								decodeInit();
								break;
							}

							case NAV_SOL: {
//								printf("GOT NAV_SOL MESSAGE\n");
								gps_bin_nav_sol_packet_t *packet = (gps_bin_nav_sol_packet_t *) _rx_buffer;

								//Check if checksum is valid and the store the gps information
								if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {

									gps_position->fix_type = packet->gpsFix;

									gps_position->counter++;
									gps_position->s_variance = packet->sAcc;
									gps_position->p_variance = packet->pAcc;

									_new_nav_sol = true;

								} else {
									warnx("NAV_SOL: checksum invalid");
								}

								// Reset state machine to decode next packet
								decodeInit();
								break;
							}

							case NAV_DOP: {
//								printf("GOT NAV_DOP MESSAGE\n");
								gps_bin_nav_dop_packet_t *packet = (gps_bin_nav_dop_packet_t *) _rx_buffer;

								//Check if checksum is valid and the store the gps information
								if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {

									gps_position->eph =  packet->hDOP;
									gps_position->epv =  packet->vDOP;

									gps_position->counter++;

									_new_nav_dop = true;

								} else {
									warnx("NAV_DOP: checksum invalid");
								}

								// Reset state machine to decode next packet
								decodeInit();
								break;
							}

							case NAV_TIMEUTC: {
//								printf("GOT NAV_TIMEUTC MESSAGE\n");
								gps_bin_nav_timeutc_packet_t *packet = (gps_bin_nav_timeutc_packet_t *) _rx_buffer;

								//Check if checksum is valid and the store the gps information
								if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {
									//convert to unix timestamp
									struct tm timeinfo;
									timeinfo.tm_year = packet->year - 1900;
									timeinfo.tm_mon = packet->month - 1;
									timeinfo.tm_mday = packet->day;
									timeinfo.tm_hour = packet->hour;
									timeinfo.tm_min = packet->min;
									timeinfo.tm_sec = packet->sec;

									time_t epoch = mktime(&timeinfo);

									gps_position->time_gps_usec = (uint64_t)epoch * 1000000; //TODO: test this
									gps_position->time_gps_usec += (uint64_t)(packet->time_nanoseconds * 1e-3f);

									gps_position->counter++;

									_new_nav_timeutc = true;

								} else {
									warnx("NAV_TIMEUTC: checksum invalid");
								}

								// Reset state machine to decode next packet
								decodeInit();
								break;
							}

							case NAV_SVINFO: {
//								printf("GOT NAV_SVINFO MESSAGE\n");

								//this is a more complicated message: the length depends on the number of satellites. This number is extracted from the first part of the message
								const int length_part1 = 8;
								char _rx_buffer_part1[length_part1];
								memcpy(_rx_buffer_part1, _rx_buffer, length_part1);
								gps_bin_nav_svinfo_part1_packet_t *packet_part1 = (gps_bin_nav_svinfo_part1_packet_t *) _rx_buffer_part1;

								//read checksum
								const int length_part3 = 2;
								char _rx_buffer_part3[length_part3];
								memcpy(_rx_buffer_part3, &(_rx_buffer[_rx_count - 1]), length_part3);
								gps_bin_nav_svinfo_part3_packet_t *packet_part3 = (gps_bin_nav_svinfo_part3_packet_t *) _rx_buffer_part3;

								//Check if checksum is valid and then store the gps information
								if (_rx_ck_a == packet_part3->ck_a && _rx_ck_b == packet_part3->ck_b) {
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
										gps_position->satellite_prn[i] = packet_part2->svid;

										//if satellite information is healthy store the data
										uint8_t unhealthy = packet_part2->flags & 1 << 4; //flags is a bitfield

										if (!unhealthy) {
											if ((packet_part2->flags) & 1) { //flags is a bitfield
												gps_position->satellite_used[i] = 1;
												satellites_used++;

											} else {
												gps_position->satellite_used[i] = 0;
											}

											gps_position->satellite_snr[i] = packet_part2->cno;
											gps_position->satellite_elevation[i] = (uint8_t)(packet_part2->elev);
											gps_position->satellite_azimuth[i] = (uint8_t)((float)packet_part2->azim * 255.0f / 360.0f);

										} else {
											gps_position->satellite_used[i] = 0;
											gps_position->satellite_snr[i] = 0;
											gps_position->satellite_elevation[i] = 0;
											gps_position->satellite_azimuth[i] = 0;
										}

									}

									for (i = packet_part1->numCh; i < 20; i++) { //these channels are unused
										/* Unused channels have to be set to zero for e.g. MAVLink */
										gps_position->satellite_prn[i] = 0;
										gps_position->satellite_used[i] = 0;
										gps_position->satellite_snr[i] = 0;
										gps_position->satellite_elevation[i] = 0;
										gps_position->satellite_azimuth[i] = 0;
									}

									/* set flag if any sat info is available */
									if (!packet_part1->numCh > 0) {
										gps_position->satellite_info_available = 1;

									} else {
										gps_position->satellite_info_available = 0;
									}

									gps_position->satellites_visible = satellites_used++; // visible ~= used but we are interested in the used ones
									gps_position->counter++;

									// as this message arrives only with 1Hz and is not essential, we don't take it into account for the report

								} else {
									warnx("NAV_SVINFO: checksum invalid");
								}

								// Reset state machine to decode next packet
								decodeInit();
								break;
							}

							case NAV_VELNED: {
//								printf("GOT NAV_VELNED MESSAGE\n");
								gps_bin_nav_velned_packet_t *packet = (gps_bin_nav_velned_packet_t *) _rx_buffer;

								//Check if checksum is valid and the store the gps information
								if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {

									gps_position->vel = (uint16_t)packet->speed;
									gps_position->vel_n = packet->velN / 100.0f;
									gps_position->vel_e = packet->velE / 100.0f;
									gps_position->vel_d = packet->velD / 100.0f;
									gps_position->vel_ned_valid = true;
									gps_position->cog = (uint16_t)((float)(packet->heading) * 1e-3f);

									gps_position->counter++;

									_new_nav_velned = true;

								} else {
									warnx("NAV_VELNED: checksum invalid");
								}

								// Reset state machine to decode next packet
								decodeInit();
								break;
							}

//							case RXM_SVSI: {
//								printf("GOT RXM_SVSI MESSAGE\n");
//								const int length_part1 = 7;
//								char _rx_buffer_part1[length_part1];
//								memcpy(_rx_buffer_part1, _rx_buffer, length_part1);
//								gps_bin_rxm_svsi_packet_t *packet = (gps_bin_rxm_svsi_packet_t *) _rx_buffer_part1;
//
//								//Check if checksum is valid and the store the gps information
//								if (_rx_ck_a == _rx_buffer[_rx_count - 1] && _rx_ck_b == _rx_buffer[_rx_count]) {
//
//									gps_position->satellites_visible = packet->numVis;
//									gps_position->counter++;
//									_last_message_timestamps[RXM_SVSI - 1] = hrt_absolute_time();
//									ret = 1;
//
//								} else {
//									warnx("RXM_SVSI: checksum invalid\n");
//								}
//
//								// Reset state machine to decode next packet
//								decodeInit();
//								return ret;
//
//								break;
//							}

						case ACK_ACK: {
//							printf("GOT ACK_ACK\n");
							gps_bin_ack_ack_packet_t *packet = (gps_bin_ack_ack_packet_t *) _rx_buffer;

							//Check if checksum is valid
							if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {

								_waiting_for_ack = false;

								switch (_config_state) {
									case UBX_CONFIG_STATE_PRT:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_PRT)
											_config_state = UBX_CONFIG_STATE_PRT_NEW_BAUDRATE;
										break;
									case UBX_CONFIG_STATE_PRT_NEW_BAUDRATE:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_PRT)
											_config_state = UBX_CONFIG_STATE_RATE;
										break;
									case UBX_CONFIG_STATE_RATE:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_RATE)
											_config_state = UBX_CONFIG_STATE_NAV5;
										break;
									case UBX_CONFIG_STATE_NAV5:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_NAV5)
											_config_state = UBX_CONFIG_STATE_MSG_NAV_POSLLH;
										break;
									case UBX_CONFIG_STATE_MSG_NAV_POSLLH:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
											_config_state = UBX_CONFIG_STATE_MSG_NAV_TIMEUTC;
										break;
									case UBX_CONFIG_STATE_MSG_NAV_TIMEUTC:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
											_config_state = UBX_CONFIG_STATE_MSG_NAV_DOP;
										break;
									case UBX_CONFIG_STATE_MSG_NAV_DOP:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
											_config_state = UBX_CONFIG_STATE_MSG_NAV_SVINFO;
										break;
									case UBX_CONFIG_STATE_MSG_NAV_SVINFO:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
											_config_state = UBX_CONFIG_STATE_MSG_NAV_SOL;
										break;
									case UBX_CONFIG_STATE_MSG_NAV_SOL:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
											_config_state = UBX_CONFIG_STATE_MSG_NAV_VELNED;
										break;
									case UBX_CONFIG_STATE_MSG_NAV_VELNED:
										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
											_config_state = UBX_CONFIG_STATE_CONFIGURED;
										break;
//									case UBX_CONFIG_STATE_MSG_RXM_SVSI:
//										if (packet->clsID == UBX_CLASS_CFG && packet->msgID == UBX_MESSAGE_CFG_MSG)
//											_config_state = UBX_CONFIG_STATE_CONFIGURED;
//										break;
									default:
										break;
								}
							} else {
								warnx("ACK_ACK: checksum invalid");
							}

							// Reset state machine to decode next packet
							decodeInit();

							break;
						}

						case ACK_NAK: {
//							printf("GOT ACK_NAK\n");
							gps_bin_ack_nak_packet_t *packet = (gps_bin_ack_nak_packet_t *) _rx_buffer;

							//Check if checksum is valid
							if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {

								warnx("UBX: Received: Not Acknowledged");
								ret = 1;

							} else {
								warnx("ACK_NAK: checksum invalid\n");
							}

							// Reset state machine to decode next packet
							decodeInit();
							return ret;

							break;
						}

						default: //we don't know the message
							warnx("UBX: Unknown message received: %d-%d\n",_message_class,_message_id);
							decodeInit();

							break;
						}
					} // end if _rx_count high enough
					else if (_rx_count < RECV_BUFFER_SIZE) {
						_rx_count++;
					} else {
						warnx("buffer full, restarting");
						decodeInit();
						ret = -1;
					}
				break;
		default:
			break;
	}

	/* return 1 when position updates and the remaining packets updated at least once */
	if(_new_nav_posllh &&_new_nav_timeutc && _new_nav_dop && _new_nav_sol && _new_nav_velned) {

		/* Add timestamp to finish the report */
		gps_position->timestamp = hrt_absolute_time();
		/* Reset the flags */

		/* update on every position change, accept minor delay on other measurements */
		_new_nav_posllh = false;
		// _new_nav_timeutc = false;
		// _new_nav_dop = false;
		// _new_nav_sol = false;
		// _new_nav_velned = false;

		ret = 1;
	}

	return ret;
}

void
UBX::decodeInit(void)
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
UBX::addByteToChecksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
UBX::addChecksumToMessage(uint8_t* message, const unsigned length)
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
