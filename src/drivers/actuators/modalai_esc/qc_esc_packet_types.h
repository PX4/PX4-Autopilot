/****************************************************************************
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
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
 * 3. Neither the name The Linux Foundation nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE.
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
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 *
 ****************************************************************************/

/*
 * This file contains the type values of all supported ESC UART packets
 */

#ifndef QC_ESC_PACKET_TYPES
#define QC_ESC_PACKET_TYPES

#define ESC_PACKET_TYPE_VERSION_REQUEST 0
#define ESC_PACKET_TYPE_PWM_CMD         1
#define ESC_PACKET_TYPE_RPM_CMD         2
#define ESC_PACKET_TYPE_SOUND_CMD       3
#define ESC_PACKET_TYPE_STEP_CMD        4
#define ESC_PACKET_TYPE_LED_CMD         5
#define ESC_PACKET_TYPE_RESET_CMD       10
#define ESC_PACKET_TYPE_SET_ID_CMD      11
#define ESC_PACKET_TYPE_SET_DIR_CMD     12
#define ESC_PACKET_TYPE_CONFIG_BOARD_REQUEST  20
#define ESC_PACKET_TYPE_CONFIG_USER_REQUEST   21
#define ESC_PACKET_TYPE_CONFIG_UART_REQUEST   22
#define ESC_PACKET_TYPE_CONFIG_TUNE_REQUEST   23
#define ESC_PACKET_TYPE_VERSION_EXT_REQUEST   24

#define ESC_PACKET_TYPE_SET_FEEDBACK_MODE 50   //reserved for future use

#define ESC_PACKET_TYPE_EEPROM_WRITE_UNLOCK 70
#define ESC_PACKET_TYPE_EEPROM_READ_UNLOCK  71
#define ESC_PACKET_TYPE_EEPROM_WRITE  72

#define ESC_PACKET_TYPE_VERSION_RESPONSE 109
#define ESC_PACKET_TYPE_PARAMS           110
#define ESC_PACKET_TYPE_BOARD_CONFIG     111
#define ESC_PACKET_TYPE_USER_CONFIG      112
#define ESC_PACKET_TYPE_UART_CONFIG      113
#define ESC_PACKET_TYPE_TUNE_CONFIG      114
#define ESC_PACKET_TYPE_FB_RESPONSE      128
#define ESC_PACKET_TYPE_VERSION_EXT_RESPONSE 131

#endif
