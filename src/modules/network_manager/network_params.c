/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file network_params.c
 * Parameter definition for network.
 *
 */


/**
 * Board Unique MAC Address byte #0
 * Address is read by MAC EEPROM from the board and should be not modified by user
 *
 * @group Ethernet
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(ETH_MAC_ADDR_0, 0);

/**
 * Board Unique MAC Address byte #1
 * Address is read by MAC EEPROM from the board and should be not modified by user
 *
 * @group Ethernet
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(ETH_MAC_ADDR_1, 0);

/**
 * Board Unique MAC Address byte #2
 * Address is read by MAC EEPROM from the board and should be not modified by user
 *
 * @group Ethernet
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(ETH_MAC_ADDR_2, 0);

/**
 * Board Unique MAC Address byte #3
 * Address is read by MAC EEPROM from the board and should be not modified by user
 *
 * @group Ethernet
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(ETH_MAC_ADDR_3, 0);

/**
 * Board Unique MAC Address byte #4
 * Address is read by MAC EEPROM from the board and should be not modified by user
 *
 * @group Ethernet
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(ETH_MAC_ADDR_4, 0);

/**
 * Board Unique MAC Address byte #5
 * Address is read by MAC EEPROM from the board and should be not modified by user
 *
 * @group Ethernet
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(ETH_MAC_ADDR_5, 0);

/**
 * Board IP Address
 *
 * @group Ethernet
 *
 */
PARAM_DEFINE_INT32(ETH_IP_ADDR, -1062731775);

/**
 * Board NETMASK Address byte #0
 *
 * @group Ethernet
 * @min 0
 * @max 32
 */
PARAM_DEFINE_INT32(ETH_SUB_PREF_S, 24);

/**
 * Board GATEWAY Address
 *
 * @group Ethernet
 *
 */
PARAM_DEFINE_INT32(ETH_GATEWAY, 0);
