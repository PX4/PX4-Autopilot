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

/**
 * @file syslink_params.c
 *
 * Parameters defined by the syslink module and the exposed NRF51 radio
 *
 * @author Dennis Shtatnov <densht@gmail.com>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>


/**
 * Operating channel of the NRF51
 *
 * @min 0
 * @max 125
 * @group Syslink
 */
PARAM_DEFINE_INT32(SLNK_RADIO_CHAN, 80);

/**
 * Operating datarate of the NRF51
 *
 * @min 0
 * @max 2
 * @group Syslink
 */
PARAM_DEFINE_INT32(SLNK_RADIO_RATE, 2);

/**
 * Operating address of the NRF51 (most significant byte)
 *
 * @group Syslink
 */
PARAM_DEFINE_INT32(SLNK_RADIO_ADDR1, 231); // 0xE7

/**
 * Operating address of the NRF51 (least significant 4 bytes)
 *
 * @group Syslink
 */
PARAM_DEFINE_INT32(SLNK_RADIO_ADDR2, 3890735079); // 0xE7E7E7E7
