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
 * This file contains function prototypes for crc16 computations using polynomial 0x8005
 */

#ifndef CRC16_H_
#define CRC16_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

// Returns the seed of crc output, which should be used when computing crc16 of a byte sequence
uint16_t voxl2_io_crc16_init(void);

// Process one byte by providing crc16 from previous step and new byte to consume.
// Output is the new crc16 value
uint16_t voxl2_io_crc16_byte(uint16_t prev_crc, const uint8_t new_byte);

// Process an array of bytes by providing crc16 from previous step (or seed), array of bytes and its length
// Output is the new crc16 value
uint16_t voxl2_io_crc16(uint16_t prev_crc, uint8_t const *input_buffer, uint16_t input_length);

#ifdef __cplusplus
}
#endif

#endif //CRC16_H_
