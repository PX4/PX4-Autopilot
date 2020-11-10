/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#pragma once

/* The modern flash controllers manage flash for ECC
 * On the H7 there is a block size (256) bits
 * This code translates between the bootloaders word read/write
 * interface and the H7 need for block based writes.
 *
 * All writes* are buffered until 8 word can be written.
 * Verification is made from the cache. On the verification of the
 * 8th word all the wordfs in the the cache are written to flash.
 * If the write fails the value returned for the last word is negated
 * to ensure an error on verification.
 *
 * *writes to the first 8 words of flash at APP_LOAD_ADDRESS
 * are buffered until the "first word" is written with the real value (not 0xffffffff)
 *
 */

#define FC_NUMBER_LINES  2                                  // Number of cache lines.
#define FC_NUMBER_WORDS  8                                  // Number of words per cache line.
#define FC_LAST_WORD     FC_NUMBER_WORDS-1                  // The index of the last word in cache line.
#define FC_ADDRESS_MASK  ~(sizeof(flash_cache[0].words)-1)  // Cache tag from address
#define FC_ADDR2INDX(a)   (((a) / sizeof(flash_cache[0].words[0])) % FC_NUMBER_WORDS) // index from address
#define FC_CLEAN  ((uint32_t)-1)                            // Cache clean

// Cache line
typedef struct flash_cache_line_t {
	uint32_t index;                   // Index of word written
	uint32_t start_address;           // cache tag (address in FLASH this is buffering)
	uint32_t words[FC_NUMBER_WORDS];  // Buffered data
} flash_cache_line_t;

// Resets the cache - all lines flashed and cache_line[0] start_address == APP_LOAD_ADDRESS
void fc_reset(void);
// Cache operations
uint32_t fc_read(uint32_t address);
int fc_write(uint32_t address, uint32_t word);
