/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file bar100_registers.h
 *
 * Shared defines for the BAR100 driver.
 */

#pragma once

// I2C address for BAR100 sensor
#define BAR100_I2C_ADDR      0x40

// Command to request measurement
#define BAR100_REQUEST       0xAC

// Register addresses for memory map (example, adjust as needed)
#define BAR100_REG_CUST_ID0  0x00
#define BAR100_REG_CUST_ID1  0x01
#define BAR100_REG_SCALING0  0x12
#define BAR100_REG_SCALING1  0x13
#define BAR100_REG_SCALING2  0x14
#define BAR100_REG_SCALING3  0x15
#define BAR100_REG_SCALING4  0x16

// Conversion interval in microseconds (20ms)
#define BAR100_CONVERSION_INTERVAL_US 20000

// Number of registers in memory map
#define BAR100_NUM_REGISTERS 7

// Structure for holding memory map values (optional)
struct bar100_memory_map_t {
    uint16_t cust_id0;
    uint16_t cust_id1;
    uint16_t scaling0;
    uint16_t scaling1;
    uint16_t scaling2;
    uint16_t scaling3;
    uint16_t scaling4;
};
