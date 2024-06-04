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

/* special symbols (the chip comes preflashed with these) */

#define OSD_SYMBOL_RSSI                    0x01
// Throttle Position (%)
#define OSD_SYMBOL_THR                     0x04
#define OSD_SYMBOL_THR1                    0x05
// Map mode
#define OSD_SYMBOL_HOME                    0x04
#define OSD_SYMBOL_AIRCRAFT                0x05
// Unit Icon (Metric)
#define OSD_SYMBOL_M                       0x0C
// Unit Icon (Imperial)
#define OSD_SYMBOL_FT                      0x0F
// Heading Graphics
#define OSD_SYMBOL_HEADING_N               0x18
#define OSD_SYMBOL_HEADING_S               0x19
#define OSD_SYMBOL_HEADING_E               0x1A
#define OSD_SYMBOL_HEADING_W               0x1B
#define OSD_SYMBOL_HEADING_DIVIDED_LINE    0x1C
#define OSD_SYMBOL_HEADING_LINE            0x1D
// Artificial Horizon Center screen Graphics
#define OSD_SYMBOL_AH_CENTER_LINE          0x26
#define OSD_SYMBOL_AH_CENTER_LINE_RIGHT    0x27
#define OSD_SYMBOL_AH_CENTER               0x7E
#define OSD_SYMBOL_AH_RIGHT                0x02
#define OSD_SYMBOL_AH_LEFT                 0x03
#define OSD_SYMBOL_AH_DECORATION           0x13
// Satellite Graphics
#define OSD_SYMBOL_SAT_L                   0x1E
#define OSD_SYMBOL_SAT_R                   0x1F
// Direction arrows
#define OSD_SYMBOL_ARROW_SOUTH             0x60
#define OSD_SYMBOL_ARROW_2                 0x61
#define OSD_SYMBOL_ARROW_3                 0x62
#define OSD_SYMBOL_ARROW_4                 0x63
#define OSD_SYMBOL_ARROW_EAST              0x64
#define OSD_SYMBOL_ARROW_6                 0x65
#define OSD_SYMBOL_ARROW_7                 0x66
#define OSD_SYMBOL_ARROW_8                 0x67
#define OSD_SYMBOL_ARROW_NORTH             0x68
#define OSD_SYMBOL_ARROW_10                0x69
#define OSD_SYMBOL_ARROW_11                0x6A
#define OSD_SYMBOL_ARROW_12                0x6B
#define OSD_SYMBOL_ARROW_WEST              0x6C
#define OSD_SYMBOL_ARROW_14                0x6D
#define OSD_SYMBOL_ARROW_15                0x6E
#define OSD_SYMBOL_ARROW_16                0x6F
// Artifical Horizon Bars
#define OSD_SYMBOL_AH_BAR9_0               0x80
#define OSD_SYMBOL_AH_BAR9_1               0x81
#define OSD_SYMBOL_AH_BAR9_2               0x82
#define OSD_SYMBOL_AH_BAR9_3               0x83
#define OSD_SYMBOL_AH_BAR9_4               0x84
#define OSD_SYMBOL_AH_BAR9_5               0x85
#define OSD_SYMBOL_AH_BAR9_6               0x86
#define OSD_SYMBOL_AH_BAR9_7               0x87
#define OSD_SYMBOL_AH_BAR9_8               0x88
// Progress bar
#define OSD_SYMBOL_PB_START                0x8A
#define OSD_SYMBOL_PB_FULL                 0x8B
#define OSD_SYMBOL_PB_HALF                 0x8C
#define OSD_SYMBOL_PB_EMPTY                0x8D
#define OSD_SYMBOL_PB_END                  0x8E
#define OSD_SYMBOL_PB_CLOSE                0x8F
// Batt evolution
#define OSD_SYMBOL_BATT_FULL               0x90
#define OSD_SYMBOL_BATT_5                  0x91
#define OSD_SYMBOL_BATT_4                  0x92
#define OSD_SYMBOL_BATT_3                  0x93
#define OSD_SYMBOL_BATT_2                  0x94
#define OSD_SYMBOL_BATT_1                  0x95
#define OSD_SYMBOL_BATT_EMPTY              0x96
// Batt Icon
#define OSD_SYMBOL_MAIN_BATT               0x97
// Voltage and amperage
#define OSD_SYMBOL_VOLT                    0x06
#define OSD_SYMBOL_AMP                     0x9A
#define OSD_SYMBOL_MAH                     0x07
#define OSD_SYMBOL_WATT                    0x57
// Time
#define OSD_SYMBOL_ON_M                    0x9B
#define OSD_SYMBOL_FLY_M                   0x9C
#define OSD_SYMBOL_FLIGHT_TIME             0x70
