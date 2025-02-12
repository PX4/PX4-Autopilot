/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

// System keys
#define KEY_FORMAT_VERSION "fmv"
#define KEY_INIT_TIMESTAMP "its"
#define KEY_BOOT_CYCLES "btc"
#define KEY_ON_DURATION "ond"
#define KEY_ARM_CYCLES "arc"

// Flight history keys
#define KEY_FLIGHT_TOTAL_DURATION "tfd"
#define KEY_FLIGHT_HIGH_ALTITUDE "had"
#define KEY_FLIGHT_MEDIUM_ALTITUDE "mad"
#define KEY_FLIGHT_LOW_ALTITUDE "lad"
#define KEY_FLIGHT_HIGH_AMBIENT_TEMP "htd"
#define KEY_FLIGHT_LOW_AMBIENT_TEMP "ltd"

// Structural history keys
#define KEY_STRUCTURAL_HIGH_VIBRATION "hvd"
#define KEY_STRUCTURAL_MODERATE_VIBRATION "mvd"
#define KEY_STRUCTURAL_LOW_VIBRATION "lvd"

// ESC temperature keys
#define KEY_ESC_TEMP_MAX "ext"
#define KEY_ESC_TEMP_WARN_COUNT "ewc"
#define KEY_ESC_TEMP_MAX_COUNT "emc"
#define KEY_ESC_TEMP_WARN_DURATION "ewd"
#define KEY_ESC_TEMP_MAX_DURATION "emd"

// Motor temperature keys
#define KEY_MOTOR_TEMP_MAX "mxt"
#define KEY_MOTOR_TEMP_WARN_COUNT "mwc"
#define KEY_MOTOR_TEMP_MAX_COUNT "mmc"
#define KEY_MOTOR_TEMP_WARN_DURATION "mwd"
#define KEY_MOTOR_TEMP_MAX_DURATION "mmd"

// Electrical keys
#define KEY_ELECTRICAL_MAX_VOLTAGE "mxv"
#define KEY_ELECTRICAL_MIN_VOLTAGE "mnv"
#define KEY_ELECTRICAL_MAX_CURRENT "mxc"
#define KEY_ELECTRICAL_MIN_CURRENT "mnc"
#define KEY_ELECTRICAL_WARN_CURRENT_COUNT "wcc"
#define KEY_ELECTRICAL_MAX_CURRENT_COUNT "mcc"
#define KEY_ELECTRICAL_WARN_CURRENT_DURATION "wcd"
#define KEY_ELECTRICAL_MAX_CURRENT_DURATION "mcd"

// RPM keys
#define KEY_RPM_TOTAL_REVOLUTIONS "trv"
#define KEY_RPM_MAX "mrp"
#define KEY_RPM_MEDIUM_DURATION "mrd"
#define KEY_RPM_HIGH_DURATION "hrd"

// Prop dynamics keys
#define KEY_PROP_MODERATE_FLOP_DURATION "mfd"
#define KEY_PROP_SEVERE_FLOP_DURATION "sfd"
#define KEY_PROP_MODERATE_LIFT_ASYMMETRY "mld"
#define KEY_PROP_SEVERE_LIFT_ASYMMETRY "sld"
