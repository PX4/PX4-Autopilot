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

#include "ESCSettingsInterface.h"
#include <uORB/Publication.hpp>
#include <uORB/topics/am32_eeprom_read.h>

class AM32Settings : public ESCSettingsInterface
{
public:
	AM32Settings(int index);

	struct EEPROMData {
		uint8_t eeprom_start;           // 0: must be 1
		uint8_t eeprom_version;         // 1: version 0-255
		uint8_t bootloader_version;     // 2: bootloader version 0-255
		uint8_t firmware_major;         // 3: firmware version major
		uint8_t firmware_minor;         // 4: firmware version minor
		uint8_t max_ramp_speed;         // 5: value/10 percent per ms (default 160 = 16%/ms)
		uint8_t min_duty_cycle;         // 6: value/2 (default 4 = 2%)
		uint8_t stick_calibration;      // 7: disable stick calibration (default 0)
		uint8_t voltage_cutoff;         // 8: absolute voltage cutoff (default 10)
		uint8_t current_pid_p;          // 9: P value x2 (default 100 = 200)
		uint8_t current_pid_i;          // 10: I value (default 0)
		uint8_t current_pid_d;          // 11: D value x10 (default 50 = 500)
		uint8_t active_brake_power;     // 12: active brake power
		uint8_t reserved[4];            // 13-16: reserved bytes
		uint8_t direction_reversed;     // 17: direction reversed
		uint8_t bidirectional_mode;     // 18: bidirectional mode (1=on, 0=off)
		uint8_t sinusoidal_startup;     // 19: sinusoidal startup
		uint8_t complementary_pwm;      // 20: complementary PWM
		uint8_t variable_pwm_freq;      // 21: variable PWM frequency
		uint8_t stuck_rotor_protection; // 22: stuck rotor protection
		uint8_t timing_advance;         // 23: timing advance x0.9375 (16 = 15 degrees)
		uint8_t pwm_frequency;          // 24: PWM freq in kHz (default 24)
		uint8_t startup_power;          // 25: startup power 50-150% (default 100)
		uint8_t motor_kv;               // 26: KV in increments of 40 (55 = 2200kv)
		uint8_t motor_poles;            // 27: motor poles (default 14)
		uint8_t brake_on_stop;          // 28: brake on stop (default 0)
		uint8_t anti_stall;             // 29: anti-stall protection
		uint8_t beep_volume;            // 30: beep volume 0-11 (default 5)
		uint8_t telemetry_30ms;         // 31: 30ms telemetry output (0 or 1)
		uint8_t servo_low;              // 32: servo low (value*2)+750us
		uint8_t servo_high;             // 33: servo high (value*2)+1750us
		uint8_t servo_neutral;          // 34: servo neutral 1374+value us (128=1500us)
		uint8_t servo_deadband;         // 35: servo deadband 0-100
		uint8_t low_voltage_cutoff;     // 36: low voltage cutoff
		uint8_t low_voltage_threshold;  // 37: threshold value+250/10V (50=3.0V)
		uint8_t rc_car_reversing;       // 38: RC car type reversing (default 0)
		uint8_t hall_sensors;           // 39: hall sensor options
		uint8_t sine_mode_range;        // 40: sine mode range 5-25% (default 15)
		uint8_t drag_brake_strength;    // 41: drag brake 1-10 (default 10)
		uint8_t running_brake_amount;   // 42: brake when running (default 10)
		uint8_t temperature_limit;      // 43: temp limit 70-140C (141=disabled)
		uint8_t current_protection;     // 44: current limit value x2 (102=disabled)
		uint8_t sine_mode_strength;     // 45: sine mode strength 1-10 (default 6)
		uint8_t input_type;             // 46: input type selector
		uint8_t auto_timing;            // 47: auto timing
	} __attribute__((packed));

	int getExpectedResponseSize() override;
	bool decodeInfoResponse(const uint8_t *buf, int size) override;

	void publish_latest() override;

private:
	int _esc_index{};
	EEPROMData _eeprom_data{};

	static uORB::Publication<am32_eeprom_read_s> _am32_eeprom_read_pub;
};
