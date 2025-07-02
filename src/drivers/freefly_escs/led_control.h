/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;


class LedControl : public ModuleParams
{
public:
	static constexpr int8_t MOTOR_COUNT{4};
	LedControl(ModuleParams *parent);
	virtual ~LedControl() = default;
	void init(const int file_descriptor) { _file_descriptor = file_descriptor; }
	void updateLeds(bool motors_armed, uint8_t online_esc_mask);

private:
	void updateParams() override;

	static constexpr hrt_abstime LED_BLINK_SPEED_SLOW{500_ms};
	static constexpr hrt_abstime LED_BLINK_SPEED_FAST{200_ms};

	enum class BoomColor {
		Off = 0,
		Red,
		Orange,
		Yellow,
		Green,
		Cyan,
		Blue,
		Purple,
		White,
		RgbWheel
	};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionMultiArray<battery_status_s> _battery_status_subs{ORB_ID::battery_status};
	uORB::Subscription _failure_detector_status_sub{ORB_ID(failure_detector_status)};
	uORB::Subscription _health_report_sub{ORB_ID(health_report)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	void updateSubscriptions();
	bool _battery_low{false}; // Sticky until disarm
	bool _kill_switch_engaged{false};
	bool _battery_fault{false}; // Sticky until disarm
	bool _mag_unhealthy{false}; // Sticky until disarm
	bool _failure_detector_failure{false}; // Sticky until disarm
	bool _arming_state_high{false}; // Sticky until disarm

	/**
	 * If constantly called it blinks all LEDs
	 * according to the parameter configuration and in full brightness
	 * @param brightness to be able to dimm LED [0, 1]
	 */
	void blinkLeds(hrt_abstime blink_mode);
	bool _blink_state_on{false};
	hrt_abstime _last_blink_time{0};

	/**
	 * Set All LED colors according to the parameter configuration
	 * @param brightness to be able to dimm LED [0, 1]
	 */
	void setLedsToParameterColors(float brightness = 1.f);
	BoomColor _led_colors_from_parameters[4];

	/**
	 * Set All LEDs to the same color
	 * @param color one of @see BoomColor
	 * @param brightness to be able to dimm LED [0, 1]
	 */
	void setLedsToColor(BoomColor color, float brightness = 1.f);

	/**
	 * Set all LEDs to turn off
	 */
	void setLedsOff();

	/**
	 * Update an LED and set a new color in an RGB color wheel
	 * @param esc_id 0 based number of the ESC [0,MOTOR_COUNT-1]
	 */
	void updateRgbWheel(uint8_t esc_id);
	int _rgb_counter{0};

	/**
	 * Set RGB LED color and brighness of one ESC
	 * @param esc_id 0 based number of the ESC [0,MOTOR_COUNT-1]
	 * @param color one of @see BoomColor
	 * @param brightness to be able to dimm LED [0, 1]
	 */
	void setLedColor(uint8_t esc_id, BoomColor color, float brightness = 1.f);

	/**
	 * Set RGB LED lighting of one ESC to be sent out at the right time
	 * @param esc_id 0 based number of the ESC [0,MOTOR_COUNT-1]
	 * @param red brightness of that color [0,255]
	 * @param green brightness of that color [0,255]
	 * @param blue brightness of that color [0,255]
	 */
	void setLedRgb(uint8_t esc_id, uint8_t red, uint8_t green, uint8_t blue);
	uint8_t _led_color_sent_mask{0}; ///< Mask of LEDs that are up to date and received their latest RGB values
	uint8_t _led_rgb_state[MOTOR_COUNT][3] {}; ///< current state of RGB values for all LEDs TODO: MOTOR_COUNT array size

	/**
	 * Send out RGB values for LED lighting of one ESC through CAN
	 * Don't use this function directly to change the LED state!
	 * Use setLed...() functions instead and let the update send it out.
	 * @param esc_id 0 based number of the ESC [0,MOTOR_COUNT-1]
	 * @param red brightness of that color [0,255]
	 * @param green brightness of that color [0,255]
	 * @param blue brightness of that color [0,255]
	 */
	void sendLedRgbCanMessage(uint8_t esc_id, uint8_t red, uint8_t green, uint8_t blue);

	int send_can_msg(const struct can_msg_s &msg_p);
	int _file_descriptor{-1};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOOM_BRT>) _param_boom_brt,
		(ParamInt<px4::params::BOOM1_COLOR>) _param_boom_color_1,
		(ParamInt<px4::params::BOOM2_COLOR>) _param_boom_color_2,
		(ParamInt<px4::params::BOOM3_COLOR>) _param_boom_color_3,
		(ParamInt<px4::params::BOOM4_COLOR>) _param_boom_color_4
	)
};
