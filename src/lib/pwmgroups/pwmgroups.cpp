/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file pwmgroups.cpp
 *
 * Programmable multi-channel pwm library.
 */

#include <cstring>
#include <ctype.h>

#include "pwmgroups.h"
#include <drivers/drv_pwm_output.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

uint8_t PwmGroups::all_timers = 0;
uint32_t PwmGroups::all_channels = 0;
uint32_t PwmGroups::default_rate_channels = 0;
uint32_t PwmGroups::alt_rate_channels = 0;

PwmGroups::PwmGroups(const char *name, const char *devname) :
	CDev(name, devname)
{

}

/**
 * Return the timer map under current working mode.
 *
 *
 * @return            timer map of current working mode.
 */
uint8_t
PwmGroups::working_timer_map (void) {
    return 0;
}

/**
 * Return the channel map under current working mode.
 *
 * @return            channel map of current working mode.
 */
uint32_t
PwmGroups::working_channel_map (void) {
    return _working_channel_map;
}

uint8_t
PwmGroups::group_channel_map_offset(uint32_t channel_map) {
    //calculate channel map offset;
    //for static pmw device only?
    uint8_t offset = 0;
    if (channel_map == 0) {
        //invalid map;
        return 0xFF;
    }
    
    while ((channel_map & 1) == 0) {
        offset ++;
        channel_map >>= 1;
    }
    return offset;
}

uint8_t
PwmGroups::group_timer_map_offset(uint8_t timer_map) {
    //calculate channel map offset;
    //for static pmw device only?
    uint8_t offset = 0;
    if (timer_map == 0) {
        //invalid map;
        return 0xFF;
    }
    while ((timer_map & 1) == 0) {
        offset ++;
        timer_map >>= 1;
    }
    return offset;
}

/**
 * @brief initialize all channels in global channel map.
 *
 * @return the number of outputs this mixer feeds to
 */
int
PwmGroups::register_working_channels() {
    uint32_t res = _working_channel_map & all_channels;
    if (res != 0) {
        //at least one channel overlap, failed.
        return -EINVAL;

    }
    
    //free channel available, check conflicts on rate;
    //select working rate;
    uint16_t working_rate;
    if ( _default_rate <= _working_pwm_rate_up_limit && _default_rate >= _working_pwm_rate_low_limit ) {
        working_rate = _default_rate;
    }
    else if (_alt_rate <= _working_pwm_rate_up_limit && _alt_rate >= _working_pwm_rate_low_limit) {
        working_rate = _alt_rate;
    }
    else {
        //illegal working rate;
        return -EINVAL;
    }
    
    if (working_rate == _default_rate) {
        res = _channel_rate_map & alt_rate_channels;
    }
    else {
        res = _channel_rate_map & default_rate_channels;
    }
    
    if (res != 0) {
        //rate conflicts;
        return -EINVAL;
    }
    
    //no conflicts, allocate the channel
    all_channels |= _working_channel_map;
    printf("[pwmgroups] registered channels: 0x%04X\n", all_channels);
    return OK;
}


int
PwmGroups::set_pwm_rate_map (uint32_t device_rate_map) {
    //device rate map -> global channel rate map;
    _channel_rate_map = device_rate_map << _channel_map_offset;
    return OK;
}


/**
 * Analyses the mix configuration and updates a bitmask of groups
 * that are required.
 *
 * @param groups        A bitmask of groups (0-31) that the mixer requires.
 */
int
PwmGroups::set_pwm_channel_rates (uint8_t alt_rate, uint8_t default_rate) {
    //device rate map -> global channel rate map;
    //_channel_rate_map = device_rate_map << _channel_map_offset;
    return OK;
}

/**
 * @brief      Empty method, only implemented for MultirotorMixer and MixerGroup class.
 *
 * @param[in]  delta_out_max  Maximum delta output.
 *
 */
int
PwmGroups::set_pwm_channel_value_single (uint8_t device_channel_idx, uint8_t value) {
    device_channel_idx += _channel_map_offset;
    up_pwm_servo_set(device_channel_idx, value);
    return OK;
}

