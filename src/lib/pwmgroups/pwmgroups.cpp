/****************************************************************************
 *
 *   Copyright (C) 2018 AirMind Development Team. All rights reserved.
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
 * 3. Neither the name AirMind nor the names of its contributors may be
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
 * Pwm device resource coordination library.
 *
 * @Author: ning.roland@mindpx.net (or ning.roland@gmail.com)
 *
 */

#include <cstring>
#include <ctype.h>

#include "pwmgroups.h"
#include <drivers/drv_pwm_output.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

uint8_t PwmGroups::all_timers = 0;
uint32_t PwmGroups::all_channels = 0;
uint32_t PwmGroups::_default_rate_channels = 0;
uint32_t PwmGroups::_alt_rate_channels = 0;
uint8_t PwmGroups::_alt_rate_timers = 0;
uint8_t PwmGroups::_default_rate_timers = 0;

PwmGroups::PwmGroups(const char *name, const char *devname) :
	CDev(devname)
{
    _default_rate = PWM_OUTPUT_RATE_DEFAULT;
    _alt_rate = PWM_OUTPUT_RATE_ALT;
    _working_rate = PWM_GROUP_RATE_UNSPECIFIED;
    _capture_channel_map = 0;
    _working_channel_map = 0;
    _channel_rate_map = 0;

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
    //for static PWM device only?
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
    //for static PWM device only?
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
    if (_working_rate == PWM_GROUP_RATE_MIXED) {
        //both alt rate and default rate at work.
        uint32_t default_rate_channel_map = (~_channel_rate_map) & _working_channel_map;
        if (!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_ALT)
            || !is_free_channels_available(default_rate_channel_map, PWM_GROUP_RATE_DEFAULT)) {
            return -EINVAL;
        }
        else {
            all_channels |= _working_channel_map;
            update_default_alt_rate_map (PWM_GROUP_RATE_ALT, _channel_rate_map);
            update_default_alt_rate_map (PWM_GROUP_RATE_DEFAULT, default_rate_channel_map);
            printf("[pwmgroups] mixed rate, registered channels all: 0x%04X\n", all_channels);
            return OK;
        }
    }
    
    //obligation to use specified rate;
    //TODO: static pwm device needs to specify working rate on start. move to subclass
    if (_working_rate != PWM_GROUP_RATE_UNSPECIFIED) {
        printf("[pwmgroups] use specified pwm rate....\n");

        if (_working_rate == PWM_GROUP_RATE_ALT) {
            if (!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_ALT)) {
                return -EINVAL;
            }
        }
        else {
            if (!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_DEFAULT)) {
                return -EINVAL;
            }

        }
        all_channels |= _working_channel_map;
        update_default_alt_rate_map (_working_rate, _working_channel_map);
        printf("[pwmgroups] registered channels all: 0x%04X\n", all_channels);
        return OK;
        
    }
    
    //select a working rate;
    //TODO: floating pwm device can select a rate on start. move to subclass
    if (_default_rate >= _working_pwm_rate_low_limit && _alt_rate <= _working_pwm_rate_up_limit) {
        if (!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_DEFAULT)) {
            if (!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_ALT)) {
                return -EINVAL;
            }
            else {
                _working_rate = PWM_GROUP_RATE_ALT;
            }
        }
        else {
            _working_rate = PWM_GROUP_RATE_DEFAULT;
        }
    }
    else {
        if ( _default_rate <= _working_pwm_rate_up_limit && _default_rate >= _working_pwm_rate_low_limit ) {
            
            _working_rate = PWM_GROUP_RATE_DEFAULT;
            //check rate conflicts;
            if(!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_DEFAULT)) {
                return -EINVAL;
            }
        }
        else if (_alt_rate <= _working_pwm_rate_up_limit && _alt_rate >= _working_pwm_rate_low_limit) {
            
            _working_rate = PWM_GROUP_RATE_ALT;
            //check rate conflicts;
            if(!is_free_channels_available(_channel_rate_map, PWM_GROUP_RATE_ALT)) {
                return -EINVAL;
            }
        }
        else {
            //illegal working rate;
            return -EINVAL;
        }
    }
    
    //no conflicts, allocate requested channels
    all_channels |= _working_channel_map;
    update_default_alt_rate_map (_working_rate, _channel_rate_map);
    printf("[pwmgroups] registered channels all: 0x%04X\n", all_channels);
    return OK;
}

void
PwmGroups::update_default_alt_rate_map (pwm_groups_rate_t rate_type, uint32_t channel_map) {
    //TODO: add critical section protection;
    if (rate_type == PWM_GROUP_RATE_ALT) {
        _alt_rate_channels |= channel_map;
        //remove corresponding bit in opposite rate map if its there;
        //_default_rate_channels ^= channel_map;

        for (uint8_t i=_channel_map_offset; i<_channel_map_offset+PWM_OUTPUT_MAX_CHANNELS; i++) {
            if (((1 << i) & _alt_rate_channels) != 0) {
                //printf("[pwmgroups] group for channel %d : %d\n", i, up_pwm_servo_get_group_of_channel(i));
                _alt_rate_timers |= (1 << up_pwm_servo_get_group_of_channel(i));
                //delete corresponding bit in opposite rate map;
                
            }
        }
    }
    else {
        printf("[pwmgroups] selected default rate\n");
        _default_rate_channels |= channel_map;
        //printf("[pwmgroups] offset: %d, default channel 0x%04X\n", _channel_map_offset, _default_rate_channels);
        
        for (uint8_t i=_channel_map_offset; i<_channel_map_offset+PWM_OUTPUT_MAX_CHANNELS; i++) {
            if (((1 << i) & _default_rate_channels) != 0) {
                //printf("[pwmgroups] group for channel %d : %d\n", i, up_pwm_servo_get_group_of_channel(i));
                _default_rate_timers |= (1 << up_pwm_servo_get_group_of_channel(i));
            }
        }
    }
    printf("[pwmgroups] device name: %s\n", get_devname());
    printf("[pwmgroups] occupation channel map: 0x%04X\n", channel_map);
    printf("[pwmgroups] alt rate channel: 0x%04X default rate channel: 0x%04X\n", _alt_rate_channels, _default_rate_channels);
    printf("[pwmgroups] alt rate timer: 0x%04X, default rate timer: 0x%04X\n", _alt_rate_timers, _default_rate_timers);
}

bool
PwmGroups::is_free_channels_available(uint32_t channel_map, pwm_groups_rate_t rate_type) {
    //uint8_t rate_group;
    //uint32_t rate_channel;
    uint32_t res;
    
    //check free channel;
    if (rate_type == PWM_GROUP_RATE_ALT) {
        res = channel_map & _alt_rate_channels;
    }
    else {
        res = channel_map & _default_rate_channels;
    }
    
    if (res != 0) {
        //channel conflicts;
        return false;
    }
    
    //check rate conflicts;
    if (rate_type == PWM_GROUP_RATE_ALT) {
        res = get_channel_mask_of_rate_type(PWM_GROUP_RATE_DEFAULT) & channel_map;
    }
    else {
        res = get_channel_mask_of_rate_type(PWM_GROUP_RATE_ALT) & channel_map;
    }
    
    if (res != 0) {
        //rate conflicts;
        return false;
    }
    
    return true;

}

uint32_t
PwmGroups::get_channel_mask_of_rate_type (pwm_groups_rate_t rate_type) {
    uint8_t target_map;
    uint32_t ret_channel = 0;
    if (rate_type == PWM_GROUP_RATE_ALT) {
        target_map = _alt_rate_timers;
    }
    else {
        target_map = _default_rate_timers;
    }
    
    if (target_map == 0) {
        return 0;
    }
    else {
        //uint8_t start = 0;
        for (uint8_t i=0 ; i<sizeof(uint8_t); i++) {
            if (((1 << i) & target_map) != 0) {
                ret_channel |= up_pwm_servo_get_rate_group(i);
            }
        }
    }
    return ret_channel;
    
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
PwmGroups::set_pwm_channel_value_single (uint8_t device_channel_idx, unsigned value) {
    device_channel_idx += _channel_map_offset;
    up_pwm_servo_set(device_channel_idx, value);
    return OK;
}

