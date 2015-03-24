/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file LidarLitePWM.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via PWM.
 */
#include "LidarLitePWM.h"
#include <stdio.h>
#include <drivers/drv_hrt.h>

/* oddly, ERROR is not defined for c++ */
#ifdef __cplusplus
static const int ERROR = -1;
#endif

LidarLitePWM::LidarLitePWM() :
    _pwmSub(-1),
    _pwm{},
    _rangePub(-1),
    _range{}
{

}

int LidarLitePWM::init()
{
    _pwmSub = orb_subscribe(ORB_ID(pwm_input));

    if(_pwmSub == -1) {
        return ERROR;
    }

    _range.type = RANGE_FINDER_TYPE_LASER;
    _range.valid = false;
    _rangePub = orb_advertise(ORB_ID(sensor_range_finder), &_range);

    return OK;
}

void LidarLitePWM::print_info()
{
    printf("poll interval:  %u ticks\n", getMeasureTicks());
}

void LidarLitePWM::print_registers()
{
    printf("Not supported in PWM mode\n");
}

void LidarLitePWM::start()
{
    //TODO: start measurement task
}

void LidarLitePWM::stop()
{
    //TODO: stop measurement task
}

int LidarLitePWM::measure()
{
    bool update;
    orb_check(_pwmSub, &update);

    if(update) {
        orb_copy(ORB_ID(pwm_input), _pwmSub, &_pwm);

        _range.timestamp = hrt_absolute_time();
        _range.error_count = _pwm.error_count;
        _range.valid = true;
        _range.maximum_distance = get_maximum_distance();
        _range.minimum_distance = get_minimum_distance();
        _range.distance = _pwm.pulse_width / 1000.0f;   //10 usec = 1 cm distance for LIDAR-Lite
        _range.distance_vector[0] = _range.distance;
        _range.just_updated = 0;

        orb_publish(ORB_ID(sensor_range_finder), _rangePub, &_range);
    }

    return OK;
}
