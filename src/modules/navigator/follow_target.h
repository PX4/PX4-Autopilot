/***************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#pragma once

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include "navigator_mode.h"
#include "mission_block.h"
#include <lib/matrix/matrix/Vector2.hpp>

class FollowTarget : public MissionBlock
{

public:
    FollowTarget(Navigator *navigator, const char *name);

    ~FollowTarget();

    virtual void on_inactive();

    virtual void on_activation();

    virtual void on_active();

private:
    Navigator *_navigator;
    control::BlockParamFloat _param_min_alt;
    matrix::Vector2f pos_pair[2];
    matrix::Vector2f gps_pair;
    bool gps_valid;
    uint64_t _last_message_time;
    uint64_t _last_publish_time;
    float _steps;
    bool follow_target_reached;
    int _index;

    struct pos_history_s{
       struct position_setpoint_triplet_s pos_history[6];
       uint64_t wp_time;
    };

    pos_history_s wp_history[6];
    int wp_cnt;
    matrix::Vector2f _current_vel;
    matrix::Vector2f target_vel;
    void update_position_sp(matrix::Vector2f & vel);
    void loiter();
    float target_dist_x;
    float target_dist_y;
    follow_target_s target;
};
