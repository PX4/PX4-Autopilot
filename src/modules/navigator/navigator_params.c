/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file navigator_params.c
 *
 * Parameters defined by the navigator task.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Navigator parameters, accessible via MAVLink
 */

/**
 * Minimum altitude (fixed wing only)
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_MIN_ALT, 50.0f);

/**
 * Waypoint acceptance radius.
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_ACCEPT_RAD, 10.0f);

/**
 * Loiter radius  (fixed wing only)
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_LOITER_RAD, 50.0f);

/**
 * Enable onboard mission.
 *
 * @group Navigation
 */
PARAM_DEFINE_INT32(NAV_ONB_MIS_EN, 0);

/**
 * Take-off altitude.
 *
 * Even if first waypoint has altitude less then NAV_TAKEOFF_ALT, system will climb to NAV_TAKEOFF_ALT on takeoff, then go to waypoint.
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_TAKEOFF_ALT, 10.0f);

/**
 * Landing altitude.
 *
 * Stay at this altitude after RTL descending. Slowly descend from this altitude if autolanding allowed.
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_LAND_ALT, 5.0f);

/**
 * Return-to-land altitude.
 *
 * Minimum altitude for going home in RTL mode.
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_RTL_ALT, 30.0f);

/**
 * Return-to-land delay.
 *
 * Delay after descend before landing in RTL mode.
 * If set to -1 the system will not land but loiter at NAV_LAND_ALT.
 *
 * @group Navigation
 */
PARAM_DEFINE_FLOAT(NAV_RTL_LAND_T, -1.0f);

/**
 * Enable parachute deployment.
 *
 * @group Navigation
 */
PARAM_DEFINE_INT32(NAV_PARACHUTE_EN, 0);
