/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file bottle_drop_params.c
 * Bottle drop parameters
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Ground drag property
 *
 * This parameter encodes the ground drag coefficient and the corresponding
 * decrease in wind speed from the plane altitude to ground altitude.
 *
 * @min 0.001
 * @max 0.1
 * @group Payload drop
 */
PARAM_DEFINE_FLOAT(BD_GPROPERTIES, 0.03f);

/**
 * Plane turn radius
 *
 * The planes known minimal turn radius - use a higher value
 * to make the plane maneuver more distant from the actual drop
 * position. This is to ensure the wings are level during the drop.
 *
 * @unit m
 * @min 30.0
 * @max 500.0
 * @group Payload drop
 */
PARAM_DEFINE_FLOAT(BD_TURNRADIUS, 120.0f);

/**
 * Drop precision
 *
 * If the system is closer than this distance on passing over the
 * drop position, it will release the payload. This is a safeguard
 * to prevent a drop out of the required accuracy.
 *
 * @unit m
 * @min 1.0
 * @max 80.0
 * @group Payload drop
 */
PARAM_DEFINE_FLOAT(BD_PRECISION, 30.0f);

/**
 * Payload drag coefficient of the dropped object
 *
 * The drag coefficient (cd) is the typical drag
 * constant for air. It is in general object specific,
 * but the closest primitive shape to the actual object
 * should give good results:
 * http://en.wikipedia.org/wiki/Drag_coefficient
 *
 * @min 0.08
 * @max 1.5
 * @group Payload drop
 */
PARAM_DEFINE_FLOAT(BD_OBJ_CD, 0.1f);

/**
 * Payload mass
 *
 * A typical small toy ball:
 *   0.025 kg
 *
 * OBC water bottle:
 *   0.6 kg
 *
 * @unit kg
 * @min 0.001
 * @max 5.0
 * @group Payload drop
 */
PARAM_DEFINE_FLOAT(BD_OBJ_MASS, 0.6f);

/**
 * Payload front surface area
 *
 * A typical small toy ball:
 *   (0.045 * 0.045) / 4.0 * pi = 0.001590 m^2
 *
 * OBC water bottle:
 *   (0.063 * 0.063) / 4.0 * pi = 0.003117 m^2
 *
 * @unit m^2
 * @min 0.001
 * @max 0.5
 * @group Payload drop
 */
PARAM_DEFINE_FLOAT(BD_OBJ_SURFACE, 0.00311724531f);
