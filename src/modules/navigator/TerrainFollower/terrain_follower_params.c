/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * Enable Terrain Data upload
 *
 * If enabled, configures the terrain data grid spacing.
 * The recommended value is 100m, or 50m on boards with enough RAM.
 * Whenever the mission, home position or safe points change,
 * terrain data is requested within a rectangular area that includes all of
 * these points.
 *
 * This will then be used during RTL.
 *
 * Note:
 * - this feature requres several KB of RAM.
 * - latitudes beyond +-80 degrees are not supported.
 *
 * @value 0 Disabled
 * @value 50 50m grid spacing
 * @value 100 100m grid spacing
 * @value 200 200m grid spacing
 * @value 400 400m grid spacing
 * @reboot_required true
 * @group Terrain Follower
 */
PARAM_DEFINE_INT32(TF_TERRAIN_EN, 0);

/**
 * Terrain follower check distance.
 *
 * Look ahead distance the terrain follower will use to detect terrain violation.
 *
 * @unit m
 * @min 10.0
 * @max 1000.0
 * @decimal 2
 * @increment 1.0
 * @group Terrain Follower
 */
PARAM_DEFINE_FLOAT(TF_CHECK_DIST, 100);


/**
 * Terrain follower minimum distance to terrain.
 *
 *
 * @unit m
 * @min 10.0
 * @decimal 2
 * @increment 1.0
 * @group Terrain Follower
 */
PARAM_DEFINE_FLOAT(TF_TERR_DIST_MIN, 100);

/**
 * Terrain follower maximum distance to terrain.
 *
 *
 * @unit m
 * @min 10.0
 * @decimal 2
 * @increment 1.0
 * @group Terrain Follower
 */
PARAM_DEFINE_FLOAT(TF_TERR_DIST_MAX, 200);