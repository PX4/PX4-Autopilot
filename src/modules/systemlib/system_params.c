/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

/*
 * @file system_params.c
 *
 * System wide parameters
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Auto-start script index.
 *
 * Defines the auto-start script used to bootstrap the system.
 *
 * @group System
 */
PARAM_DEFINE_INT32(SYS_AUTOSTART, 0);

/**
 * Automatically configure default values.
 *
 * Set to 1 to reset parameters on next system startup (setting defaults).
 * Platform-specific values are used if available.
 * RC* parameters are preserved.
 *
 * @min 0
 * @max 1
 * @group System
 */
PARAM_DEFINE_INT32(SYS_AUTOCONFIG, 0);

/**
* Set usage of IO board
*
* Can be used to use a standard startup script but with a FMU only set-up. Set to 0 to force the FMU only set-up.
*
* @min 0
* @max 1
* @group System
*/
PARAM_DEFINE_INT32(SYS_USE_IO, 1);

/**
* Set restart type
*
* Set by px4io to indicate type of restart
*
* @min 0
* @max 2
* @group System
*/
PARAM_DEFINE_INT32(SYS_RESTART_TYPE, 2);

/**
* Companion computer interface
*
* Configures the baud rate of the companion computer interface.
* Set to zero to disable, set to these values to enable (NO OTHER VALUES SUPPORTED!)
* 921600: enables onboard mode at 921600 baud, 8N1. 57600: enables onboard mode at 57600 baud, 8N1.
* 157600: enables OSD mode at 57600 baud, 8N1.
*
* @min 0
* @max 921600
* @group System
*/
PARAM_DEFINE_INT32(SYS_COMPANION, 0);

/**
* Parameter version
*
* This monotonically increasing number encodes the parameter compatibility set.
* whenever it increases parameters might not be backwards compatible and
* ground control stations should suggest a fresh configuration.
*
* @min 0
* @group System
*/
PARAM_DEFINE_INT32(SYS_PARAM_VER, 1);
