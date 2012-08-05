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
 * @file Mixer ioctl interface.
 *
 * This interface can/should be exported by any device that supports
 * control -> actuator mixing.
 */

#ifndef _DRV_MIXER_H
#define _DRV_MIXER_H

#include <stdint.h>
#include <sys/ioctl.h>

#include <systemlib/mixer.h>

/**
 * Structure used for receiving mixers.
 *
 * Note that the mixers array is not actually an array of mixers; it
 * simply represents the first mixer in the buffer.
 */
struct MixInfo
{
	unsigned	num_controls;
	struct MixMixer	mixer;
};

/**
 * Handy macro for determining the allocation size of a MixInfo structure.
 */
#define MIXINFO_SIZE(_num_controls)	(sizeof(struct MixInfo) + ((_num_controls) * sizeof(struct MixScaler)))

/*
 * ioctl() definitions
 */

#define _MIXERIOCBASE		(0x2400)
#define _MIXERIOC(_n)		(_IOC(_MIXERIOCBASE, _n))

/** get the number of actuators that require mixers in *(unsigned)arg */
#define MIXERIOCGETMIXERCOUNT	_MIXERIOC(0)

/**
 * Copy a mixer from the device into *(struct MixInfo *)arg.
 *
 * The num_controls field indicates the number of controls for which space
 * is allocated following the MixInfo structure.  If the allocation
 * is too small, no mixer data is retured.  The control_count field in
 * the MixInfo.mixer structure is always updated.
 *
 * If no mixer is assigned for the given index, the ioctl returns ENOENT.
 */
#define MIXERIOCGETMIXER(_mixer)	_MIXERIOC(0x20 + _mixer)

/**
 * Copy a mixer from *(struct MixMixer *)arg to the device.
 *
 * If arg is zero, the mixer is deleted.
 */
#define MIXERIOCSETMIXER(_mixer)	_MIXERIOC(0x40 + _mixer)

#endif /* _DRV_ACCEL_H */
