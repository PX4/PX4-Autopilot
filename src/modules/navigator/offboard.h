/***************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file offboard.h
 *
 * Helper class for offboard commands
 *
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef NAVIGATOR_OFFBOARD_H
#define NAVIGATOR_OFFBOARD_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/offboard_control_setpoint.h>

#include "navigator_mode.h"

class Navigator;

class Offboard : public NavigatorMode
{
public:
	/**
	 * Constructor
	 */
	Offboard(Navigator *navigator, const char *name);

	/**
	 * Destructor
	 */
	~Offboard();

	/**
	 * This function is called while the mode is inactive
	 */
	virtual void on_inactive();

	/**
	 * This function is called while the mode is active
	 */
	virtual bool on_active(struct position_setpoint_triplet_s *pos_sp_triplet);
private:
	void update_offboard_control_setpoint();

	struct offboard_control_setpoint_s _offboard_control_sp;
};

#endif
