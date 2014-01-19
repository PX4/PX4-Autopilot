/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Julian Oes <joes@student.ethz.ch>
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
 * @file navigator_mission.h
 * Helper class to access missions
 */

#ifndef NAVIGATOR_MISSION_H
#define NAVIGATOR_MISSION_H

#include <uORB/topics/mission.h>


class __EXPORT Mission
{
public:
	/**
	 * Constructor
	 */
	Mission();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Mission();

	void		set_offboard_dataman_id(int new_id);
	void		set_current_offboard_mission_index(int new_index);
	void		set_current_onboard_mission_index(int new_index);
	void		set_offboard_mission_count(unsigned new_count);
	void		set_onboard_mission_count(unsigned new_count);

	void		set_onboard_mission_allowed(bool allowed);

	bool		current_mission_available();
	bool		next_mission_available();

	int		get_current_mission_item(struct mission_item_s *mission_item, bool *onboard, unsigned *index);
	int		get_next_mission_item(struct mission_item_s *mission_item);

	void		move_to_next();

	void		add_home_pos(struct mission_item_s *new_mission_item);

private:
	bool		current_onboard_mission_available();
	bool		current_offboard_mission_available();
	bool		next_onboard_mission_available();
	bool		next_offboard_mission_available();

	int 		_offboard_dataman_id;
	unsigned	_current_offboard_mission_index;
	unsigned	_current_onboard_mission_index;
	unsigned	_offboard_mission_item_count;		/** number of offboard mission items available */
	unsigned	_onboard_mission_item_count;		/** number of onboard mission items available */

	bool		_onboard_mission_allowed;

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_ONBOARD,
		MISSION_TYPE_OFFBOARD,
	} 		_current_mission_type;
};

#endif