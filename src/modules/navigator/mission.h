/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @author Julian Oes <julian@oes.ch>
 *
 * @author Julian Oes <joes@student.ethz.ch>
 */

#ifndef NAVIGATOR_MISSION_H
#define NAVIGATOR_MISSION_H

#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <dataman/dataman.h>


class __EXPORT Mission
{
public:
	/**
	 * Constructor
	 */
	Mission();

	/**
	 * Destructor
	 */
	~Mission();

	void		set_offboard_dataman_id(const int new_id);
	void		set_offboard_mission_count(const int new_count);
	void		set_onboard_mission_count(const int new_count);
	void		set_onboard_mission_allowed(const bool allowed);

	bool		command_current_offboard_mission_index(const int new_index);
	bool		command_current_onboard_mission_index(const int new_index);

	bool		get_current_mission_item(struct mission_item_s *mission_item, bool *onboard, int *index);
	bool		get_next_mission_item(struct mission_item_s *mission_item);

	void		move_to_next();

private:
	bool		read_mission_item(const dm_item_t dm_item, const bool is_current, int *mission_index, struct mission_item_s *new_mission_item);

	void		report_mission_item_reached();
	void		report_current_offboard_mission_item();

	void		publish_mission_result();

	int 		_offboard_dataman_id;
	int		_current_offboard_mission_index;
	int		_current_onboard_mission_index;
	int		_offboard_mission_item_count;		/** number of offboard mission items available */
	int		_onboard_mission_item_count;		/** number of onboard mission items available */
	bool		_onboard_mission_allowed;

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_ONBOARD,
		MISSION_TYPE_OFFBOARD,
	} 		_current_mission_type;

	orb_advert_t	 _mission_result_pub;			/**< publish mission result topic */
	mission_result_s _mission_result;			/**< mission result for commander/mavlink */
};

#endif
