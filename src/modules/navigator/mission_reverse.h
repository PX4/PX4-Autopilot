/***************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file mission_reverse.h
 *
 * Helper class to fly a offboard mission in reverse.
 * Only the positions of the waypoints are used, the type is converted to
 * MAV_CMD_NAV_WAYPOINT.
 *
 * @author Florian Achermann <florian.achermann@mavt.ethz.ch>
 */

#ifndef NAVIGATOR_MISSIONREVERSE_H
#define NAVIGATOR_MISSIONREVERSE_H

#include "mission_block.h"
#include "mission.h"

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <cfloat>


class Navigator;
class Mission;

class MissionReverse final : public MissionBlock
{
public:
	MissionReverse(Navigator *navigator, Mission *mission);

	~MissionReverse();

	virtual void on_inactive() override;
	virtual void on_inactivation() override;
	virtual void on_activation() override;
	virtual void on_active() override;

	bool get_mission_reverse_finished() const { return _mission_reverse_finished; }

private:
	/**
	 * Advance the mission to the prior waypoint containing a position. Waypoints without
	 * a position are skipped.
	 */
	void advance_mission();

	/**
	 * Set the position setpoint triplet according to the current mission item.
	 */
	void set_mission_items();

	/**
	 * Return the index of the closest offboard mission item to the current global position.
	 */
	uint16_t index_closest_mission_item() const;

	/**
	 * Return if the mission has changed since the last iteration.
	 */
	bool mission_changed();

	/**
	 * Issue a vtol transition to the FW mode.
	 */
	void command_vtol_transition();

	Mission *_mission{nullptr};

	bool _mission_reverse_finished{false}; /**< Indicates if flying back the mission is completed */

	struct mission_s _previous_mission {}; /**< Copy of the offboard mission used to detect changes in the mission. */
};

#endif
