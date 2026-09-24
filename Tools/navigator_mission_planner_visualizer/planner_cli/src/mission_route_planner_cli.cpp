/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file mission_route_planner_cli.cpp
 *
 * Runs the mission route planner on a scenario read from stdin and prints the plan,
 * one field per line, plus the planner's debug traces. Built out of tree through
 * EXTERNAL_MODULES_LOCATION for Tools/navigator_mission_planner_visualizer, so the
 * flight code stays untouched.
 *
 * Input, one entry per line, items in mission order:
 *   ITEM <WAYPOINT|TAKEOFF|LAND|...> <lat> <lon> <alt>
 *   JUMP <target index> <repeats> [current count]
 *   TRANSITION <FW|MC>
 *   RALLY <lat> <lon> <alt>
 *   VEHICLE <lat> <lon> <alt>
 *   SET <request field> <value>
 *   ACTION <RTL|RESUME>
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_planner.h"
#include "mission_route_provider.h"
#include "mission_route_types.h"
#include "navigation.h"

#include <uORB/topics/vtol_vehicle_status.h>

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

using namespace mission_route;

namespace
{

class VectorProvider final : public Provider
{
public:
	std::vector<mission_item_s> mission;
	std::vector<mission_item_s> safe_points;

	int missionCount() const override { return static_cast<int>(mission.size()); }
	bool loadMissionItem(int index, mission_item_s &item) const override { return load(mission, index, item); }
	int safePointCount() const override { return static_cast<int>(safe_points.size()); }
	bool loadSafePointItem(int index, mission_item_s &item) const override { return load(safe_points, index, item); }

private:
	static bool load(const std::vector<mission_item_s> &items, int index, mission_item_s &item)
	{
		if (index < 0 || index >= static_cast<int>(items.size())) {
			return false;
		}

		item = items[index];
		return true;
	}
};

struct Scenario {
	VectorProvider provider;
	std::string action{"RTL"};
	bool has_vehicle{false};
	// Superset of MissionResumeRequest, so one SET table serves both entry points.
	RtlRouteRequest request{};
};

struct NavCmdName {
	const char *name;
	uint16_t nav_cmd;
};

constexpr NavCmdName kNavCmdNames[] = {
	{"WAYPOINT", NAV_CMD_WAYPOINT},
	{"TAKEOFF", NAV_CMD_TAKEOFF},
	{"LAND", NAV_CMD_LAND},
	{"VTOL_TAKEOFF", NAV_CMD_VTOL_TAKEOFF},
	{"VTOL_LAND", NAV_CMD_VTOL_LAND},
	{"LOITER_TO_ALT", NAV_CMD_LOITER_TO_ALT},
	{"LOITER_UNLIMITED", NAV_CMD_LOITER_UNLIMITED},
	{"LOITER_TIME_LIMIT", NAV_CMD_LOITER_TIME_LIMIT},
};

bool parseNavCmd(const std::string &text, uint16_t &nav_cmd)
{
	const std::string name = text.rfind("NAV_CMD_", 0) == 0 ? text.substr(8) : text;

	for (const NavCmdName &entry : kNavCmdNames) {
		if (name == entry.name) {
			nav_cmd = entry.nav_cmd;
			return true;
		}
	}

	char *end = nullptr;
	const long value = strtol(text.c_str(), &end, 10);

	if (end == text.c_str() || *end != '\0' || value < 0 || value > UINT16_MAX) {
		return false;
	}

	nav_cmd = static_cast<uint16_t>(value);
	return true;
}

// Same item layout as the test helpers in mission_route_test_helpers.h.
mission_item_s makePositionItem(uint16_t nav_cmd, double lat, double lon, float alt)
{
	mission_item_s item{};
	item.nav_cmd = nav_cmd;
	item.lat = lat;
	item.lon = lon;
	item.altitude = alt;
	item.frame = NAV_FRAME_GLOBAL;
	item.altitude_is_relative = false;
	item.autocontinue = nav_cmd != NAV_CMD_TAKEOFF && nav_cmd != NAV_CMD_LAND;
	return item;
}

bool parseBool(const std::string &text, bool &value)
{
	if (text == "1" || text == "true") {
		value = true;
		return true;
	}

	if (text == "0" || text == "false") {
		value = false;
		return true;
	}

	return false;
}

bool parseVtolState(const std::string &text, uint8_t &state)
{
	if (text == "UNDEFINED") {
		state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;

	} else if (text == "MC") {
		state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	} else if (text == "FW") {
		state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

	} else {
		return false;
	}

	return true;
}

bool setRequestField(RtlRouteRequest &request, const std::string &key, const std::string &value)
{
	const float number = strtof(value.c_str(), nullptr);
	const long integer = strtol(value.c_str(), nullptr, 10);
	bool flag = false;

	if (key == "mission_index") { request.mission_index = static_cast<int32_t>(integer); return true; }

	if (key == "mission_land_index") { request.mission_land_index = static_cast<int32_t>(integer); return true; }

	if (key == "active_jump_anchor") { request.active_jump_anchor.jump_item_index = static_cast<int32_t>(integer); return true; }

	if (key == "home_altitude_amsl") { request.home_altitude_amsl = number; return true; }

	if (key == "projection_search_distance_m") { request.projection_search_distance_m = number; return true; }

	if (key == "safe_point_projection_search_distance_m") { request.safe_point_projection_search_distance_m = number; return true; }

	if (key == "acceptance_radius_m") { request.acceptance_radius_m = number; return true; }

	if (key == "direct_goal_acceptance_radius_m") { request.direct_goal_acceptance_radius_m = number; return true; }

	if (key == "altitude_acceptance_radius_m") { request.altitude_acceptance_radius_m = number; return true; }

	if (key == "fw_u_turn_penalty_m") { request.fw_u_turn_penalty_m = number; return true; }

	if (key == "velocity_north_m_s") { request.velocity_north_m_s = number; return true; }

	if (key == "velocity_east_m_s") { request.velocity_east_m_s = number; return true; }

	if (key == "vtol_state_on_mission_upload") { return parseVtolState(value, request.vtol_state_on_mission_upload); }

	if (!parseBool(value, flag)) {
		return false;
	}

	if (key == "current_route_direction_reversed") { request.current_route_direction_reversed = flag; return true; }

	if (key == "is_fixed_wing") { request.is_fixed_wing = flag; return true; }

	if (key == "in_transition_to_fw") { request.in_transition_to_fw = flag; return true; }

	if (key == "is_vtol") { request.is_vtol = flag; return true; }

	if (key == "require_vtol_approach") { request.require_vtol_approach = flag; return true; }

	return false;
}

bool readScenario(FILE *input, Scenario &scenario, std::string &error)
{
	char buffer[512];
	int line_number = 0;

	while (fgets(buffer, sizeof(buffer), input) != nullptr) {
		++line_number;
		std::istringstream line(buffer);
		std::string tag;

		if (!(line >> tag) || tag[0] == '#') {
			continue;
		}

		bool ok = true;

		if (tag == "ITEM") {
			std::string cmd;
			double lat = 0., lon = 0.;
			float alt = 0.f;
			uint16_t nav_cmd = NAV_CMD_WAYPOINT;
			ok = static_cast<bool>(line >> cmd >> lat >> lon >> alt) && parseNavCmd(cmd, nav_cmd);

			if (ok) {
				scenario.provider.mission.push_back(makePositionItem(nav_cmd, lat, lon, alt));
			}

		} else if (tag == "JUMP") {
			int target = -1, repeats = 0, current = 0;
			ok = static_cast<bool>(line >> target >> repeats);
			line >> current;

			if (ok) {
				mission_item_s item{};
				item.nav_cmd = NAV_CMD_DO_JUMP;
				item.do_jump_mission_index = static_cast<int16_t>(target);
				item.do_jump_repeat_count = static_cast<uint16_t>(repeats);
				item.do_jump_current_count = static_cast<uint16_t>(current);
				scenario.provider.mission.push_back(item);
			}

		} else if (tag == "TRANSITION") {
			std::string state_name;
			uint8_t state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_UNDEFINED;
			ok = static_cast<bool>(line >> state_name) && parseVtolState(state_name, state);

			if (ok) {
				mission_item_s item{};
				item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
				item.params[0] = static_cast<float>(state);
				scenario.provider.mission.push_back(item);
			}

		} else if (tag == "RALLY") {
			double lat = 0., lon = 0.;
			float alt = 0.f;
			ok = static_cast<bool>(line >> lat >> lon >> alt);

			if (ok) {
				scenario.provider.safe_points.push_back(makePositionItem(NAV_CMD_RALLY_POINT, lat, lon, alt));
			}

		} else if (tag == "VEHICLE") {
			Position &vehicle = scenario.request.vehicle_position;
			ok = static_cast<bool>(line >> vehicle.lat >> vehicle.lon >> vehicle.alt);
			scenario.has_vehicle = ok;

		} else if (tag == "SET") {
			std::string key, value;
			ok = static_cast<bool>(line >> key >> value) && setRequestField(scenario.request, key, value);

		} else if (tag == "ACTION") {
			ok = static_cast<bool>(line >> scenario.action)
			     && (scenario.action == "RTL" || scenario.action == "RESUME");

		} else {
			ok = false;
		}

		if (!ok) {
			std::string text(buffer);

			while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
				text.pop_back();
			}

			error = "line " + std::to_string(line_number) + ": " + text;
			return false;
		}
	}

	if (!scenario.has_vehicle) {
		error = "no VEHICLE line";
		return false;
	}

	return true;
}

const char *transitionName(VtolTransitionAction action)
{
	switch (action) {
	case VtolTransitionAction::kFrontTransition:
		return "front_transition";

	case VtolTransitionAction::kBackTransition:
		return "back_transition";

	case VtolTransitionAction::kNone:
	default:
		return "none";
	}
}

void printPosition(const char *prefix, const Position &position)
{
	printf("PLAN %s_lat %.9f\n", prefix, position.lat);
	printf("PLAN %s_lon %.9f\n", prefix, position.lon);
	printf("PLAN %s_alt %.3f\n", prefix, static_cast<double>(position.alt));
}

template<typename PlanT>
void printJoin(const PlanT &plan)
{
	printf("PLAN valid %d\n", static_cast<int>(plan.valid()));
	printPosition("join", plan.join_position);
	printf("PLAN first_mission_item_index %d\n", static_cast<int>(plan.first_mission_item_index));
	printf("PLAN direction_reversed %d\n", static_cast<int>(plan.direction_reversed));
	printf("PLAN use_current_altitude %d\n", static_cast<int>(plan.use_current_altitude));
	printf("PLAN active_jump_anchor %d\n", static_cast<int>(plan.active_jump_anchor.jump_item_index));
	printf("PLAN vtol_transition_action %s\n", transitionName(plan.vtol_transition_action));
}

void printStatus(FailureReason status)
{
	if (status == FailureReason::kNone) {
		printf("STATUS OK\n");

	} else {
		printf("STATUS FAIL %s\n", failureReasonString(status));
	}
}

void runPlanner(const Scenario &scenario)
{
	const MissionRoutePlanner planner{scenario.provider};

	if (scenario.action == "RTL") {
		RtlRoutePlan plan{};
		const FailureReason status = planner.planRtlRoute(scenario.request, plan);
		printStatus(status);

		if (status == FailureReason::kNone) {
			printJoin(plan);
			printf("PLAN goal_type %s\n", goalTypeString(plan.goal_type));
			printPosition("goal", plan.goal_position);
			printf("PLAN safe_point_index %d\n", static_cast<int>(plan.safe_point_index));
			printPosition("branch_off", plan.branch_off_position);
			printf("PLAN branch_off_mission_item_index %d\n", static_cast<int>(plan.branch_off_mission_item_index));
			printf("PLAN fly_direct_to_goal %d\n", static_cast<int>(plan.fly_direct_to_goal));
		}

		return;
	}

	const RtlRouteRequest &source = scenario.request;
	MissionResumeRequest request{};
	request.vehicle_position = source.vehicle_position;
	request.mission_index = source.mission_index;
	request.current_route_direction_reversed = source.current_route_direction_reversed;
	request.active_jump_anchor = source.active_jump_anchor;
	request.home_altitude_amsl = source.home_altitude_amsl;
	request.projection_search_distance_m = source.projection_search_distance_m;
	request.acceptance_radius_m = source.acceptance_radius_m;
	request.is_fixed_wing = source.is_fixed_wing;
	request.in_transition_to_fw = source.in_transition_to_fw;
	request.is_vtol = source.is_vtol;
	request.vtol_state_on_mission_upload = source.vtol_state_on_mission_upload;
	request.velocity_north_m_s = source.velocity_north_m_s;
	request.velocity_east_m_s = source.velocity_east_m_s;
	request.fw_u_turn_penalty_m = source.fw_u_turn_penalty_m;

	MissionResumePlan plan{};
	const FailureReason status = planner.planMissionResumeJoin(request, plan);
	printStatus(status);

	if (status == FailureReason::kNone) {
		printJoin(plan);
	}
}

} // namespace

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	Scenario scenario{};
	std::string error;

	if (!readScenario(stdin, scenario, error)) {
		fprintf(stderr, "invalid scenario: %s\n", error.c_str());
		return 2;
	}

	printf("ACTION %s\n", scenario.action.c_str());
	runPlanner(scenario);
	printf("END\n");
	return 0;
}
