"""Turn a scenario and the plan the CLI returned into a planner unit test."""

import re
from typing import Optional

from .generator import cpp_float, generate_cpp
from .models import Base, Position, Scene
from .planner_cli import PlanRequest, PlanResult, first_mission, vehicle_position

# Values makeRtlRouteRequest() / makeMissionResumeRequest() already set in test_mission_route_planner.cpp.
_TEST_HELPER_DEFAULTS = {
    "home_altitude_amsl": 500.0,
    "projection_search_distance_m": 60.0,
    "safe_point_projection_search_distance_m": 60.0,
    "acceptance_radius_m": 10.0,
    "direct_goal_acceptance_radius_m": 10.0,
    "altitude_acceptance_radius_m": 10.0,
    "fw_u_turn_penalty_m": 4000.0,  # the request struct default
}
_RESUME_FIELDS = (
    "current_route_direction_reversed",
    "active_jump_anchor",
    "home_altitude_amsl",
    "projection_search_distance_m",
    "acceptance_radius_m",
    "is_fixed_wing",
    "in_transition_to_fw",
    "is_vtol",
    "vtol_state_on_mission_upload",
    "fw_u_turn_penalty_m",
)
_LAT_LON_TOLERANCE = "1e-6"


def test_name(text: str) -> str:
    """CamelCase identifier: capitalise each word, keep capitals already there, drop the rest."""
    name = re.sub(r"[^A-Za-z0-9]", "", "".join(word[:1].upper() + word[1:] for word in text.split()))
    return name if name and not name[0].isdigit() else "GeneratedScenario"


def generate_unit_test(
    scene: Scene,
    base: Optional[Base],
    request: PlanRequest,
    result: PlanResult,
    name: str,
    vehicle: Optional[Position] = None,
) -> str:
    """A TEST_F for test_mission_route_planner.cpp asserting what the planner returned.

    The expectations are whatever the planner did, so the test is only as right as the
    review of the plan on the map. Say what the scenario checks in the comment. Only the
    vehicle the plan was made for is emitted; unused positions would fail -Werror.
    """
    rtl = request.action == "RTL"
    _, items = first_mission(scene)
    vehicle = vehicle or vehicle_position(scene)
    fixtures = _fixtures(scene, base, vehicle)
    lines = [
        "// Describe the intent of this scenario here, not the numbers.",
        f"TEST_F(MissionRoutePlannerTest, {test_name(name)})",
        "{",
    ]
    lines += ["\t" + line if line else "" for line in fixtures.splitlines()]
    planner_args = "mission, safe_points" if scene.safe_points else "mission"
    lines.append(f"\tTestPlanner planner({planner_args});")
    request_type = "RtlRouteRequest" if rtl else "MissionResumeRequest"
    helper = "makeRtlRouteRequest" if rtl else "makeMissionResumeRequest"
    vehicle_name = vehicle.name if vehicle is not None else "vehicle_position"
    lines.append(
        f"\tmission_route::{request_type} request = {helper}({vehicle_name}, {request.mission_index});"
    )
    lines += [f"\t{line}" for line in _request_overrides(request, scene, rtl)]
    plan_type = "RtlRoutePlan" if rtl else "MissionResumePlan"
    method = "planRtlRoute" if rtl else "planMissionResumeJoin"
    lines += [
        f"\tmission_route::{plan_type} plan{{}};",
        "",
        f"\tconst mission_route::FailureReason status = planner.{method}(request, plan);",
        "",
    ]
    if not result.ok:
        lines += [
            f"\t// The planner reported: {result.failure}",
            "\tEXPECT_NE(status, mission_route::FailureReason::kNone);",
            "\tEXPECT_FALSE(plan.valid());",
            "}",
        ]
        return "\n".join(lines)
    lines.append(
        "\tASSERT_EQ(status, mission_route::FailureReason::kNone) << mission_route::failureReasonString(status);"
    )
    lines.append("\tEXPECT_TRUE(plan.valid());")
    lines += [f"\t{line}" for line in _expectations(result, base, rtl, len(items))]
    lines.append("}")
    return "\n".join(lines)


def _fixtures(scene: Scene, base: Optional[Base], vehicle: Optional[Position]) -> str:
    # One mission named `mission`, whatever the pasted source called it, and one vehicle.
    name, items = first_mission(scene)
    named = Scene(
        missions={"mission": items},
        safe_points=scene.safe_points,
        positions=[vehicle] if vehicle is not None else [],
        constants=scene.constants,
    )
    return generate_cpp(named, base)


def _request_overrides(request: PlanRequest, scene: Scene, rtl: bool):
    for name, value in request.settings().items():
        if name in ("mission_index",) or (not rtl and name not in _RESUME_FIELDS):
            continue
        default = _TEST_HELPER_DEFAULTS.get(name)
        if default is not None and abs(float(value) - default) < 1e-6:
            continue
        if name == "mission_land_index" and int(value) < 0:
            continue
        if name == "active_jump_anchor":
            if int(value) >= 0:
                yield f"request.active_jump_anchor = {{{value}}};"
            continue
        if name == "vtol_state_on_mission_upload":
            if value != "UNDEFINED":
                yield f"request.vtol_state_on_mission_upload = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_{value};"
            continue
        if value in ("0", "1") and not name.endswith(("_m", "_amsl", "_index")):
            if value == "1":
                yield f"request.{name} = true;"
            continue
        if name.endswith("_index"):
            yield f"request.{name} = {value};"
            continue
        yield f"request.{name} = {cpp_float(float(value))};"
    if scene.velocity is not None:
        yield f"request.velocity_north_m_s = {cpp_float(scene.velocity[0])};"
        yield f"request.velocity_east_m_s = {cpp_float(scene.velocity[1])};"


def _expectations(result: PlanResult, base: Optional[Base], rtl: bool, item_count: int):
    yield f"EXPECT_EQ(plan.first_mission_item_index, {result.int_field('first_mission_item_index')});"
    yield _expect_bool("plan.direction_reversed", result.bool_field("direction_reversed"))
    yield _expect_bool("plan.use_current_altitude", result.bool_field("use_current_altitude"))
    yield from _expect_position("join_position", result.position("join"), base)
    if not rtl:
        return
    goal_type = result.plan.get("goal_type", "")
    yield f"EXPECT_EQ(plan.goal_type, mission_route::GoalType::k{_goal_enum(goal_type)});"
    if goal_type == "safe_point":
        yield f"EXPECT_EQ(plan.safe_point_index, {result.int_field('safe_point_index')});"
        yield f"EXPECT_EQ(plan.branch_off_mission_item_index, {result.int_field('branch_off_mission_item_index')});"
        yield from _expect_position("branch_off_position", result.position("branch_off"), base)
    yield _expect_bool("plan.fly_direct_to_goal", result.bool_field("fly_direct_to_goal"))


def _goal_enum(goal_type: str) -> str:
    return {
        "safe_point": "SafePoint",
        "mission_land": "MissionLand",
        "mission_takeoff": "MissionTakeoff",
    }.get(goal_type, "None")


def _expect_bool(expression: str, value: bool) -> str:
    return f"EXPECT_{'TRUE' if value else 'FALSE'}({expression});"


def _expect_position(field: str, position, base: Optional[Base]):
    if position is None:
        return
    lat, lon, alt = position
    yield f"EXPECT_NEAR(plan.{field}.lat, {lat:.7f}, {_LAT_LON_TOLERANCE});"
    yield f"EXPECT_NEAR(plan.{field}.lon, {lon:.7f}, {_LAT_LON_TOLERANCE});"
    yield f"EXPECT_NEAR(plan.{field}.alt, {cpp_float(alt)}, 0.01f);"
