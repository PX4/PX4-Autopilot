"""Streamlit UI: paste a test, click the map to add or edit items, run the planner, copy the C++."""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import pandas as pd
import streamlit as st
from streamlit_folium import st_folium

from . import planner_cli
from .cpp_parser import header_constants, parse_cpp
from .editor import ClickSettings, Editor
from .generator import generate_cpp
from .geometry import fit_view, latlon_to_offset, offset_to_latlon
from .map2d import PLAN_COLORS, add_plan_layer, build_base_map, build_new_items_layer
from .models import (
    DEFAULT_CENTER,
    DEFAULT_ZOOM,
    ITEM_TYPES,
    MISSION,
    MISSION_COMMANDS,
    VEHICLE,
    Base,
    Position,
    Scene,
    Waypoint,
    base_from,
)
from .planner_cli import PlanRequest, PlanResult
from .unit_test import generate_unit_test

MAP_HEIGHT = 620
OFFSETS = "Offsets from kBaseLat/kBaseLon"
ABSOLUTE = "Absolute lat/lon"
PLANNER_ACTIONS = (("Plan Return", "RTL"), ("Plan Mission rejoin", "RESUME"))
# What st_folium reports back: map clicks add or move items, marker clicks select them.
MAP_EVENTS = ["last_clicked", "last_object_clicked", "last_object_clicked_tooltip"]


@dataclass
class VehicleRun:
    """The planner's answer for one vehicle position."""

    vehicle: Position
    scenario: str
    result: PlanResult


@dataclass
class PlanRun:
    """One press of a plan button: the request, the scene it ran on and one answer per vehicle."""

    request: PlanRequest
    scene: Scene
    runs: List[VehicleRun]

    def names(self) -> List[str]:
        return [run.vehicle.name for run in self.runs]

    def run_for(self, name: Optional[str]) -> VehicleRun:
        return next((run for run in self.runs if run.vehicle.name == name), self.runs[0])


def init_state() -> None:
    state = st.session_state
    state.setdefault("editor", Editor())
    state.setdefault("parsed", None)
    state.setdefault("map_key", 0)
    state.setdefault("plan", None)
    state.setdefault("plan_error", None)
    # The new item being edited, whether the next map click moves it, the marker click already
    # handled, and whether the edit form must be refilled from the item.
    state.setdefault("selected", None)
    state.setdefault("moving", False)
    state.setdefault("last_object_click", None)
    state.setdefault("edit_sync", False)
    if "view" not in state:
        base = base_from(header_constants())
        center = (base.lat, base.lon) if base else DEFAULT_CENTER
        state["view"] = (center, DEFAULT_ZOOM)


def current_base() -> Optional[Base]:
    """kBaseLat/kBaseLon of the parsed source, else the ones of the helper header."""
    parsed = st.session_state["parsed"]
    return parsed.base() if parsed else base_from(header_constants())


def recenter_on_base(base: Base) -> None:
    """Put kBaseLat/kBaseLon back in the middle of the map at the default zoom.

    st_folium only moves when center or zoom change, so a fresh widget key remounts
    the map. Otherwise a pan away from an unchanged base view could not be undone.
    """
    st.session_state["view"] = ((base.lat, base.lon), DEFAULT_ZOOM)
    st.session_state["map_key"] += 1


def render_source_panel() -> None:
    with st.expander("C++ source", expanded=False):
        st.text_area("Paste one test, with the mission data it uses.", height=240, key="source")
        parse_column, clear_column, status_column = st.columns([1, 1, 4])
        if parse_column.button("Parse", type="primary", use_container_width=True):
            scene = parse_cpp(st.session_state["source"])
            if scene.is_empty():
                st.warning("No mission items, safe points or positions found.")
            else:
                st.session_state["parsed"] = scene
                st.session_state["view"] = fit_view(scene.all_points(), MAP_HEIGHT)
        if clear_column.button("Clear parsed", use_container_width=True):
            st.session_state["parsed"] = None
        parsed = st.session_state["parsed"]
        if parsed is not None:
            status_column.caption(
                f"Showing {parsed.summary()}. Hover the map for names, altitudes and leg lengths."
            )


def render_sidebar(editor: Editor, base: Optional[Base]) -> Tuple[str, ClickSettings, bool, bool]:
    st.header("Add items")
    item_type = st.selectbox("Item type", ITEM_TYPES, key="item_type")
    cmd = "NAV_CMD_WAYPOINT"
    if item_type == MISSION:
        kind = st.radio("Mission item", list(MISSION_COMMANDS), horizontal=True, key="mission_kind")
        cmd = MISSION_COMMANDS[kind]
    alt = st.number_input("Altitude (m AMSL)", value=500.0, step=10.0, key="alt")
    if item_type == VEHICLE:
        north = st.number_input("Velocity north (m/s)", value=0.0, step=1.0, key="vel_north")
        east = st.number_input("Velocity east (m/s)", value=0.0, step=1.0, key="vel_east")
        editor.velocity = (north, east)
    grid = st.number_input(
        "Snap clicks to a grid (m, 0 = off)", min_value=0.0, value=10.0, step=5.0, key="grid"
    )
    st.caption("Click the map to add a point with these values.")

    undo_column, clear_column = st.columns(2)
    # Undo and Load can change the item under the selection, so the edit form is refilled from it.
    if undo_column.button("Undo", disabled=not editor.history, use_container_width=True):
        editor.undo()
        st.session_state["edit_sync"] = True
        st.rerun()
    if clear_column.button(
        f"Clear {item_type.lower()}", disabled=not editor.count(item_type), use_container_width=True
    ):
        editor.clear(item_type)
        st.rerun()
    if st.button("Clear all new items", disabled=not editor.count(), use_container_width=True):
        editor.clear()
        st.rerun()
    parsed = st.session_state["parsed"]
    if st.button(
        "Load parsed items into the editor",
        disabled=parsed is None,
        use_container_width=True,
        help="Replaces the new items with the parsed ones, to extend an existing test.",
    ):
        editor.load(parsed)
        st.session_state["edit_sync"] = True
        st.rerun()
    st.caption(f"{editor.count(item_type)} in {item_type.lower()}, {editor.count()} new items in total.")

    render_edit_panel(editor, base)

    st.header("Output")
    coordinates = st.radio("Coordinates", (OFFSETS, ABSOLUTE), key="coordinates")
    if base is None:
        st.caption("kBaseLat/kBaseLon are unknown. Define them in the source or use absolute coordinates.")
    else:
        alt_text = f", kAlt {base.alt:.0f} m" if base.alt is not None else ""
        st.caption(f"Base: {base.lat:.6f}, {base.lon:.6f}{alt_text}")
    if st.button(
        "Recenter map on base",
        disabled=base is None,
        use_container_width=True,
        help="Moves the map back to kBaseLat/kBaseLon at the default zoom.",
    ):
        recenter_on_base(base)
    show_3d = st.checkbox("Show 3D view", key="show_3d")

    settings = ClickSettings(alt=alt, cmd=cmd, grid_m=grid)
    return item_type, settings, coordinates == OFFSETS and base is not None, show_3d


# ---- Editing the selected item ----


def selected_point(editor: Editor) -> Optional[Waypoint]:
    """The selected item, dropping a selection that undo, clear or delete made stale."""
    point = editor.get(st.session_state["selected"])
    if point is None:
        st.session_state["selected"] = None
        st.session_state["moving"] = False
    return point


def fill_edit_widgets(point: Waypoint, base: Optional[Base], use_offsets: bool) -> None:
    """Seed the edit form from the item. Only allowed before the widgets are drawn in this run."""
    state = st.session_state
    if use_offsets:
        north, east = latlon_to_offset(base.lat, base.lon, point.lat, point.lon)
        state["edit_north"], state["edit_east"] = round(north, 2), round(east, 2)
    else:
        state["edit_lat"], state["edit_lon"] = point.lat, point.lon
    state["edit_alt"] = point.alt
    kinds = {cmd: kind for kind, cmd in MISSION_COMMANDS.items()}
    state["edit_cmd"] = kinds.get(point.cmd, point.cmd)
    state["edit_offsets"] = use_offsets


def render_edit_panel(editor: Editor, base: Optional[Base]) -> None:
    """The selected new item: position, altitude and command, applied as they change."""
    st.header("Edit item")
    point = selected_point(editor)
    if point is None:
        st.caption("Click one of the new items on the map to edit it here.")
        return
    selection = st.session_state["selected"]
    st.caption(f"Editing {editor.name(selection)}. Changes apply as you type.")
    # The coordinates radio is drawn later in the sidebar, so its value is the previous run's.
    use_offsets = base is not None and st.session_state.get("coordinates", OFFSETS) == OFFSETS
    if st.session_state.pop("edit_sync", False) or st.session_state.get("edit_offsets") != use_offsets:
        fill_edit_widgets(point, base, use_offsets)
    if use_offsets:
        north = st.number_input("North of base (m)", step=10.0, format="%.2f", key="edit_north")
        east = st.number_input("East of base (m)", step=10.0, format="%.2f", key="edit_east")
        lat, lon = offset_to_latlon(base.lat, base.lon, north, east)
    else:
        lat = st.number_input("Latitude", step=1e-5, format="%.7f", key="edit_lat")
        lon = st.number_input("Longitude", step=1e-5, format="%.7f", key="edit_lon")
    alt = st.number_input("Altitude (m AMSL)", step=10.0, key="edit_alt")
    cmd = point.cmd
    if selection[0] == MISSION:
        kinds = dict(MISSION_COMMANDS)
        if point.cmd not in kinds.values():
            kinds[point.cmd] = point.cmd  # a parsed command the sidebar does not offer
        cmd = kinds[st.selectbox("Command", list(kinds), key="edit_cmd")]
    editor.update(selection, lat, lon, alt, cmd)
    if st.session_state["moving"]:
        st.info("Click the map to place the item.")
    move_column, delete_column = st.columns(2)
    if st.session_state["moving"]:
        if move_column.button("Cancel move", use_container_width=True):
            st.session_state["moving"] = False
            st.rerun()
    elif move_column.button("Move to next map click", use_container_width=True):
        st.session_state["moving"] = True
        st.rerun()
    if delete_column.button("Delete", use_container_width=True):
        editor.remove(selection)
        st.session_state["selected"] = None
        st.session_state["moving"] = False
        st.rerun()
    if st.button("Deselect", use_container_width=True):
        st.session_state["selected"] = None
        st.session_state["moving"] = False
        st.rerun()


def select_marker(editor: Editor, event: Optional[Dict]) -> bool:
    """Select the new item whose marker was clicked. st_folium repeats the last click, so it is remembered."""
    event = event or {}
    tooltip = event.get("last_object_clicked_tooltip")
    clicked = event.get("last_object_clicked") or {}
    key = (tooltip, clicked.get("lat"), clicked.get("lng"))
    if not tooltip or key == st.session_state["last_object_click"]:
        return False
    st.session_state["last_object_click"] = key
    selection = editor.locate(tooltip)
    if selection is None or selection == st.session_state["selected"]:
        return False
    st.session_state["selected"] = selection
    st.session_state["moving"] = False
    st.session_state["edit_sync"] = True
    return True


# ---- The map ----


def add_plan_layers(layer, plan: PlanRun) -> None:
    """Every vehicle's path and join; branch-off and goal pins only for the vehicle shown in the panel."""
    shown = plan.run_for(st.session_state.get("plan_vehicle")).vehicle.name
    _, items = planner_cli.first_mission(plan.scene)
    for index, run in enumerate(plan.runs):
        path = planner_cli.plan_path(items, run.vehicle, run.result)
        name = run.vehicle.name if len(plan.runs) > 1 else ""
        color = PLAN_COLORS[index % len(PLAN_COLORS)]
        add_plan_layer(layer, run.result, path, name, color, goal_pins=run.vehicle.name == shown)


def render_map(editor: Editor, item_type: str, settings: ClickSettings, base: Optional[Base]) -> None:
    center, zoom = st.session_state["view"]
    base_map = build_base_map(center, zoom, st.session_state["parsed"])
    selection = st.session_state["selected"]
    new_items = build_new_items_layer(editor.scene(), item_type == MISSION, editor.get(selection))
    plan = st.session_state["plan"]
    if plan is not None:
        add_plan_layers(new_items, plan)
    # st_folium sizes its iframe from the body height measured while Leaflet's CSS is still
    # loading, and never corrects it, so the frame is often far taller than the map.
    st.markdown(
        f"<style>iframe[title='streamlit_folium.st_folium'] {{ height: {MAP_HEIGHT}px !important; }}</style>",
        unsafe_allow_html=True,
    )
    # center and zoom only move the map when they change, so the user's pan survives edits.
    event = st_folium(
        base_map,
        key=f"map-{st.session_state['map_key']}",
        height=MAP_HEIGHT,
        use_container_width=True,
        center=center,
        zoom=zoom,
        feature_group_to_add=new_items,
        returned_objects=MAP_EVENTS,
    )
    if select_marker(editor, event):
        st.rerun()
    if st.session_state["moving"] and selection is not None:
        if editor.move_to_click(event, selection, base, settings.grid_m):
            st.session_state["moving"] = False
            st.session_state["edit_sync"] = True
            st.rerun()
    elif editor.add_click(event, item_type, settings, base):
        st.rerun()


def render_code(editor: Editor, base: Optional[Base]) -> None:
    code = generate_cpp(editor.scene(), base)
    if not code:
        st.info("Click the map to add items. Their C++ appears here and follows every edit.")
        return
    st.caption("New items only. Hover the block and use the copy button in its top right corner.")
    st.code(code, language="cpp")


# ---- The planner ----


def planning_scene(editor: Editor) -> Tuple[Scene, str]:
    """New items when there are any, else the parsed test."""
    if editor.count():
        return editor.scene(), "the new items"
    parsed = st.session_state["parsed"]
    return (parsed, "the parsed test") if parsed is not None else (Scene(), "nothing yet")


def render_request(items: List[Waypoint]) -> PlanRequest:
    """Vehicle state in view, planner parameters in an expander, labelled with their PX4 parameter."""
    defaults = PlanRequest()
    last_index = max(len(items) - 1, 0)

    def index_input(column, label, key, value, min_value=-1):
        return int(
            column.number_input(label, min_value=min_value, max_value=last_index, value=value, key=key)
        )

    def metres(label, key, value, step=1.0):
        return st.number_input(label, min_value=0.0, value=value, step=step, key=key)

    left, right = st.columns(2)
    mission_index = index_input(
        left, "Mission index (current_seq)", "req_mission_index", min(1, last_index), 0
    )
    mission_land_index = index_input(
        right, "Mission land index (-1 = none)", "req_land_index", planner_cli.land_index(items)
    )
    active_jump_anchor = index_input(left, "Active DO_JUMP item (-1 = none)", "req_jump", -1)
    reversed_now = right.checkbox("Currently flying the route in reverse", key="req_reversed")
    with st.expander("Parameters", expanded=False):
        home_altitude = st.number_input(
            "Home altitude AMSL (m)", value=defaults.home_altitude_amsl, step=10.0, key="req_home_alt"
        )
        vehicle_margin = metres(
            "Vehicle cross-track margin (m), MIS_MC_SEG_DIST / MIS_FW_SEG_DIST",
            "req_veh_margin",
            defaults.projection_search_distance_m,
            10.0,
        )
        rally_margin = metres(
            "Rally cross-track margin (m), RTL_RP_SEG_DIST",
            "req_rally_margin",
            defaults.safe_point_projection_search_distance_m,
            10.0,
        )
        acceptance = metres("Acceptance radius (m), NAV_ACC_RAD", "req_acc", defaults.acceptance_radius_m)
        direct_acceptance = metres(
            "Direct-to-goal radius (m)", "req_direct_acc", defaults.direct_goal_acceptance_radius_m
        )
        altitude_acceptance = metres(
            "Altitude acceptance (m)", "req_alt_acc", defaults.altitude_acceptance_radius_m
        )
        u_turn = metres(
            "Fixed-wing U-turn penalty (m), RTL_FW_UTURN_PEN",
            "req_uturn",
            defaults.fw_u_turn_penalty_m,
            500.0,
        )
        fixed_wing = st.checkbox("Fixed-wing", key="req_fw")
        is_vtol = st.checkbox("VTOL", key="req_vtol")
        in_transition = st.checkbox("In transition to fixed-wing", key="req_transition", disabled=not is_vtol)
        upload_state = st.selectbox(
            "VTOL state at mission upload",
            planner_cli.VTOL_STATES,
            key="req_upload_state",
            disabled=not is_vtol,
        )
        require_approach = st.checkbox(
            "Require a VTOL landing approach", key="req_approach", disabled=not is_vtol
        )
    return PlanRequest(
        mission_index=mission_index,
        mission_land_index=mission_land_index,
        current_route_direction_reversed=reversed_now,
        active_jump_anchor=active_jump_anchor,
        home_altitude_amsl=home_altitude,
        projection_search_distance_m=vehicle_margin,
        safe_point_projection_search_distance_m=rally_margin,
        acceptance_radius_m=acceptance,
        direct_goal_acceptance_radius_m=direct_acceptance,
        altitude_acceptance_radius_m=altitude_acceptance,
        fw_u_turn_penalty_m=u_turn,
        is_fixed_wing=fixed_wing,
        is_vtol=is_vtol,
        in_transition_to_fw=in_transition and is_vtol,
        vtol_state_on_mission_upload=upload_state if is_vtol else "UNDEFINED",
        require_vtol_approach=require_approach and is_vtol,
    )


def run_planner(binary, scene: Scene, request: PlanRequest) -> None:
    """One CLI run per vehicle position, all sharing the request."""
    try:
        vehicles = planner_cli.vehicle_positions(scene)
        if not vehicles:
            raise ValueError("The scenario has no vehicle position.")
        runs = []
        for vehicle in vehicles:
            scenario = planner_cli.encode(scene, request, vehicle)
            runs.append(VehicleRun(vehicle, scenario, planner_cli.run(binary, scenario)))
    except (ValueError, RuntimeError, OSError) as error:
        st.session_state["plan"] = None
        st.session_state["plan_error"] = str(error)
    else:
        st.session_state["plan"] = PlanRun(request, scene, runs)
        st.session_state["plan_error"] = None
        if st.session_state.get("plan_vehicle") not in [run.vehicle.name for run in runs]:
            st.session_state["plan_vehicle"] = runs[0].vehicle.name
    st.rerun()


def render_planner(scene: Scene, label: str) -> None:
    """The panel next to the map: build state, the request, the two actions and the last plan."""
    st.subheader("Planner")
    root = planner_cli.repo_root()
    binary = planner_cli.binary_path(root) if root else None
    if binary is None or not binary.is_file():
        st.info("Build the planner CLI once, from the PX4 repository root, then reload this page:")
        command = (
            planner_cli.build_command(root) if root else "Run this tool from a checkout of PX4-Autopilot."
        )
        st.code(command, language="bash")
        return
    stale = planner_cli.stale_sources(root, binary)
    if stale:
        st.warning(
            f"The planner binary is older than {len(stale)} source file(s), for example {stale[0]}. Rebuild it:"
        )
        st.code(planner_cli.build_command(root), language="bash")
    _, items = planner_cli.first_mission(scene)
    vehicles = planner_cli.vehicle_positions(scene)
    ready = bool(items) and bool(vehicles)
    if ready:
        st.caption(
            f"Runs on {label}: {len(items)} mission items, {len(scene.safe_points)} safe points, "
            f"{len(vehicles)} vehicle position(s). DO_JUMP and VTOL transitions are not sent."
        )
    else:
        st.caption("Parse a test, or add mission items and a vehicle position, to enable the planner.")
    request = render_request(items)
    for column, (button_label, action) in zip(st.columns(2), PLANNER_ACTIONS):
        if column.button(button_label, use_container_width=True, disabled=not ready):
            request.action = action
            run_planner(binary, scene, request)
    render_plan_result(scene, request)


def plan_summary(result: PlanResult) -> str:
    direction = "reverse" if result.bool_field("direction_reversed") else "nominal"
    return f"first item {result.int_field('first_mission_item_index')}, {direction} direction"


def summary_row(run: VehicleRun) -> Dict[str, str]:
    result = run.result
    return {
        "vehicle": run.vehicle.name,
        "result": plan_summary(result) if result.ok else f"failed: {result.failure}",
        "goal": result.plan.get("goal_type", ""),
    }


def plan_current(plan: PlanRun, scene: Scene, request: PlanRequest) -> bool:
    """Whether the plan on the map still matches the scene and the request."""
    vehicles = planner_cli.vehicle_positions(scene)
    if [vehicle.name for vehicle in vehicles] != plan.names():
        return False
    try:
        return all(
            planner_cli.encode(scene, request, vehicle) == run.scenario
            for vehicle, run in zip(vehicles, plan.runs)
        )
    except ValueError:
        return False


def render_plan_result(scene: Scene, request: PlanRequest) -> None:
    error = st.session_state["plan_error"]
    if error:
        st.error(error)
    plan = st.session_state["plan"]
    if plan is None:
        return
    if len(plan.runs) > 1:
        st.table(pd.DataFrame([summary_row(run) for run in plan.runs]).set_index("vehicle"))
        st.selectbox("Details, unit test and trace for", plan.names(), key="plan_vehicle")
    result = plan.run_for(st.session_state.get("plan_vehicle")).result
    if result.ok:
        goal = result.plan.get("goal_type", "")
        st.success(f"{result.action}: {plan_summary(result)}" + (f", goal {goal}" if goal else ""))
    else:
        st.error(f"{result.action} failed: {result.failure}")
    request.action = plan.request.action
    if not plan_current(plan, scene, request):
        st.caption("The scenario or the request changed since this plan. Run it again to refresh the map.")
    if result.plan:
        st.table(pd.DataFrame(result.rows()).set_index("field"))


def render_outputs(editor: Editor, base: Optional[Base], use_offsets: bool) -> None:
    """Under the map: the fixtures' C++, the unit test and the trace of the vehicle shown in the panel."""
    code_tab, test_tab, trace_tab = st.tabs(["Generated C++", "Unit test", "Planner trace"])
    with code_tab:
        render_code(editor, base if use_offsets else None)
    plan = st.session_state["plan"]
    run = plan.run_for(st.session_state.get("plan_vehicle")) if plan is not None else None
    with test_tab:
        if run is None:
            st.info("Run the planner first. The test asserts what it returned.")
        else:
            st.caption("Check the plan on the map first, then describe the intent in the comment at the top.")
            name = st.text_input("Test name", value="GeneratedScenario", key="test_name")
            code = generate_unit_test(plan.scene, base, plan.request, run.result, name, run.vehicle)
            st.code(code, language="cpp")
    with trace_tab:
        if run is None:
            st.info("Run the planner first. Its debug output appears here.")
        else:
            st.code("\n".join(run.result.trace) or "No trace lines.", language="text")


def render_3d(editor: Editor) -> None:
    try:
        from .map3d import build_deck
    except ModuleNotFoundError:
        st.info("Install pydeck for the 3D view: pip install pydeck")
        return
    center, zoom = st.session_state["view"]
    st.pydeck_chart(build_deck(st.session_state["parsed"], editor.scene(), center, zoom))


def main() -> None:
    st.set_page_config(layout="wide", page_title="PX4 mission route test helper")
    init_state()
    editor = st.session_state["editor"]
    st.title("Mission route test helper")
    st.caption(
        "Paste a planner test to see its fixtures, click the map to add items and click an item to "
        "edit it, run the real planner on the result, and copy the C++."
    )
    render_source_panel()
    base = current_base()
    scene, label = planning_scene(editor)
    with st.sidebar:
        item_type, settings, use_offsets, show_3d = render_sidebar(editor, base)
    # The tabs share the map column, so they start right under the map whatever the panel height.
    map_column, planner_column = st.columns([2, 1], gap="medium")
    with map_column:
        render_map(editor, item_type, settings, base)
        render_outputs(editor, base, use_offsets)
    with planner_column:
        render_planner(scene, label)
    if show_3d:
        render_3d(editor)
