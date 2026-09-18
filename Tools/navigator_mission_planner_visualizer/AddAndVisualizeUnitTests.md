# Mission route test helper

Streamlit tool for the navigator mission route planner tests. Paste a test to see its mission, safe points and vehicle position on a map, click the map to add items, and copy the generated C++ back into the test.

The generated code uses the helpers of `src/modules/navigator/test/support/mission_route_test_helpers.h`. With the optional planner CLI built, the real planner runs on the scenario and its answer is drawn on the map.

## Running

```bash
pip install streamlit folium streamlit-folium pydeck
cd Tools/navigator_mission_planner_visualizer
streamlit run mission_planner_tools.py
```

Checked with Python 3.8, Streamlit 1.40, folium 0.18, streamlit-folium 0.22 and pydeck 0.9.

## Workflow

1. Paste one test, including the mission data it references, into **C++ source** and press **Parse**. The map zooms to the fixtures. Hover an item for its name and altitude, or a mission leg for its length.
2. Pick an item type in the sidebar and click the map. Clicks snap to a grid around kBaseLat/kBaseLon (10 m by default) so offsets come out as round numbers. Mission items can be waypoints, takeoff, land or loiter to altitude. The vehicle position carries the request velocity.
3. Click one of the new items to select it. **Edit item** in the sidebar shows its offsets from the base (or its lat/lon), its altitude and, for mission items, its command; changes apply as you type. **Move to next map click** puts it where you click next, **Delete** removes it, **Undo** covers all of this. Parsed items are read-only until they are loaded into the editor.
4. The **Generated C++** tab below the map follows every edit. Copy it with the button in the corner of the code block.

**Load parsed items into the editor** copies the parsed fixtures into the editor, to extend an existing test. DO_JUMP and VTOL transition items have no position and are dropped; add them back by hand.

## What the parser reads

- Mission containers: `std::vector<mission_item_s> mission{...}` and `= {...}`, `auto m = std::vector<mission_item_s>{...}`, `std::array<mission_item_s, N>`, `push_back(...)`, and functions returning `{...}` such as the ones in `test_mission_route_data.h`. Item helpers outside a container are read in order as one mission.
- Item helpers: `makePositionItem`, `makeTakeoffItem`, `makeLandItem` and their `FromOffset` variants, with an optional nav command. `makeDoJump` and `makeVtolTransitionItem` are skipped without breaking the route.
- Safe points: `makeSafePointAbsolute`, `makeSafePointFromOffset`.
- Positions: `makePositionAbsolute`, `makePositionFromOffset`, `mission_route::Position p{...}` and `x = mission_route::Position{...}`. The variable name is kept, and names containing "vehicle" get the velocity arrow.
- Velocity: `request.velocity_north_m_s = ...;` and `velocity_east_m_s`.
- Constants: numeric `const`/`constexpr` definitions and `+ - * /` between them. `kBaseLat`, `kBaseLon` and `kAlt` default to the values of the helper header; a definition in the pasted source overrides them.

The parser does not execute code, follow includes or unroll loops. Paste one scenario at a time.

## Output

With **Offsets from kBaseLat/kBaseLon** (the default) the output matches the style of the planner tests:

```cpp
std::vector<mission_item_s> mission{
	makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt), // 0
	makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f), // 1
	makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f), // 2
};

std::vector<mission_item_s> safe_points{
	makeSafePointFromOffset(kBaseLat, kBaseLon, 60.f, 5.f, kAlt), // 0
};

const mission_route::Position vehicle_position = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

request.velocity_north_m_s = 5.f;
request.velocity_east_m_s = 0.f;
```

**Absolute lat/lon** emits `makePositionItem(lat, lon, alt)`, `makeSafePointAbsolute(...)` and `makePositionAbsolute(...)` instead.

Offsets use the same spherical earth model as `add_vector_to_global_position()`, so the generated code reproduces the clicked point.

## Running the planner

The tool can run the real `MissionRoutePlanner` on the scenario shown on the map. The planner is not linked into Python: a small command-line binary, `mission_route_planner_cli`, is built with the normal PX4 toolchain, generated headers and board configuration, and the tool starts it for every run. Nothing under `src/` changes for this; the binary lives in `planner_cli/` and enters the build through PX4's `EXTERNAL_MODULES_LOCATION` hook. Build it once from the repository root:

```bash
make px4_sitl_test EXTERNAL_MODULES_LOCATION=$PWD/Tools/navigator_mission_planner_visualizer/planner_cli mission_route_planner_cli
```

Rebuild the same way after changing the planner sources. Each run starts the binary afresh, so a rebuilt binary is picked up on the next click without restarting the tool. The **Planner** panel next to the map compares the binary's timestamp with the planner sources and shows this command when the binary is stale, or when it has not been built yet.

The panel works on the new items when there are any, otherwise on the parsed test, and needs mission items and a vehicle position. Only position items reach the planner: `DO_JUMP` and VTOL transition items are not part of a parsed scene.

Every vehicle position gets its own run, all with the same request. Each path is drawn in its own colour with the vehicle name in its tooltip, a table under the buttons summarises the runs, and the plan fields, the unit test and the trace follow the vehicle picked under that table. A drawn path stops at the first `LAND` item of the route.

- **Plan Return** runs the route-following Return: join point, route direction, branch-off and the selected goal.
- **Plan Mission rejoin** runs the Mission resume join.

The planned path is drawn as a dashed red line, the join point as a red pin, the branch-off as a dark red pin and the goal as a green flag. The plan fields appear under the buttons.

The vehicle state sits above the buttons: mission index (`current_seq`), land index, the active `DO_JUMP` item and whether the route is currently flown in reverse. **Parameters** holds the rest, labelled with the PX4 parameter it comes from: cross-track margins (`MIS_MC_SEG_DIST`, `MIS_FW_SEG_DIST`, `RTL_RP_SEG_DIST`), acceptance radii, the fixed-wing U-turn penalty (`RTL_FW_UTURN_PEN`), home altitude, vehicle type and VTOL state. Change them freely between runs; nothing is rebuilt.

Under the map, **Planner trace** shows the planner's own debug output, which the CLI compiles in; the text is informal and may change.

**Unit test** turns the scenario and the returned plan into a `TEST_F` for `test_mission_route_planner.cpp`, using the same helpers as the existing tests. It asserts what the planner returned, so review the plan on the map first and describe the intent in the comment at the top; a test that merely records the current output is worth little.

## Tests

```bash
python3 -m unittest discover -s Tools/navigator_mission_planner_visualizer -p 'test_*.py'
```

`test_parser.py`, `test_generator.py`, `test_editor.py`, `test_geometry.py` and `test_planner_cli.py` need only the standard library; the last one uses a fake binary, so it does not need the CLI built. The rendering and app tests need the UI dependencies.
