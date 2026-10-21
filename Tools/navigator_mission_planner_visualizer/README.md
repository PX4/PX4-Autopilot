# Mission route test helper

Streamlit tool for the navigator mission route planner tests. Paste a test to see its mission, safe points, vehicle position and geofences on a map, click the map to add items, and copy the generated C++ back into the test.

The generated code uses the helpers of `src/modules/navigator/test/support/mission_route_test_helpers.h`. With the optional planner CLI built, the real planner runs on the scenario and its answer is drawn on the map.

## Running

From the repository root:

```bash
pip install streamlit folium streamlit-folium
make px4_sitl_test EXTERNAL_MODULES_LOCATION=$PWD/Tools/navigator_mission_planner_visualizer/planner_cli mission_route_planner_cli
cd Tools/navigator_mission_planner_visualizer
streamlit run mission_planner_tools.py
```

The `make` line builds the optional planner CLI, see [Running the planner](#running-the-planner).

Checked with Python 3.8, Streamlit 1.40, folium 0.18 and streamlit-folium 0.22.

## Workflow

1. Paste one test, including the mission data it references, into **C++ source** and press **Parse**. The map zooms to the fixtures. Hover an item for its name and altitude, or a mission leg for its length.
2. Pick an item type in the sidebar and click the map. Clicks snap to a grid around kBaseLat/kBaseLon (10 m by default) so offsets come out as round numbers. Mission items can be waypoints, takeoff, land or loiter to altitude. The vehicle position carries the request velocity.
3. Click one of the new items to select it. **Edit item** in the sidebar shows its offsets from the base (or its lat/lon), its altitude and, for mission items, its command; changes apply as you type. **Move to next map click** puts it where you click next, **Delete** removes it, **Undo** covers all of this. Parsed items are read-only until they are loaded into the editor.
4. The **Generated C++** tab below the map follows every edit. Copy it with the button in the corner of the code block.

**Load parsed items into the editor** copies the parsed fixtures into the editor, to extend an existing test. Only the first mission is loaded, the same one the planner uses for the parsed test. DO_JUMP and VTOL transition items have no position and are dropped; add them back by hand. Dropping them shifts the index of the items after them.

Parsing imports geometry and velocity. Set the mission index and other planner parameters to match the pasted test before running it.

## Drawing geofences

To add fences to a pasted test, first select **Load parsed items into the editor**. The planner uses the editor's items whenever it contains any; it does not combine them with the parsed test.

1. Set **Item type** to **Geofences**.
2. Choose **Inclusion** (green, flight allowed inside) or **Exclusion** (red, flight prohibited inside).
3. Choose a shape:
   - **Polygon**: click at least three vertices in boundary order, then select **Finish polygon**. Do not repeat the first vertex. The dashed draft is not used by the planner or exported to C++. **Cancel polygon** discards it.
   - **Circle**: set **Radius (m)** and click the center.
4. Select **Plan Return** or **Plan Mission rejoin** to check the edited scene. Rerun after each change.

The grid setting also applies to fence vertices and circle centers. **Undo** restores previous edits, including polygon drafts. To remove a fence, choose it under **Existing geofence** and select **Delete geofence**. **Clear geofences** removes all editor fences. Fences have no editable altitude bounds.

The CLI loads these fences through PX4's `Geofence` and uses `GeofencePathChecker`, the same checks as Navigator. It checks the vehicle-to-branch-in segment, branch-off-to-safe-point segments and route-skip shortcuts. If the vehicle already breaches the fence, segments that start at the vehicle are not checked, to allow recovery; safe point segments still are. The trace reports this.

This tool does not run mission-upload feasibility validation. Drawing a fence across an existing mission leg does not remove that leg from the route. It also does not validate landing approaches or find detours around fences. Home altitude is configurable, but Home latitude/longitude and distance-from-home limits are not part of the scene.

## What the parser reads

- Mission containers: `std::vector<mission_item_s> mission{...}` and `= {...}`, `auto m = std::vector<mission_item_s>{...}`, `std::array<mission_item_s, N>`, `push_back(...)`, and functions returning `{...}` such as the ones in `test_mission_route_data.h`. Item helpers outside a container are read in order as one mission.
- Item helpers: `makePositionItem`, `makeTakeoffItem`, `makeLandItem` and their `FromOffset` variants, with an optional nav command. `makeDoJump` and `makeVtolTransitionItem` are skipped without breaking the route.
- Safe points: `makeSafePointAbsolute`, `makeSafePointFromOffset`.
- Positions: `makePositionAbsolute`, `makePositionFromOffset`, `routePosition`, `mission_route::Position p{...}` and `x = mission_route::Position{...}`. The variable name is kept, and names containing "vehicle" get the velocity arrow.
- Velocity: `request.velocity_north_m_s = ...;` and `velocity_east_m_s`.
- Geofences: the exported `makeFencePoint(...)` records, plus `polygon(...)`, `circle(...)` and `exclusionSquare()` from the current geofence tests, and `exclusionCircle(...)` from the Return tests. The geofence fixture helpers, including `routePosition`, use offsets from (47, 8); `exclusionCircle` uses `kBaseLat`/`kBaseLon`. Paste the relevant fixture calls along with the mission and vehicle.
- Constants: numeric `const`/`constexpr` definitions and `+ - * /` between them. `kBaseLat`, `kBaseLon` and `kAlt` default to the values of the helper header; a definition in the pasted source overrides them.

Frame arguments are ignored and every altitude is read as AMSL, so a `NAV_FRAME_GLOBAL_RELATIVE_ALT` safe point is planned at the wrong altitude. The parser does not execute code, follow includes or unroll loops. Paste one scenario at a time.

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

Scenes with fences always export absolute latitude/longitude for every item, at the CLI's nine-decimal precision. This avoids moving a point across a fence boundary when rounding offsets. The generated `makeFencePoint` helper creates `mission_fence_point_s` records for polygons and circles. These records can be pasted back into the tool.

## Running the planner

The tool runs the real `MissionRoutePlanner` through a command-line binary, `mission_route_planner_cli`, built with the PX4 toolchain. Its source is in `planner_cli/` and enters the build through PX4's `EXTERNAL_MODULES_LOCATION` hook. The executable is `build/px4_sitl_test/mission_route_planner_cli`. It links Navigator and Dataman to load geofences into RAM for each run. Without fences, it runs the planner without geofence checks. It is built by the `make` line under [Running](#running).

Rebuild the same way after changing the planner sources. Each run starts the binary afresh, so a rebuilt binary is picked up on the next click without restarting the tool. The **Planner** panel next to the map compares the binary's timestamp with the planner sources and shows this command when the binary is stale, or when it has not been built yet.

The panel works on the new items when there are any, otherwise on the parsed test, and needs mission items and a vehicle position. Only position items reach the planner: `DO_JUMP` and VTOL transition items are not part of a parsed scene.

Every vehicle position gets its own run, all with the same request. Each path is drawn in its own colour with the vehicle name in its tooltip, a table under the buttons summarises the runs, and the plan fields, the unit test and the trace follow the vehicle picked under that table. A drawn path stops at the first `LAND` item of the route.

- **Plan Return** runs the route-following Return: join point, route direction, branch-off and the selected goal.
- **Plan Mission rejoin** runs the Mission resume join.

The planned path is drawn as a dashed line, with pins for branch-in, branch-off and the goal. The plan fields appear under the buttons, including `geofence_prevents_shortcut` when a fence requires following the route instead of flying directly to the goal.

The vehicle state sits above the buttons: mission index (`current_seq`), land index and whether the route is currently flown in reverse. **Parameters** holds the rest, labelled with the PX4 parameter it comes from: cross-track margins (`MIS_MC_SEG_DIST`, `MIS_FW_SEG_DIST`, `RTL_RP_SEG_DIST`), acceptance radii, the fixed-wing U-turn penalty (`RTL_FW_UTURN_PEN`), home altitude, vehicle type and VTOL state. Change them freely between runs; nothing is rebuilt.

Under the map, **Planner trace** shows the planner's own debug output, which the CLI compiles in; the text is informal and may change.

**Unit test** turns the scenario and returned plan into a `TEST_F`:

- Without fences, paste it into `src/modules/navigator/test/test_mission_route_planner.cpp`.
- With fences, paste it into `src/modules/navigator/test/test_mission_route_geofence.cpp` and add `#include "mission_route_planner.h"` and `#include "support/mission_route_test_helpers.h"` to that file if they are missing. It loads the drawn fences and uses the real `GeofencePathChecker`. Like the CLI, it leaves Home latitude/longitude unset so the loader does not discard fences based on an invented Home position.

The test asserts what the planner returned. Review the plan on the map first and describe the intent in the comment at the top; a test that merely records the current output is worth little.

## Tests

```bash
python3 -m unittest discover -s Tools/navigator_mission_planner_visualizer -p 'test_*.py'
```

The parser, generator, editor, geometry and CLI protocol tests need only the Python standard library. The rendering and app tests need the UI dependencies and are skipped without them.

`test_planner_cli.py` also runs integration tests against the real CLI when it is built and current. Otherwise those tests are skipped. Build the CLI first to check polygon/circle pruning, recovery behavior and invalid input.
