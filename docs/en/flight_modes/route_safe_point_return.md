# Route Safe Point Return

_Route Safe Point Return_ is a mission-aware [Return Mode](./return.md) that uses the uploaded mission geometry as the return corridor. PX4 projects the vehicle and every uploaded safe point onto the mission route, chooses the closest safe point along the route, follows the route in nominal or reverse direction, reaches the branch-off point defined as the projection of the selected safe-point on the route and only then branches off horizontally from the flight plan and proceeds to land at the safe point location.

Set [RTL_TYPE=6](../advanced_config/parameter_reference.md#RTL_TYPE) to enable it.

This mode is intended for operations where the mission itself is the safest known path through terrain, obstacles, or airspace constraints.Unlike direct RTL variants, it does not assume that the safest way home is a straight line.

::: info
- If no [safety points (rally points)](../flying/plan_safety_points.md) can be selected, PX4 falls back to the closer mission endpoint (landing or takeoff) while staying in the route-based return logic.
- Regardless of the direction of flight, the vehicle automatically skips special commands such as loops.
- If the mission itself cannot be projected, PX4 falls back to a direct return (RTL_TYPE=0 behavior).
:::

## Setup and Configuration

Route Safe Point Return requires:

- A valid mission with at least two position items.
- [RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) set to `6`.

Tuning parameters:

| Parameter | Description |
| --- | --- |
| [RTL_MC_SEG_DIST](../advanced_config/parameter_reference.md#RTL_MC_SEG_DIST) | Extra cross-track search window for multicopter or VTOL-in-MC vehicle projection. |
| [RTL_FW_SEG_DIST](../advanced_config/parameter_reference.md#RTL_FW_SEG_DIST) | Extra cross-track search window for fixed-wing or VTOL-in-FW vehicle projection. |
| [RTL_RP_SEG_DIST](../advanced_config/parameter_reference.md#RTL_RP_SEG_DIST) | Extra cross-track search window for safe-point projection. Increase if safe points are placed far from the mission route. |
| [NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD) | Affects join acceptance, branch-off acceptance, and the direct-to-safe-point shortcut. |

::: tip
Larger search windows expose more candidate segments but also admit more distant branch-off options. Smaller windows keep the behavior closer to the nominal route.
:::

## How It Works

### Mode Entry and Selection

When `RTL_TYPE=6` is evaluated, PX4 performs these steps:

1. Project the vehicle onto the mission route.
2. Project all safe points onto the mission route.
3. Score the reachable safe-point projections by along-route cost.
4. If no safe point is usable, fall back to the closer mission endpoint (takeoff or land).
5. Build a route-join, route-follow, and branch-off plan from the result.

### Vehicle Projection

When the RTL mode is triggered, the vehicle might have deviated from the route e.g. with a GoTo command. Since the _Route Safe Point Return_ follows the mission path, the very first step is to rejoin the mission path at a "branch-in" point on the planned route.

The algorithm for selecting a branch-in point is executed in three phases:

**Phase 1 - Identifying valid candidates:**

The system first identifies up to three potential projection points on the flight route by calculating perpendicular lines from the vehicle's current position to all route segments. A projection point is only considered a valid candidate if the crosstrack distance from the vehicle to the segment does not not exceed the crosstrack distance to the closest available segment plus an allowed margin. This margin is determined by the vehicle's current flight mode:
 - Multicopter: [RTL_MC_SEG_DIST](../advanced_config/parameter_reference.md#RTL_MC_SEG_DIST) by default, 30 m.
 - Fixed-wing: [RTL_FW_SEG_DIST](../advanced_config/parameter_reference.md#RTL_FW_SEG_DIST) by default, 150 m.

**Phase 2 - Selecting the best branch-in point:**
From the potential candidates, the system selects the best branch-in point using a priority-based system.
 - Priority 1: if one of the valid projection points falls on the route segment the vehicle is currently expected to be flying (last targeted waypoint index), that point is immediately selected.
 - Priority 2: if no point satisfies Priority 1, the system calculates the combined distance score (A + B) for the remaining candidates and selects the point with the lowest score for the most efficient path:
    - A (crosstrack distance): the distance from the vehicle to the projection point on the route.
    - B (distance to last waypoint): the distance from the projection point to either the start or the end of the last flown segment (whichever is closer).

**Phase 3 - Determining the branch-in altitude:**

Once the lateral branch-in point is established, the system calculates the target altitude for that point using the following logic:
 - Linear interpolation: by default, the altitude is calculated via linear interpolation between the altitudes of the segment's start and end waypoints (e.g., if rejoining on segment 2-3, the system interpolates between the altitudes of waypoint 2 and waypoint 3).
 - Special case (Land): if the branch-in point falls on a land segment, the system defaults to the altitude of the previous waypoint.
 - Special case (short segments): if the segment is too short for reliable interpolation (such as in the case of superimposed waypoints), the system uses the altitude of the segment's end waypoint.

### Safe-Point Scoring

When the RTL mode is active, the vehicle follows the mission route until it reaches a branch-off point and flies straight to the safe point. The safe point selection is based on the along mission distance from the vehicle projection to the safe point. In order to evaluate the mission along distance, the first step is to find candidate "branch-off" points on the planned route. The point at which the vehicle will leave the flight plan to go straight to the safe point.

The algorithm for selecting a safe point projection (potential branch-off) is executed in two main phases:

**Phase 1 - Identifying valid candidates:**

The process for identifying valid projection candidates is identical to Phase 1 of the vehicle projection algorithm: a projection point is only considered a valid candidate if the crosstrack distance from the vehicle to the segment does not exceed the crosstrack distance to the closest available segment plus an allowed margin. This margin is determined by [RTL_RP_SEG_DIST](../advanced_config/parameter_reference.md#RTL_RP_SEG_DIST).

**Phase 2 - Selecting the best projection point:**

From the valid candidates, the system evaluates the travel path from each projection point to the safe point destination and selects the one with the lowest total path cost. The cost is calculated based on the following factors:
 - Path Distance: The distance required to fly from the projected point along the mission path to the safe point goal.
 - U-turn Penalty: For Fixed-wing and VTOL-in-FW, a distance penalty ([RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN), default 4,000 m) is added to the cost if the path requires the vehicle to perform a U-turn. This prioritizes forward-flowing paths. Reduce the value for smaller airframes with tighter turn radii, or set to 0 to disable the penalty.

Safe points are loaded once and evaluated in one batched route scan:

- Valid safe points are read from the dataman store.
- Invalid coordinates or unsupported frames are skipped.
- Every valid safe point gets up to three local-minimum route projections.

### Direct-to-Safe-Point Shortcut

Multicopters (and VTOLs currently in MC mode) that are already within `NAV_ACC_RAD` of a safe point may skip route following and navigate straight towards the safe point to land there.

<!-- TODO: this below is not clear, can be improved e.g. with an example as the comment below -->
If the RTL was aborted and is re-triggered while the vehicle is still near the stored branch-off leg (path between the safe point and the branch-off), PX4 keeps the previous safe-point choice and continues navigating straight to that safe point instead of forcing a route rejoin. This prevents repeated mode toggles from pulling the vehicle back to the route after it has already branched off.
 <!-- - Example: RTL is triggered, the vehicle follows the route, branches off towards the safe point, the operator decides to resume the mission and the vehicle starts flying back to the route following the same path towards the branch-off point. If at this point the operator re-triggers a new RTL, there is no need to continue flying towards the branch-off point to reach it and then branch-off again towards the safe point. In that case, the vehicle immediately goes to the safe point. -->


::: info
The cached plan is invalidated whenever the mission or safe-point data changes (new upload or safe-point set update), so the planner never reuses stale geometry.
:::

### Mission-Endpoint Fallback

If no safe point can be selected, Route Safe Point Return falls back to the closest between:

- The mission landing endpoint, flown in the nominal direction.
- The mission takeoff endpoint, flown in reverse.


## Execution Stages

The active executor runs through these stages:

<!-- TODO: add links to the relevant sections here. E.g. a link to Vehicle projection where we explain the branch-in -->

1. **Join route**: fly to a virtual branch-in waypoint at the vehicle projection point.
2. **Post-join transition**: if the new mission segment is expected to be flown in multi-copter mode, apply any required VTOL back-transition before following the route.
3. **Follow route**: follow the mission path in nominal or reverse direction.
4. If a safe point is available:
   - **Branch off**: replace the mission target with the virtual branch-off waypoint defined as the orthogonal projection of the safe point on the route.
   - **Navigate to the safe point**: once the branch-off is reached, navigate straight to the safe point.
5. **Land at goal**: land at the safe point or the selected mission endpoint fallback.

### Join Route

The join point is a virtual `NAV_CMD_WAYPOINT` placed at the vehicle projection.

- If the target route segment requires MC flight while the vehicle is currently in FW mode, the join context requests a VTOL back-transition after the join waypoint is reached.
- If the selected goal is already within the acceptance radius of a landing endpoint, the join altitude requirement is skipped so landing can start immediately.
- If the join projection is already within acceptance radius of the branch-off projection, PX4 goes straight to landing instead of following a zero-length route segment.

### Route Following

During route following, PX4 treats the mission as geometry rather than as a full mission replay:

- Nominal direction walks forward through position items, skipping `DO_JUMP` entries instead of following them as control flow.
- Reverse direction walks backward through position items, also skipping `DO_JUMP` entries.
- Loiter items are converted to plain waypoints with `autocontinue = true` and zero hold time so the vehicle keeps moving.
- `NAV_CMD_DELAY` items are clamped to zero hold time.
- Other non-position mission commands are skipped.

VTOL transition handling is still preserved during route following: PX4 uses the same segment-end anchor rules as the planner so that reverse traversal correctly handles transition items.

### Branch-Off and Landing

The branch-off point is not an actual mission item — PX4 injects it as a virtual waypoint.
As soon as the route target becomes the selected branch-off index, the executor replaces the current target with the branch-off waypoint.
The vehicle branches off at the projected point on the segment, not after flying all the way to the real mission waypoint.

All final landings run through the same `handleLanding()` pipeline used by other mission-based RTL modes, preserving VTOL landing sequences, move-to-land waypoints, and precision landing settings.

::: warning
When falling back to the mission takeoff endpoint in reverse, PX4 lands at ground-level altitude (not the takeoff waypoint altitude).
:::

## Time Estimation

Route Safe Point Return publishes a remaining-flight-time estimate via `rtl_time_estimate`.
The estimate divides the remaining route distance by the active cruising speed.
During the FollowRoute stage, the estimate subtracts the distance already covered so the reported time decreases as the vehicle progresses.

This approximation ignores altitude changes, wind, and VTOL transitions, but gives the operator a useful indication of remaining flight time.

## Mission Size Limit

The route planner caches the entire mission in RAM for non-blocking access during flight. The maximum supported mission size is defined by the board-level Kconfig option `CONFIG_RTL_MISSION_CACHE_SIZE` (default: 300 items). Each cached item uses approximately 76 bytes of heap.

**Missions within the cache limit** are fully cached on upload. The planner evaluates every segment and optimal safe-point selection is guaranteed.

**Missions exceeding the cache limit** cannot use Route Safe Point Return. PX4 logs a warning and automatically falls back to the closest safe destination using direct-path RTL logic (`RTL_TYPE=3` behavior).

To increase the limit for a specific board, set the following in the board's `.px4board` file:

```
CONFIG_RTL_MISSION_CACHE_SIZE=500
```

::: tip
For most real-world operations, 300 waypoints is sufficient. If your mission requires more waypoints, either increase `CONFIG_RTL_MISSION_CACHE_SIZE` for your board or consider splitting the mission into shorter segments.
:::

## Current Limitations

- Missions exceeding `CONFIG_RTL_MISSION_CACHE_SIZE` items (default 300) are not supported; PX4 falls back to direct-path RTL.
- Geofence-aware pruning for vehicle and safe-point projections is not yet implemented.
- No dedicated reverse-turn execution module: U-turns are penalized in path scoring but not executed as a specific maneuver.

## Related Topics

- [Return Mode (all types)](./return.md)
- [Safety Points (Rally Points)](../flying/plan_safety_points.md)
