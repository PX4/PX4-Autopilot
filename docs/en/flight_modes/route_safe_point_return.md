# Route Safe Point Return

_Route Safe Point Return_ is a mission-aware [Return Mode](./return.md) that uses the uploaded mission geometry as the return corridor. PX4 projects the vehicle and every uploaded safe point onto the mission route, chooses the closest safe point along the route, follows the route in nominal or reverse direction, reaches the branch-off point defined as the projection of the selected safe-point on the route and only then branches off horizontally from the flight plan and proceeds to land at the safe point location.

Set [RTL_TYPE=6](../advanced_config/parameter_reference.md#RTL_TYPE) to enable it.

This mode is intended for operations where the mission itself is the safest known path through terrain, obstacles, or airspace constraints. Unlike direct RTL variants, it does not assume that the safest way home is a straight line.

::: info
- If no [safety points (rally points)](../flying/plan_safety_points.md) can be selected, PX4 falls back to the closer mission endpoint (landing or takeoff) while staying in the route-based return logic.
- Regardless of the direction of flight, the vehicle skips `DO_JUMP` commands. Note however that the route planner evaluates jump segments to accurately project the vehicle segment.
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
| [RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN) | U-turn distance penalty for fixed-wing and VTOL-in-FW safe-point scoring. Penalizes paths that require reversing direction. Set to 0 to disable. |
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

The system first identifies up to three potential projection points on the flight route by calculating perpendicular lines from the vehicle's current position to all route segments. A projection point is only considered a valid candidate if the crosstrack distance from the vehicle to the segment does not exceed the crosstrack distance to the closest available segment plus an allowed margin. This margin is determined by the vehicle's current flight mode:
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

When the RTL mode is active, the vehicle follows the mission route until it reaches a branch-off point and flies straight to the safe point. The safe point selection is based on the along-route distance from the vehicle projection to the safe-point projection. In order to evaluate this along-route distance, the first step is to find candidate "branch-off" points on the planned route — the point at which the vehicle will leave the flight plan to go straight to the safe point.

The algorithm for selecting a safe point projection (potential branch-off) is executed in two main phases:

**Phase 1 - Identifying valid candidates:**

The process for identifying valid projection candidates is identical to Phase 1 of the vehicle projection algorithm, except the reference point is the safe point (not the vehicle): a projection point is only considered a valid candidate if the crosstrack distance from the safe point to the segment does not exceed the crosstrack distance to the closest available segment plus an allowed margin. This margin is determined by [RTL_RP_SEG_DIST](../advanced_config/parameter_reference.md#RTL_RP_SEG_DIST).

**Phase 2 - Selecting the best projection point:**

From the valid candidates, the system evaluates the travel path from each projection point to the safe point destination and selects the one with the lowest total path cost. The cost is calculated based on the following factors:
 - Along-Route Distance: The distance along the mission route from the vehicle projection to the safe-point projection (branch-off point). This is measured along the route geometry with straight lines between waypoints.
 - U-turn Penalty: For Fixed-wing and VTOL-in-FW, a distance penalty ([RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN), default 4,000 m) is added to the cost if the path requires the vehicle to perform a U-turn. This prioritizes forward-flowing paths. Reduce the value for smaller airframes with tighter turn radii, or set to 0 to disable the penalty.

Safe points are loaded once and evaluated in one batched route scan:

- Valid safe points are read from the dataman store.
- Invalid coordinates or unsupported frames are skipped.
- Every valid safe point gets up to three local-minimum route projections.

### Direct-to-Safe-Point Shortcut

Multicopters (and VTOLs currently in MC mode) that are already within `NAV_ACC_RAD` of a safe point may skip route following and navigate straight to the safe point to land there.

If the RTL was aborted and is re-triggered while the vehicle is still near the stored branch-off leg (the path between the branch-off point and the safe point), PX4 keeps the previous safe-point choice and continues navigating straight to that safe point instead of forcing a route rejoin. This prevents repeated mode toggles from pulling the vehicle back to the route after it has already branched off.

Example: RTL is triggered, the vehicle follows the route, branches off towards the safe point, and the operator resumes the mission. The vehicle starts flying back towards the branch-off point. If the operator re-triggers RTL at this point, there is no need to continue flying to the branch-off point and then branch off again. Instead, the vehicle immediately goes straight to the safe point.

::: info
The cached plan is invalidated whenever the mission or safe-point data changes (new upload or safe-point set update), so the planner never reuses stale geometry.
:::

### Mission-Endpoint Fallback

If no safe point can be selected, Route Safe Point Return falls back to the closest between:

- The mission landing endpoint, flown in the nominal direction.
- The mission takeoff endpoint, flown in reverse.


## Execution Stages

The active executor runs through these stages:

1. **[Join route](#join-route)**: fly to a virtual branch-in waypoint at the [vehicle projection](#vehicle-projection) point.
2. **Post-join transition**: if the new mission segment is expected to be flown in multi-copter mode, apply any required VTOL back-transition before following the route.
3. **[Follow route](#route-following)**: follow the mission path in nominal or reverse direction.
4. **Transition during route**: if the next route segment requires a different VTOL state, apply the transition and resume route following.
5. If a safe point is available:
   - **[Branch off](#branch-off-and-landing)**: replace the mission target with the virtual branch-off waypoint defined as the orthogonal projection of the safe point on the route.
   - **Navigate to the safe point**: once the branch-off is reached, navigate straight to the safe point.
6. **Land at goal**: land at the safe point or the selected mission endpoint fallback.

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
- When route traversal is completed (no more waypoints in either direction or failure to read the next item), if the vehicle has not landed, the vehicle holds position rather than flying a straight line to the goal.

VTOL transition handling is preserved during route following: PX4 detects the expected VTOL state for each target segment using the same anchor rules as the planner. When a transition is needed, the executor enters a dedicated `TransitionDuringRoute` stage that issues the transition command once and waits for completion before resuming route following. This prevents the transition command from being re-issued on every control cycle.

### Branch-Off and Landing

The branch-off point is not an actual mission item — PX4 injects it as a virtual waypoint.
As soon as the route target becomes the selected branch-off index, the executor replaces the current target with the branch-off waypoint.
The vehicle branches off at the projected point on the segment, not after flying all the way to the real mission waypoint.

All final landings run through the same `handleLanding()` pipeline used by other mission-based RTL modes, preserving VTOL landing sequences, move-to-land waypoints, and precision landing settings.

::: warning
When falling back to the mission takeoff endpoint in reverse, PX4 lands at ground-level altitude (not the takeoff waypoint altitude).
:::

## Time Estimation

Route Safe Point Return publishes a remaining-flight-time estimate via `rtl_time_estimate` using the same wind-aware, vehicle-type-aware `RtlTimeEstimator` used by other RTL modes.

The estimator walks the remaining route items from the current position, summing horizontal distance (with wind correction) and vertical distance at the appropriate climb/descend rates. For VTOL vehicles, the time estimator correctly switches to MC descent rates before the final leg, ensuring the descent-time portion of the estimate reflects the actual multicopter descent profile.

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

## Developer Deep Dive

This section covers the internal architecture for developers working on the Route Safe Point Return code.

### Three-Role Architecture

Route Safe Point Return separates concerns into three roles:

| Role | Class | Responsibility |
| --- | --- | --- |
| **Orchestrator** | `RTL` (in `rtl.cpp`) | Owns the planner and executor instances, triggers planning on RTL entry or mission change, passes the plan to the executor, and selects the active RTL type. |
| **Brain** | `RtlRoutePlanner` (in `rtl_route_planner.h/cpp`) | Planning logic: projects the vehicle and safe points onto the mission route, scores candidates, and builds the `Plan` struct. Stateless between calls and fully testable via the `Provider` interface. |
| **Pilot** | `RtlMissionSafePointFollow` (in `rtl_mission_safe_point_follow.h/cpp`) | Executes the plan built by the Brain. Manages a stage-based state machine (Join → Transition → Follow → BranchOff → Land) and publishes setpoints to the flight controller. Inherits from `RtlBase → MissionBase → MissionBlock`. |

### Data Flow

```
RTL::setRtlTypeAndDestination()
  │
  ├─ RtlRoutePlanner::buildPlan(config)
  │    ├─ collectVehicleProjection()    → ProjectionContext
  │    ├─ selectSafePoint()             → Selection
  │    └─ choosePath()                  → Path + JoinContext
  │
  ├─ Plan {projection_context, selection, join_context}
  │
  └─ RtlMissionSafePointFollow::setRoutePlan(plan)
       └─ Executor stage machine drives setpoints
```

### State Machine

The executor's stage machine progresses through these states:

```
Idle → JoinRoute → TransitionAfterJoin → FollowRoute ⇄ TransitionDuringRoute → BranchOff → LandAtGoal
```

- `JoinRoute`: fly to the virtual branch-in waypoint.
- `TransitionAfterJoin`: VTOL back-transition if the first route segment expects MC flight.
- `FollowRoute`: walk mission items as geometry (skipping `DO_JUMP`), advancing via `advanceRouteTarget()`. When route traversal is exhausted (no more waypoints in either direction), the vehicle holds position if not already landed.
- `TransitionDuringRoute`: a VTOL transition was detected mid-route. The transition command is issued once, and the stage waits for completion before returning to `FollowRoute`. This prevents transition command spamming.
- `BranchOff`: replace the mission target with the virtual branch-off waypoint.
- `LandAtGoal`: hand off to `handleLanding()` for the final descent.

### Virtual Waypoints

The executor injects three types of virtual waypoints that exist only in RAM:

1. **Join waypoint**: placed at the vehicle's projection on the route. Altitude is interpolated from the segment endpoints.
2. **Branch-off waypoint**: placed at the safe-point's projection on the route. The vehicle flies here before leaving the route.
3. **Landing item**: a synthetic `NAV_CMD_LAND` or `NAV_CMD_VTOL_LAND` at the goal position.

These virtual items are published through `publishRouteItems()` or `publishLandingItems()` and are indistinguishable from real mission items to the flight controller.

### Activation Bypass

The executor bypasses `MissionBase::on_activation()` camera/gimbal state replay to prevent CPU lockup on high-index missions. Because the route follower treats the mission as geometry only, replaying every camera and gimbal command from item zero up to the current index is unnecessary and would block the control loop on large missions.

### Provider Interface

`RtlRoutePlanner` accesses mission data through the `Provider` interface, which abstracts the dataman storage:

- In production: `RtlRoutePlannerProvider` (in `rtl.cpp`) reads from the dataman cache.
- In tests: `VectorProvider` (in `test_RTL_helpers.h`) provides mission items from an in-memory vector.

This separation allows the planner to be fully unit-tested without SD card access or dataman dependencies.

### Shared Methods in MissionBase

The executor reuses several traversal methods from `MissionBase` to avoid code duplication:

- `loadMissionItemFromCache()`: load a single mission item by index.
- `findNextPositionIndexNoJump()` / `findPreviousPositionIndexNoJump()`: walk forward/backward skipping non-position and `DO_JUMP` items. `findPreviousPositionIndexNoJump()` returns `false` on dataman load failure instead of silently skipping, ensuring SD card read errors are surfaced to the caller.
- `findAttachedPositionIndex()`: find the nearest position item at or before a given index.
- `vtolTransitionActionForTarget()`: determine if a VTOL transition is needed for a given target index.

### Dataman Cache Architecture

PX4 missions can contain thousands of items, which heavily exceeds the RAM limits of typical flight controller microcontrollers. To solve this, PX4 stores missions on the SD Card or FRAM using the Dataman service.

Because reading from an SD card is slow and blocking, Navigator modes use `DatamanCache` as a RAM buffer. However, the Route Safe Point Return mode requires a different caching strategy than standard mission execution.

#### The Sequential Sliding Window (MissionBase)

Standard mission execution (Takeoff, Mission, direct RTL) only flies from waypoint A to waypoint B. It does not need the entire mission in RAM. `MissionBase` implements a sliding-window cache (typically 5 items). As the drone flies, it asynchronously fetches the next few waypoints. This keeps RAM usage low.

#### The Random-Access Route Cache (RTL Orchestrator)

Route Safe Point Return (`RTL_TYPE=6`) must perform complex geometric projections. To find the safest branch-off point, `RtlRoutePlanner` must rapidly scan every segment of the mission against every safe point. Doing this with a 5-item sliding window would trigger thousands of synchronous SD card reads, completely locking up the flight controller.

To solve this, the RTL orchestrator pre-loads the entire mission (up to `CONFIG_RTL_MISSION_CACHE_SIZE`, default 300) into a dedicated large RAM cache when the mode activates.

```
Mission on SD Card:  [ 0 ][ 1 ][ 2 ][ 3 ][ 4 ][ 5 ][ 6 ][ 7 ]...[ 299 ]
                       │    │    │    │    │    │    │    │        │
                       ▼    ▼    ▼    ▼    ▼    ▼    ▼    ▼        ▼
RAM Cache (300):     [ 0 ][ 1 ][ 2 ][ 3 ][ 4 ][ 5 ][ 6 ][ 7 ]...[ 299 ]
                       ▲                                           ▲
                       └─── entire mission loaded on activation ───┘
                            instant random access by any index
```

#### The Bridge (loadMissionItemFromCache Override)

`MissionBase` provides internal methods (`getNonJumpItem()`, `checkClimbRequired()`, `setMissionToClosestItem()`, etc.) that need to load mission items by index. All of these call the virtual method `loadMissionItemFromCache()` rather than accessing the cache directly.

This design allows each subclass to supply its own storage backend through a single override.

- **Standard Mission mode** calls the base `MissionBase::loadMissionItemFromCache()`, which safely uses the 5-item sliding window.
- **RtlMissionSafePointFollow** overrides `loadMissionItemFromCache()` to pull instantly from the RTL orchestrator's large pre-loaded cache.

This means the same traversal code in `MissionBase` works correctly for both modes without any code duplication or mode-specific branching.

## Related Topics

- [Return Mode (all types)](./return.md)
- [Safety Points (Rally Points)](../flying/plan_safety_points.md)
