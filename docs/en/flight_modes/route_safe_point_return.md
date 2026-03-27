# Route Safe Point Return

_Route Safe Point Return_ is a mission-aware [Return Mode](./return.md) that uses the uploaded mission geometry as the return corridor. PX4 projects the vehicle and every uploaded safe point onto the mission route, chooses the safe point with the lowest total return cost, follows the route in nominal or reverse direction, reaches the branch-off point defined as the projection of the selected safe point on the route and only then branches off horizontally from the flight plan. For VTOL vehicles flying in FW mode, if the selected safe point has valid `NAV_CMD_LOITER_TO_ALT` approach items, PX4 then flies the same wind-selected landing approach used by direct RTL before entering the final landing sequence. The wind ranking is computed from the selected land location to each approach loiter, not from home.

Set [RTL_TYPE=6](../advanced_config/parameter_reference.md#RTL_TYPE) to enable it.

This mode is intended for operations where the mission itself is the safest known path through terrain, obstacles, or airspace constraints. Unlike direct RTL variants, it does not assume that the safest way home is a straight line.

::: info
- If route planning succeeds but no [safety points (rally points)](../flying/plan_safety_points.md) are usable, PX4 falls back to the closer mission endpoint (landing or takeoff) while staying in the route-based return logic.
- During execution, route following skips `DO_JUMP` items as mission commands. During planning, jump segments can still be evaluated as route geometry. Mission smart rejoin preserves the repeat count of the selected loop edge, while `RTL_TYPE=6` clears that effective repeat count after projection and chooses the shorter continue-vs-rewind path through the loop exit.
- If the mission cannot be projected, the route cache is not ready, or the mission exceeds `CONFIG_RTL_MISSION_CACHE_SIZE`, PX4 falls back to the same direct RTL destination selection used by `RTL_TYPE=3`: home, the closest eligible safe point, or the mission landing point.
:::

## Setup and Configuration

Route Safe Point Return requires:

- A valid mission with at least two position items.
- [RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) set to `6`.

Tuning parameters:

| Parameter | Description |
| --- | --- |
| [MIS_MC_SEG_DIST](../advanced_config/parameter_reference.md#MIS_MC_SEG_DIST) | Extra cross-track search window for multicopter or VTOL-in-MC vehicle projection. |
| [MIS_FW_SEG_DIST](../advanced_config/parameter_reference.md#MIS_FW_SEG_DIST) | Extra cross-track search window for fixed-wing or VTOL-in-FW vehicle projection. |
| [MIS_ROUTE_JOIN](../advanced_config/parameter_reference.md#MIS_ROUTE_JOIN) | Shared mission smart-rejoin switch. `RTL_TYPE=6` always uses the route planner, but Mission mode uses the same planner for off-route mission resume only when this parameter is enabled. |
| [RTL_RP_SEG_DIST](../advanced_config/parameter_reference.md#RTL_RP_SEG_DIST) | Extra cross-track search window for safe-point projection. Increase if safe points are placed far from the mission route. |
| [RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN) | U-turn distance penalty for fixed-wing and VTOL-in-FW safe-point scoring. Penalizes paths that require reversing direction. Set to 0 to disable. |
| [RTL_APPR_FORCE](../advanced_config/parameter_reference.md#RTL_APPR_FORCE) | For VTOL in FW mode, only safe points with a valid VTOL approach are considered. |
| [RTL_PLD_MD](../advanced_config/parameter_reference.md#RTL_PLD_MD) | Precision landing mode used for the synthetic safe-point landing item and the reverse-takeoff landing fallback. |
| [NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD) | Sets the default shortcut radius for direct-to-safe-point. Join and branch-off acceptance use the Navigator dynamic acceptance radius, which starts from `NAV_ACC_RAD` and may be enlarged by the fixed-wing controller switch distance. |

::: tip
Larger search windows expose more candidate segments but also admit more distant branch-off options. Smaller windows keep the behavior closer to the nominal route.
:::

## How It Works

### Mode Entry and Selection

When `RTL_TYPE=6` is evaluated, PX4 performs these steps:

1. Project the vehicle onto the mission route.
2. Project all safe points onto the mission route.
3. Score the reachable safe-point projections by total return cost.
4. If route planning succeeds but no safe point is usable, fall back to the closer mission endpoint (takeoff or land).
5. If route planning itself cannot run, fall back to the same direct RTL destination selection used by `RTL_TYPE=3` (home, closest eligible safe point, or mission landing).
6. Build a route-join, route-follow, and branch-off plan from the selected result.

### Vehicle Projection

When the RTL mode is triggered, the vehicle might have deviated from the route e.g. with a GoTo command. Since the _Route Safe Point Return_ follows the mission path, the very first step is to rejoin the mission path at a "branch-in" point on the planned route.

The algorithm for selecting a branch-in point is executed in three phases:

**Phase 1 - Identifying valid candidates:**

The system first identifies up to three potential projection points on the flight route by calculating perpendicular lines from the vehicle's current position to all route segments. A projection point is only considered a valid candidate if the crosstrack distance from the vehicle to the segment does not exceed the crosstrack distance to the closest available segment plus an allowed margin. This margin is determined by the vehicle's current flight mode:
 - Multicopter: [MIS_MC_SEG_DIST](../advanced_config/parameter_reference.md#MIS_MC_SEG_DIST) by default, 30 m.
 - Fixed-wing: [MIS_FW_SEG_DIST](../advanced_config/parameter_reference.md#MIS_FW_SEG_DIST) by default, 150 m.

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

When the RTL mode is active, the vehicle follows the mission route until it reaches a branch-off point and flies straight to the safe point. The safe point selection is based on the total return cost from the vehicle projection to the safe-point destination. The first step is therefore to find candidate branch-off points on the planned route: the points where the vehicle could leave the flight plan and fly straight to the safe point.

The algorithm for selecting a safe point projection (potential branch-off) is executed in two main phases:

**Phase 1 - Identifying valid candidates:**

The process for identifying valid projection candidates is identical to Phase 1 of the vehicle projection algorithm, except the reference point is the safe point (not the vehicle): a projection point is only considered a valid candidate if the crosstrack distance from the safe point to the segment does not exceed the crosstrack distance to the closest available segment plus an allowed margin. This margin is determined by [RTL_RP_SEG_DIST](../advanced_config/parameter_reference.md#RTL_RP_SEG_DIST).

**Phase 2 - Selecting the best projection point:**

From the valid candidates, the system evaluates the travel path from each projection point to the safe point destination and selects the one with the lowest total path cost. The cost is calculated based on the following factors:
 - Along-Route Distance: The distance along the mission route from the vehicle projection to the safe-point projection (branch-off point). This is measured along the route geometry with straight lines between waypoints.
 - Branch-Off Leg: The straight-line distance from the safe-point projection to the safe point itself. This is the final off-route leg flown after the vehicle leaves the mission geometry.
 - U-turn Penalty: For Fixed-wing and VTOL-in-FW, a distance penalty ([RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN), default 4,000 m) is added to the cost if the path requires the vehicle to perform a U-turn. This prioritizes forward-flowing paths. Reduce the value for smaller airframes with tighter turn radii, or set to 0 to disable the penalty.

Safe points are filtered while the planner builds its safe-point batch:

- Valid safe points are read from the dataman store.
- Invalid coordinates, unsupported frames, or filtered safe points are skipped.
- For VTOL approach checks, a rally point owns the contiguous block of following `NAV_CMD_LOITER_TO_ALT` items up to the next rally point.
- For VTOL in FW mode with [RTL_APPR_FORCE](../advanced_config/parameter_reference.md#RTL_APPR_FORCE)=1, only safe points with a valid VTOL landing approach remain eligible.
- If a safe point is later selected for a VTOL-in-FW return, RTL reads the full approach block for that rally point and chooses the best approach using the same wind-based logic as direct RTL.
- Every remaining valid safe point gets up to three local-minimum route projections.

The route-based scorer evaluates a fixed-size batch of uploaded safe points; see [Safe-Point Evaluation Limit](#safe-point-evaluation-limit).

### Route-Skip Shortcuts

After the planner has already selected the best safe point by total route cost, PX4 applies two execution shortcuts to that selected goal:

- **Direct-to-safe-point**: if the vehicle is already within the direct acceptance radius of the selected safe point, RTL skips route join/follow and goes straight to the safe point.
- **Close-to-branch-leg**: if the vehicle is already within the route acceptance radius of the selected branch-off leg (the segment from the branch-off projection to the safe point), RTL also skips route join/follow and continues straight toward the selected goal.

Both shortcuts are evaluated only after safe-point selection has finished. They therefore never change which safe point wins the cost-based ranking.

If RTL is aborted and later re-triggered, PX4 computes a fresh type-6 plan again. If the newly selected safe point still yields one of the two shortcuts above, the vehicle skips route rejoin based on that new plan instead of reusing the old safe-point choice.

### Mission-Endpoint Fallback

If route planning succeeds but no safe point can be selected, Route Safe Point Return falls back to the closest between:

- The mission landing endpoint, flown in the nominal direction.
- The mission takeoff endpoint, flown in reverse.

If route planning itself cannot run, PX4 falls back to the same direct RTL destination selection used by `RTL_TYPE=3` instead of staying in the route-following executor.


## Execution Stages

The active executor runs through these stages:

1. **[Join route](#join-route)**: fly to a virtual branch-in waypoint at the [vehicle projection](#vehicle-projection) point.
2. **Post-join transition**: if the resumed mission segment requires a different VTOL state, apply the required front-transition or back-transition before following the route.
3. **[Follow route](#route-following)**: follow the mission path in nominal or reverse direction.
4. **Transition during route**: if the next route segment requires a different VTOL state, apply the transition and resume route following.
5. If a safe point is available:
   - **[Branch off](#branch-off-and-landing)**: replace the mission target with the virtual branch-off waypoint defined as the orthogonal projection of the safe point on the route.
   - **Navigate to the safe point**: once the branch-off is reached, navigate straight to the safe point or, for VTOL-in-FW with a valid approach, to the selected approach loiter.
6. **Approach at goal**: for VTOL-in-FW safe-point returns with a valid approach, fly the chosen `NAV_CMD_LOITER_TO_ALT` approach.
7. **Land at goal**: land at the safe point or the selected mission endpoint fallback.

### Join Route

The join point is a virtual `NAV_CMD_WAYPOINT` placed at the vehicle projection.

- If the target route segment requires a different VTOL state than the current vehicle state, the join context requests the required front-transition or back-transition after the join waypoint is reached.
- The branch-in planner only resumes the route on position mission items, so the post-join transition logic always aligns against a real waypoint target.
- For fixed-wing and VTOL-in-FW joins that stay in FW, PX4 doubles the join waypoint acceptance radius so the vehicle can curve onto the route instead of flying backward to hit the exact branch-in point.
- For post-join front-transitions, the requested yaw aligns with the active route target. If the vehicle is already within that waypoint's acceptance radius, PX4 instead aligns with the next position-bearing waypoint in the active traversal direction (which is the previous mission item when the route is being followed in reverse).
- If PX4 cannot derive a valid alignment target, it leaves yaw unset and performs the front-transition without an explicit heading command.
- If the selected goal is already within the acceptance radius of a landing endpoint, the join altitude requirement is skipped so landing can start immediately.
- If the join projection is already within acceptance radius of the branch-off projection, PX4 goes straight to landing instead of following a zero-length route segment.

### Route Following

During route following, PX4 treats the mission as geometry rather than as a full mission replay:

- Nominal direction walks forward through position items, skipping `DO_JUMP` entries instead of following them as control flow.
- Reverse direction walks backward through position items, also skipping `DO_JUMP` entries.
- Loiter items are converted to plain waypoints with `autocontinue = true` and zero hold time so the vehicle keeps moving.
- `NAV_CMD_DELAY` and other non-position mission commands are skipped during route traversal.
- Other non-position mission commands are skipped.
- When route traversal can no longer advance, the executor transitions to the already-selected goal and continues with the landing stage instead of completing RTL in loiter.

VTOL transition handling is preserved during route following: PX4 detects the expected VTOL state for each target segment using the same anchor rules as the planner. When a transition is needed, the executor enters a dedicated `TransitionDuringRoute` stage that issues the transition command once and waits for completion before resuming route following. This prevents the transition command from being re-issued on every control cycle.

### Branch-Off and Landing

The branch-off point is not an actual mission item — PX4 injects it as a virtual waypoint.
As soon as the route target becomes the selected branch-off index, the executor replaces the current target with the branch-off waypoint.
The vehicle branches off at the projected point on the segment, not after flying all the way to the real mission waypoint.

For VTOL vehicles flying in FW mode, if the selected safe point has valid approach items, RTL reads that rally point's `NAV_CMD_LOITER_TO_ALT` block, picks the best approach from wind, and injects the chosen loiter as an explicit pre-landing stage. This matches direct RTL behavior after the destination has already been chosen.

All final landings still run through the same `handleLanding()` pipeline used by other mission-based RTL modes, preserving VTOL landing sequences, move-to-land waypoints, and precision landing settings.

::: warning
When falling back to the mission takeoff endpoint in reverse, PX4 lands at ground-level altitude (not the takeoff waypoint altitude).
:::

## Time Estimation

Route Safe Point Return publishes a remaining-flight-time estimate via `rtl_time_estimate` using the same wind-aware, vehicle-type-aware `RtlTimeEstimator` used by other RTL modes.

The estimator walks the remaining route items from the current position, summing horizontal distance (with wind correction) and vertical distance at the appropriate climb/descend rates. For VTOL vehicles, the time estimator correctly switches to MC descent rates before the final leg, ensuring the descent-time portion of the estimate reflects the actual multicopter descent profile.

## Mission Size Limit

The route planner caches the entire mission in RAM for non-blocking access during flight. The maximum supported mission size is defined by the board-level Kconfig option `CONFIG_RTL_MISSION_CACHE_SIZE` (default: 300 items). Each cached item uses approximately 76 bytes of heap.

**Missions within the cache limit** are fully cached on upload. The planner evaluates every segment and optimal safe-point selection is guaranteed.

**Missions exceeding the cache limit** cannot use Route Safe Point Return. PX4 logs a warning and automatically falls back to the same direct RTL destination selection used by `RTL_TYPE=3`.

To increase the limit for a specific board, set the following in the board's `.px4board` file:

```
CONFIG_RTL_MISSION_CACHE_SIZE=500
```

::: tip
For most real-world operations, 300 waypoints is sufficient. If your mission requires more waypoints, either increase `CONFIG_RTL_MISSION_CACHE_SIZE` for your board or consider splitting the mission into shorter segments.
:::

## Safe-Point Evaluation Limit

The route-based scorer used by `RTL_TYPE=6` evaluates at most 64 eligible rally points per planning cycle. This limit is independent of the mission cache size and comes from the fixed-size safe-point batch inside `MissionRoutePlanner`. Additional uploaded rally points remain stored in dataman, but they are not considered by the type-6 route scorer. `NAV_CMD_LOITER_TO_ALT` approach items do not consume those 64 rally-point slots because the planner keeps them attached to the preceding rally point and filters rally-point eligibility before filling the batch. Direct safe-point RTL modes still use their own direct-distance selection path, but they use the same rally-to-approach association rule.

## Current Limitations

- Missions exceeding `CONFIG_RTL_MISSION_CACHE_SIZE` items (default 300) are not supported; PX4 falls back to direct-path RTL.
- VTOL safe-point approaches are associated purely by upload order: a rally point owns the following `NAV_CMD_LOITER_TO_ALT` items up to the next rally point, and only the first 8 approaches in that block are retained.
- Geofence-aware pruning for vehicle and safe-point projections is not yet implemented.
- No dedicated reverse-turn execution module: U-turns are penalized in path scoring but not executed as a specific maneuver.

## Developer Deep Dive

This section covers the internal architecture for developers working on the Route Safe Point Return code.

### Architecture Overview

`RTL_TYPE=6` is split across planner, cache, outer RTL orchestration, inner execution, and shared mission helpers. The main files are:

| File | Role in the type-6 pipeline |
| --- | --- |
| `rtl.cpp` / `rtl.h` | Outer RTL orchestrator. Owns only the continuity hints needed by the next planning pass, builds planner config, applies fallback policy, chooses the goal landing approach, and hands the fresh type-6 plan to the nested executor. |
| `rtl_base.h` | Shared base interface for mission-backed RTL executors, including the route-safe-point handoff contract. |
| `rtl_mission_safe_point_follow.cpp` / `rtl_mission_safe_point_follow.h` | RTL_TYPE 6 executor. Runs the route-follow / branch-off / approach / land stages and reuses `MissionBase` for join-route helper work items. |
| `mission_route_planner.cpp` / `mission_route_planner.h` | Geometry and scoring engine. Projects the vehicle and safe points, evaluates route paths, and builds `Plan`, `JoinPlan`, and `Selection`. |
| `mission_route_cache.cpp` / `mission_route_cache.h` | Shared provider/cache for full mission geometry, mission-land item, and safe points. |
| `mission_base.cpp` / `mission_base.h` | Shared execution helpers: join-route pipeline, traversal helpers, VTOL transition logic, and loop-anchor tracking. |
| `mission.cpp` / `mission.h` | Mission-mode caller that reuses the same planner and join-route path for smart mission resume. |
| `mission_block.cpp` / `mission_block.h` | Low-level mission-item utilities used by MissionBase and the RTL executors. |
| `navigator.h` / `navigator_main.cpp` | Own the shared `MissionRouteCache` instance and wire the planner/caches into Navigator runtime. |
| `rtl_params.c` / `mission_params.c` | Parameters that tune Route Safe Point Return and Mission smart rejoin. |
| `msg/RtlStatus.msg` | Status exported to the rest of the system. |
| `src/modules/navigator/test/*` | Unit and integration coverage for planner, executor, MissionBase helpers, and Navigator-level orchestration. |

```text
Navigator
├─ MissionRouteCache
├─ Mission
│  └─ MissionBase
│     └─ MissionBlock
└─ RTL
   ├─ MissionRoutePlanner(provider = MissionRouteCache)
   ├─ continuity hints: route direction / last loop segment
   └─ RtlMissionSafePointFollow
      ├─ active Plan / stage machine
      └─ RtlBase
         └─ MissionBase
            └─ MissionBlock
```

### Three Roles

Route Safe Point Return separates concerns into three roles:

| Role | Class | Responsibility |
| --- | --- | --- |
| **Orchestrator** | `RTL` (in `rtl.cpp`) | Owns the planner and executor instances, triggers planning on RTL entry or mission change, preserves only the continuity hints needed for the next planning pass, passes the fresh route-safe-point plan to the executor, selects the active RTL type, and chooses the final VTOL landing approach once a safe point has been selected. |
| **Brain** | `MissionRoutePlanner` (in `mission_route_planner.h/cpp`) | Planning logic: projects the vehicle and safe points onto the mission route, scores candidates, and builds the `Plan` struct. |
| **Pilot** | `RtlMissionSafePointFollow` (in `rtl_mission_safe_point_follow.h/cpp`) | Executes the plan built by the Brain. Manages the route-follow / branch-off / goal-approach / landing stages and reuses `MissionBase` for join-route work items. Inherits from `RtlBase → MissionBase → MissionBlock`. |

### Runtime Data Flow

```
RTL::setRtlTypeAndDestination()
  │
  ├─ MissionRouteCache::update(mission)
  │
  ├─ buildRouteSafePointConfig()
  │    └─ continuity hints:
  │         route direction / last loop segment
  │
  ├─ MissionRoutePlanner::planRouteToGoal(config)
  │    ├─ collectVehicleProjection()    → ProjectionContext
  │    ├─ selectBestGoal()              → Selection
  │    └─ fill geometric context        → JoinContext
  │
  ├─ RouteSafePointConfig {
  │      plan,
  │      goal_land_approach,
  │      rtl_alt
  │  }
  │
  └─ RtlMissionSafePointFollow::configureRouteSafePoint(...)
       ├─ MissionBase join-route work items
       └─ Type-6 executor stage machine publishes setpoints
```

### State Machine

Join-route handling is now shared in `MissionBase`, while the RTL executor keeps only the route-specific stages.

```
MissionBase work items: Default → JoinRoute → TransitionAfterJoin → Default
Executor stages:       Idle → FollowRoute ⇄ TransitionDuringRoute → BranchOff → ApproachAtGoal? → LandAtGoal
```

- `JoinRoute`: a shared `MissionBase` work item that flies the virtual branch-in waypoint.
- `TransitionAfterJoin`: a shared `MissionBase` work item that performs the required VTOL front-transition or back-transition after the join when needed.
- `FollowRoute`: walk mission items as geometry (skipping `DO_JUMP`), advancing via `advanceRouteTarget()`. If traversal can no longer advance, the executor goes straight to the already-selected landing goal instead of terminating RTL in loiter.
- `TransitionDuringRoute`: a VTOL transition was detected mid-route. The transition command is issued once, and the stage waits for completion before returning to `FollowRoute`. This prevents transition command spamming.
- `BranchOff`: replace the mission target with the virtual branch-off waypoint.
- `ApproachAtGoal`: if the selected safe point has a valid VTOL approach and the vehicle is currently flying in FW mode, inject the chosen `NAV_CMD_LOITER_TO_ALT` approach before landing.
- `LandAtGoal`: hand off to `handleLanding()` for the final descent.

### Virtual Waypoints

The executor injects three types of virtual waypoints that exist only in RAM:

1. **Join waypoint**: placed at the vehicle's projection on the route. Altitude is interpolated from the segment endpoints.
2. **Branch-off waypoint**: placed at the safe-point's projection on the route. The vehicle flies here before leaving the route.
3. **Landing item**: a synthetic `NAV_CMD_LAND` or `NAV_CMD_VTOL_LAND` at the goal position.

These virtual items are published through `publishRouteItems()` or `publishLandingItems()` and are indistinguishable from real mission items to the flight controller.

### Activation Ordering

Mission mode and `RtlMissionSafePointFollow` now determine the final target index and arm any join-route work item **before** calling `MissionBase::on_activation()`. This matters for two reasons:

- `MissionBase` caches camera, gimbal, trigger, and speed history up to `current_seq - 1`, so the target index must already point at the segment end that will be resumed.
- `MissionBase` publishes the first active setpoint on activation, so the join route must already be armed to avoid a double publication of first the nominal waypoint and then the virtual branch-in waypoint.

The result is a single clean publication on activation and correct replay of mission history when resuming after a route rejoin.

### Shared Join-Route Execution Flow

Mission smart rejoin and `RTL_TYPE=6` share the same join-route execution path in `MissionBase`. The caller decides the route target and join geometry, then `MissionBase` runs the temporary work items that get the vehicle back onto the route before normal mission or RTL execution resumes.

The flow is:

1. Mission mode (`trySetRouteJoinOnActivation()`) or RTL (`RtlMissionSafePointFollow::on_activation()`) computes the route target and calls `setupJoinRoute()` before `MissionBase::on_activation()`.
   - The planner contributes only the geometric join state: projection point plus route direction.
   - `MissionBase::setupJoinRoute()` then applies the execution-side corrections: the required VTOL transition and any skip-altitude override.
2. `MissionBase::update_mission()` runs during activation. It normally clears transient `_work_item_type` state for a newly accepted mission, but it preserves `WORK_ITEM_TYPE_JOIN_ROUTE` and `WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN` so the join pipeline is not dropped before first publication.
3. In the active loop, both Mission mode and RTL call `handleJoinRouteWorkItems()` from their `setActiveMissionItems()` implementation:
   - `WORK_ITEM_TYPE_JOIN_ROUTE` publishes the virtual branch-in waypoint until the cached helper-item reached flags show that the join is complete.
   - `WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN` publishes the required VTOL front-transition or back-transition if it is still needed, otherwise it clears the join state immediately.
4. When one of those helper items is reached or completed, `advance_mission()` does not advance the real mission sequence because `_work_item_type != DEFAULT`. The next `set_mission_items()` pass then lets `handleJoinRouteWorkItems()` advance the temporary join pipeline during setpoint generation, matching `handleLanding()`:
   - After `JOIN_ROUTE`, it either promotes the pipeline to `TRANSITION_AFTER_JOIN` or clears the join state when the vehicle is already in the correct VTOL mode.
   - After `TRANSITION_AFTER_JOIN`, it clears the temporary join state and returns to normal mission or RTL progression.
5. `shouldReportMissionItemReached()` suppresses `seq_reached` updates for `JOIN_ROUTE` and `TRANSITION_AFTER_JOIN`, because those helper items are synthetic and do not mean the uploaded mission item at `current_seq` was reached.
6. If the uploaded mission items are replaced from outside, `onMissionUpdate()` clears the join pipeline because the cached join context may no longer match the mission that will be flown.

### Shared Smart Rejoin Path Selection

Mission smart rejoin now uses the planner's dedicated `planMissionResumeJoin()` entry point, which shares the same vehicle-projection, path, and geometric join-context logic as `RTL_TYPE=6` while still preserving the active mission loop count:

- The resumed target index comes from the shortest valid nominal path to the mission landing goal instead of hard-coding the projected segment end.
- If the projection lies on a `DO_JUMP` loop segment with repeats remaining, the resumed path continues to the loop segment end.
- If the selected projection lies on an exhausted loop edge, the planner compares continuing versus rewinding through the loop exit and chooses the shorter path.
- The shared join-route work item finalizes the execution corrections when it is armed: it applies the required front-transition or back-transition and, when appropriate, skips the join altitude to avoid an unnecessary climb near landing or other caller-provided special cases.

### Loop Geometry and Cached Anchors

`MissionRoutePlanner::Segment` can describe either a nominal forward route segment or an active synthetic loop edge created by a `DO_JUMP`.

- For a nominal segment, `start.idx < end.idx`.
- For an active `DO_JUMP` loop segment, `start` is the position item attached to the jump command and `end` is the first position item reached at `do_jump_mission_index`, so the segment can run backward in mission-index order.
- `segment.is_loop` marks that synthetic jump edge.
- `segment.is_loop` only means "this projection candidate came from a synthetic jump edge"; it does not imply that Mission mode will necessarily replay loop control flow from that point onward.

The cached loop state is intentionally temporary. It means "the active loop edge the vehicle was most recently committed to", not "the last `DO_JUMP` seen anywhere in the mission".

Example mission:

`[WP0, WP1, WP2, DO_JUMP->0 repeat=3 current=1, WP3]`

- If `current_seq == 2`, the next nominal forward advance will execute the `DO_JUMP`, so the cached active loop edge is `[2 -> 0]` with `loops_remaining = 2`.
- If `current_seq == 1`, the scan stops at `WP2` and caches nothing, because the upcoming nominal advance ends at `WP2` before the later `DO_JUMP` is reached.

`MissionBase::updateLastFlownLoopSegmentForNominalAdvance()` intentionally scans only from `current_seq + 1` until the first position item:

- Non-position items in that window still belong to the same nominal advance and may redirect control flow.
- Reaching the next position item ends that advance, so later `DO_JUMP` items belong to future legs and must not bias the current projection/rejoin decision.

The same concept exists at different lifetimes:

- `Mission` stores `_last_flown_loop_segment` between Mission activations so smart rejoin can stay anchored on the active loop.
- `RtlMissionSafePointFollow` stores its active `_last_flown_loop_segment` while the executor is running.
- `RTL` stores `_last_route_safe_point_loop_segment` across nested executor recreation and feeds it back into the next planning pass.

The loop-repeat count belongs to the selected projected loop segment itself. That matters for missions with multiple `DO_JUMP` items: the active loop must keep its own repeat count even if a later unrelated loop also exists in the same mission.

After projection, the two mode families diverge:

- Mission smart rejoin honors `mission_loops_remaining`. If repeats are still pending, the resumed path must continue to the loop end.
- `RTL_TYPE=6` clears the effective remaining-loop count after projection. The loop edge is still useful as geometry, but return execution must not replay mission loop control flow.

### Type-6 State Ownership

The type-6 state split is intentionally small and explicit:

- `MissionRoutePlanner` remains stateless. Any continuity hint it needs must be passed in through `MissionRoutePlanner::Config`.
- `RTL` keeps only the continuity hints that the next planning pass may need: `_route_safe_point_direction_reversed` and `_last_route_safe_point_loop_segment`.
- `RtlMissionSafePointFollow` owns the active `Plan`, the executor stage machine, and its runtime copy of the loop anchor while the nested executor is alive.

The stored route direction is needed because `mission_index` alone is ambiguous at shared waypoints. When the current target lies on a waypoint between two segments, the same mission index can mean "arriving from the nominal side" or "arriving from the reverse side". Preserving the last chosen direction lets the next planning pass prioritize the correct adjacent segment instead of flipping sides at that waypoint.

The handoff is now:

1. `RTL` builds `MissionRoutePlanner::Config` from the current vehicle state plus the two continuity hints it preserves between passes.
2. `MissionRoutePlanner` returns a fresh `Plan`.
3. `RTL` copies the selected route direction and projected loop segment out of that plan so the next planning pass can stay continuous.
4. `RtlMissionSafePointFollow` receives the fresh plan through `configureRouteSafePoint()` and drives execution purely from the plan plus `_stage`.

There is no longer any outer cached type-6 plan or persisted "straight-to-goal" latch. If RTL is re-entered, PX4 replans and then applies the route-skip shortcut policy to the newly selected safe point only.

### Provider Interface

`MissionRoutePlanner` accesses mission data through the `Provider` interface, which abstracts the dataman storage:

- In production: `MissionRouteCache` implements the provider directly and serves mission geometry, safe points, and the mission-land item from its dataman-backed caches.
- In tests: `VectorProvider` (in `test_RTL_helpers.h`) provides mission items from an in-memory vector.

This separation allows the planner to be fully unit-tested without SD card access or dataman dependencies.

### Shared Methods in MissionBase

The executor reuses several traversal methods from `MissionBase` to avoid code duplication:

- `loadMissionItemFromCache()`: load a single mission item by index.
- `findNextPositionIndexNoJump()` / `findPreviousPositionIndexNoJump()`: walk forward/backward skipping non-position and `DO_JUMP` items. `findPreviousPositionIndexNoJump()` returns `false` on dataman load failure instead of silently skipping, ensuring SD card read errors are surfaced to the caller.
- `findAttachedPositionIndex()`: find the nearest position item at or before a given index.
- `vtolTransitionActionForTarget()`: determine if a VTOL transition is needed for a given target index.
- `setupJoinRoute()` / `handleJoinRouteWorkItems()`: arm and execute the shared branch-in pipeline used by both Mission smart rejoin and `RTL_TYPE=6`.
- `computeFrontTransitionAlignmentYaw()`: provide the route-aligned yaw used for front transitions after join and during route following.
- `updateLastFlownLoopSegmentForNominalAdvance()`: cache the exact active `DO_JUMP` edge that the next nominal forward advance would traverse.

### Dataman Cache Architecture

PX4 missions can contain thousands of items, which heavily exceeds the RAM limits of typical flight controller microcontrollers. To solve this, PX4 stores missions on the SD Card or FRAM using the Dataman service.

Because reading from an SD card is slow and blocking, Navigator modes use `DatamanCache` as a RAM buffer. However, the Route Safe Point Return mode requires a different caching strategy than standard mission execution.

#### The Sequential Sliding Window (MissionBase)

Standard mission execution (Takeoff, Mission, direct RTL) only flies from waypoint A to waypoint B. It does not need the entire mission in RAM. `MissionBase` implements a sliding-window cache (typically 5 items). As the drone flies, it asynchronously fetches the next few waypoints. This keeps RAM usage low.

#### The Random-Access Route Cache (`MissionRouteCache`)

Route Safe Point Return (`RTL_TYPE=6`) must perform complex geometric projections. To find the safest branch-off point, `MissionRoutePlanner` must rapidly scan every segment of the mission against every safe point. Mission resume rejoin uses the same geometry scan when it needs to branch back into the route after a GoTo, a manual reposition, or an RTL type 6 branch-off. Doing this with a 5-item sliding window would trigger thousands of synchronous SD card reads, completely locking up the flight controller.

To solve this, Navigator maintains a shared `MissionRouteCache` in RAM. It owns three caches and the asynchronous safe-point dataman state machine:

```
Mission / RTL
   │
   ├─ uses MissionRoutePlanner (geometry scan)
   │
   └─ uses MissionRouteCache
        ├─ full mission route cache      [0 ... CONFIG_RTL_MISSION_CACHE_SIZE-1]
        ├─ safe-point cache              [all rally / safe points]
        ├─ mission-land item cache       [current mission land item]
        └─ safe-point stats reader       [DM_KEY_SAFE_POINTS_STATE async state machine]
```

The full-mission cache is shared by Mission resume and RTL type 6. The safe-point and mission-land caches are also shared, which means RTL no longer needs its own parallel provider wrapper or duplicated dataman state machine.

#### The Bridge (loadMissionItemFromCache Override)

`MissionBase` provides internal methods (`getNonJumpItem()`, `checkClimbRequired()`, `setMissionToClosestItem()`, etc.) that need to load mission items by index. All of these call the virtual method `loadMissionItemFromCache()` rather than accessing the cache directly.

This design allows each subclass to supply its own storage backend through a single override.

- **Standard Mission mode** still uses the base `MissionBase::loadMissionItemFromCache()` and its 5-item sliding window for normal mission execution. When Mission mode is activated while the vehicle is off-route, it uses the shared full-mission route cache plus `MissionRoutePlanner::collectVehicleProjection()` to build a temporary branch-in waypoint before resuming the real mission item.
- **RtlMissionSafePointFollow** overrides `loadMissionItemFromCache()` to pull instantly from the same shared full-mission route cache, while safe-point and mission-land access also come from `MissionRouteCache`.

This means the same traversal code in `MissionBase` works correctly for both modes without any code duplication or mode-specific branching.

### Unit Test Coverage

The feature has automated coverage:

- `test_RTL_projection.cpp`: vehicle projection, segment ownership, stored loop-anchor preference, corner handling, and invalid inputs.
- `test_RTL_safe_point.cpp`: safe-point filtering/scoring, direct-to-safe-point shortcut, VTOL approach policy, loop-aware safe-point selection, branch-off geometry, and batch-scan behavior.
- `test_RTL_planner_integration.cpp`: full `planRouteToGoal()` / `planMissionResumeJoin()` behavior, including loop continuation vs rewind, endpoint fallback, and regression coverage for missions with multiple `DO_JUMP` loops.
- `test_RTL_mission_safe_point_follow.cpp`: lightweight executor stage-machine transitions.
- `test_mission_base_vtol.cpp`: shared MissionBase helpers such as join-route work items, VTOL transition decisions, and loop-anchor caching before nominal advance.
- `test_navigator_route_rejoin_and_rtl.cpp`: Navigator-level integration with real Mission / RTL classes and dataman-backed mission and safe-point state.

That test split mirrors the architecture split: geometry is tested close to `MissionRoutePlanner`, execution helpers are tested close to `MissionBase` and `RtlMissionSafePointFollow`, and end-to-end orchestration is covered at Navigator level.

## Related Topics

- [Return Mode (all types)](./return.md)
- [Safety Points (Rally Points)](../flying/plan_safety_points.md)
