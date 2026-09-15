# Mission Route Planning

Mission route planning helps the vehicle rejoin an uploaded mission or use that route to return to a landing destination.
It provides:

- [Smart mission rejoin](#smart-mission-rejoin): Rejoin the route after leaving it, for example after manual repositioning or a GoTo.
- [Route-following Return](#route-following-return): Follow the mission route towards a [safe point](../flying/plan_safety_points.md) or mission endpoint, then land or hold as configured.

Enable smart rejoin with [MIS_ROUTE_JOIN](../advanced_config/parameter_reference.md#MIS_ROUTE_JOIN) (off by default).
Select route-following Return with [RTL_TYPE=7](../flight_modes/return.md#rtl_type_7).
`RTL_TYPE=6` provides a separate battery-aware home/rally return mode.

::: warning
The planner requires a complete mission from the [mission route cache](../advanced/mission_route_cache.md).
[Direct fallback](#route-following-fallback) can leave the mission corridor and select a different destination using `RTL_TYPE=3` rules.
Review the fallback conditions below before relying on the mission route as a return corridor.
:::

::: info
Route-following Return supports fixed-wing, multicopter and VTOL vehicles.
VTOL vehicles change flight mode to match the selected route legs, including when following the mission in reverse.
After a fixed-wing system failure, Route-following Return keeps the selected route and continues in multicopter mode, skipping front transitions while the failure remains active.
:::

::: warning
No geofence or obstacle check is applied to a planned join, route, or branch-off leg.
:::

## How It Works

Both features choose where to join the route using the vehicle's current position and mission progress.
Mission rejoin continues the mission; route-following Return chooses a destination and may fly the route forwards or backwards to reach it.

When the vehicle is already near the selected destination or branch-off leg, [route-skip shortcuts](#route-skip-shortcuts) can avoid an unnecessary trip back to the route.
If route-following Return cannot use the route, [direct fallback](#route-following-fallback) may change the destination and fly outside the mission corridor.

### Smart Mission Rejoin {#smart-mission-rejoin}

When Mission mode is activated in flight with smart rejoin enabled, the vehicle:

1. Selects a branch-in point and altitude on the route using [Vehicle Projection](#vehicle-projection).
2. Flies to that point.
3. Continues towards the mission end in the mission direction, accounting for any active loop as described below.

If the branch-in lands on an active [`DO_JUMP` loop segment](#vehicle-projection), the loop's repeat count is preserved:

- while repeats remain the resumed path continues to the jump target so the loop is still flown
- once repeats are exhausted, the planner picks whichever loop exit gives the shorter **total** path on to the mission end: continuing forward to the jump target, or rewinding back to the waypoint before the jump command (each including any fixed-wing U-turn penalty). The comparison is over the full path, so if the mission end lies near the loop start it may rewind most of the loop rather than finish it.

The projection search margin is set by [MIS_MC_SEG_DIST](../advanced_config/parameter_reference.md#MIS_MC_SEG_DIST) (multicopter) and [MIS_FW_SEG_DIST](../advanced_config/parameter_reference.md#MIS_FW_SEG_DIST) (fixed-wing).
If a join cannot be planned, Mission mode resumes without the smart join.
This can happen while the mission is still loading into the [route cache](../advanced/mission_route_cache.md).
After pausing a camera-trigger survey, smart route rejoin can select a branch-in point that conflicts with the return to the previous survey waypoint.
Use `MIS_ROUTE_JOIN=0` for missions that rely on camera-trigger survey resume.
Selecting a mission item explicitly cancels a pending smart join.

When a VTOL rejoin requires a front transition, the vehicle follows this sequence:

1. Fly to the branch-in waypoint.
2. Wait for any active back transition to finish, retaining the branch-in position target.
3. Hold at the branch-in position and align the heading towards the next mission waypoint.
4. Start the front transition once position and heading alignment are complete, then resume the mission after reaching fixed-wing mode.

Back-transition completion starts the alignment step even if the vehicle has drifted outside the branch-in acceptance radius.
The vehicle must still reach the alignment position before starting the front transition.
These temporary steps do not mark the next uploaded mission waypoint as reached.

### Route-Following Return {#route-following-return}

With `RTL_TYPE=7`, Return uses the mission route as a return corridor:

1. Select a branch-in point using [Vehicle Projection](#vehicle-projection).
2. Choose a safe point and branch-off point using [Safe-Point Scoring](#safe-point-scoring), or a mission takeoff/landing endpoint if no safe point is usable.
3. Join and follow the route in the selected direction, then fly to the destination.

[Route-skip shortcuts](#route-skip-shortcuts) may bypass the join and route when the vehicle is already near the destination or branch-off leg.

Route-following Return does not yet support `MAV_CMD_DO_RETURN_PATH_START` to designate a return-path segment.
The planner considers the mission route without restricting its join and return path to the segment between this marker and `MAV_CMD_DO_LAND_START`.

Here an active [`DO_JUMP` loop segment](#vehicle-projection) is used as return geometry only: the vehicle does not repeat the loop (unlike [Smart Mission Rejoin](#smart-mission-rejoin)). The planner then picks whichever loop exit gives the shorter **total** return path to the goal: continuing forward to the jump target, or rewinding back to the waypoint before the jump command (each including any fixed-wing U-turn penalty). The comparison is over the full path, so if the goal lies near the loop start the planner may rewind most of the loop instead of finishing it.

Route following skips waypoint hold times and timed or unlimited loiter holds, but preserves `LOITER_TO_ALT`.
The vehicle approaches these loiters at its current altitude, then changes altitude in the loiter before continuing.

::: details How VTOL Transitions Work Along the Route

VTOL transition commands belong to the preceding position waypoint in the uploaded mission.
For example, `A → front transition → B → back transition → C` is flown in reverse as `C → B` in multicopter mode, then a front transition at `B`, followed by `B → A` in fixed-wing mode.
The vehicle flies each leg in the mode used in the forward mission, even when travelling in reverse.
Before a front transition, it waits for any back transition to finish, holds its position and aligns towards the next route target.
It advances the route after the transition completes.
An ongoing front transition may finish when route-following Return is activated; direct fallback cancels it.

PX4 determines each leg's mode from the VTOL mode recorded when it first receives the mission and any preceding transition commands.
This starting mode is not saved across reboots; a transition in progress is recorded as multicopter mode.
:::

At a rally point or mission takeoff endpoint, the vehicle first approaches at the altitude held when leaving the route.
It then follows the destination arrival policy configured by [RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT), [RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY), and [RTL_LOITER_RAD](../advanced_config/parameter_reference.md#RTL_LOITER_RAD):

- A negative landing delay holds indefinitely above the destination.
- A positive landing delay waits at the descent altitude for the configured time before landing.
- With zero landing delay, a multicopter lands after horizontal arrival; a fixed-wing vehicle first descends in the destination loiter.

The descent altitude is relative to the destination and is capped at the arrival altitude, so arrival does not introduce another return-altitude climb.
Landings at rally points and takeoff endpoints use [RTL_PLD_MD](../advanced_config/parameter_reference.md#RTL_PLD_MD) for precision landing.
An uploaded mission landing command retains its own landing and precision-landing settings; the destination descent and delay described above do not override it.
A VTOL arriving in fixed-wing mode can use the selected rally point's landing approach, then back-transitions and lands through the existing VTOL landing sequence.
If arrival starts during a front transition, the vehicle still back-transitions before landing.
An approach is retained even when Return starts in multicopter mode, but is only flown if the vehicle reaches the destination phase in fixed-wing mode.

Before Return is activated, its time estimate is refreshed every two seconds using a branch-in recomputed from the current mission index and vehicle position.
During Return, the estimate follows the remaining route and arrival stages, counting sequential loiter altitude changes and multicopter landing descent separately from horizontal approach.
With a negative landing delay, it estimates time to the indefinite hold.
For VTOL, the estimate uses the flight mode of each route leg and multicopter mode for the final vertical descent.
It does not model transition duration or turning dynamics.

#### Direct Fallback and Route Deviations {#route-following-fallback}

Direct fallback reselects the destination using [RTL_TYPE=3](../flight_modes/return.md#rtl_type_3): home, an eligible rally point or a mission landing pattern.
The destination can differ from the SRP goal, and the vehicle can fly outside the mission corridor to reach it.
The direct-RTL climb and landing approach still apply; selecting a mission landing pattern means flying directly to that pattern before following it.

Fallback occurs when:

- Route planning is compiled out (`CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE=0`).
- The current mission has no valid feasibility result, including while validation is pending or the result belongs to a previous mission, home position or geofence.
- The complete mission or rally-point cache is unavailable, still loading/reloading or does not match the current source. This includes missions exceeding the configured full-cache capacity.
- The planner cannot produce a valid join and return path: for example, required position/index/parameter inputs are invalid, required mission data cannot be read, or no usable rally point or mission endpoint can be reached through the route.
- The result cannot be accepted because its source changed during planning, its plan is invalid, or the selected rally-point index exceeds the supported range.
- There is insufficient memory to start route following.
- Vehicle status reports a type other than multicopter or fixed-wing. VTOL hover, forward flight and transitions are supported.

During route following, changes to the mission or rally points trigger replanning.
If that replan cannot be accepted, the same direct fallback applies.
Once the destination arrival or landing sequence has begun, changes to the mission or rally points do not replace it.

Pending cache or mission-validation inputs are retried on the two-second check once they are ready and the mission is valid, provided landing has not started.
Other planning or allocation failures do not automatically retry on that timer.

If no rally point is usable, the planner first tries mission takeoff/landing endpoints.
A fixed-wing system failure keeps a VTOL on the selected route in multicopter mode, skipping front transitions.
An incomplete time estimate alone does not trigger fallback.

Even with a valid plan, [route-skip shortcuts](#route-skip-shortcuts) can bypass the join and route when the vehicle is already near the selected goal or branch-off leg.
Those shortcuts retain the planner's selected goal; they are separate from direct fallback.

## Point Projection

This is the shared projection step used by both the [Vehicle Projection](#vehicle-projection) and the [Safe-Point Scoring](#safe-point-scoring) below.
The projected point is the vehicle in one case and a safe point (rally point) in the other.

The planner draws a perpendicular projection from the point onto every route segment and keeps up to three candidates per point.
A projection is a valid candidate only if its crosstrack distance is within a search margin of the closest candidate's crosstrack distance.
Separate parameters control the search margins for the vehicle and rally points; see [Parameters and Build Configuration](#parameters-and-build-configuration).
When more than three projections fall within the margin, only the three with the smallest crosstrack distance are kept and the rest are dropped.

The mission route planner supports [`DO_JUMP`](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP) mission loop commands. The active jump segment is the segment running from the waypoint before the jump command to the first position waypoint at the jump target.
A point projects onto a loop segment using the same crosstrack and margin rule as any other segment. How that candidate is then used differs for the vehicle branch-in ([Vehicle Projection](#vehicle-projection)) and for safe-point branch-offs ([Safe-Point Scoring](#safe-point-scoring)).

::: details Click here for more detail on how projections land on segments and corners

A projection is normally perpendicular to the point on a segment, but it is only kept if it is a _local minimum_ of the distance to the route.
This rule decides what happens at a waypoint, where two segments meet:

- An interior projection, one that lands part-way along a segment, is always kept.
- A projection that lands exactly on a waypoint (a corner) is kept only when the previous segment and the current segment both project onto that same shared corner.
  This happens when two consecutive segments form a V-shape and the point is closest to the apex:

```text
 A \      / C
    \    /          P projects onto corner B from both segments.
     \  /           Both projections land on B, so the shared
    B \/            corner is kept as a single candidate.

       ^
       P
```

This is why some projections in the images below land on a waypoint instead of perpendicular to a segment.
:::

The example below uses a large margin.
The mission route is drawn in white and the rally-point (`R`) projections in green.
Every rally point has three projections because the margin is large.

![Point projection with a large search margin](../../assets/mission_route_planner/mission_route_planner_large_search_margin.png)

Here is the same route with a smaller margin.
Some projections are dropped because their crosstrack distance is greater than the closest projection's crosstrack distance plus the margin.

![Point projection with a small search margin](../../assets/mission_route_planner/mission_route_planner_small_search_margin.png)

With a margin of zero, each rally point keeps only its single closest projection.

<a id="stacked-waypoints"></a>

::: details Click here to see how stacked waypoints (same latitude and longitude) are handled

Two consecutive waypoints that share latitude and longitude form a vertical segment with zero horizontal length, for example a climb waypoint directly above `TAKEOFF` or a descent waypoint directly above `LAND`.
Every point projects onto such a segment at its single horizontal location, so the segment always looks like a corner hit from both sides.
Accepting that hit at the route ends would make the endpoint a candidate for every rally point, wherever the rally point is.
With a wide enough search margin the candidate survives the cross-track window, and because the path cost adds along-route and off-route distance, a diagonal shortcut from the endpoint can score lower than following the route:

```text
Mission order: A, B, B', LAND (B, B' and LAND share latitude/longitude)

Top view:
A ----------- P ----------- B/B'/LAND    V
              |
              R

Side view at the route end:
                         B' o 75 m
                          B o 50 m
                       LAND o ground
```

`V` is the vehicle, just beyond the landing location.
`R` is a rally point 50 m back along the route and 10 m to the side, and `P` is its perpendicular projection onto A-B.
Following the route to `P` and turning toward `R` costs 50 + 10 = 60 m.
Branching straight from the landing location to `R` costs about 51 m, so the diagonal would win even though all of it is flown off the route instead of only the final 10 m.
A vertical waypoint would then change the horizontal return path.

To prevent this, vertical segments are handled as follows:

- **Rally-point projections**: a vertical segment never creates its own endpoint candidate.
  The adjacent horizontal leg decides: if `R` projects onto the interior of A-B, that interior projection is used; if `R` lies beyond `B`, the endpoint remains a candidate.
  The same rule applies at takeoff, so vertical waypoints above `TAKEOFF` do not add a branch-off point for a rally point alongside the first horizontal leg.
  Interior stacks keep the normal corner rules, and an entirely vertical mission keeps its final location as a candidate.
- **Vehicle projection**: the vertical segment is kept when it matches the current mission index and flight direction, at both `LAND` and `TAKEOFF`.
  During a landing, a small drift toward `A` must not move the vehicle's projection from B'-LAND onto A-B.
  Keeping B'-LAND lets `LAND` remain the first target, and inside its acceptance radius the join keeps the current altitude instead of climbing back to the approach altitude (see Phase 3 of [Vehicle Projection](#vehicle-projection)).
  This exception is disabled during an active `DO_JUMP`: the jump target index does not mean the vehicle is flying the nominal vertical segment.
  Rally-point projections carry no mission index, so they always use the stricter geometric rule.

For the example above the return path is therefore: follow the route back to `P`, then take the short hop to `R`.

**Limitation**: if the vehicle was targeting `LAND` and then flew a distant GoTo, the mission index still points at `LAND`, so the vertical segment stays a candidate whenever it passes the cross-track margin filter.
Mission continuity can then prefer the diagonal path over a closer horizontal leg.
Only the horizontal path is affected: the current-altitude rule requires the vehicle to be inside the endpoint's acceptance radius, so a distant vehicle flies the diagonal at the approach altitude and never bypasses the altitude requirement.
This is a rare corner case and is accepted.

:::

## Vehicle Projection

When a feature needs to rejoin the route, the first step is to project the current vehicle position onto the mission path and choose a "branch-in" point.
The vehicle may have left the route (for example with a GoTo), so the planner picks the point that best preserves mission continuity.

The branch-in point is chosen in three phases:

**Phase 1: Identifying valid candidates:**

The vehicle is projected onto the route as described in [Point Projection](#point-projection).

**Phase 2: Selecting the best branch-in point:**

The best candidate is chosen with a priority system:

- Priority 1: a candidate is selected immediately if it lies on the segment the vehicle is currently expected to be flying (the last targeted waypoint index, or the last flown `DO_JUMP` segment if the caller supplied it).
  This includes the vertical segment being flown at takeoff or landing (see [Stacked Waypoints](#stacked-waypoints)).
- Priority 2: if no candidate matches Priority 1, the planner scores each candidate by summing the following distances, and keeps the lowest:
  - Crosstrack distance: from the vehicle to the projection on the route.
  - Distance along the route to the last-flown segment: from the projection, following the mission path to whichever end of the last-flown segment is closer.
    This selects the candidate that gets the vehicle back to where it was last flying if available, or the best match for a close in-sequence segment if it is not.

**Example 1:**

The vehicle is flying a mission, where the outbound and inbound legs are close to one another at the start.
While on the outbound leg, the vehicle deviates east to a GoTo location (e.g. due to incoming traffic).
That GoTo location is closer to the inbound leg, but both legs are close enough that the projection step returns two valid candidates: one on the outbound leg and one on the inbound leg.
The outbound segment is the one the vehicle is currently expected to fly, so it wins immediately on Priority 1.
Even without that rule it would still win on Priority 2, which measures distance _along the route_ back to the last-flown segment.
The vehicle was last flying on the outbound leg, so the outbound projection is only a short way along the route from it.
Reaching that same segment from the inbound projection would mean flying almost the entire mission back, so the inbound candidate scores far higher and loses.
The vehicle rejoins the mission and continues toward B.

![Branch-in selection on a mission with outbound and inbound close segments](../../assets/mission_route_planner/mission_route_planner_segment_selection.png)

**Example 2:**

The vehicle is flying north and deviates to a GoTo location that has two valid branch-in candidates.
Neither projection lies on the current segment, so each candidate is scored by summing its crosstrack distance and the distance along the route to the last-flown segment.
Take the candidate projected onto the east segment.
Its score is the crosstrack distance (166.95 m) plus the distance along the route back to the last-flown segment (the red lines).
The closer end of the last-flown segment is used, and the along-route distance is the sum of the straight lines between waypoints.
The other candidate, projected onto the south segment, wins because its total distance is smaller.

![Branch-in scoring when no candidate lies on the current segment](../../assets/mission_route_planner/mission_route_planner_segment_selection_2.png)

**Phase 3: Determining the branch-in altitude:**

- Linear interpolation: by default the altitude is interpolated between the start and end waypoint altitudes of the segment (for example, rejoining on segment 2-3 interpolates between waypoint 2 and waypoint 3).
- Special case (land): if the branch-in point falls on a land segment, the previous waypoint altitude is used.
- Special case (short segments): if the segment is too short to interpolate reliably (such as [stacked waypoints](#stacked-waypoints)), the segment end waypoint altitude is used.
- Special case (route ends): when the first item to fly is `LAND`, or `TAKEOFF` while flying the route in reverse, and the vehicle is already inside that item's horizontal acceptance radius, the join keeps the vehicle's current altitude.
  Both items sit at ground level, and without this rule a vehicle already descending onto them would first be sent back up to the approach altitude.
  Outside the acceptance radius the rules above apply as usual: a vehicle 30 m from a stacked takeoff with a 10 m acceptance radius rejoins at the stack's end altitude.

**`DO_JUMP` loop segments:** the vehicle can also branch-in on a loop edge.
A loop edge is projected and scored like any normal segment, vehicle projection does not skip it, so it competes as an ordinary candidate. This is required because the loop edge might be the only valid candidate (e.g. after a GoTo close to the loop edge).

What happens after joining a loop depends on the selected flight mode (see [Smart Mission Rejoin](#smart-mission-rejoin) and [Route-Following Return](#route-following-return)).

## Safe-Point Scoring

For a route-following return, the vehicle follows the mission route until it reaches a "branch-off" point and then flies straight to a safe point.
The planner first finds the candidate branch-off points and then selects the safe point with the lowest total return cost.

This runs in two phases:

**Phase 1: Identifying valid candidates:**

The eligible safe points are projected onto the route using the same [Point Projection](#point-projection) described above.
Vertical segments at the route ends do not add their own branch-off candidates (see [Stacked Waypoints](#stacked-waypoints)).
The vehicle was already projected by the [Vehicle Projection](#vehicle-projection) step, so its branch-in point is reused here rather than recomputed.

**Phase 2: Selecting the best projection point:**

Each candidate branch-off is scored by total path cost, and the lowest wins. The cost is built from:

- Along-route distance: along the route geometry from the vehicle projection to the safe-point projection (branch-off point), using straight lines between waypoints.
- Branch-off leg: the straight-line distance from the safe-point projection to the safe point (the off-route leg flown after leaving the route).
- U-turn penalty: for fixed-wing and VTOL-in-FW, an extra distance penalty is added when the path would require an immediate U-turn, so forward-flowing paths are preferred. The U-turn is detected by comparing the vehicle's current velocity with its desired course: toward the branch-in point when far from the route, or along the selected route direction when close to it. If the two are more than 90° apart, the penalty is applied. Set [RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN) to 0 to disable this penalty. Multirotors and hovering VTOL are exempt, because they can turn on the spot.

As the planner gathers the safe points to consider, it filters out the ones it cannot use:

- Invalid coordinates, unsupported frames, or filtered safe points are skipped.
- Every remaining safe point gets up to three projections.

Safe-point branch-off candidates on loop segments are more restricted:

- If the vehicle is not itself flying a loop, safe-point projections on loop segments are skipped.
  This prevents a return from entering a mission loop just to reach a branch-off point.
- If the vehicle and selected safe point project onto the same active loop segment, the return path may follow that loop edge to the branch-off point.
- If the selected safe point is outside the active loop, the planner first chooses how to leave the loop (shortest overall path), then follows the normal route toward the selected branch-off point or fallback endpoint.

### Route-Skip Shortcuts

After a destination has been selected, the vehicle can skip joining and following the route in these cases:

- **Direct-to-safe-point**: if the vehicle is already within the direct acceptance radius of the selected safe point, go straight to it.
- **Close-to-branch-leg**: if the vehicle is already close to the selected branch-off leg (both horizontally and vertically), continue straight toward the goal.
- **Endpoint goal**: for a mission `LAND` or `TAKEOFF` goal, the route-ends rule of [Vehicle Projection](#vehicle-projection) Phase 3 applies. When that endpoint is the first item to fly and the vehicle is inside its acceptance radius, the Return skips route following and flies directly to it.

These shortcuts are applied only after selection, so they never change which goal wins the cost comparison.

**Example:** In the image below the vehicle is already on the branch-off leg, after a GoTo or a cancelled Return.
If a new Return is requested, the vehicle flies straight to the rally point (`R`) instead of flying back to the branch-in point only to branch off again.

![Close-to-branch-leg shortcut](../../assets/mission_route_planner/mission_route_planning_close_to_branch_off.png)

## Parameters and Build Configuration

| Setting            | Purpose                                                                           | Default |
| ------------------ | --------------------------------------------------------------------------------- | ------- |
| `MIS_ROUTE_JOIN`   | Enable route rejoin on airborne Mission activation.                               | `0`     |
| `MIS_MC_SEG_DIST`  | Extra cross-track search margin for vehicle projections in multicopter mode.      | 30 m    |
| `MIS_FW_SEG_DIST`  | Extra cross-track search margin for vehicle projections in fixed-wing mode.       | 150 m   |
| `RTL_RP_SEG_DIST`  | Extra cross-track search margin for rally-point projections.                      | 30 m    |
| `RTL_FW_UTURN_PEN` | Additional distance cost for a fixed-wing reversal during route-following Return. | 4000 m  |

`CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE` must be greater than zero and large enough for the entire uploaded mission.
SITL defaults to 500 mission items.
An oversized mission can still be flown in Mission mode, but smart rejoin and route-following Return cannot use it.
See [Safe-Point Batching](#safe-point-batching) for the rally-point batch configuration and memory tradeoff.

::: details Firmware Configuration: Safe-Point Batching

### Safe-Point Batching {#safe-point-batching}

`CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE` controls how many rally points are checked in each scan of the mission route.
Its default is `1` (`32` on testing builds), with a supported range of `1` to `32`.
More rally points than fit in one batch require additional scans, increasing planning time.
Increasing the batch size reduces the number of scans but uses more RAM.

For firmware builds with enough memory, all supported rally points can be checked in one scan:

```ini
CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE=32
```

:::
