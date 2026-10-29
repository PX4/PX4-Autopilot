# Mission Route Planning

The mission route planner computes where to join an uploaded mission and how to follow it towards a safe point or mission endpoint.
It is a library: it does not publish setpoints, change mission progress, or enable a flight mode.
Flight modes do not yet call this library.

`MissionRoutePlanner` reads mission and rally-point data through a `Provider`, such as the [mission route cache](../advanced/mission_route_cache.md).
The caller supplies the vehicle position, mission progress, search margins, acceptance radius and vehicle capabilities.
The result describes the branch-in point, route direction and, for a return plan, the branch-off point and destination.

## Point Projection

The planner projects the vehicle and safe points onto the mission route, retaining up to three candidates per reference point.
Candidates must lie within the caller's cross-track search margin of the closest eligible projection.
When more than three qualify, the three with the smallest cross-track distances are retained.

An interior projection is a local distance minimum and can be retained directly.
A waypoint at a corner is retained when both adjacent segments project onto it, or when it is an endpoint of the route.
Consecutive waypoints at the same latitude and longitude form a vertical segment.
For safe points, the adjacent horizontal segments determine whether that position is a candidate; a wholly vertical mission retains its final position.
For the vehicle, the vertical segment matching its current mission index can also be retained, except during an active `DO_JUMP`.

`DO_JUMP` edges are considered as additional geometry without following their repeats during the scan.
The vehicle can project onto these edges.
A safe point can project onto a jump edge only when the vehicle's selected branch-in lies on that same edge.
Other jump edges cannot narrow the safe point's search window.

## Vehicle Projection

A candidate on the segment the vehicle is expected to fly takes priority.
The caller can identify the active `DO_JUMP` edge through the last-flown jump information.
Otherwise, the planner minimises the sum of cross-track distance and distance along the route to the nearer end of the expected segment.

The branch-in altitude is interpolated between segment endpoints.
A landing segment uses the preceding waypoint's altitude; a vertical segment uses its end waypoint's altitude.
When the first route item is a landing, or a takeoff approached in reverse, and the vehicle is within its horizontal acceptance radius, the plan can retain the current altitude.
Callers may then proceed directly to that endpoint: a mission-resume plan checks that direct path, and a return plan checks it when the endpoint is its destination.

A mission-resume plan continues towards the mission end.
When joining a jump edge with repeats remaining, it preserves the loop direction.
For an exhausted loop, it selects the exit with the lower total cost to the mission end.
A return plan selects the lower-cost exit towards its destination without repeating the loop.

### VTOL Leg State

The planner infers each leg's mode from the preceding `VTOL_TAKEOFF` or `DO_VTOL_TRANSITION` command.
If none defines it, it looks ahead in nominal mission order: a front transition implies multicopter mode before it, and a back transition implies fixed-wing mode before it.
Lookahead passes jump commands without following their targets and stops at a landing, takeoff or unreadable item.
An undefined result leaves mode selection to the caller.
The plan reports any required transition; the caller is responsible for executing it.

## Safe-Point Scoring

The planner projects eligible safe points onto the route and chooses the branch-off with the lowest sum of:

- Distance along the route from the vehicle's branch-in.
- Horizontal distance from the branch-off to the safe point.
- The caller's fixed-wing U-turn penalty, when the desired course opposes the vehicle's velocity.

Multicopters and VTOL vehicles joining a multicopter leg are exempt from the U-turn penalty.
Invalid safe-point coordinates and unsupported frames are skipped.
The caller can require a VTOL landing approach for eligible rally points.
If no safe point is usable, the planner considers the mission landing or takeoff endpoint.

The branch-off connection is planned level at the branch-off altitude.
The caller must handle descent and arrival at the destination separately.

## Geofence Checks

An optional `PathChecker` validates connections without coupling the planner to Navigator's geofence implementation.
`GeofencePathChecker` implements this interface using the loaded geofence.
It checks vehicle-to-branch-in, branch-off-to-safe-point and direct shortcut connections against permitted areas, altitude and distance-from-home limits.
Touching or crossing an inclusion or exclusion boundary is rejected.

The initial scan checks the retained candidates in batches.
If checking removes the closest candidate, or removes any candidate from a full buffer, a blocked candidate may have hidden other usable projections.
One additional route scan serves all affected reference points, checking each candidate before it can narrow the search window or occupy a buffer slot.
A reference point outside the permitted area is excluded without a rescan.

When the vehicle breaches a polygon or circle, connections starting at the vehicle are exempt to allow recovery towards the route.
Branch-off connections remain checked.
A vehicle outside only the altitude or distance-from-home limits may still start connections; their ends and paths are checked.
An unavailable fence or an unavailable path check fails planning.

The planner does not validate the uploaded route itself, find detours, check obstacles or account for turning space.
Callers must validate the mission separately, including any jump edges they intend to follow.
VTOL landing-approach connections and loiters are outside these checks.

## Route-Skip Shortcuts

After selecting the destination, the planner can permit a direct connection when the vehicle is already within the destination's horizontal acceptance radius, or close to the branch-off leg at its altitude.
For a mission landing or takeoff endpoint, the endpoint must also be the first route item to fly.
These shortcuts do not affect which destination wins the cost comparison.
The direct connection must pass the path checker.
A rejected shortcut preserves the route unless the nearby endpoint would immediately require the same rejected connection, in which case the plan fails.

## Safe-Point Batching

`CONFIG_NAVIGATOR_SAFE_POINT_BATCH_SIZE` defaults to `16` (`32` for testing builds), with a supported range of `16` to `32`.
The planner scans the route once for the vehicle and once per safe-point batch, with an additional scan when blocked candidates may have hidden usable ones.
The minimum of `16` limits repeated route scans; `32` permits all supported rally-list items to fit in one batch.
Approach and loiter items also occupy rally-list storage, so changing the batch size does not change upload capacity.

Each projection slot takes 392 bytes of static RAM when the planner is linked: about 6.3 KB at `16` and 12.6 KB at `32`.
Geofence checks share a separate static buffer of about 1.4 KB for up to 32 paths.
The buffers are reused, so callers must serialise planning calls.
The library is compiled only with a non-zero `CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE`; unused planner and checker code is discarded by the linker.

Planning is synchronous.
Larger batches reduce route walks but do not bound the total time spent checking fences.
In the worst case, a safe point separated from every segment by a fence requires one path check per segment in the rescan, each run on its own rather than in a batch.

[Earlier measurements on a physical Pixhawk 6C running SIH][v6c-timings] compared batches of 1 and 32 in the route-following prototype:

| Mission Items | Rally Points | Batch 1 Mean / Maximum | Batch 32 Mean / Maximum |
| ------------: | -----------: | ---------------------: | ----------------------: |
|           300 |           32 |     152.00 / 152.41 ms |        57.43 / 57.89 ms |
|          1000 |           32 |     500.54 / 501.41 ms |      186.05 / 186.41 ms |

These measurements predate the default batch of `16` and the blocked-candidate rescan, so they are not timing bounds for this library revision.
Batch 1 is no longer supported.
Consumers need to measure their mission and fence workloads on the target hardware and account for the time spent outside other Navigator duties.

[v6c-timings]: https://github.com/PX4/PX4-Autopilot/pull/27687#issuecomment-6360146557
