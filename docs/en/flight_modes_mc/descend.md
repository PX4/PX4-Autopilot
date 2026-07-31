# Descend Mode (Multicopter)

_Descend_ is a [failsafe](../config/safety.md) fallback mode.
It is activated automatically by PX4 and cannot be selected by the pilot. It only appears as a status label.

The vehicle descends open-loop: it keeps its attitude and reduces altitude, but does not control its horizontal position, so it drifts with the wind.
It is the last resort used when the vehicle must come down but has no valid position estimate for a controlled descent.

::: info

- Mode is automatic and cannot be selected by the user.
- Requires only attitude, so it works without a position (or even altitude) estimate.
- The descent rate is set by [MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED).

:::

## When It Occurs

Descend is the bottom of the failsafe chain (`Hold → Return → Land → Descend`).
PX4 falls through to it whenever a failsafe needs to bring the vehicle down or hold position but the position estimate is missing, so none of the higher options can run. For example:

- Losing the position estimate while landing, e.g. GNSS fails during [Land](../flight_modes_mc/land.md): the vehicle keeps descending, but now open-loop as _Descend_.
- Losing the position estimate in [Hold](../flight_modes_mc/hold.md), [Mission](../flight_modes_mc/mission.md) or [Return](../flight_modes_mc/return.md): with no position to hold, fly to, or return with, the failsafe escalates down to _Descend_.
- A Return or Land failsafe (from manual control loss, GCS/data link loss, low battery, geofence breach, …) triggered while no valid position estimate is available: Return and Land can't run, so it degrades to _Descend_.

## Exiting Descend

Descend ends when either:

- the failsafe condition is resolved (e.g. the position estimate recovers), and the vehicle returns to its previous mode; or
- the pilot takes over by switching to a manual mode ([Position](../flight_modes_mc/position.md), [Altitude](../flight_modes_mc/altitude.md) or [Stabilized](../flight_modes_mc/manual_stabilized.md)). On a multicopter, moving the sticks does this [by default](../flight_modes_mc/land.md#MAN_OVERRIDE_SPD).

## See Also

- [Descend Mode (FW)](../flight_modes_fw/descend.md)
- [Land Mode (MC)](../flight_modes_mc/land.md)
- [Safety (Failsafes)](../config/safety.md)
