# Descend Mode (Fixed-Wing)

_Descend_ is a [failsafe](../config/safety.md) fallback mode.
It is activated automatically by PX4 and cannot be selected by the pilot. It only appears as a status label.

The vehicle descends open-loop: it circles at a fixed bank angle ([FW_GPSF_R](../advanced_config/parameter_reference.md#FW_GPSF_R)) and descends at a fixed 0.5 m/s, but does not control its ground position, so the circle drifts with the wind.
It is the last resort used when the vehicle must come down but has no valid position estimate for a controlled descent.

::: info

- Mode is automatic and cannot be selected by the user.
- Requires only attitude, so it works without a position estimate.
- The vehicle first loiters at the current altitude for [FW_GPSF_LT](../advanced_config/parameter_reference.md#FW_GPSF_LT) seconds (waiting for the estimate to recover) before starting to descend.

:::

## When It Occurs

Descend is the bottom of the failsafe chain (`Hold → Return → Land → Descend`).
PX4 falls through to it whenever a failsafe needs to bring the vehicle down or hold position but the position estimate is missing, so none of the higher options can run. For example:

- Losing the position estimate while landing, e.g. GNSS and airspeed sensors fail during [Land](../flight_modes_fw/land.md): the vehicle keeps descending, but now open-loop as _Descend_.
- Losing the position estimate in [Hold](../flight_modes_fw/hold.md), [Mission](../flight_modes_fw/mission.md) or [Return](../flight_modes_fw/return.md): with no position to hold, fly to, or return with, the failsafe escalates down to _Descend_.
- A Return or Land failsafe (from manual control loss, GCS/data link loss, low battery, geofence breach, …) triggered while no valid position estimate is available: Return and Land can't run, so it degrades to _Descend_.

## Exiting Descend

Descend ends when either:

- the failsafe condition is resolved (e.g. the position estimate recovers), and the vehicle returns to its previous mode; or
- the pilot takes over by switching to a manual mode ([Position](../flight_modes_fw/position.md), [Altitude](../flight_modes_fw/altitude.md) or [Stabilized](../flight_modes_fw/stabilized.md)).

## See Also

- [Descend Mode (MC)](../flight_modes_mc/descend.md)
- [Land Mode (FW)](../flight_modes_fw/land.md)
- [Safety (Failsafes)](../config/safety.md)
