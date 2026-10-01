# GNSS docs pass (step 8 of #28813)

Draft placeholder: this file is the plan and is deleted when the work lands. Stacked on the step 7 PR. Until this step, each step updated only the docs references it renamed.

## Goal

The docs describe the GNSS pipeline as it ships: checks per receiver, the selection, how a switch reaches EKF2, and loss reporting.

## Scope

- `advanced_config/tuning_the_ecl_ekf.md`: the checks run per receiver in the sensors module (`GNSS_CHECK`, `GNSS_REQ_*`, strict until the first pass and while disarmed, relaxed in flight). EKF2 fuses only usable samples, resets its position on a receiver switch, and reports `gnss_fusion_state`.
- `gps_compass/index.md`, multiple receivers: the selection with and without `SENS_GNSS_PRIME`, the moving base preferred automatically, and what `GPS_RAW_INT` and `GPS2_RAW` show. Blending is gone.
- RTK, heading and moving-baseline pages (`gps_compass/rtk_gps.md`, `gps_compass/u-blox_f9p_heading.md`, `gps_compass/septentrio.md`, `advanced/rtk_gps.md`) and the ARK RTK pages under `dronecan/`: configuration under the new selection. A moving base pair no longer needs `SENS_GNSS_PRIME`.
- `advanced_config/gnss_degraded_or_denied_flight.md`: loss reporting and the failsafe as the user sees them.
- Every page that still describes blending or `SENS_GNSS_MASK`.
- `releases/main.md`: an entry for each user-visible change of steps 5 to 7 that lacks one.

Only `docs/en` changes; the translations are regenerated from it.

## Out of scope

- Renaming GPS to GNSS in docs prose and page titles: step 9.
