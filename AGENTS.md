# AGENTS.md -- PX4-Autopilot

Open-source flight controller (C/C++, NuttX/POSIX/QURT). Docs live in-tree at `docs/en/` (rendered at https://docs.px4.io).

## Docs (read these first)

- **Building & targets:** `docs/en/dev_setup/building_px4.md`
- **Code style, conventions & commits:** `docs/en/contribute/code.md`
- **Architecture & modules:** `docs/en/concept/architecture.md`
- **uORB messaging:** `docs/en/middleware/uorb.md`
- **Simulation (incl. choosing SIH vs Gazebo):** `docs/en/simulation/index.md`
- **Testing:** `docs/en/test_and_ci/unit_tests.md`
- **Test flights:** `docs/en/test_and_ci/test_flights.md`

## Build Verification

Always test across architectures before submitting:

```bash
make px4_sitl_default           # POSIX / SITL
make px4_fmu-v6x_default        # STM32H7 (Pixhawk 6X)
make px4_fmu-v5x_default        # STM32F7 (Pixhawk 5X)
make nxp_fmuk66-v3_default      # NXP K66
```

## Areas That Benefit From Extra Review

| Directory | Controls | Before modifying |
|-----------|----------|------------------|
| `src/modules/commander/` | Arming, failsafe, mode transitions | Verify failsafe in SITL for all vehicle types |
| `src/modules/ekf2/` | State estimation | Run EKF replay tests |
| `src/modules/mc_*_control/` | Multicopter controllers | Test in SIH; verify all MC airframes |
| `src/modules/fw_*/` | Fixed-wing controllers | Test FW airframes in SITL |
| `src/modules/vtol_att_control/` | VTOL transitions | Test MC + FW modes and transitions |
| `msg/*.msg` | uORB schemas | Check all publishers/subscribers |
| `ROMFS/px4fmu_common/init.d/` | Startup & default params | Can change behavior on all boards |

## Rules

- Never bypass safety checks (arming, geofence, failsafe) without justification
- Document parameter changes, they affect flight behavior
- Specify units in comments for physical quantities
- No magic numbers, use named `constexpr` constants

## Agent Decision Framework

**Do without asking:** write or update the docs, ensure successful build, ensure CI checks are successful, run `make format`, add tests for new features or fixes.

**Ask first:** parameter defaults, control algorithms, failsafe logic, uORB schema changes, board configs.

**Stop, do not proceed:** if you can't verify flight safety, if modifying EKF2/controller math without SITL/SIH, if removing safety guards.
