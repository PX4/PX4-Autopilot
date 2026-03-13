# PX4 Lockstep (Strategy A)

Goal: run **Navigator + flight_mode_manager + Mission + mc_pos_control + mc_att_control + mc_rate_control + control_allocator**
**single-threaded and deterministic** in a Julia-driven lockstep loop.

⚠️ **Commander-in-loop is not supported yet.** `enable_commander != 0` is a hard error.
That means this lockstep sim does not validate arming/failsafe/mode logic unless you
implement it in the bridge.

Key principles:

- **Reuse uORB as-is** (shared memory, no extra threads required).
- **Reuse Mission/Navigator** (no mission re-implementation). Load missions by preloading Dataman.
- **No module threads/tasks**: we instantiate modules as objects and call `run_once()` in a fixed order.
- **PX4 time driven by Julia**: `hrt_absolute_time()` returns an injected sim time in lockstep mode.

This directory builds `libpx4_lockstep` (shared library) exposing a small C ABI
(`include/px4_lockstep/px4_lockstep.h`).

## Build integration

1. Copy this folder into:

```
PX4-Autopilot/src/lib/px4_lockstep
```

2. Add to `PX4-Autopilot/src/lib/CMakeLists.txt`:

```
add_subdirectory(px4_lockstep)
```

3. Ensure your PX4 build generates the required static module libraries (names in `CMakeLists.txt`).

4. Configure the lockstep SITL build (recommended; trims default sim modules):

```
make px4_sitl_lockstep
```

This uses the minimal lockstep board config in `boards/px4/sitl/lockstep.px4board`.

5. Build or rebuild just the lockstep library:

```
ninja -C build/px4_sitl_lockstep px4_lockstep
```

The output should contain something like:

```
build/px4_sitl_lockstep/src/lib/px4_lockstep/libpx4_lockstep.dylib
```

On Linux the library ends with `.so` (instead of `.dylib`).

If you prefer the default SITL build, use:

```
make px4_sitl_default
ninja -C build/px4_sitl_default px4_lockstep
```

## Runtime

From Julia, `dlopen` the library and `ccall`:

- `px4_lockstep_create()`
- `px4_lockstep_load_mission_qgc_wpl()`
- `px4_lockstep_step_uorb()` each sim tick

Inputs/outputs are exchanged via the generic uORB publish/subscribe API:

- Queue uORB publishes with `px4_lockstep_orb_queue_publish()` before each step.
- Read outputs via uORB subscriptions (`actuator_motors`, `vehicle_attitude_setpoint`,
  `mission_result`, etc.).

## Julia wrapper

A minimal Julia wrapper lives in `Tools/px4_lockstep_julia` and provides a clean
starting point for a simulator.

`PX4Lockstep.jl` searches `build/px4_sitl_lockstep` and `build/px4_sitl_default`
for the shared library; override with `PX4_LOCKSTEP_LIB` if needed.

Example (after building `px4_sitl_lockstep`):

```
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/iris_mission_lockstep_sim.jl
```

The Julia example writes `sim_log.csv` and a `sim_plot.png` overview
plot of position, setpoints, and velocity for quick visualization.

You can override the shared library and mission path via:

```
PX4_LOCKSTEP_LIB=build/px4_sitl_lockstep/src/lib/px4_lockstep/libpx4_lockstep.dylib \
PX4_LOCKSTEP_MISSION=/path/to/mission.waypoints \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/iris_mission_lockstep_sim.jl
```

The example keeps Commander disabled (lockstep is not implemented yet), but
enables the control allocator and includes a quad-physics loop so missions can
actually advance.

## Julia lockstep sim (Iris)

`Tools/px4_lockstep_julia/examples/iris_mission_lockstep_sim.jl` implements a per-motor physics
loop tuned to the Gazebo Iris parameters:

- Mass/inertia are pulled from
  `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja`.
- Rotor geometry/yaw moments match the Iris airframe defaults in
  `ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris`.
- Per-motor outputs use `actuator_motors` (normalized thrust fraction). The
  sim scales thrust so `hover_thrust=0.5` corresponds to weight; tune this if
  you change vehicle mass.

Run it like this:

```
ninja -C build/px4_sitl_lockstep px4_lockstep
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/iris_mission_lockstep_sim.jl
```

To capture PX4 debug logs emitted on stderr:

```
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/iris_mission_lockstep_sim.jl \
  2>&1 | tee julia_out.txt
```

## Required PX4 patches

This library expects small, low-paranoia patches in PX4 core:

- POSIX HRT: allow lockstep time injection (`hrt_lockstep_set_absolute_time()`)
- Dataman: allow lockstep init + synchronous read/write (`dm_lockstep_init()` / `dm_lockstep_set_sync()`)
- Navigator + flight_mode_manager + controllers: add `enable_lockstep()`, `init_lockstep()` and `run_once()` wrappers
- Control allocator: add `enable_lockstep()`, `init_lockstep()` and `run_once()` wrappers
- Commander: add `enable_lockstep()`, `init_lockstep()` and `run_once()` wrappers (and disable its low-priority thread in lockstep)

See the git diff in this branch for the exact changes.
