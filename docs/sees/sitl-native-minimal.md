# PX4 SITL with Minimal External Dependencies

This workflow runs PX4 SITL natively on Linux, using PX4's built-in Software-in-the-Loop (SIH) simulator. SIH runs the vehicle physics inside the PX4 process, so it does not require an external simulator such as Gazebo. Known to work on v1.17 branch.

## Host and Python dependencies

Install the native build tools for your distribution: `make`, CMake, Ninja, a C++ compiler, and Python 3. Then create a virtual environment and install PX4's Python requirements from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r Tools/setup/requirements.txt
```

The requirements file constrains EmPy to `>=3.3,<4`. EmPy 4.x is incompatible with PX4's template generators; in particular, configuration can fail with `AttributeError: module 'em' has no attribute 'RAW_OPT'`. Using the requirements file in the active environment avoids accidentally selecting an incompatible system or user-installed EmPy.

## Build and run SIH

From the repository root, with the virtual environment active:

```sh
make px4_sitl sihsim_quadx
```

This builds and starts PX4 SITL with SIH's quadrotor-X model. The simulator runs headless by default. QGroundControl can connect over UDP port 14550 and show the vehicle on its map.

Other SIH models and their support status are listed in the [SIH documentation](../en/sim_sih/index.md). The quadrotor is the stable model; the other listed vehicle types are experimental.

