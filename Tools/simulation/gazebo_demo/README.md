# Local X500 Gazebo Demo

Run a repeatable PX4 SITL demonstration on macOS with a bright yellow X500, orange propellers, blue sky and a pale ground grid.
The grid covers 500 × 500 metres around the takeoff point, with 2-metre cells and darker 10-metre lines.
The camera follows the drone while the ground provides a stationary reference for movement.

The demo uses the upstream X500 mass, inertia, joints, collisions, sensors and four normal motor plugins.
It changes visual materials and adds a visual-only yellow panel and ground texture.
Motor-failure injection, cameras and optical flow are outside this example.

## Quick Start On This Mac

From the repository root, run:

```sh
bash Tools/simulation/gazebo_demo/run_demo.sh
```

Keep that terminal open.
The launcher opens one Gazebo GUI and QGroundControl, then waits for PX4 to report readiness.
In QGroundControl, wait for the simulated vehicle to show **Ready to Fly**, choose **Takeoff**, and confirm the simulated takeoff.
Use the flight controls or a small local mission to demonstrate movement across the grid.
Choose **Land** when finished, then press `Ctrl+C` in the launch terminal to stop the demo.

Every launch resets the simulated drone to the takeoff point.
QGroundControl remains open after the simulation stops.
The macOS application menu may say `ruby` because Gazebo's `gz` executable is a Ruby launcher.
The window title is Gazebo GUI or Gazebo Sim.

## Install On Another Mac

This launcher supports macOS on Apple Silicon and Intel.
The demo was exercised on Apple Silicon with Gazebo Sim 8.10.0.
The repository's pinned environment supports both architectures, but this demo has not been verified on Intel or Linux.
The linked QGroundControl build requires macOS 13 or later.

1. Install Xcode Command Line Tools:

   ```sh
   xcode-select --install
   ```

2. Install [Homebrew](https://brew.sh/) if it is not already available.
   Clone your fork with its submodules and select the branch or commit containing this example:

   ```sh
   git clone --recursive <your-repository-url> PX4-Autopilot
   cd PX4-Autopilot
   git checkout <your-demo-branch-or-commit>
   git submodule update --init --recursive
   ```

3. Install the repository's development and simulation dependencies:

   ```sh
   bash Tools/setup/macos.sh --sim-tools
   ```

   This installs the toolchain, creates `.venv`, and installs the locked Gazebo environment under `Tools/setup/macos/.pixi`.
   If XQuartz has just been installed, log out and back in when required by macOS.

4. Install the official [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) macOS application in `/Applications/QGroundControl.app`.

5. Build and launch:

   ```sh
   ulimit -S -n 2048
   bash Tools/simulation/gazebo_demo/run_demo.sh --build
   ```

   `--build` runs `make px4_sitl_default` using the activated environment before starting the simulation.
   On later launches, omit `--build` unless the source, toolchain or build has changed.

There is no cloud deployment step.
PX4, Gazebo and QGroundControl run locally and communicate through loopback.
Install dependencies once; subsequent demonstrations use the single launch command.

## Dependencies

The setup script and lock file are the authoritative dependency lists.
Do not combine a Homebrew Gazebo installation with this checkout's pinned Gazebo libraries.

- System tools: macOS, Xcode Command Line Tools, Git, Bash, Homebrew, XQuartz and Python 3.
- Controller application: QGroundControl installed in `/Applications`, listening for MAVLink on UDP 14550.
- Homebrew development packages installed by `Tools/setup/macos.sh`: `astyle`, `bash-completion`, `ccache`, `cmake`, `fastdds`, `genromfs`, `kconfig-frontends`, `ncurses`, `ninja`, `osx-cross/arm/arm-gcc-bin@13`, `flock`, `python` and `python-tk`.
- Homebrew simulation packages installed by the same script: `exiftool`, `glog`, `graphviz` and `pixi`.
- Python build dependencies: all packages in [`Tools/setup/requirements.txt`](../../setup/requirements.txt), including `empy<4`, `jinja2`, `numpy`, `pyyaml`, `pymavlink`, `kconfiglib`, `packaging` and the other listed build dependencies.
- Pinned simulation packages: [`Tools/setup/macos/pixi.toml`](../../setup/macos/pixi.toml) requests `gz-harmonic`, `cppzmq`, headless `libopencv` 4, `gstreamer`, and the base, good, bad and ugly GStreamer plugins.
- Exact simulation package versions and transitive dependencies: [`Tools/setup/macos/pixi.lock`](../../setup/macos/pixi.lock), including Gazebo Sim, GUI, rendering, transport, messages, sensors, physics, SDFormat, Qt, Ogre, Ruby, ZeroMQ, Protobuf and Abseil.
- Repository assets: the X500 and X500 base models and their meshes under `Tools/simulation/gz/models`, the default world, and PX4's Gazebo sensor configuration.

The demo launcher itself uses only Python's standard library.
It adds no pip packages and generates its grid PNG without an image-processing dependency.
The standard setup installs the full PX4 simulation stack; the demo does not load the optional custom optical-flow and camera system libraries.
ROS, Docker, a physical flight controller and a physical joystick are not required for this example.

## Launcher Options

```sh
# Rebuild the SITL target, then launch the full demonstration.
bash Tools/simulation/gazebo_demo/run_demo.sh --build

# Use an already-open controller, or run without opening QGroundControl.
bash Tools/simulation/gazebo_demo/run_demo.sh --no-qgc

# Run PX4 and Gazebo without a GUI or opening QGroundControl.
bash Tools/simulation/gazebo_demo/run_demo.sh --headless --no-qgc

# Generate the portable world, texture and server configuration only.
bash Tools/simulation/gazebo_demo/run_demo.sh --prepare-only
```

With `--no-qgc`, startup success does not imply flight readiness.
Check the controller and `px4.log` before commanding flight.
With the default options, a missing ground station or unresolved preflight problem causes startup to time out rather than reporting success.

## How The Example Is Saved

Commit this directory's `demo.py`, `run_demo.sh`, `gui.config` and `README.md` to your fork.
The launcher derives paths from its location, so there are no username-specific paths or dependencies on files in `/tmp`.
It builds the demonstration world from the repository's X500 assets on each launch, avoiding a second copy of its flight dynamics.

Generated worlds, textures and logs go into a fresh directory under `build/px4_sitl_default/demo/`.
These files are build outputs and should not be committed.
The launcher prints the directory containing `server.log`, `px4.log` and, for graphical launches, `gui.log`.

Each launch gets a distinct Gazebo transport partition and `GZ_IP=127.0.0.1` for the server, PX4 and GUI.
The launcher finishes PX4 startup before opening the GUI, avoiding competing scene requests during initialisation.
The GUI layout and plugin paths are supplied explicitly.
Appearance is configured before startup, so the demo does not depend on live material-update requests.
The launcher owns its child processes and stops them on `Ctrl+C`, startup failure or GUI closure.
It refuses to start another instance when its lock or the PX4 UDP ports are occupied.

## Troubleshooting

### Another Simulation Is Already Running

Stop the previous demo using `Ctrl+C` in its original terminal.
This also applies to manually launched `make px4_sitl gz_x500` sessions.
Do not start `gz gui` or `gz sim -g` separately while this launcher is running.
QGroundControl can remain open and reconnect on the next launch.

### A Lock Remains After A Crash

The lock is `build/px4_sitl_default/demo/running.lock/launcher.pid`.
Read the PID and confirm the launcher and its simulation processes have exited before removing the stale lock:

```sh
cat build/px4_sitl_default/demo/running.lock/launcher.pid
ps -p <launcher-pid> -o pid,args
```

After confirming that no old demo processes remain:

```sh
rm build/px4_sitl_default/demo/running.lock/launcher.pid
rmdir build/px4_sitl_default/demo/running.lock
```

### Blank Window, Frozen Scene Or Missing Sensors

Inspect the printed log directory.
Stop the launcher and start it again to reset both server and GUI.
A process existing by itself does not prove the server is responsive.
`px4.log` should eventually contain `Ready for takeoff!` when QGroundControl is connected.
Repeated missing-accelerometer, gyroscope or barometer messages indicate that the sensor systems have not initialised.

Use the provided launcher rather than launching an empty `gz gui` shell.
It supplies the GUI plugins, matching local transport settings and the PX4 sensor configuration.
The display remains a normal Gazebo GUI window even if the macOS menu calls the hosting process `ruby`.
Bring the Gazebo window forward if it is obscured; macOS can defer rendering in a background window.
The launcher configures camera following when the renderer initialises, including after a delayed window activation.

### Dependencies Or ABI Changed

Run `bash Tools/setup/macos.sh --sim-tools` from this checkout, then launch with `--build`.
Keep the pinned environment and the build from the same checkout.
If you move the checkout, rerun setup to regenerate its environment paths.
For a fresh deployment, use a fresh clone rather than copying `.venv`, `.pixi` or compiled binaries from another Mac.

### Drone Reaches The Edge Of The Grid

The marked ground extends 250 metres in each direction from home.
Keep the demonstration mission inside that area.
The original large ground plane remains beneath the visual grid, so the grid does not add obstacles or change collisions.

## Validation Of This Example

The launcher was exercised on the existing Apple Silicon installation with its existing PX4 SITL build.
Asset generation, the rendered yellow drone and pale grid, PX4 readiness with QGroundControl, refusal of a duplicate launch, and process/lock cleanup after `Ctrl+C` were checked.
Shell syntax and Python compilation checks also passed.
Fresh-machine dependency installation, the optional rebuild flag and Intel execution have not been exercised as part of this example.
