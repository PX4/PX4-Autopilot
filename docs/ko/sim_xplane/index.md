# X-Plane Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4 and may be removed in future releases.
See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[X-Plane](https://www.x-plane.com/) can be used with PX4 for [Software-In-The-Loop (SITL)](../simulation/index.md#sitl-simulation-environment) simulation through the community-supported [px4xplane](https://github.com/alireza787b/px4xplane) bridge plugin.
PX4 runs as the flight controller, while X-Plane provides the vehicle dynamics, visual scene, and simulated sensor inputs.
The px4xplane bridge is an X-Plane plugin: it runs inside X-Plane, accepts PX4's simulator MAVLink connection, publishes simulated sensors to PX4, and writes PX4 actuator outputs to X-Plane datarefs.

## Supported Vehicles

PX4 includes X-Plane SITL airframes for the following vehicles:

| Vehicle        | 형식              | PX4 target                                 |
| -------------- | --------------- | ------------------------------------------ |
| Cessna 172     | Plane           | `make px4_sitl_default xplane_cessna172`   |
| TB2            | Plane           | `make px4_sitl_default xplane_tb2`         |
| Ehang 184      | 멀티콥터            | `make px4_sitl_default xplane_ehang184`    |
| Alia-250       | VTOL quadplane  | `make px4_sitl_default xplane_alia250`     |
| QuadTailsitter | VTOL tailsitter | `make px4_sitl_default xplane_qtailsitter` |

The matching X-Plane aircraft files and bridge configuration are distributed by the px4xplane project.
Additional X-Plane aircraft or vehicle models can also be integrated when PX4 has a matching SITL-capable control path and px4xplane can map the required actuator outputs to writable X-Plane datarefs.

## 설치

1. Install the [PX4 development environment](../dev_setup/dev_env.md) on the machine, container, or WSL2 environment that will run PX4 SITL.

2. Install X-Plane 11 or X-Plane 12.

3. Download the latest OS-specific px4xplane release package from the [px4xplane releases page](https://github.com/alireza787b/px4xplane/releases).

4. The release package is an archive, not an installer.
   Extract it and copy the complete `px4xplane` plugin folder into:

   ```sh
   X-Plane/Resources/plugins/px4xplane
   ```

5. Copy the matching X-Plane aircraft folders from the same extracted package into an X-Plane aircraft directory.
   Keep the packaged `64/config.ini` with the `px4xplane` plugin unless the release notes say otherwise.

The [px4xplane build and installation guide](https://github.com/alireza787b/px4xplane/blob/master/docs/BUILD.md#installation) has more detail on the plugin folder layout.

## Network Setup

PX4 connects to the X-Plane bridge on TCP port `4560`.
The default host is `localhost`, which is suitable when PX4 and X-Plane run on the same Linux or macOS machine.

For Windows, the usual setup is X-Plane running natively on Windows and PX4 running in WSL2.
In that case, set `PX4_SIM_HOSTNAME` in the WSL2 shell to the Windows host IP before starting PX4.
This variable is set on the PX4/SITL side and points to the host where X-Plane and the px4xplane plugin are running:

```sh
export PX4_SIM_HOSTNAME=$(ip route | awk '/default/ {print $3; exit}')
```

Then start the desired PX4 target from the same shell.
If PX4 runs in Docker or on another computer, set `PX4_SIM_HOSTNAME` to the reachable IP address or hostname of the X-Plane machine instead.
Make sure inbound TCP port `4560` is allowed by the firewall on the X-Plane host.

## Running SITL

1. Start X-Plane and load the aircraft that matches the PX4 target.

2. Confirm the px4xplane plugin is loaded in the X-Plane plugin menu.

3. Start PX4 SITL. 예:

   ```sh
   cd PX4-Autopilot
   make px4_sitl_default xplane_alia250
   ```

4. Connect QGroundControl to PX4 as usual.

PX4 prints `Simulator connected on TCP port 4560` when the bridge is connected.

## Custom Aircraft

To add another X-Plane aircraft:

1. Create a PX4 airframe file in `ROMFS/px4fmu_common/init.d-posix/airframes`.
2. Add a matching SITL target in `src/modules/simulation/simulator_mavlink/sitl_targets_xplane.cmake`.
3. Configure the px4xplane bridge to map PX4 actuator outputs to the aircraft datarefs.
4. Tune the PX4 parameters from ULog data and X-Plane truth logs.

See the [px4xplane custom airframe guide](https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md) for bridge configuration details.
For X-Plane-side aircraft design and dataref mapping, see the official [Plane Maker manual](https://developer.x-plane.com/manuals/planemaker/) and [X-Plane datarefs reference](https://developer.x-plane.com/datarefs/).

## 문제 해결

If PX4 waits for the simulator, verify that X-Plane is running, the px4xplane plugin is loaded, `PX4_SIM_HOSTNAME` points to the X-Plane host, and TCP port `4560` is not blocked.

If the aircraft loads but does not respond correctly, verify that the X-Plane aircraft, PX4 target, and px4xplane configuration all refer to the same vehicle.

If estimator warnings appear, keep the simulator frame rate stable, check that the intended airframe parameters were loaded, and inspect the ULog sensor and estimator topics before changing bridge or airframe parameters.
