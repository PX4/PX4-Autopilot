# PteroSim Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4 and may be removed in future releases.
See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[PteroSim](https://github.com/PteroLabsAI/PteroSim-UAV-Simulator) is proprietary simulation software that can be used with PX4 for [Software-In-The-Loop (SITL)](../simulation/index.md#sitl-simulation-environment) simulation.
PX4 runs as the flight controller, while PteroSim provides the vehicle dynamics, visual scene, and simulated sensor inputs through a built-in HIL bridge.
PteroSim uses the [Simulator MAVLink API](../simulation/index.md#simulator-mavlink-api) over TCP port `4560` with lockstep synchronization.

PteroSim runs on **Windows 10/11** and **Ubuntu 22.04+** (64-bit).
Install PteroSim from [GitHub Releases](https://github.com/PteroLabsAI/PteroSim-UAV-Simulator/releases) and set up PX4 SITL using the [PX4 development environment](../dev_setup/dev_env.md).
For full installation, UI walkthrough, and QGroundControl setup, see the [PteroSim PX4 SITL guide](https://pterosimdocs.readthedocs.io/en/latest/sitl_simulation_px4.html).

## Videos

PX4 tutorial:

<lite-youtube videoid="gUSc84v3f44" title="PteroSim PX4 tutorial"/>

PteroSim demo:

<lite-youtube videoid="4QMwmZL_3O4" title="PteroSim PX4 demo"/>

## Supported Vehicles

The free tier includes the **F450** multicopter, which is covered by the setup steps on this page:

| Vehicle | Type        | PX4 target                        |
| ------- | ----------- | --------------------------------- |
| F450    | Multicopter | `make px4_sitl_default none_iris` |

Paid licenses also include fixed-wing, VTOL, and helicopter templates with PX4 SITL support.
See [pterolabs.ai](https://pterolabs.ai/#pricing) for licensing details.

## Network Setup

PX4 connects to PteroSim on TCP port `4560`.

When PteroSim and PX4 SITL run on the same Ubuntu machine, the default host `localhost` is sufficient.

When PteroSim runs on Windows and PX4 runs in WSL2, set `PX4_SIM_HOSTNAME` in the WSL2 shell to the Windows host IP before starting PX4:

```sh
export PX4_SIM_HOSTNAME=$(ip route | awk '/default/ {print $3; exit}')
```

Make sure inbound TCP port `4560` is allowed by the firewall on the PteroSim host.

## Running SITL

1. Start PteroSim, spawn the F450, select **PX4** as the control source, and press **Start**.
2. Start PX4 SITL with lockstep enabled. For example:

   ```sh
   cd PX4-Autopilot
   PX4_LOCKSTEP=1 PX4_SIM_SPEED_FACTOR=1 make px4_sitl_default none_iris
   ```

   On Windows with PX4 in WSL2, include `PX4_SIM_HOSTNAME` as shown in [Network Setup](#network-setup).

3. Connect [QGroundControl](https://qgroundcontrol.com) to PX4 as usual.

PX4 prints `Simulator connected on TCP port 4560` when the bridge is connected.

## Troubleshooting

If PX4 waits for the simulator, verify that PteroSim is running, the simulation has been started, the control source is set to **PX4**, `PX4_SIM_HOSTNAME` points to the PteroSim host when using WSL2, and TCP port `4560` is not blocked.

If PX4 connects but the aircraft does not respond to commands, verify that the control source is set to **PX4** in PteroSim.
