# SkySim Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.
See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[SkySim](https://github.com/vishwagw/Sky-Sim-drone-simulator-3.0.git) is an open-source multirotor simulator that can be used with PX4 for [Software-In-The-Loop (SITL)](../simulation/index.md#sitl-simulation-environment) simulation.
PX4 runs as the flight controller, while SkySim provides the vehicle dynamics and simulated sensor inputs.
SkySim uses the [Simulator MAVLink API](../simulation/index.md#simulator-mavlink-api) over TCP port `4560` with lockstep synchronization: SkySim listens on the port and PX4 connects to it.

SkySim runs natively and also entirely in the browser via WebAssembly, so an autonomy stack can be exercised against PX4 SITL without a local install.

## Why Use SkySim?

SkySim models multirotor flight with a blade-element rotor model, ground effect, vortex-ring state, an ISA atmosphere, Dryden turbulence, and a battery model.
This makes it useful when testing estimation, planning, or control against more detailed vehicle dynamics than a point-mass model.

SkySim also exposes a [Gymnasium](https://gymnasium.farama.org/) environment, so reinforcement-learning and other Python research pipelines can be connected to a PX4-controlled multirotor without rewriting them around PX4-specific APIs.

## Installation

Install SkySim from its [repository](https://github.com/vishwagw/Sky-Sim-drone-simulator-3.0.git) by following the quick-start there, and set up PX4 SITL using the [PX4 development environment](../dev_setup/dev_env.md).

## Supported Vehicles

The setup steps on this page use a quadcopter:

| Vehicle | Type        | PX4 target                        |
| ------- | ----------- | --------------------------------- |
| Quad    | Multicopter | `make px4_sitl_default none_iris` |

## Network Setup

PX4 connects to SkySim on TCP port `4560`.

When SkySim and PX4 SITL run on the same machine, the default host `localhost` is sufficient.

When SkySim runs on Windows and PX4 runs in WSL2, set `PX4_SIM_HOSTNAME` in the WSL2 shell to the Windows host IP before starting PX4:

```sh
export PX4_SIM_HOSTNAME=$(ip route | awk '/default/ {print $3; exit}')
```

Make sure inbound TCP port `4560` is allowed by the firewall on the SkySim host.

## Running SITL

1. Start SkySim and select the **PX4 (MAVLink)** bridge so it listens on TCP port `4560`.
2. Start PX4 SITL with lockstep enabled. For example:

   ```sh
   cd PX4-Autopilot
   PX4_LOCKSTEP=1 PX4_SIM_SPEED_FACTOR=1 make px4_sitl_default none_iris
   ```

   On Windows with PX4 in WSL2, include `PX4_SIM_HOSTNAME` as shown in [Network Setup](#network-setup).

3. Connect [QGroundControl](https://qgroundcontrol.com) to PX4 as usual.

PX4 prints `Simulator connected on TCP port 4560` when the bridge is connected.

## Troubleshooting

If PX4 waits for the simulator, verify that SkySim is running with the **PX4 (MAVLink)** bridge selected and listening on port `4560`, that `PX4_SIM_HOSTNAME` points to the SkySim host when using WSL2, and that TCP port `4560` is not blocked by a firewall.
