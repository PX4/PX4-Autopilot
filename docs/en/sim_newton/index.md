# Newton Simulation

**Supported Vehicles:** Quadrotor

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[Newton](https://developer.nvidia.com/newton-physics) is an open-source (Apache 2.0) physics engine written in Python.
It is developed by NVIDIA, Google DeepMind, and Disney Research, and managed by the Linux Foundation.
It leverages the NVIDIA Warp Python framework for accelerated simulation using JIT compilation of kernel functions for GPU or CPU.
It comes with modern features such as OpenUSD support and differentiable simulation.

PX4 runs decoupled from the simulator: the `make` target below builds and launches PX4 in SITL with **no in-process simulator**, and an external Newton-based simulator connects to it over the simulator MAVLink API on TCP port 4560.

## Supported Vehicles

The supported vehicles and `make` commands are listed below:

| Vehicle                   | Command                        |
| ------------------------- | ------------------------------ |
| Freefly Systems Astro Max | `make px4_sitl none_astro_max` |

## Integration

See the [Newton framework documentation](https://breuerpeter.github.io/newton/) for details on the Newton simulator integration.

## See Also

- See [Simulation](../simulation/index.md) for general information about simulators in PX4.
- 
