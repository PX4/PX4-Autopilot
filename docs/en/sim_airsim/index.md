# AirSim Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[AirSim](https://microsoft.github.io/AirSim/) is a open-source, cross platform simulator for drones, built on _Unreal Engine_.
It provides physically and visually realistic simulations of Pixhawk/PX4 using either Hardware-In-The-Loop (HITL) or Software-In-The-Loop (SITL).

<lite-youtube videoid="-WfTr1-OBGQ" title="AirSim Demo"/>

<!-- datestamp:video:youtube:20170216:AirSim Demo -->

## PX4 Setup

[PX4 Setup for AirSim](https://microsoft.github.io/AirSim/px4_setup/) describes how to use PX4 with AirSim using both [SITL](https://microsoft.github.io/AirSim/px4_sitl/) and [HITL](https://microsoft.github.io/AirSim/px4_setup/#setting-up-px4-hardware-in-loop).

## Videos

#### AirSim with PX4 on WSL 2

<lite-youtube videoid="DiqgsWIOoW4" title="AirSim with PX4 on WSL 2"/>

<!-- datestamp:video:youtube:20210401:AirSim with PX4 on WSL 2 -->

::: info
WSL 2 is not a supported [PX4 Windows development environment](../dev_setup/dev_env_windows_cygwin.md), mainly because it is non-trivial to display simulator UIs running within WSL 2 in the normal Windows environment.
This limitation does not apply for AirSim because its UI is run natively in Windows.
:::

#### Microsoft AirSim: Applications to Research and Industry (PX4 Developer Summit Virtual 2020)

<lite-youtube videoid="-YMiKaJYl44" title="Microsoft AirSim: Applications to Research and Industry"/>

<!-- datestamp:video:youtube:20200716:Microsoft AirSim: Applications to Research and Industry — PX4 Developer Summit Virtual 2020 -->

#### Autonomous Drone Inspections using AirSim and PX4 (PX4 Developer Summit Virtual 2020)

<lite-youtube videoid="JDx0MPTlhrg" title="Autonomous Drone Inspections using AirSim and PX4"/>

<!-- datestamp:video:youtube:20200716:Autonomous Drone Inspections using AirSim and PX4 — PX4 Developer Summit Virtual 2020 -->

## Further Information

- [AirSim Documentation](https://microsoft.github.io/AirSim/)
- [Using AirSim to Simulate Aircraft Inspection by Autonomous Drones](https://gaas.gitbook.io/guide/case-study/using-airsim-to-simulate-aircraft-inspection-by-autonomous-drones) (Case Study from Generalized Autonomy Aviation System (GAAS) project).
