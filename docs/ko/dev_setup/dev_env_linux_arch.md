# Arch Linux 개발 환경

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

The PX4-Autopilot repository provides a convenient script to set your Arch installation up for PX4 development: [Tools/setup/arch.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/arch.sh). <!-- NEED px4_version -->

The script installs (by default) all tools to build PX4 for NuttX targets and run simulation with [JMAVSim](../sim_jmavsim/index.md).
You can additionally install the [Gazebo Classic](../sim_gazebo_classic/index.md) simulator by specifying the command line argument: `--gazebo`.

![Gazebo on Arch](../../assets/simulation/gazebo_classic/arch-gazebo.png)

:::info
The instructions have been tested on [Manjaro](https://manjaro.org/) (Arch based distribution) as it is much easier to set up than Arch Linux.
:::

스크립트를 가져와 실행하려면 다음 중 하나를 실행합니다.

- [Download PX4 Source Code](../dev_setup/building_px4.md) and run the scripts in place:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  bash PX4-Autopilot/Tools/setup/arch.sh
  ```

- 필요한 스크립트만 다운로드하여 실행합니다.

  ```sh
  wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/arch.sh
  wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt
  bash arch.sh
  ```

스크립트는 다음의 매개변수를 사용합니다.

- `--gazebo`: Add this parameter to install Gazebo from the [AUR](https://aur.archlinux.org/packages/gazebo/).

  ::: info
  Gazebo gets compiled from source.
  It takes some time to install and requires entering the `sudo` password multiple times (for dependencies).

:::

- `--no-nuttx`: Do not install the NuttX/Pixhawk toolchain (i.e. if only using simulation).

- `--no-sim-tools`: Do not install jMAVSim/Gazebo (i.e. if only targeting Pixhawk/NuttX targets)
