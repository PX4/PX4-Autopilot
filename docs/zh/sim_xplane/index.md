# X-Plane 仿真

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4 and may be removed in future releases.
See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

[X-Plane] (https://www.x-plane.com/) 可通过社区支持的 [px4xplane] (https://github.com/alireza787b/px4xplane)桥接插件与 PX4 配合，用于软件在环 (SITL) 仿真(../simulation/index.md#sitl-simulation-environment)。
PX4 作为飞行控制器运行，而 X-Plane 提供机体动力学、可视场景和模拟传感器输入。
Px4xplane 桥接器是一个 X-Plane 插件：它在 X-Plane 内部运行，接受 PX4 的模拟 MAVLink 连接，将模拟传感器数据发布至 PX4，并将 PX4 执行器输出写入 X-Plane dataref。

## 支持的载具

PX4 为以下机体提供 X-Plane SITL 机架配置：

| Vehicle        | 类型              | PX4 构建目标                                   |
| -------------- | --------------- | ------------------------------------------ |
| Cessna 172     | Plane           | `make px4_sitl_default xplane_cessna172`   |
| TB2            | Plane           | `make px4_sitl_default xplane_tb2`         |
| Ehang 184      | 多旋翼             | `make px4_sitl_default xplane_ehang184`    |
| Alia-250       | VTOL quadplane  | `make px4_sitl_default xplane_alia250`     |
| QuadTailsitter | VTOL tailsitter | `make px4_sitl_default xplane_qtailsitter` |

匹配的 X-Plane 飞机文件和桥接配置由 px4xplane 项目分发。
如果 PX4 具备支持 SITL 的相应控制路径，并且 px4xplane 能够将所需的执行器输出映射到可写的 X-Plane dataref，也可以集成其他 X-Plane 飞机或机体模型。

## 安装

1. 在将运行 PX4 SITL 的计算机、容器或 WSL2 环境中安装 [PX4 开发环境](../dev_setup/dev_env.md)。

2. 安装 X-Plane 11 或 X-Plane 12。

3. 从 [px4xplane 发布页面](https://github.com/alireza787b/px4xplane/releases)下载适用于当前操作系统的最新 px4xplane 发布包。

4. 该发布包是压缩归档文件，而不是安装程序。
   解压发布包，并将完整的 px4xplane 插件文件夹复制到：

   ```sh
   X-Plane/Resources/plugins/px4xplane
   ```

5. 将同一解压包中匹配的 X-Plane 飞机文件夹复制到 X-Plane 的飞机目录中。
   除非发布说明另有要求，否则应将发布包中的 64/config.ini 与 px4xplane 插件放在一起。

[px4xplane 构建和安装指南](https://github.com/alireza787b/px4xplane/blob/master/docs/BUILD.md#installation)提供了有关插件文件夹结构的更多详细信息。

## 网络设置

PX4 通过 TCP 端口 4560 连接到 X-Plane 桥接器。
默认主机为 localhost，适用于 PX4 和 X-Plane 在同一台 Linux 或 macOS 计算机上运行的情况。

在 Windows 上，通常是在 Windows 中原生运行 X-Plane，并在 WSL2 中运行 PX4。
在这种情况下，启动 PX4 前，应在 WSL2 shell 中将 PX4_SIM_HOSTNAME 设置为 Windows 主机的 IP 地址。
该变量在 PX4/SITL 端设置，并指向运行 X-Plane 和 px4xplane 插件的主机：

```sh
export PX4_SIM_HOSTNAME=$(ip route | awk '/default/ {print $3; exit}')
```

然后从同一 shell 启动所需的 PX4 构建目标。
如果 PX4 在 Docker 或另一台计算机上运行，则应将 PX4_SIM_HOSTNAME 设置为 X-Plane 计算机可访问的 IP 地址或主机名。
确保 X-Plane 主机的防火墙允许 TCP 端口 4560 的入站连接。

## Running SITL

1. 启动 X-Plane，并加载与 PX4 构建目标匹配的飞机。

2. 确认 X-Plane 插件菜单中已加载 px4xplane 插件。

3. 启动 PX4 SITL。 例如：

   ```sh
   cd PX4-Autopilot
   make px4_sitl_default xplane_alia250
   ```

4. 像往常一样将 QGroundControl 连接到 PX4。

桥接连接成功时，PX4 会输出 Simulator connected on TCP port 4560。

## 自定义飞行器

如需添加其他 X-Plane 飞机：

1. 在 ROMFS/px4fmu_common/init.d-posix/airframes 中创建 PX4 机架配置文件。
2. 在 src/modules/simulation/simulator_mavlink/sitl_targets_xplane.cmake 中添加匹配的 SITL 构建目标。
3. 配置 px4xplane 桥接器，将 PX4 执行器输出映射到该飞机的 dataref。
4. 根据 ULog 数据和 X-Plane 真值日志调优 PX4 参数。

有关桥接器配置的详细信息，请参阅 px4xplane 自定义机架指南(https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md)。
有关 X-Plane 端的飞机设计和 dataref 映射，请参阅官方 Plane Maker 手册(https://developer.x-plane.com/manuals/planemaker/)和 X-Plane dataref 参考文档(https://developer.x-plane.com/datarefs/)。

## 故障处理

如果 PX4 一直等待仿真器，请确认 X-Plane 正在运行、px4xplane 插件已加载、PX4_SIM_HOSTNAME 指向 X-Plane 主机，并且 TCP 端口 4560 未被阻止。

如果飞机能够加载但无法正确响应，请确认 X-Plane 飞机、PX4 构建目标和 px4xplane 配置均对应同一机体。

如果出现估计器警告，请保持仿真器帧率稳定，确认已加载预期的机架参数，并在修改桥接器参数或机架参数之前检查 ULog 中的传感器和估计器主题。
