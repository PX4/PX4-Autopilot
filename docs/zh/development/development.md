# PX4 开发

本节解释如何支持新型载具及其变体，修改飞行算法。 添加新模式，整合新的硬件，并与飞行控制器外的 PX4 通信。

:::tip
本节针对软件开发者和(新)硬件集成器。
如果要构建现有的机身或者PX4已有的，可以跳过此章节。
:::

它解释了如何：

- 获取一个[最小的开发者设置](../dev_setup/config_initial.md), [从源代码生成PX4](../dev_setup/building_px4.md) 并部署在[众多支持的自动化设备](../flight_controller/index.md)。
- 理解[PX4系统架构](../concept/architecture.md)及其他核心概念。
- 学习如何更改飞行栈及中间层：
  - 修改飞行算法并添加新的 [飞行模式](../concept/flight_modes.md)。
  - 支持新的 [airframes](../dev_airframes/index.md)。
- 学习如何将PX4集成到新的硬件上：
  - 支持新的传感器和执行器, 包括摄像头、测距仪等。
  - 修改PX4使之能够在新的自驾仪硬件上运行。
- 获取一个[最小的开发者设置](../dev_setup/config_initial.md), [从源代码生成PX4](../dev_setup/building_px4.md) 并部署在[众多支持的自动化设备](../flight_controller/index.md)。
- 与外部机器人的 API 进行联调通信/集成。

## 开发者可用的关键链接

- [支持](../contribute/support.md)：使用 [讨论板](https://discuss.px4.io//) 和其他支持渠道获得帮助。
- [每周开发者电话会议](../contribute/dev_call.md)：这是一个很好的机会来会见 PX4 开发团队，讨论平台技术细节(包括pull requests， 主要问题，一般性问答）。
- [Licences](../contribute/licenses.md): What you can do with the code (free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/license/BSD-3-Clause)!)
- [贡献](../contribute/index.md): 如何使用我们的 [源代码](../contribute/code.md)。
