# PX4 开发

本章节讲述了如何支持新的无人机/无人车类型及变种，如何修改飞行算法，如何添加新的飞行模式，如何集成新的硬件，如何通过外部飞控和PX4通信。

:::tip
This section is for software developers and (new) hardware integrators.
如果要构建现有的机身或者PX4已有的，可以跳过此章节。
:::

它解释了如何：

- Get a [minimum developer setup](../dev_setup/config_initial.md), [build PX4 from source](../dev_setup/building_px4.md) and deploy on [numerous supported autopilots](../flight_controller/index.md).
- 理解[PX4系统架构](../concept/architecture.md)及其他核心概念。
- 学习如何更改飞行栈及中间层：
  - 修改飞行算法并添加新的 [飞行模式](../concept/flight_modes.md)。
  - 支持新的 [airframes](../dev_airframes/index.md)。
- 学习如何将PX4集成到新的硬件上：
  - 支持新的传感器和执行器, 包括摄像头、测距仪等。
  - 修改PX4使之能够在新的自驾仪硬件上运行。
- [Simulate](../simulation/index.md), [test](../test_and_ci/index.md) and [debug/log](../debug/index.md) PX4.
- 与外部机器人的 API 进行联调通信/集成。

## 开发者可用的关键链接

- [Support](../contribute/support.md): Get help using the [discussion boards](https://discuss.px4.io//) and other support channels.
- [Weekly Dev Call](../contribute/dev_call.md): A great opportunity to meet the PX4 dev team and discuss platform technical details (including pull requests, major issues, general Q&A).
- [Licences](../contribute/licenses.md): What you can do with the code (free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)!)
- [Contributing](../contribute/index.md): How to work with our [source code](../contribute/code.md).
