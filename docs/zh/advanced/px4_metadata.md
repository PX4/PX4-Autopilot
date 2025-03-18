# PX4 Metadata

PX4 使用并生成人类和机器可读的元数据:

- [参数](../advanced_config/parameters.md) 配置 PX4 的行为。
  - 一个参数由一个 ID 字符串表示，并映射到 PX4 中存储的一个值。
  - 相关的元数据包含对设置的描述，可能的取值，有关值如何呈现的信息 (比如位掩码)。
- [Events](../concept/events_interface.md) 提供事件通知，例如失效保护的原因，电池低电量告警，校准结束等等。
  - 事件是由一个 id 表示，并且发送带有日志级别和一些参数。
  - 相关的元数据包括每个事件的消息、描述和参数列表 (包括它们的类型)。
- [Actuators](../config/actuators.md) 配置自定义飞行器的特定几何形状，将执行器和电机绑定到飞控的输出，并测试执行器和电机响应。
  - 元数据包含所支持的飞行器几何形状信息，驱动输出列表，以及如何配置它们。
  - _QGroundControl_  地面站使用该信息动态构建配置界面。

元数据和元数据翻译与外部系统共享，例如 QGroundControl 地面站，使其能够显示有关参数和事件的信息，并配置飞行器几何形状和执行器输出映射。

本节解释了您如何定义元数据并翻译成字符串 (以及"仅供您参考"，它是如何工作的)。

## 元数据翻译

在 Crowdin 项目 [PX4-Metadata-Translations](https://crowdin.com/project/px4-metadata-translations) 中翻译 PX4 元数据。
更多关于 PX4 和 Crowdin 信息请参阅[Translation](../contribute/translation.md)。

## 元数据定义

PX4 元数据是在 PX4 源代码及其相关数据中定义的。
这可以通过在 C/C++ 注释中使用特殊标记来指示元数据字段及其值，或者使用 YAML 文件来完成。

更多信息请参阅每种数据类型的章节：

- [参数与配置> 创建/定义参数](../advanced/parameters_and_configurations.md#creating-defining-parameters)
- [事件接口](../concept/events_interface.md)
- [执行器元数据](#actuator-metadata)（下文）

## 元数据工具链

处理元数据的过程对所有元数据类型都是相同的。

每次构建 PX4 时，元数据都会被收集到 JSON 文件中。

对于大多数飞控（因为大多数都有足够的可用 FLASH 存储空间），JSON文件经过 xz 压缩并存储在生成的二进制文件中。
然后使用 MAVLink [Component Metadata Protocol](https://mavlink.io/en/services/component_information.html)共享该文件。
使用组件元数据协议确保接收者总是能够为运行在载具的代码获取最新元数据。
事件元数据也会被添加到日志文件中，允许日志分析工具 (如飞行回放) 使用正确的元数据来显示事件。

内存受限的飞控二进制文件不会在二进制文件中存储参数元数据，而是引用存储在`px4-travis.s3.amazonaws.com`上的相同数据。
例如，这适用于[Umnibus F4 SD](../flight_controller/omnibus_f4_sd.md)。
元数据是通过 [github CI](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/metadata.yml) 上传的，用于所有构建目标（因此只有在参数被合并到主体后才能使用）。

:::info
You can identify memory constrained boards because they specify `CONFIG_BOARD_CONSTRAINED_FLASH=y` in their [px4board definition file](https://github.com/PX4/PX4-Autopilot/blob/main/boards/omnibus/f4sd/default.px4board).

如果在 FLASH 受限板上进行自定义开发，您可以调整此处的 URL 以指向另一台服务器[here](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/component_information/CMakeLists.txt#L41)。
:::

如果载具上没有显示参数元数据，则使用 `px4-travis.s3.amazonaws.com` 上的元数据。
它也可以用作后退，以避免对低速遥测链接进行非常缓慢的下载。

用于 CI 构建  `main` 分支的元数据 JSON 文件也拷贝到了 github 仓库：[PX4/PX4-Metadata-Translations](https://github.com/PX4/PX4-Metadata-Translations/)。
这与Crowdin集成，用于获取翻译，这些翻译存储在 [translated](https://github.com/PX4/PX4-Metadata-Translations/tree/main/translated) 文件夹中，每种语言都有 xz 压缩的翻译文件。
这些是由载具的组件元数据引用的，并在需要时下载。
有关更多信息，请参阅 [PX4-Metadata-Translations](https://github.com/PX4/PX4-Metadata-Translations/) 和 [Component Metadata Protocol > Translation](https://mavlink.io/en/services/component_information.html#translation)。

:::info
主分支的参数 XML 文件通过 CI 复制到 QGC 源代码树中，并在没有通过组件元数据协议获取到元数据的情况下用作后备方案 (该方法早于组件元数据协议)。
:::

### 元数据执行器

下面的图表显示了执行器元数据是如何从源代码中集成和如何被 QGroundControl 使用的：

![执行器元数据](../../assets/diagrams/actuator_metadata_processing.svg)

<!-- Source: https://docs.google.com/drawings/d/1hMQmIijdFjr21rREcXj50qz0C1b47JW0OEa6p5P231k/edit -->

- **Left**: 元数据在不同模快的 `module.yml` 文件中定义。
  `control_allocator` 模块定义几何形状，而每个输出驱动程序则定义其通道集和配置参数。
  [schema file](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) 描写了这些 yaml 文件的结构。
- **Middle**: 在构建时间, 当前构建目标的所有已启用模块的 `module.yml` 文件都会被解析并变成一个驱动程序。 使用 [Tools/module_config/generate_actuators_metadata.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/module_config/generate_actuators_metadata.py)脚本的文件。
  这里还有[schema file](https://github.com/mavlink/mavlink/blob/master/component_metadata/actuators.schema.json)。
- **Right**: 运行时，QGroundControl 通过 MAVLink 组件元数据 API 请求 JSON  文件(上文已经描述)。

## 更多信息

- [参数和配置](../advanced/parameters_and_configurations.md)
- [事件接口](../concept/events_interface.md)
- [翻译](../contribute/translation.md)
- [组件元数据协议](https://mavlink.io/en/services/component_information.html) (mavlink.io)
- [PX4 元数据翻译](https://github.com/PX4/PX4-Metadata-Translations/) (Github)
