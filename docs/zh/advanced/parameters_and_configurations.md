# Parameters & Configurations

PX4 uses the _param subsystem_ (a flat table of `float` and `int32_t` values) and text files (for startup scripts) to store its configuration.

This section discusses the _param_ subsystem in detail.
它涵盖如何列出、保存和加载参数，以及如何定义这些参数并使这些参数在地面站上显示。

:::tip
[System startup](../concept/system_startup.md) and the way that [frame configuration](../dev_airframes/adding_a_new_frame.md) startup scripts work are detailed on other pages.
:::

## 命令行使用方法

The PX4 [system console](../debug/system_console.md) offers the [param](../modules/modules_command.md#param) tool, which can be used to set parameters, read their value, save them, and export and restore to/from files.

### 获取和设置参数

The `param show` command lists all system parameters:

```sh
param show
```

为了更有选择性，部分参数名称可以使用通配符 "\*" ：

```sh
nsh> param show RC_MAP_A*
Symbols: x = used, + = saved, * = unsaved
x   RC_MAP_AUX1 [359,498] : 0
x   RC_MAP_AUX2 [360,499] : 0
x   RC_MAP_AUX3 [361,500] : 0
x   RC_MAP_ACRO_SW [375,514] : 0

 723 parameters total, 532 used.
```

You can use the `-c` flag to show all parameters that have changed (from their defaults):

```sh
param show -c
```

You can use `param show-for-airframe` to show all parameters that have changed from their defaults for just the current airframe's definition file (and defaults it imports).

### 导出和加载参数

You can save any parameters that have been _changed_ (that are different from airframe defaults).

The standard `param save` command will store the parameters in the current default file:

```sh
param save
```

如果提供了参数，它会将参数存储到这个新位置:

```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to _load_ parameters:

- `param load` first does a full reset of all parameters to their defaults, and then overwrites parameter values with any values stored in the file.
- `param import` just overwrites parameter values with the values from the file and then saves the result (i.e. effectively calls `param save`).

The `load` effectively resets the parameters to the state when the parameters were saved (we say "effectively" because any parameters saved in the file will be updated, but other parameters may have different firmware-defined default values than when the parameters file was created).

By contrast, `import` merges the parameters in the file with the current state of the vehicle.
例如，这可以用来只导入包含校准数据的参数文件，而不覆盖系统配置的其余部分。

这两种情况的示例如下所示:

```sh
# 文件保存时重置参数
param load /fs/microsd/vtol_param_backup
# 选择性的保存参数 (不自动加载)
param save
```

```sh
# 将保存的参数与当前参数合并
param import /fs/microsd/vtol_param_backup
```

## 参数创建/定义

参数定义有两部分:

- [Parameter metadata](#parameter-metadata) specifies the default value for each parameter in firmware along with other metadata for presentation (and editing) of parameters in ground control stations and documentation.
- [C/C++ Code](#c-c-api) that provides access to get and/or subscribe to parameter values from within PX4 modules and drivers.

以下描述了编写元数据和代码的几种方法。
Where possible code should use newer [YAML metadata](#yaml-metadata) and [C++ API](#c-api) over the older C parameter/code definitions, as these are more flexible and robust.

Parameter metadata is [compiled into the firmware](#publishing-parameter-metadata-to-a-gcs),
and made available to ground stations via the [MAVLink Component Information service](https://mavlink.io/en/services/component_information.html).

### 参数名称

参数名称不得超过 16 个 ASCII 字符。

By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed-wing systems.
此惯例不强制执行。

The name must match in both code and [parameter metadata](#parameter-metadata) to correctly associate the parameter with its metadata (including default value in Firmware).

### C / C++ API

有单独的 C 和 C++ 的 API 可用于从 PX4 模块和驱动程序中访问参数值。

API 之间的一个重要区别是，C++ 版本具有更有效的标准化机制，可与参数值的更改（即来自 GCS 的更改）同步。

同步很重要，因为参数可能随时被更改为另一个值。
Your code should _always_ use the current value from the parameter store.
If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).

此外，C++ 版本有更好的类型安全和更小的 RAM 开销。
缺点是参数名称必须在编译时知道，而 C 语言 API 可以将动态创建的名称作为字符串。

#### C++ API

The C++ API provides macros to declare parameters as _class attributes_.
You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with _any_ parameter update.
框架代码然后 (在不可见的情况下) 处理追踪影响 uORB 消息，并保持参数属性和 uORB 消息同步。
在代码的其余部分中，您只能使用定义的参数属性，它们将始终是最新的！

首先在您的模块或驱动程序的类头文件中包含所需的头文件:

- **px4_platform_common/module_params.h** to get the `DEFINE_PARAMETERS` macro:

 ```cpp
 #include <px4_platform_common/module_params.h>
 ```

- **parameter_update.h** to access the uORB `parameter_update` message:

 ```cpp
 #include <uORB/topics/parameter_update.h>
 ```

- **Subscription.hpp** for the uORB C++ subscription API:

 ```cpp
 #include <uORB/Subscription.hpp>
 ```

Derive your class from `ModuleParams`, and use `DEFINE_PARAMETERS` to specify a list of parameters and their associated parameter attributes.
参数的名称必须与其参数元数据定义相同。

```cpp
class MyModule : ..., public ModuleParams
{
public:
	...

private:

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */
		(ParamFloat<px4::params::ATT_BIAS_MAX>) _att_bias_max  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
```

使用模板更新 CPP 文件，以检查与参数更新相关的 uORB 消息。

Call `parameters_update();` periodically in code to check if there has been an update:

```cpp
void Module::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}
```

在上述方法中：

- `_parameter_update_sub.updated()` tells us if there is _any_ update to the `param_update` uORB message (but not what parameter is affected).
- If there has been "some" parameter updated, we copy the update into a `parameter_update_s` (`param_update`), to clear the pending update.
- Then we call `ModuleParams::updateParams()`.
 This "under the hood" updates all parameter attributes listed in our `DEFINE_PARAMETERS` list.

The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.

:::tip
The [Application/Module Template](../modules/module_template.md) uses the new-style C++ API but does not include [parameter metadata](#parameter-metadata).
:::

#### C API

C API 可以在模块和驱动程序中使用。

首先包括参数 API 头文件:

```C
#include <parameters/param.h>
```

Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`.
The variable `my_param` can then be used in your module code.

```C
int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

:::info
If `PARAM_NAME` was declared in parameter metadata then its default value will be set, and the above call to find the parameter should always succeed.
:::

`param_find()` is an "expensive" operation, which returns a handle that can be used by `param_get()`.
If you're going to read the parameter multiple times, you may cache the handle and use it in `param_get()` when needed

```cpp
# Get the handle to the parameter
param_t my_param_handle = PARAM_INVALID;
my_param_handle = param_find("PARAM_NAME");

# Query the value of the parameter when needed
int32_t my_param = 0;
param_get(my_param_handle, &my_param);
```

### 参数元数据

PX4 使用广泛的参数元数据系统来驱动面向用户的参数表示，并在固件中设置的每个参数的默认值。

:::tip
Correct metadata is critical for good user experience in a ground station.
:::

Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible).
通常，它与关联的模块一起存储。

The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced_config/parameter_reference.md) and the parameter information [used by ground stations](#publishing-parameter-metadata-to-a-gcs).

:::warning
After adding a _new_ parameter file you should call `make clean` before building to generate the new parameters (parameter files are added as part of the _cmake_ configure step, which happens for clean builds and if a cmake file is modified).
:::

#### YAML 元数据

:::info
At time of writing YAML parameter definitions cannot be used in _libraries_.
:::

YAML meta data is intended as a full replacement for the **.c** definitions.
它支持所有相同的元数据，以及多实例定义等新功能。

- The YAML parameter metadata schema is here: [validation/module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml).
- An example of YAML definitions being used can be found in the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml).
- 通过添加到 cmake 构建系统中注册一个 YAML 文件

 ```cmake
 MODULE_CONFIG
 	module.yaml
 ```

 to the `px4_add_module` section of the `CMakeLists.txt` file of that module.

#### 多实例（模块化）YAML 元数据

Templated parameter definitions are supported in [YAML parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) (templated parameter code is not supported).

The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`.
例如，下面将生成 MY_PARAM_1_RATE、MY_PARAM_2_RATE 等。

```yaml
MY_PARAM_${i}_RATE:
  description:
    short: Maximum rate for instance ${i}
```

以下 YAML 定义提供起始和结束索引。

- `num_instances` (default 1): Number of instances to generate (>=1)
- `instance_start` (default 0): First instance number. If 0, `${i}` expands to [0, N-1]\`.

For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml)

#### c 参数元数据

The legacy approach for defining parameter metadata is in a file with extension **.c** (at time of writing this is the approach most commonly used in the source tree).

参数的元数据部分看起来像下面的例子:

```cpp
/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @reboot_required true
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

```cpp
/**
 * Acceleration compensation based on GPS
 * velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);
```

The `PARAM_DEFINE_*` macro at the end specifies the type of parameter (`PARAM_DEFINE_FLOAT` or `PARAM_DEFINE_INT32`), the name of the parameter (which must match the name used in code), and the default value in firmware.

注释块中的行都是可选的，主要用于控制地面站内的显示和编辑选项。
The purpose of each line is given below (for more detail see [module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml)).

```cpp
/**
 * <title>
 *
 * <longer description, can be multi-line>
 *
 * @unit <the unit, e.g. m for meters>
 * @min <the minimum sane value. Can be overridden by the user>
 * @max <the maximum sane value. Can be overridden by the user>
 * @decimal <the minimum sane value. Can be overridden by the user>
 * @increment <the "ticks" in which this value will increment in the UI>
 * @reboot_required true <add this if changing the param requires a system restart.>
 * @boolean <add this for integer parameters that represent a boolean value>
 * @group <a title for parameters that form a group>
 */
```

## 发布参数的元数据到地面站

The parameter metadata JSON file is compiled into firmware (or hosted on the Internet), and made available to ground stations via the [MAVLink Component Metadata service](https://mavlink.io/en/services/component_information.html).
这确保了元数据始终与载具上运行的代码保持最新。

This process is the same as for [events metadata](../concept/events_interface.md#publishing-event-metadata-to-a-gcs).
For more information see [PX4 Metadata (Translation & Publication)](../advanced/px4_metadata.md)

## 更多信息

- [Finding/Updating Parameters](../advanced_config/parameters.md)
- [Parameter Reference](../advanced_config/parameter_reference.md)
- [Param implementation](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/param.h#L129) (information on `.get()`, `.commit()`, and other methods)
