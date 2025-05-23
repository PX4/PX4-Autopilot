# Parameters & Configurations

PX4 uses the _param subsystem_ (a flat table of `float` and `int32_t` values) and text files (for startup scripts) to store its configuration.

This section discusses the _param_ subsystem in detail.
It covers how to list, save and load parameters, and how to define them and make them available to ground stations.

:::tip
[System startup](../concept/system_startup.md) and the way that [frame configuration](../dev_airframes/adding_a_new_frame.md) startup scripts work are detailed on other pages.
:::

## Command Line Usage

The PX4 [system console](../debug/system_console.md) offers the [param](../modules/modules_command.md#param) tool, which can be used to set parameters, read their value, save them, and export and restore to/from files.

### Getting and Setting Parameters

The `param show` command lists all system parameters:

```sh
param show
```

To be more selective, a partial parameter name with wildcard "\*" can be used:

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

### Exporting and Loading Parameters

You can save any parameters that have been _changed_ (that are different from airframe defaults).

The standard `param save` command will store the parameters in the current default file:

```sh
param save
```

If provided with an argument, it will store the parameters instead to this new location:

```sh
param save /fs/microsd/vtol_param_backup
```

There are two different commands to _load_ parameters:

- `param load` first does a full reset of all parameters to their defaults, and then overwrites parameter values with any values stored in the file.
- `param import` just overwrites parameter values with the values from the file and then saves the result (i.e. effectively calls `param save`).

The `load` effectively resets the parameters to the state when the parameters were saved (we say "effectively" because any parameters saved in the file will be updated, but other parameters may have different firmware-defined default values than when the parameters file was created).

By contrast, `import` merges the parameters in the file with the current state of the vehicle.
This can be used, for example, to just import a parameter file containing calibration data, without overwriting the rest of the system configuration.

Examples for both cases are shown below:

```sh
# Reset the parameters to when file was saved
param load /fs/microsd/vtol_param_backup
# Optionally save params (not done automatically with load)
param save
```

```sh
# Merge the saved parameters with current parameters
param import /fs/microsd/vtol_param_backup
```

## Creating/Defining Parameters

Parameters definitions have two parts:

- [Parameter metadata](#parameter-metadata) specifies the default value for each parameter in firmware along with other metadata for presentation (and editing) of parameters in ground control stations and documentation.
- [C/C++ Code](#c-c-api) that provides access to get and/or subscribe to parameter values from within PX4 modules and drivers.

Several approaches are described below for writing both the metadata and code.
Where possible code should use newer [YAML metadata](#yaml-metadata) and [C++ API](#c-api) over the older C parameter/code definitions, as these are more flexible and robust.

Parameter metadata is [compiled into the firmware](#publishing-parameter-metadata-to-a-gcs),
and made available to ground stations via the [MAVLink Component Information service](https://mavlink.io/en/services/component_information.html).

### Parameter Names

Parameter names must be no more than 16 ASCII characters.

By convention, every parameter in a group should share the same (meaningful) string prefix followed by an underscore, and `MC_` and `FW_` are used for parameters related specifically to Multicopter or Fixed-wing systems.
This convention is not enforced.

The name must match in both code and [parameter metadata](#parameter-metadata) to correctly associate the parameter with its metadata (including default value in Firmware).

### C / C++ API

There are separate C and C++ APIs that can be used to access parameter values from within PX4 modules and drivers.

One important difference between the APIs is that the C++ version has a more efficient standardized mechanism to synchronize with changes to parameter values (i.e. from a GCS).

Synchronization is important because a parameter can be changed to another value at any time.
Your code should _always_ use the current value from the parameter store.
If getting the latest version is not possible, then a reboot will be required after the parameter is changed (set this requirement using the `@reboot_required` metadata).

In addition, the C++ version has also better type-safety and less overhead in terms of RAM.
The drawback is that the parameter name must be known at compile-time, while the C API can take a dynamically created name as a string.

#### C++ API

The C++ API provides macros to declare parameters as _class attributes_.
You add some "boilerplate" code to regularly listen for changes in the [uORB Topic](../middleware/uorb.md) associated with _any_ parameter update.
Framework code then (invisibly) handles tracking uORB messages that affect your parameter attributes and keeping them in sync.
In the rest of the code you can just use the defined parameter attributes and they will always be up to date!

First include the required needed headers in the class header for your module or driver:

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
The names of the parameters must be the same as their parameter metadata definitions.

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

Update the cpp file with boilerplate to check for the uORB message related to parameter updates.

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

In the above method:

- `_parameter_update_sub.updated()` tells us if there is _any_ update to the `param_update` uORB message (but not what parameter is affected).
- If there has been "some" parameter updated, we copy the update into a `parameter_update_s` (`param_update`), to clear the pending update.
- Then we call `ModuleParams::updateParams()`.
  This "under the hood" updates all parameter attributes listed in our `DEFINE_PARAMETERS` list.

The parameter attributes (`_sys_autostart` and `_att_bias_max` in this case) can then be used to represent the parameters, and will be updated whenever the parameter value changes.

:::tip
The [Application/Module Template](../modules/module_template.md) uses the new-style C++ API but does not include [parameter metadata](#parameter-metadata).
:::

#### C API

The C API can be used within both modules and drivers.

First include the parameter API:

```C
#include <parameters/param.h>
```

Then retrieve the parameter and assign it to a variable (here `my_param`), as shown below for `PARAM_NAME`.
The variable `my_param` can then be used in your module code.

```C
int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

::: info
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

### Parameter Metadata

PX4 uses an extensive parameter metadata system to drive the user-facing presentation of parameters, and to set the default value for each parameter in firmware.

:::tip
Correct metadata is critical for good user experience in a ground station.
:::

Parameter metadata can be stored anywhere in the source tree as either **.c** or **.yaml** parameter definitions (the YAML definition is newer, and more flexible).
Typically it is stored alongside its associated module.

The build system extracts the metadata (using `make parameters_metadata`) to build the [parameter reference](../advanced_config/parameter_reference.md) and the parameter information [used by ground stations](#publishing-parameter-metadata-to-a-gcs).

:::warning
After adding a _new_ parameter file you should call `make clean` before building to generate the new parameters (parameter files are added as part of the _cmake_ configure step, which happens for clean builds and if a cmake file is modified).
:::

#### YAML Metadata

::: info
At time of writing YAML parameter definitions cannot be used in _libraries_.
:::

YAML meta data is intended as a full replacement for the **.c** definitions.
It supports all the same metadata, along with new features like multi-instance definitions.

- The YAML parameter metadata schema is here: [validation/module_schema.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml).
- An example of YAML definitions being used can be found in the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml).
- A YAML file is registered in the cmake build system by adding

  ```cmake
  MODULE_CONFIG
  	module.yaml
  ```

  to the `px4_add_module` section of the `CMakeLists.txt` file of that module.

#### Multi-Instance (Templated) YAML Meta Data

Templated parameter definitions are supported in [YAML parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) (templated parameter code is not supported).

The YAML allows you to define instance numbers in parameter names, descriptions, etc. using `${i}`.
For example, below will generate MY_PARAM_1_RATE, MY_PARAM_2_RATE etc.

```yaml
MY_PARAM_${i}_RATE:
  description:
    short: Maximum rate for instance ${i}
```

The following YAML definitions provide the start and end indexes.

- `num_instances` (default 1): Number of instances to generate (>=1)
- `instance_start` (default 0): First instance number. If 0, `${i}` expands to [0, N-1]`.

For a full example see the MAVLink parameter definitions: [/src/modules/mavlink/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/module.yaml)

#### c Parameter Metadata

The legacy approach for defining parameter metadata is in a file with extension **.c** (at time of writing this is the approach most commonly used in the source tree).

Parameter metadata sections look like the following examples:

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

The lines in the comment block are all optional, and are primarily used to control display and editing options within a ground station.
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

## Publishing Parameter Metadata to a GCS

The parameter metadata JSON file is compiled into firmware (or hosted on the Internet), and made available to ground stations via the [MAVLink Component Metadata service](https://mavlink.io/en/services/component_information.html).
This ensures that metadata is always up-to-date with the code running on the vehicle.

This process is the same as for [events metadata](../concept/events_interface.md#publishing-event-metadata-to-a-gcs).
For more information see [PX4 Metadata (Translation & Publication)](../advanced/px4_metadata.md)

## Further Information

- [Finding/Updating Parameters](../advanced_config/parameters.md)
- [Parameter Reference](../advanced_config/parameter_reference.md)
- [Param implementation](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/param.h#L129) (information on `.get()`, `.commit()`, and other methods)
