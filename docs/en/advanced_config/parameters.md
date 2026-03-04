# Finding/Updating Parameters

PX4 behaviour can be configured/tuned using [parameters](../advanced_config/parameter_reference.md) (e.g. [Multicopter PID gains](../config_mc/pid_tuning_guide_multicopter.md), calibration information, etc.).

The _QGroundControl Parameters_ screen allows you to find and modify **any** of the parameters associated with the vehicle.
The screen is accessed by clicking the **Q** application icon > **Vehicle Setup**, and then _Parameters_ in the sidebar.

::: info
Most of the more commonly used parameters are more conveniently set using the dedicated setup screens, as described in the [Standard Configuration](../config/index.md) section.
The _Parameters_ screen is needed when modifying less commonly modified parameters - for example while tuning a new vehicle.
:::

:::warning
While some parameters can be changed in flight, this is not recommended (except where explicitly stated in the guide).
:::

<a id="finding"></a>

## Finding a Parameter

You can search for a parameter by entering a term in the _Search_ field.
This will show you a list of all parameter names and descriptions that contain the entered substring (press **Clear** to reset the search, and use the **Show modified only** checkbox to filter out unchanged parameters).

![Parameters Search](../../assets/qgc/setup/parameters/parameters_search.png)

You can also browse the parameters by type and group by clicking on the buttons to the left (in the image below the _DShot_ group in the _Standard_ parameters is selected).

![Parameters Screen](../../assets/qgc/setup/parameters/parameters_px4.png)

You can expand/collapse the "type" groupings as shown.
Note that the groups at the bottom named _Component X_ are attached [DroneCAN peripherals](../dronecan/index.md#qgc-cannode-parameter-configuration) ("X" is the node id).
[QGC can set the parameters](../dronecan/index.md#qgc-cannode-parameter-configuration) of these peripherals if they are attached to the Flight Controller when QGC is started.

![Parameters Types - collapsed](../../assets/qgc/setup/parameters/parameters_types.png)

:::tip
If you can't find an expected parameter, see the [next section](#missing).
:::

<a id="missing"></a>

## Missing Parameters

Parameters are usually not visible because either they are conditional on other parameters, or they are not present in the firmware (see below).

### Conditional Parameters

A parameter may not be displayed if it is conditional on another parameter that is not enabled.

You can usually find out what parameters are conditional by searching the [full parameter reference](../advanced_config/parameter_reference.md) and other documentation.
In particular [serial port configuration parameters](../peripherals/serial_configuration.md) depend on what service is assigned to a serial port.

### Parameter Not In Firmware

A parameter may not be present in the firmware because you're using a different version of PX4 or because you're using a build in which the associated module is not included.

New parameters are added in each PX4 version, and existing parameters are sometimes removed or renamed.
You can check whether a parameter _should_ be present by reviewing the [full parameter reference](../advanced_config/parameter_reference.md) for the version you're targeting.
You can also search for the parameter in the source tree and in the release notes.

The other reason that a parameter might not be in firmware is if its associated module has not been included.
This is a problem (in particular) for _FMUv2 firmware_, which omits many modules so that PX4 can fit into the 1MB of available flash.
There are two options to solve this problem:

- Check if you can update your board to run FMUv3 firmware, which includes all modules: [Firmware > FMUv2 Bootloader Update](../config/firmware.md#bootloader)
- If your board can only run FMUv2 firmware you will need to [rebuild PX4](../dev_setup/building_px4.md) with the missing modules enabled.
  You need reconfigure the PX4 firmware itself through make px4_fmuv2_default boardconfig where you can enabled/disable modules.

  ::: info
  You may also need to disable other modules in order to fit the rebuilt firmware into 1MB flash.
  Finding modules to remove requires some trial/error and depends on what use cases you need the vehicle to meet.
  :::

<a id="changing"></a>

## Changing a Parameter

To change the value of a parameter click on the parameter row in a group or search list.
This will open a side dialog in which you can update the value (this dialog also provides additional detailed information about the parameter - including whether a reboot is required for the change to take effect).

![Changing a parameter value](../../assets/qgc/setup/parameters/parameters_changing.png)

::: info
When you click **Save** the parameter is automatically and silently uploaded to the connected vehicle.
Depending on the parameter, you may then need to reboot the flight controller for the change to take effect.
:::

## Tools

You can select additional options from the **Tools** menu on the top right hand side of the screen.

![Tools menu](../../assets/qgc/setup/parameters/parameters_tools_menu.png)

**Refresh**
<br>Refresh the parameter values by re-requesting all of them from the vehicle.

**Reset all to firmware defaults**
<br>Reset all parameters to their original default values in the firmware.

**Reset to vehicle's configuration defaults**
<br>Reset all parameters to their original default values for the particular selected airfame configuration.

**Load from file / Save to file**
<br>Load parameters from an existing file or save your current parameter settings to a file.

**Clear all RC to Param**
<br>This clears all associations between RC transmitter controls and parameters.
For more information see: [Radio Setup > Param Tuning Channels](../config/radio.md#param-tuning-channels).

**Reboot Vehicle**
<br>Reboot the vehicle (required after changing some parameters).
