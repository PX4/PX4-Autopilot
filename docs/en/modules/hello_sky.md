# First Application Tutorial (Hello Sky)

This topic explains how to create and run your first onboard application.
It covers all the basic concepts and APIs required for app development on PX4.

The application is a _module_ that runs on a _work queue_, subscribes to a uORB topic and publishes to another one.
This is the pattern most PX4 modules follow.

::: info
For simplicity, more advanced features like parameters and command-line arguments are omitted.
These are covered in [Application/Module Template](../modules/module_template.md).
:::

## Prerequisites

You will require the following:

- [Gazebo Simulator](../sim_gazebo_gz/index.md) (or another [PX4 SITL Simulator](../simulation/index.md)) _or_ a [PX4-compatible flight controller](../flight_controller/index.md).
- [PX4 Development Toolchain](../dev_setup/dev_env.md) for the desired target.
- [Download the PX4 Source Code](../dev_setup/building_px4.md#download-the-px4-source-code) from Github

The source code [PX4-Autopilot/src/examples/px4_simple_app](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/px4_simple_app) directory contains a completed version of this tutorial that you can review if you get stuck.

::: tip
Rename (or delete) the **px4_simple_app** directory.
:::

## Minimal Application

In this section we create a _minimal application_ that just prints out `Hello Sky!`.
This consists of a single _C++_ file, a _cmake_ definition (which tells the toolchain how to build the application) and a _Kconfig_ definition (which allows the application to be enabled in a build).

1. Create a new directory **PX4-Autopilot/src/examples/px4_simple_app**.
2. Create a new C++ file in that directory named **px4_simple_app.cpp**:
   - Copy in the default header to the top of the page.
     This should be present in all contributed files!

     ```cpp
     /****************************************************************************
      *
      *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
      *
      * Redistribution and use in source and binary forms, with or without
      * modification, are permitted provided that the following conditions
      * are met:
      *
      * 1. Redistributions of source code must retain the above copyright
      *    notice, this list of conditions and the following disclaimer.
      * 2. Redistributions in binary form must reproduce the above copyright
      *    notice, this list of conditions and the following disclaimer in
      *    the documentation and/or other materials provided with the
      *    distribution.
      * 3. Neither the name PX4 nor the names of its contributors may be
      *    used to endorse or promote products derived from this software
      *    without specific prior written permission.
      *
      * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
      * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
      * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
      * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
      * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
      * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
      * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
      * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
      * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
      * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
      * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
      * POSSIBILITY OF SUCH DAMAGE.
      *
      ****************************************************************************/
     ```

   - Copy the following code below the default header.

     ```cpp
     /**
      * @file px4_simple_app.cpp
      * Minimal module example for PX4 autopilot
      *
      * @author Example User <mail@example.com>
      */

     #include <px4_platform_common/log.h>
     #include <px4_platform_common/module.h>
     #include <px4_platform_common/px4_work_queue/WorkItem.hpp>

     class PX4SimpleApp : public ModuleBase, public px4::WorkItem
     {
     public:
     	static Descriptor desc;

     	PX4SimpleApp() : WorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

     	static int task_spawn(int argc, char *argv[]);
     	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
     	static int print_usage(const char *reason = nullptr);

     private:
     	void Run() override;
     };

     ModuleBase::Descriptor PX4SimpleApp::desc{task_spawn, custom_command, print_usage};

     void PX4SimpleApp::Run()
     {
     	PX4_INFO("Hello Sky!");
     	exit_and_cleanup(desc);
     }

     int PX4SimpleApp::task_spawn(int argc, char *argv[])
     {
     	PX4SimpleApp *instance = new PX4SimpleApp();

     	if (instance == nullptr) {
     		PX4_ERR("alloc failed");
     		return PX4_ERROR;
     	}

     	desc.object.store(instance);
     	desc.task_id = task_id_is_work_queue;

     	instance->ScheduleNow();
     	return PX4_OK;
     }

     int PX4SimpleApp::print_usage(const char *reason)
     {
     	if (reason) {
     		PX4_WARN("%s\n", reason);
     	}

     	PRINT_MODULE_USAGE_NAME("px4_simple_app", "template");
     	PRINT_MODULE_USAGE_COMMAND("start");
     	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

     	return 0;
     }

     extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[])
     {
     	return ModuleBase::main(PX4SimpleApp::desc, argc, argv);
     }
     ```

     The parts of the code are:
     - `ModuleBase` provides the `start`, `stop` and `status` commands, and ensures that only one instance of the module runs.
       `px4_simple_app_main()` is the entry point of the command, and `ModuleBase::main()` dispatches to `task_spawn()`, `custom_command()` or `print_usage()` through the `desc` descriptor.
     - `px4::WorkItem` makes the module run on a work queue: a thread shared with other modules.
       PX4 calls `Run()` on that thread whenever the module is scheduled, so the module doesn't need a thread and stack of its own.
       The constructor selects the work queue, here the low-priority `lp_default`.
     - `task_spawn()` creates the module object, registers it with `ModuleBase` and schedules it.
       For now we schedule `Run()` just once with `ScheduleNow()`.
     - `Run()` prints the message and then removes the module again with `exit_and_cleanup()`.

     :::tip

     The main function must be named `<module_name>_main` and exported from the module as shown.

     :::

     :::tip

     `PX4_INFO` is the equivalent of `printf` for the PX4 shell (included from **px4_platform_common/log.h**).
     There are different log levels: `PX4_INFO`, `PX4_WARN`, `PX4_ERR`, `PX4_DEBUG`.
     Warnings and errors are additionally added to the [ULog](../dev_log/ulog_file_format.md) and shown on [Flight Review](https://logs.px4.io/).

     :::

3. Create and open a new _cmake_ definition file named **CMakeLists.txt**.
   Copy in the text below:

   ```cmake
   ############################################################################
   #
   #   Copyright (c) 2015 PX4 Development Team. All rights reserved.
   #
   # Redistribution and use in source and binary forms, with or without
   # modification, are permitted provided that the following conditions
   # are met:
   #
   # 1. Redistributions of source code must retain the above copyright
   #    notice, this list of conditions and the following disclaimer.
   # 2. Redistributions in binary form must reproduce the above copyright
   #    notice, this list of conditions and the following disclaimer in
   #    the documentation and/or other materials provided with the
   #    distribution.
   # 3. Neither the name PX4 nor the names of its contributors may be
   #    used to endorse or promote products derived from this software
   #    without specific prior written permission.
   #
   # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
   # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
   # OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
   # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   # POSSIBILITY OF SUCH DAMAGE.
   #
   ############################################################################
   px4_add_module(
   	MODULE examples__px4_simple_app
   	MAIN px4_simple_app
   	SRCS
   		px4_simple_app.cpp
   	DEPENDS
   		px4_work_queue
   	)
   ```

   Note that in your own modules you'd use the current copyright year!
   We're using `2015` here to match the example.

   The `px4_add_module()` method builds a static library from a module description.
   - The `MODULE` block is the Firmware-unique name of the module (by convention the module name is prefixed by parent directories back to `src`).
   - The `MAIN` block lists the entry point of the module, which registers the command with NuttX so that it can be called from the PX4 shell or SITL console.
   - The `DEPENDS` block lists the libraries the module uses, here the work queue library.

   :::tip
   The `px4_add_module()` format is documented in [PX4-Autopilot/cmake/px4_add_module.cmake](https://github.com/PX4/PX4-Autopilot/blob/main/cmake/px4_add_module.cmake). <!-- NEED px4_version -->
   :::

   ::: info
   If you specify `DYNAMIC` as an option to `px4_add_module`, a _shared library_ is created instead of a static library on POSIX platforms (these can be loaded without having to recompile PX4, and shared to others as binaries rather than source code).
   Your app will not become a builtin command, but ends up in a separate file called `examples__px4_simple_app.px4mod`.
   You can then run your command by loading the file at runtime using the `dyn` command: `dyn ./examples__px4_simple_app.px4mod`
   :::

4. Create and open a new _Kconfig_ definition file named **Kconfig** and define your symbol for naming (see [Kconfig naming convention](../hardware/porting_guide_config.md#px4-kconfig-symbol-naming-convention)).
   Copy in the text below:

   ```txt
   menuconfig EXAMPLES_PX4_SIMPLE_APP
   	bool "px4_simple_app"
   	default n
   	---help---
   		Enable support for px4_simple_app
   ```

## Build the Application/Firmware

The application is now complete.
In order to run it you first need to make sure that it is built as part of PX4.
Applications are added to the build/firmware in the appropriate board-level _px4board_ file for your target:

- PX4 SITL (Simulator): [PX4-Autopilot/boards/px4/sitl/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/sitl/default.px4board)
- Pixhawk 6X (px4/fmu-v6x): [PX4-Autopilot/boards/px4/fmu-v6x/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/default.px4board)
- _px4board_ files for other boards can be found in [PX4-Autopilot/boards/](https://github.com/PX4/PX4-Autopilot/tree/main/boards)

To enable the compilation of the application into the firmware add the corresponding Kconfig key `CONFIG_EXAMPLES_PX4_SIMPLE_APP=y` in the _px4board_ file or run [boardconfig](../hardware/porting_guide_config.md#px4-menuconfig-setup).

For example, to edit the board config for FMUv6x you would do:

```sh
make fmu-v6x_default boardconfig
```

And then enable the app in the _boardconfig_ UI as shown:

```txt
examples  --->
    [x] PX4 Simple app  ----
```

::: info
Examples are opt-in and not included in firmware by default (although they are in SITL).
You must explicitly enable them as shown above.
:::

Build the example using the board-specific command:

- Gazebo Simulator: `make px4_sitl gz_x500`
- Pixhawk 6X: `make px4_fmu-v6x_default`
- Other boards: [Building the Code](../dev_setup/building_px4.md)

## Test App (Hardware)

### Upload the firmware to your board

Enable the uploader and then reset the board:

- Pixhawk 6X: `make px4_fmu-v6x_default upload`

It should print before you reset the board a number of compile messages and at the end:

```sh
Loaded firmware for X,X, waiting for the bootloader...
```

Once the board is reset, and uploads, it prints:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

### Connect the Console

Now connect to the [system console](../debug/system_console.md) either via serial or USB.
Hitting **ENTER** will bring up the shell prompt:

```sh
nsh>
```

Type ''help'' and hit ENTER

```sh
nsh> help
  help usage:  help [-v] [<cmd>]

  [           df          kill        mkfifo      ps          sleep
  ?           echo        losetup     mkrd        pwd         test
  cat         exec        ls          mh          rm          umount
  cd          exit        mb          mount       rmdir       unset
  cp          free        mkdir       mv          set         usleep
  dd          help        mkfatfs     mw          sh          xd

Builtin Apps:
  reboot
  perf
  top
  ..
  px4_simple_app
  ..
  sercon
  serdis
```

Note that `px4_simple_app` is now part of the available commands.
Start it by typing `px4_simple_app start` and ENTER:

```sh
nsh> px4_simple_app start
INFO  [px4_simple_app] Hello Sky!
```

The application is now correctly registered with the system and can be extended to actually perform useful tasks.

## Test App (SITL)

If you're using SITL the _PX4 console_ is automatically started (see [Building the Code > First Build (Using a Simulator)](../dev_setup/building_px4.md#first-build-using-a-simulator)).
As with the _nsh console_ (see previous section) you can type `help` to see the list of built-in apps.

Enter `px4_simple_app start` to run the minimal app.

```sh
pxh> px4_simple_app start
INFO  [px4_simple_app] Hello Sky!
```

The application can now be extended to actually perform useful tasks.

## Subscribing to Sensor Data

To do something useful, the application needs to subscribe inputs and publish outputs (e.g. motor or servo commands).

:::tip
The benefits of the PX4 hardware abstraction comes into play here!
There is no need to interact in any way with sensor drivers and no need to update your app if the board or sensors are updated.
:::

Individual message channels between applications are called [topics](../middleware/uorb.md).
For this tutorial, we are interested in the [VehicleAcceleration](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAcceleration.msg) topic, which holds the filtered vehicle acceleration data.

A work queue module doesn't wait for data itself.
Instead, it subscribes with a `uORB::SubscriptionCallbackWorkItem`, which schedules `Run()` every time the topic is published.
Between updates the module doesn't run at all and doesn't use any CPU time.

Add the subscription as a member of the class, and limit how often it schedules `Run()` in the constructor:

```cpp
#include <drivers/drv_hrt.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_acceleration.h>

using namespace time_literals;
..
	PX4SimpleApp() : WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		// Run() at most every 200 ms (5 Hz), not on every vehicle_acceleration update
		_vehicle_acceleration_sub.set_interval_us(200_ms);
	}
..
private:
	// Schedules Run() whenever vehicle_acceleration is published
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};
```

In `task_spawn()`, register the callback instead of scheduling `Run()` once.
We also move the greeting here, because `Run()` now runs repeatedly:

```cpp
	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	PX4_INFO("Hello Sky!");
	return PX4_OK;
```

`Run()` now handles a `stop` request, and otherwise copies the new data with `update()` and prints it:

```cpp
void PX4SimpleApp::Run()
{
	if (should_exit()) {
		_vehicle_acceleration_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_acceleration_s accel;

	if (_vehicle_acceleration_sub.update(&accel)) {
		PX4_INFO("Accelerometer: %8.4f %8.4f %8.4f",
			 (double)accel.xyz[0], (double)accel.xyz[1], (double)accel.xyz[2]);
	}
}
```

`update()` returns `true` and copies the data only if there is a new sample since the last call.
For data that a module only needs to read now and then, without being scheduled by it, use a plain `uORB::Subscription` and call `update()` on it from `Run()` in the same way.

Compile the app again by entering:

```sh
make px4_sitl_default
```

### Testing the uORB Subscription

Start the module:

```sh
px4_simple_app start
```

Your app will greet you and then display the acceleration five times per second:

```sh
INFO  [px4_simple_app] Hello Sky!
INFO  [px4_simple_app] Accelerometer:   0.0176  -0.0252  -9.7370
INFO  [px4_simple_app] Accelerometer:  -0.0061  -0.0286  -9.7450
INFO  [px4_simple_app] Accelerometer:   0.0085  -0.0317  -9.8414
INFO  [px4_simple_app] Accelerometer:   0.0786  -0.0497  -9.7997
INFO  [px4_simple_app] Accelerometer:   0.0087  -0.0534  -9.7937
```

Stop it again with:

```sh
px4_simple_app stop
```

You can see the module on its work queue with `work_queue status`.

## Publishing Data

To use the calculated outputs, the next step is to _publish_ the results.
Below we publish the acceleration to the [DebugVect](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugVect.msg) topic.

::: info
We've chosen `debug_vect` because the _mavlink_ module forwards it to the ground station as the `DEBUG_VECT` message - providing an easy way to look at the results.
Don't publish example data to topics that other modules use, like `vehicle_attitude`: the vehicle would act on it.
:::

Add the publication as a member of the class:

```cpp
#include <string.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_vect.h>
..
	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
```

Then fill in the message and publish it in `Run()` whenever there is new data:

```cpp
		// Publish the values, MAVLink forwards debug_vect to the ground station as DEBUG_VECT
		debug_vect_s debug_vect{};
		strncpy(debug_vect.name, "accel", sizeof(debug_vect.name));
		debug_vect.x = accel.xyz[0];
		debug_vect.y = accel.xyz[1];
		debug_vect.z = accel.xyz[2];
		debug_vect.timestamp = hrt_absolute_time();
		_debug_vect_pub.publish(debug_vect);
```

The topic is advertised automatically the first time it is published.
Every message needs its `timestamp` set to the time of publication.

## Full Example Code

The [complete example code](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/px4_simple_app/px4_simple_app.cpp) is now:

```cpp
/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.cpp
 * Minimal module example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <string.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/vehicle_acceleration.h>

using namespace time_literals;

class PX4SimpleApp : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	PX4SimpleApp() : WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		// Run() at most every 200 ms (5 Hz), not on every vehicle_acceleration update
		_vehicle_acceleration_sub.set_interval_us(200_ms);
	}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

private:
	void Run() override;

	// Schedules Run() whenever vehicle_acceleration is published
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};

	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
};

ModuleBase::Descriptor PX4SimpleApp::desc{task_spawn, custom_command, print_usage};

void PX4SimpleApp::Run()
{
	if (should_exit()) {
		_vehicle_acceleration_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_acceleration_s accel;

	if (_vehicle_acceleration_sub.update(&accel)) {
		PX4_INFO("Accelerometer: %8.4f %8.4f %8.4f",
			 (double)accel.xyz[0], (double)accel.xyz[1], (double)accel.xyz[2]);

		// Publish the values, MAVLink forwards debug_vect to the ground station as DEBUG_VECT
		debug_vect_s debug_vect{};
		strncpy(debug_vect.name, "accel", sizeof(debug_vect.name));
		debug_vect.x = accel.xyz[0];
		debug_vect.y = accel.xyz[1];
		debug_vect.z = accel.xyz[2];
		debug_vect.timestamp = hrt_absolute_time();
		_debug_vect_pub.publish(debug_vect);
	}
}

int PX4SimpleApp::task_spawn(int argc, char *argv[])
{
	PX4SimpleApp *instance = new PX4SimpleApp();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	PX4_INFO("Hello Sky!");
	return PX4_OK;
}

int PX4SimpleApp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal example module: prints vehicle_acceleration at 5 Hz and publishes it as debug_vect.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("px4_simple_app", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[])
{
	return ModuleBase::main(PX4SimpleApp::desc, argc, argv);
}
```

## Running the Complete Example

And finally run your app:

```sh
px4_simple_app start
```

You can check the published values on the console with `listener debug_vect`.
If you start _QGroundControl_, you can also look at the `DEBUG_VECT` message in the [MAVLink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html).

## Wrap-Up

This tutorial covered everything needed to develop a basic PX4 autopilot application.
Keep in mind that the full list of uORB messages/topics is [available here](https://github.com/PX4/PX4-Autopilot/tree/main/msg/) and that the headers are well documented and serve as reference.

Further information and troubleshooting/common pitfalls can be found here: [uORB](../middleware/uorb.md).

The next page presents a template for a full application with parameters, periodic scheduling and performance counters.
