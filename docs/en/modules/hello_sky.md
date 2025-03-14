# First Application Tutorial (Hello Sky)

This topic explains how to create and run your first onboard application.
It covers all the basic concepts and APIs required for app development on PX4.

::: info
For simplicity, more advanced features like start/stop functionality and command-line arguments are omitted.
These are covered in [Application/Module Template](../modules/module_template.md).
:::

## Prerequisites

You will require the following:

- [PX4 SITL Simulator](../simulation/index.md) _or_ a [PX4-compatible flight controller](../flight_controller/index.md).
- [PX4 Development Toolchain](../dev_setup/dev_env.md) for the desired target.
- [Download the PX4 Source Code](../dev_setup/building_px4.md#download-the-px4-source-code) from Github

The source code [PX4-Autopilot/src/examples/px4_simple_app](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/px4_simple_app) directory contains a completed version of this tutorial that you can review if you get stuck.

- Rename (or delete) the **px4_simple_app** directory.

## Minimal Application

In this section we create a _minimal application_ that just prints out `Hello Sky!`.
This consists of a single _C_ file and a _cmake_ definition (which tells the toolchain how to build the application).

1. Create a new directory **PX4-Autopilot/src/examples/px4_simple_app**.
1. Create a new C file in that directory named **px4_simple_app.c**:

   - Copy in the default header to the top of the page.
     This should be present in all contributed files!

     ```c
     /****************************************************************************
      *
      *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
     This should be present in all contributed files!

     ```c
     /**
      * @file px4_simple_app.c
      * Minimal application example for PX4 autopilot
      *
      * @author Example User <mail@example.com>
      */

     #include <px4_platform_common/log.h>

     __EXPORT int px4_simple_app_main(int argc, char *argv[]);

     int px4_simple_app_main(int argc, char *argv[])
     {
     	PX4_INFO("Hello Sky!");
     	return OK;
     }
     ```

     :::tip
     The main function must be named `<module_name>_main` and exported from the module as shown.
     :::

     :::tip
     `PX4_INFO` is the equivalent of `printf` for the PX4 shell (included from **px4_platform_common/log.h**).
     There are different log levels: `PX4_INFO`, `PX4_WARN`, `PX4_ERR`, `PX4_DEBUG`.
     Warnings and errors are additionally added to the [ULog](../dev_log/ulog_file_format.md) and shown on [Flight Review](https://logs.px4.io/).
     :::

1. Create and open a new _cmake_ definition file named **CMakeLists.txt**.
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
   	STACK_MAIN 2000
   	SRCS
   		px4_simple_app.c
   	DEPENDS
   	)
   ```

   The `px4_add_module()` method builds a static library from a module description.

   - The `MODULE` block is the Firmware-unique name of the module (by convention the module name is prefixed by parent directories back to `src`).
   - The `MAIN` block lists the entry point of the module, which registers the command with NuttX so that it can be called from the PX4 shell or SITL console.

   :::tip
   The `px4_add_module()` format is documented in [PX4-Autopilot/cmake/px4_add_module.cmake](https://github.com/PX4/PX4-Autopilot/blob/main/cmake/px4_add_module.cmake). <!-- NEED px4_version -->
   :::

   ::: info
   If you specify `DYNAMIC` as an option to `px4_add_module`, a _shared library_ is created instead of a static library on POSIX platforms (these can be loaded without having to recompile PX4, and shared to others as binaries rather than source code).
   Your app will not become a builtin command, but ends up in a separate file called `examples__px4_simple_app.px4mod`.
   You can then run your command by loading the file at runtime using the `dyn` command: `dyn ./examples__px4_simple_app.px4mod`
   :::

1. Create and open a new _Kconfig_ definition file named **Kconfig** and define your symbol for naming (see [Kconfig naming convention](../hardware/porting_guide_config.md#px4-kconfig-symbol-naming-convention)).
   Copy in the text below:

   ```
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
- Pixhawk v1/2: [PX4-Autopilot/boards/px4/fmu-v2/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v2/default.px4board)
- Pixracer (px4/fmu-v4): [PX4-Autopilot/boards/px4/fmu-v4/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v4/default.px4board)
- _px4board_ files for other boards can be found in [PX4-Autopilot/boards/](https://github.com/PX4/PX4-Autopilot/tree/main/boards)

To enable the compilation of the application into the firmware add the corresponding Kconfig key `CONFIG_EXAMPLES_PX4_SIMPLE_APP=y` in the _px4board_ file or run [boardconfig](../hardware/porting_guide_config.md#px4-menuconfig-setup) `make px4_fmu-v4_default boardconfig`:

```
examples  --->
    [x] PX4 Simple app  ----
```

::: info
The line will already be present for most files, because the examples are included in firmware by default.
:::

Build the example using the board-specific command:

- jMAVSim Simulator: `make px4_sitl_default jmavsim`
- Pixhawk v1/2: `make px4_fmu-v2_default` (or just `make px4_fmu-v2`)
- Pixhawk v3: `make px4_fmu-v4_default`
- Other boards: [Building the Code](../dev_setup/building_px4.md#building-for-nuttx)

## Test App (Hardware)

### Upload the firmware to your board

Enable the uploader and then reset the board:

- Pixhawk v1/2: `make px4_fmu-v2_default upload`
- Pixhawk v3: `make px4_fmu-v4_default upload`

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
Start it by typing `px4_simple_app` and ENTER:

```sh
nsh> px4_simple_app
Hello Sky!
```

The application is now correctly registered with the system and can be extended to actually perform useful tasks.

## Test App (SITL)

If you're using SITL the _PX4 console_ is automatically started (see [Building the Code > First Build (Using a Simulator)](../dev_setup/building_px4.md#first-build-using-a-simulator)).
As with the _nsh console_ (see previous section) you can type `help` to see the list of built-in apps.

Enter `px4_simple_app` to run the minimal app.

```sh
pxh> px4_simple_app
INFO  [px4_simple_app] Hello Sky!
```

The application can now be extended to actually perform useful tasks.

## Subscribing to Sensor Data

To do something useful, the application needs to subscribe inputs and publish outputs (e.g. motor or servo commands).

:::tip
The benefits of the PX4 hardware abstraction comes into play here!
There is no need to interact in any way with sensor drivers and no need to update your app if the board or sensors are updated.
:::

Individual message channels between applications are called [topics](../middleware/uorb.md). For this tutorial, we are interested in the [SensorCombined](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorCombined.msg) topic, which holds the synchronized sensor data of the complete system.

Subscribing to a topic is straightforward:

```cpp
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
```

The `sensor_sub_fd` is a topic handle and can be used to very efficiently perform a blocking wait for new data.
The current thread goes to sleep and is woken up automatically by the scheduler once new data is available, not consuming any CPU cycles while waiting.
To do this, we use the [poll()](http://pubs.opengroup.org/onlinepubs/007908799/xsh/poll.html) POSIX system call.

Adding `poll()` to the subscription looks like (_pseudocode, look for the full implementation below_):

```cpp
#include <poll.h>
#include <uORB/topics/sensor_combined.h>
..
int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

/* one could wait for multiple topics with this technique, just using one here */
px4_pollfd_struct_t fds[] = {
    { .fd = sensor_sub_fd,   .events = POLLIN },
};

while (true) {
	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
	int poll_ret = px4_poll(fds, 1, 1000);
	..
	if (fds[0].revents & POLLIN) {
		/* obtained data for the first file descriptor */
		struct sensor_combined_s raw;
		/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
		PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					(double)raw.accelerometer_m_s2[0],
					(double)raw.accelerometer_m_s2[1],
					(double)raw.accelerometer_m_s2[2]);
	}
}
```

Compile the app again by entering:

```sh
make
```

### Testing the uORB Subscription

The final step is to start your application as a background process/task by typing the following in the nsh shell:

```sh
px4_simple_app &
```

Your app will display 5 sensor values in the console and then exit:

```sh
[px4_simple_app] Accelerometer:   0.0483          0.0821          0.0332
[px4_simple_app] Accelerometer:   0.0486          0.0820          0.0336
[px4_simple_app] Accelerometer:   0.0487          0.0819          0.0327
[px4_simple_app] Accelerometer:   0.0482          0.0818          0.0323
[px4_simple_app] Accelerometer:   0.0482          0.0827          0.0331
[px4_simple_app] Accelerometer:   0.0489          0.0804          0.0328
```

:::tip
The [Module Template for Full Applications](../modules/module_template.md) can be used to write background process that can be controlled from the command line.
:::

## Publishing Data

To use the calculated outputs, the next step is to _publish_ the results.
Below we show how to publish the attitude topic.

::: info
We've chosen `attitude` because we know that the _mavlink_ app forwards it to the ground control station - providing an easy way to look at the results.
:::

The interface is pretty simple: initialize the `struct` of the topic to be published and advertise the topic:

```c
#include <uORB/topics/vehicle_attitude.h>
..
/* advertise attitude topic */
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));
orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
```

In the main loop, publish the information whenever its ready:

```c
orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
```

## Full Example Code

The [complete example code](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/px4_simple_app/px4_simple_app.c) is now:

```c
/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
```

## Running the Complete Example

And finally run your app:

```sh
px4_simple_app
```

If you start _QGroundControl_, you can check the sensor values in the real time plot ([Analyze > MAVLink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html)).

## Wrap-Up

This tutorial covered everything needed to develop a basic PX4 autopilot application.
Keep in mind that the full list of uORB messages/topics is [available here](https://github.com/PX4/PX4-Autopilot/tree/main/msg/) and that the headers are well documented and serve as reference.

Further information and troubleshooting/common pitfalls can be found here: [uORB](../middleware/uorb.md).

The next page presents a template for writing a full application with start and stop functionality.
