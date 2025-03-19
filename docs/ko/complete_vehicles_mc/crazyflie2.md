# Crazyflie 2.0 (Discontinued)

<Badge type="info" text="Discontinued" />

:::warning
_Crazyflie 2.0_ has been [discontinued/superseded](../flight_controller/autopilot_experimental.md).
Try [Bitcraze Crazyflie 2.1](../complete_vehicles_mc/crazyflie21.md) instead!
:::

:::warning

- PX4 does not manufacture this (or any) autopilot.
  Contact the [manufacturer](https://www.bitcraze.io/) for hardware support or compliance issues.
- PX4 support for this flight controller is [experimental](../flight_controller/autopilot_experimental.md).

:::

The Crazyflie line of micro quads was created by Bitcraze AB.
An overview of the Crazyflie 2.0 can be [found here](https://www.bitcraze.io/crazyflie-2/).

![Crazyflie2 Image](../../assets/flight_controller/crazyflie/crazyflie2_hero.png)

## 요약

:::info
The main hardware documentation is here: https://wiki.bitcraze.io/projects:crazyflie2:index
:::

- Main System-on-Chip: STM32F405RG
  - CPU : 단정밀도 FPU의 168MHz ARM Cortex M4
  - RAM : 192KB SRAM
- nRF51822 radio and power management MCU
- MPU9250 Accel / Gyro / Mag
- LPS25H barometer

## 구매처

- [Crazyflie 2.0](https://store.bitcraze.io/collections/kits/products/crazyflie-2-0).
- [Crazyradio PA 2.4 GHz USB dongle](https://store.bitcraze.io/collections/kits/products/crazyradio-pa): used for wireless communication between _QGroundControl_ and Crazyflie 2.0.
- [Breakout deck](https://store.bitcraze.io/collections/decks/products/breakout-deck): breakout expansion board for connecting new peripherals.
- [Flow deck](https://store.bitcraze.io/collections/decks/products/flow-deck): contains an optical flow sensor to measure movements of the ground and a distance sensor to measure the distance to the ground.
  This will be useful for precise altitude and position control.
- [Z-ranger deck](https://store.bitcraze.io/collections/decks/products/z-ranger-deck) has the same distance sensor as the Flow deck to measure the distance to the ground.
  This will be useful for precise altitude control.
- [SD-card deck](https://store.bitcraze.io/collections/decks/products/sd-card-deck): used for high speed onboard logging to a micro SD card.
- [Logitech Joystick](https://support.logi.com/hc/en-us/articles/360024326793--Getting-Started-Gamepad-F310).

## Flashing PX4

After setting up the PX4 development environment, follow these steps to install the PX4 Autopilot on the Crazyflie 2.0:

1. Download the source code of the PX4 Bootloader:

  ```sh
  git clone https://github.com/PX4/Bootloader.git
  ```

2. Navigate into the top directory of the source code and compile it using:

  ```sh
  make crazyflie_bl
  ```

3. Put the Crazyflie 2.0 into DFU mode by following these steps:
  - Ensure it is initially unpowered.
  - Hold down the reset button (see figure below...).
    ![Crazyflie2 Reset Button](../../assets/flight_controller/crazyflie/crazyflie_reset_button.jpg)
  - Plug into computer's USB port.
  - After a second, the blue LED should start blinking and after 5 seconds should start blinking faster.
  - Release button.

4. Install _dfu-util_:

  ```sh
  sudo apt-get update
  sudo apt-get install dfu-util
  ```

5. Flash bootloader using _dfu-util_ and unplug Crazyflie 2.0 when done:

  ```sh
  sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D ./build/crazyflie_bl/crazyflie_bl.bin
  ```

  When powering on the Crazyflie 2.0 the yellow LED should blink.

6. Download the source code of the PX4 autopilot:

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git
  ```

7. Navigate into the top directory of the source code and compile it using:

  ```sh
  make bitcraze_crazyflie_default upload
  ```

8. When prompted to plug in device, plug in Crazyflie 2.0.
  The yellow LED should start blinking indicating bootloader mode.
  Then the red LED should turn on indicating that the flashing process has started.

9. Wait for completion.

10. Done! Calibrate the sensors using [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors.html).

:::info
If QGroundControl does not connect with the vehicle, ensure that in [nuttx-config](https://github.com/PX4/PX4-Autopilot/blob/main/boards/bitcraze/crazyflie/nuttx-config/nsh/defconfig) for crazyflie `# CONFIG_DEV_LOWCONSOLE is not set` is replaced by `CONFIG_DEV_LOWCONSOLE=y`.
This should be done using _menuconfig_:

```sh
make bitcraze_crazyflie_default menuconfig
```

or _qconfig_ (Check _Low-level console support_ under _Serial Driver Support_ in GUI):

```sh
make bitcraze_crazyflie_default qconfig
```

:::

## Wireless Setup Instructions

The onboard nRF module allows connecting to the board via Bluetooth or through the proprietary 2.4GHz Nordic ESB protocol.

- A [Crazyradio PA](https://www.bitcraze.io/crazyradio-pa/) is recommended.
- To fly the Crazyflie 2.0 right away, the Crazyflie phone app is supported via Bluetooth.

Using the official Bitcraze **Crazyflie phone app**:

- Connect via Bluetooth.
- Change mode in settings to 1 or 2.
- Calibrate via QGroundControl.

Connecting via **MAVLink**:

- Use a Crazyradio PA alongside a compatible GCS.
- Download the _crazyflie-lib-python_ source code:

  ```sh
  git clone https://github.com/bitcraze/crazyflie-lib-python.git
  ```

:::info
We will use [cfbridge.py](https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/cfbridge.py) to setup a wireless MAVlink communication link between Crazyflie 2.0 (flashed with PX4) and QGroundControl. _Cfbridge_ enables QGroundControl to communicate with the crazyradio PA.
The [C based cfbridge](https://github.com/dennisss/cfbridge) is currently experiencing data loss issues, which is why we have chosen to use **cfbridge.py**.
:::

- Make sure you have set the udev permissions to use the USB Radio. To do this, follow the steps listed [here](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/) and **restart** your computer.

- Connect a Crazyradio PA via USB.

- Build a [virtual environment (local python environment)](https://virtualenv.pypa.io/en/latest/) with package dependencies using the following method:

  ```sh
  pip install tox --user
  ```

- Navigate to the crazyflie-lib-python folder and type:

  ```sh
  make venv
  ```

- Activate the virtual environment:

  ```sh
  source venv-cflib/bin/activate
  ```

- Install required dependencies:

  ```sh
  pip install -r requirements.txt --user
  ```

To connect Crazyflie 2.0 with crazyradio, **launch cfbridge** by following these steps:

- Power off and power on Crazyflie 2.0 and wait for it to boot up.

- Connect a Crazyflie radio device via USB.

- Navigate to the crazyflie-lib-python folder.

- Activate the environment:

  ```sh
  source venv-cflib/bin/activate
  ```

- Navigate to the examples folder:

  ```sh
  cd examples
  ```

- Launch cfbridge:

```sh
python cfbridge.py
```

:::info
_Cfbridge_ by default tries to initiate the radio link communication on channel 80 and with crazyflie address 0xE7E7E7E7E7.
If you are using [multiple crazyflies and/or crazyradios](https://github.com/dennisss/cfbridge/blob/master/index.md#advanced-swarming) in the same room and want to use a different channel and/or address for each, first connect the crazyflie with QGroundControl via a USB cable and change the syslink parameters (channel, address) in QGroundControl.
Next, launch the cfbridge by giving the same channel and address as the first and second arguments respectively, e.g: `python cfbridge.py 90 0x0202020202`
:::

- Open QGroundControl.
- After using _cfbridge_, you can deactivate the virtualenv if you activated it by pressing `CTRL+z`.
  Most of the time, launching _cfbridge_ again from the same terminal doesn't connect to crazyflie, this can be solved by closing the terminal and relaunching _cfbridge_ in a new terminal.

:::tip
If you change any driver in [crazyflie-lib-python](https://github.com/bitcraze/crazyflie-lib-python) or if launching _cfbridge_ in a new terminal does not find crazyflie, you can try navigating to the crazyflie-lib-python folder and run the script below to rebuild cflib.

```sh
make venv
```

:::

:::info
To use Joystick, set `COM_RC_IN_MODE` in QGroundControl to "Joystick/No RC Checks".
Calibrate the Joystick and set the Joystick message frequency in QGroundControl to any value between 5 to 14 Hz (10 Hz is recommended).
To be able to set the frequency, the advanced option should be enabled.
This is the rate at which Joystick commands are sent from QGroundControl to Crazyflie 2.0 (to do this, you will need to follow the instructions [here](https://github.com/mavlink/qgroundcontrol) to obtain the latest QGroundControl source code (master) and build it).
:::

![](../../assets/hardware/joystick-message-frequency.png)

## 하드웨어 설정

Crazyflie 2.0 is able to fly with precise control in [Stabilized mode](../flight_modes_mc/manual_stabilized.md), [Altitude mode](../flight_modes_mc/altitude.md) and [Position mode](../flight_modes_mc/position.md).

- You will need the [Z-ranger deck](https://store.bitcraze.io/collections/decks/products/z-ranger-deck) to fly in _Altitude_ mode.
  If you also want to fly in the _Position_ mode, it is recommended you buy the [Flow deck](https://store.bitcraze.io/collections/decks/products/flow-deck) which also has the integrated Z-ranger sensor.
- The onboard barometer is highly susceptible to any external wind disturbances including those created by Crazyflie's own propellers. Hence, we isolated the barometer with a piece of foam, and then mounted the distance sensor on top of it as shown below:

![Crazyflie barometer](../../assets/flight_controller/crazyflie/crazyflie_barometer.jpg)

![Crazyflie barometer foam](../../assets/flight_controller/crazyflie/crazyflie_baro_foam.jpg)

![Crazyflie optical flow](../../assets/flight_controller/crazyflie/crazyflie_opticalflow.jpg)

In order to log flight details, you can mount SD card deck on top of crazyflie as shown below:

![Crazyflie SDCard](../../assets/flight_controller/crazyflie/crazyflie_sdcard.jpg)

Then, you need to stick the battery on top of the SD card deck using a double sided tape:

![Crazyflie battery setup](../../assets/flight_controller/crazyflie/crazyflie_battery_setup.jpg)

## Altitude Control

Crazyflie is able to fly in _Altitude_ mode if you use a [Z-ranger deck](https://store.bitcraze.io/collections/decks/products/z-ranger-deck).
According to the datasheet, the maximum height (above ground) the range finder can sense is 2 m. However, when tested on dark surfaces this value decreases to 0.5 m. On a light floor, it goes up to max 1.3 m. This means you cannot hold altitudes above this value in _Altitude_ or _Position_ flight modes.

:::tip
If the Crazyflie 2.0 height drifts at mid-throttle command in _Altitude mode_ or _Position mode_, first try rebooting the vehicle. If this does not fix the problem, recalibrate the accel and mag (compass).\
:::

:::info
Since the onboard barometer is highly susceptible to wind disturbances created by the Crazyflie's own propellers, you cannot rely on it to hold altitude.
:::

## Position Control

With [Flow deck](https://store.bitcraze.io/collections/decks/products/flow-deck), you can fly Crazyflie 2.0 in _Position mode_.
Unlike [PX4FLOW](../sensor/px4flow.md), the flow deck does not house a gyro, hence the onboard gyro is used for flow fusion to find the local position estimates.
Moreover, the flow deck shares the same SPI bus as the SD card deck, therefore logging at high rate on SD card is not recommended when flying in _Position mode_.

## Using FrSky Taranis RC Transmitter as Joystick

If you already own a Taranis RC transmitter and want to use it as a controller, it can be configured as a USB Joystick:

- Create a new model in Taranis.

  ![Taranis - new model](../../assets/flight_controller/crazyflie/taranis_model.jpg)

- In _MODEL SETUP_ menu page, turn off both internal and external TX modules.

  ![Taranis - model setup](../../assets/flight_controller/crazyflie/taranis_model_setup.jpg)

- In _OUTPUTS_ menu page (also called “SERVOS” page in some Taranis transmitters), invert Throttle (CH1) and Aileron (CH3).

  ![Taranis - outputs](../../assets/flight_controller/crazyflie/taranis_outputs.jpg)

To use Taranis switches to arm/disarm and switch to different flight modes:

- In Taranis UI _MIXER_ menu page, you can assign the switches to any channel in the range channel 9-16 which map to the buttons 0-7 in the QGroundControl Joystick setup. For example, Taranis “SD” switch can be set to channel 9 in Taranis UI:

  ![Taranis switch setup](../../assets/flight_controller/crazyflie/taranis_switch_setup.jpg)

- Connect Taranis to PC with a USB cable and Open QGroundControl.

- In QGroundControl Joystick Setup, you can see the buttons turning yellow when you switch them on. For example, channel 9 in Taranis maps to button 0 in QGroundControl Joystick setup. You can assign any mode to this button e.g. _Altitude_ mode. Now when you lower the switch "SD", flight mode will change to _Altitude_.

  ![Joystick setup](../../assets/flight_controller/crazyflie/crazyflie_QGCjoystick_setup.png)

### ROS

To connect to Crazyflie 2.0 via MAVROS:

- Start up _cfbridge_ using the above instructions.

- Change the UDP port QGroundControl listens to:
  - In QGroundControl, navigate to **Application Settings > General** and uncheck all the boxes under _Autoconnect to the following devices_.
  - Add in **Comm Links** a link of type _UDP_, check the _Automatically Connect on Start_ option, change the _Listening Port_ to 14557, add Target Hosts: 127.0.0.1 and then press **OK**.

- Make sure you have [MAVROS](https://github.com/mavlink/mavros/tree/master/mavros#installation) installed.

- Start MAVROS with command:

  ```sh
  roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14551" gcs_url:="udp://@127.0.0.1:14557"
  ```

- Restart QGroundControl if it doesn't connect.
