# CRSF Telemetry (TBS Crossfire Telemetry)

CRSF is a telemetry protocol that can be used for both [RC control](../getting_started/rc_transmitter_receiver.md) and to get telemetry information from the vehicle/flight controller on a compatible RC transmitter.

The protocol was developed by [Team BlackSheep](https://www.team-blacksheep.com/) for their Crossfire RC system, but is also used by [ExpressLRS (ELRS)](https://www.expresslrs.org/) RC systems.
It is a bidirectional protocol that only needs a single UART for communicating both RC and telemetry.

The [supported telemetry messages are listed here](#telemetry-messages), and include: flight mode, battery level, GPS data RC signal strength, speed, altitude, and so on.

:::info
If you don't need telemetry you can connect a TBS Crossfire to the `RCIN` port and configure the receiver to use S.BUS.
Crossfire radio systems can also be used as [Telemetry Radios](../telemetry/index.md).
:::

:::warning
PX4 does not include the CRSF protocol support by default.
The [instructions below](#px4-configuration) explain how to build and upload custom PX4 firmware that includes the required modules.
:::

## Radio System Setup

To use CRSF telemetry you will need a [TBS Crossfire radio system](#tbs-radio-systems) or [ExpressLRS radio system](#expresslrs-radio-systems) that includes an [RC controller](#rc-controllers) with a transmitter, and a receiver (from the same vendor).

:::info
An RC radio system historically consisted of a ground-based controller that transmitted to an on-vehicle receiver.
Even though many radio systems are now bidirectional, the ground module may still be referred to as the transmitter, and the air unit may be called a receiver.
:::

Generally you will need to separately setup and configure the transmitter and receiver, and then _bind_ them together.

A transmitter might come as an integral part of an [RC controller](#rc-controllers), or it might be a separate module that you plug into a controller.
If it is a separate module then you may also need to update the module software on the transmitter to firmware that supports CRSF, such as OpenTX or EdgeTx.
In both cases you will need to configure the transmitter to enable CRSF.

The receiver must be [wired](#wiring) to a spare port (UART) on the Flight Controller.
Then you can _bind_ the transmitter and receiver together.

Instructions for the steps above are covered in

- [TBS Crossfire Manual](https://www.team-blacksheep.com/tbs-crossfire-manual.pdf)
- [Express LRS: QuickStart](https://www.expresslrs.org/quick-start/getting-started/)

### 배선

The TX and RX on your selected Flight Controller UART should be connected to separate channels on the receiver.
The signal is usually uninverted, and can be directly connected (no additional inverter logic is required in the cable).
You should check the manual for your specific receiver though!

#### TBS Receiver Wiring

For TBS receivers you wire the FC UART and receiver as shown (this assumes the TBS Nano RX).

| FC UART | Nano RX |
| ------- | ------- |
| TX      | Ch2     |
| RX      | Ch1     |

#### ExpressLRS Receiver Wiring

For ExpressLRS receivers wire to the flight controller UART as shown below (wiring is covered [in detail here](https://www.expresslrs.org/quick-start/receivers/wiring-up/)):

| FC UART | ExpressLRS |
| ------- | ---------- |
| TX      | RX         |
| RX      | TX         |
| VCC     | VCC        |
| GND     | GND        |

## PX4 설정

### Firmware Configuration/Build

CRSF telemetry support is not included in any PX4 firmware by default.
To use this feature you must build and upload custom firmware that includes [crsf-rc](../modules/modules_driver.md#crsf-rc) and removes [rc_input](../modules/modules_driver.md#rc-input).

단계는 다음과 같습니다:

1. [Setup a development environment](../dev_setup/dev_env.md) for building PX4.

  As part of this process you will have used `git` to fetch source code into the **PX4-Autopilot** directory.

2. Open a terminal and `cd` into the `PX4-Autopilot` directory.

  ```sh
  cd PX4-Autopilot
  ```

3. Launch the [PX4 board config tool (`menuconfig`)](../hardware/porting_guide_config.md#px4-menuconfig-setup) for your `make` target using the `boardconfig` option (here the target is the [ARK Electronics ARKV6X](../flight_controller/ark_v6x.md) flight controller):

  ```sh
  make ark_fmu-v6x_default boardconfig
  ```

4. In the PX4 board config tool:

  - Disable the default `rc_input` module
    1. Navigate to the `drivers` submenu, then scroll down to highlight `rc_input`.
    2. Use the enter key to remove the `*` from `rc_input` checkbox.
  - Enable the `crsf_rc` module
    1. Scroll to highlight the `RC` submenu, then press enter to open it.
    2. Scroll to highlight `crsf_rc` and press enter to enable it.

  Save and exit the PX4 board config tool.

5. [Build the PX4 source code](../dev_setup/building_px4.md) with your changes (again assuming you are using ARKV6X):

  ```sh
  make ark_fmu-v6x_default
  ```

This will build your custom firmware, which must now be uploaded to your flight controller.

### Firmware Upload

To upload the custom firmware, first connect your flight controller to the development computer via USB.

You can upload firmware as part of the build process using the `upload` options:

```sh
make ark_fmu-v6x_default upload
```

Alternatively you can use QGroundControl to install the firmware, as described in [Firmware > Installing PX4 master, beta, or custom firmware](../config/firmware.md#installing-px4-main-beta-or-custom-firmware).

### Parameter Configuration

[Find and set](../advanced_config/parameters.md) the following parameters:

1. [RC_CRSF_PRT_CFG](../advanced_config/parameter_reference.md#RC_CRSF_PRT_CFG) — Set to the port that is connected to the CRSF receiver (such as `TELEM1`).

  This [configures the serial port](../peripherals/serial_configuration.md) to use the CRSF protocol.
  Note that some serial ports may already have a [default serial port mapping](../peripherals/serial_configuration.md#default-serial-port-configuration) or [default MAVLink serial port mapping](../peripherals/mavlink_peripherals.md#default-mavlink-ports) that you will have to un-map before you can assign the port to CRSF.
  For example, if you want to use `TELEM1` or `TELEM2` you first need to modify [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) or [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) to stop setting those ports.

  포트 전송속도는 드라이버에 의해 설정되므로, 추가로 설정하지 않아도 됩니다.

2. [RC_CRSF_TEL_EN](../advanced_config/parameter_reference.md#RC_CRSF_TEL_EN) — Enable to activate Crossfire telemetry.

### 무선 조종기 설정

[Radio Control Setup](../config/radio.md) explains how to map your RC controller's attitude control sticks (roll, pitch, yaw, throttle) to channels, and to calibrate the minimum, maximum, trim and reverse settings for all other transmitter controls/RC channels.

## RC Controllers

A transmitter might come as an integral part of an RC controller, or it might be a separate module that you plug into a controller.

RC Controllers that support TBS Crossfire and ExpressLRS TX modules:

- [FrSky Taranis X9D Plus](https://www.frsky-rc.com/product/taranis-x9d-plus-2/) has an external module bay that can be used with TBS or ExpressLRS transmitter modules that are "JR module bay" compatible.
  You will need to install OpenTX software, which supports CRSF, and enable the external module and CRSF.
- [Radiomaster TX16S](https://www.radiomasterrc.com/collections/tx16s-mkii) has an internal ExpressLRS transmitter module.
  It also has an external module bay that can be used with TBS or ExpressLRS transmitter modules that are "JR module bay" compatible.
  It runs both OpenTX and EdgeTx software, either of which can support CRSF.

## TBS Radio Systems

[TBS Crossfire Radio Systems are listed here](https://www.team-blacksheep.com/shop/cat:cat_crossfire#product_listing).
A few options are listed below.

Transmitter modules:

- [TBS CROSSFIRE TX - LONG RANGE R/C TRANSMITTER](https://www.team-blacksheep.com/products/prod:crossfire_tx)

Receivers:

- [TBS Crossfire Nano RX](http://team-blacksheep.com/products/prod:crossfire_nano_rx) - designed for small quadcopters.

## ExpressLRS Radio Systems

Express LRS provide Radio System guidance in the [Hardware Selection](https://www.expresslrs.org/hardware/hardware-selection/) page.
A few tested options are listed below.

Transmitter modules:

- TBD

Receivers:

- [ExpressLRS Matek Diversity RX](http://www.mateksys.com/?portfolio=elrs-r24).

  ::: info
  This is used in the [Reptile Dragon 2 Build Log](../frames_plane/reptile_dragon_2.md).
  See sections [ELRS Rx](../frames_plane/reptile_dragon_2.md#elrs-rx) and [Radio Setup](../frames_plane/reptile_dragon_2.md#radio-setup).

:::

## 텔레메트리 메시지

The supported telemetry messages and their source are listed below (this table is reproduced from the [TBS Crossfire Manual: "Available sensors with OpenTX"](https://www.team-blacksheep.com/tbs-crossfire-manual.pdf)).

| Datapoint | 설명                                                                    | Data source                      |
| --------- | --------------------------------------------------------------------- | -------------------------------- |
| 1RSS      | Uplink - received signal strength antenna 1 (RSSI) | TBS CROSSFIRE RX                 |
| 2RSS      | Uplink - received signal strength antenna 2 (RSSI) | TBS CROSSFIRE RX                 |
| RQly      | Uplink - link quality (valid packets)              | TBS CROSSFIRE RX                 |
| RSNR      | Uplink - signal-to-noise ratio                                        | TBS CROSSFIRE RX                 |
| RFMD      | Uplink - update rate, 0 = 4Hz, 1 = 50Hz, 2 = 150Hz                    | TBS CROSSFIRE RX                 |
| TPWR      | Uplink - transmitting power                                           | TBS CROSSFIRE TX                 |
| TRSS      | Downlink - signal strength antenna                                    | TBS CROSSFIRE TX                 |
| TQly      | Downlink - link quality (valid packets)            | TBS CROSSFIRE TX                 |
| TSNR      | Downlink - signal-to-noise ratio                                      | TBS CROSSFIRE TX                 |
| ANT       | Sensor for debugging only                                             | TBS CROSSFIRE TX                 |
| GPS       | GPS Coordinates                                                       | TBS GPS / FC                     |
| Alt       | GPS Altitudes                                                         | TBS GPS / FC                     |
| Sats      | GPS Satellites acquired                                               | TBS GPS / FC                     |
| Hdg       | Magnetic orientation                                                  | TBS GPS / FC                     |
| RXBt      | Battery voltage                                                       | TBS GPS / FC/ CROSSFIRE RX/ CORE |
| Curr      | Current draw                                                          | TBS GPS / FC// CORE              |
| Capa      | Current consumption                                                   | TBS GPS / FC/ CORE               |
| Ptch      | FC Pitch angle                                                        | FC                               |
| Roll      | FC Roll angle                                                         | FC                               |
| Yaw       | FC Yaw angle                                                          | FC                               |
| FM        | Flight mode                                                           | FC                               |
| VSPD      | 기압계                                                                   | FC                               |

## See Also

- [TBS Crossfire Manual](https://www.team-blacksheep.com/tbs-crossfire-manual.pdf)
- [ExpressLRS Documentation](https://www.expresslrs.org/quick-start/getting-started/)
- [FrSky Telemetry](../peripherals/frsky_telemetry.md)
- [Radio Control Setup](../config/radio.md)
- [Radio Control Systems](../getting_started/rc_transmitter_receiver.md)
