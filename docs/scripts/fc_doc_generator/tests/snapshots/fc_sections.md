# PX4 Board FC Doc Sections

Auto-generated from source. See `fc_doc_generator.py`.

---

### 3dr/ctrl-zero-h7-oem-revg

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 3
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "3dr/ctrl-zero-h7-oem-revg",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ctrl-zero-h7-oem-revg/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "3dr/ctrl-zero-h7-oem-revg",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/3dr_ctrl-zero-h7-oem-revg/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "3dr/ctrl-zero-h7-oem-revg",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/3dr_ctrl-zero-h7-oem-revg/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "3dr/ctrl-zero-h7-oem-revg",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ctrl-zero-h7-oem-revg/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "3dr/ctrl-zero-h7-oem-revg",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ctrl-zero-h7-oem-revg board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ctrl-zero-h7-oem-revg/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "3dr/ctrl-zero-h7-oem-revg",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 |  | - |
| UART7 | /dev/ttyS4 | TELEM3 | - |
| UART8 | /dev/ttyS5 | Debug Console | - |
```

---

### accton-godwit/ga1

**Doc:** [https://docs.px4.io/main/en/flight_controller/accton-godwit_ga1](https://docs.px4.io/main/en/flight_controller/accton-godwit_ga1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-42688P (SPI), ICM-42688P (SPI)
- **Barometer**: TODO: confirm which is installed — BMP388, ICP-20100 (I2C, internal), ICP-20100 (I2C, external), MS5611
- **Magnetometer**: TODO: confirm which is installed — BMM150, IST8308, IST8310 (I2C, bus 1, external), MMC5983MA, RM3100 (I2C, bus 4, internal)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - ICP-20100 (barometer, internal)
  - ICP-20100 (barometer, external)
  - RM3100 (magnetometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - ICM-42688P (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "accton-godwit/ga1",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-42688P"
    ],
    "baro": [
      "BMP388",
      "ICP-20100",
      "MS5611"
    ],
    "mag": [
      "BMM150",
      "IST8308",
      "IST8310",
      "MMC5983MA",
      "RM3100"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ga1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "accton-godwit/ga1",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/accton-godwit_ga1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "accton-godwit/ga1",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/accton-godwit_ga1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "accton-godwit/ga1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ga1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "accton-godwit/ga1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ga1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ga1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "accton-godwit/ga1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### airmind/mindpx-v2

**Doc:** [https://docs.px4.io/main/en/flight_controller/mindpx](https://docs.px4.io/main/en/flight_controller/mindpx)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20948, MPU-6000 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: confirm which is installed — HMC5883L (I2C, internal), QMC5883L (I2C, internal)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: 2
  - HMC5883L (magnetometer, internal)
  - QMC5883L (magnetometer, internal)
- **SPI buses**: 3
  - MPU-6000 (IMU)
  - MS5611 (barometer)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "airmind/mindpx-v2",
  "chip_model": "STM32F42",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20948",
      "MPU-6000"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "HMC5883L",
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/mindpx-v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "airmind/mindpx-v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mindpx/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "airmind/mindpx-v2",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/mindpx/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "airmind/mindpx-v2",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/mindpx-v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "airmind/mindpx-v2",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: mindpx-v2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/mindpx-v2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "airmind/mindpx-v2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### ark/cannode

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F412

### Sensors

- **IMU**: TODO: confirm which is installed — ADIS16507, ICM-20948, ICM-42688P (SPI)
- **Barometer**: SPL06 (I2C, external)
- **Magnetometer**: TODO: confirm which is installed — HMC5883L (I2C, external), IST8308 (I2C, external), IST8310 (I2C, external), IIS2MDC (I2C, external), LIS3MDL (I2C, external), QMC5883L (I2C, external), RM3100 (I2C, external), AK09916 (I2C, external)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: 1
  - SPL06 (barometer, external)
  - HMC5883L (magnetometer, external)
  - IST8308 (magnetometer, external)
  - IST8310 (magnetometer, external)
  - IIS2MDC (magnetometer, external)
  - LIS3MDL (magnetometer, external)
  - QMC5883L (magnetometer, external)
  - RM3100 (magnetometer, external)
  - AK09916 (magnetometer, external)
- **SPI buses**: 2
  - ICM-42688P (IMU)
- **CAN buses**: 1
- **USB**: TODO: confirm USB connector type
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "ark/cannode",
  "chip_model": "STM32F412",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 2,
  "num_can_buses": 1,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ADIS16507",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "SPL06"
    ],
    "mag": [
      "HMC5883L",
      "IST8308",
      "IST8310",
      "IIS2MDC",
      "LIS3MDL",
      "QMC5883L",
      "RM3100",
      "AK09916"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/cannode/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "ark/cannode",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 8 outputs are in 3 groups:

- Outputs 1-3 in group1 (Timer2)
- Outputs 4-7 in group2 (Timer3)
- Output 8 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/ark_cannode/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "ark/cannode",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/ark_cannode/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "ark/cannode",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/cannode/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "ark/cannode",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "ark/cannode",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### ark/fmu-v6x

**Doc:** [https://docs.px4.io/main/en/flight_controller/ark_v6x](https://docs.px4.io/main/en/flight_controller/ark_v6x)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: IIM-42652 (SPI, variant ARKV6X000), ICM-42688P (SPI, variant ARKV6X000)
- **Barometer**: BMP388 (I2C, internal)
- **Magnetometer**: BMM150 (I2C, internal)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMP388 (barometer, internal)
  - BMM150 (magnetometer, internal)
- **SPI buses**: 5
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "ark/fmu-v6x",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ADIS16507",
      "ICM-42688P",
      "IIM-42652"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [
      "BMM150",
      "IST8310",
      "LIS3MDL",
      "RM3100",
      "IIS2MDC"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6x/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "ark/fmu-v6x",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `PX4IO/RC` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/ark_v6x/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "ark/fmu-v6x",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC",
    "side": "IO"
  },
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/ark_v6x/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "ark/fmu-v6x",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6x/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "ark/fmu-v6x",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6x board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6x/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "ark/fmu-v6x",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO/RC | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### ark/fpv

**Doc:** [https://docs.px4.io/main/en/flight_controller/ark_fpv](https://docs.px4.io/main/en/flight_controller/ark_fpv)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: BMP388 (I2C, bus 2, internal)
- **Magnetometer**: TODO: confirm which is installed — BMM150, HMC5883L, QMC5883L, IST8308, IST8310, LIS3MDL, LSM303AGR, RM3100, IIS2MDC (I2C, bus 4, internal)

### Interfaces

- **PWM outputs**: 9 (FMU)
- **Serial ports**: 7
- **I2C ports**: 3
  - BMP388 (barometer, internal, bus 2)
  - IIS2MDC (magnetometer, internal, bus 4)
- **SPI buses**: 2
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "ark/fpv",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 9,
  "fmu_servo_outputs": 9,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 3,
  "num_spi_buses": 2,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [],
    "baro": [
      "BMP388"
    ],
    "mag": [
      "BMM150",
      "HMC5883L",
      "QMC5883L",
      "IST8308",
      "IST8310",
      "LIS3MDL",
      "LSM303AGR",
      "RM3100",
      "IIS2MDC"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fpv/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "ark/fpv",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "analog"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Output 9 does not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer8)
- Output 9 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/ark_fpv/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "ark/fpv",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/ark_fpv/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "ark/fpv",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fpv/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "ark/fpv",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fpv board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fpv/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "ark/fpv",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | - |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | RC | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Output 9 does not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer8)
- Output 9 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### ark/pi6x

**Doc:** [https://docs.px4.io/main/en/flight_controller/ark_pi6x](https://docs.px4.io/main/en/flight_controller/ark_pi6x)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: ICM-42688P (SPI, variant ARKPI6X000)
- **Barometer**: BMP388 (I2C, bus 4, internal)
- **Magnetometer**: MMC5983MA (I2C, bus 4, internal)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 3
  - BMP388 (barometer, internal, bus 4)
  - MMC5983MA (magnetometer, internal, bus 4)
- **SPI buses**: 4
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "ark/pi6x",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 3,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [
      "MMC5983MA",
      "IIS2MDC"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/pi6x/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "ark/pi6x",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/ark_pi6x/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "ark/pi6x",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/ark_pi6x/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "ark/pi6x",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/pi6x/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "ark/pi6x",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: pi6x board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/pi6x/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "ark/pi6x",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART3 | /dev/ttyS1 | Debug Console | - |
| UART4 | /dev/ttyS2 | TELEM4 | - |
| UART5 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | TELEM1 | Yes |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### atl/mantis-edu

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)

### Sensors

- **IMU**: ICM-20602 (SPI)
- **Barometer**: MPC2520 (I2C, bus 4, internal)
- **Magnetometer**: IST8310 (I2C, bus 2, internal)

### Interfaces

- **Serial ports**: 8
- **I2C ports**: 2
  - MPC2520 (barometer, internal, bus 4)
  - IST8310 (magnetometer, internal, bus 2)
- **SPI buses**: 2
  - ICM-20602 (IMU)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "atl/mantis-edu",
  "chip_model": "STM32F765",
  "has_io_board": false,
  "total_outputs": null,
  "fmu_servo_outputs": 0,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602"
    ],
    "baro": [
      "MPC2520"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/mantis-edu/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "atl/mantis-edu",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "analog"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

PWM output information could not be determined from source.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/atl_mantis-edu/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "atl/mantis-edu",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/atl_mantis-edu/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "atl/mantis-edu",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/mantis-edu/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "atl/mantis-edu",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: mantis-edu board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/mantis-edu/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "atl/mantis-edu",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 |  | - |
| USART2 | /dev/ttyS1 |  | Yes |
| USART3 | /dev/ttyS2 |  | Yes |
| UART4 | /dev/ttyS3 |  | - |
| UART5 | /dev/ttyS4 |  | - |
| USART6 | /dev/ttyS5 |  | Yes |
| UART7 | /dev/ttyS6 |  | - |
| UART8 | /dev/ttyS7 | Debug Console | - |
```

---

### auterion/fmu-v6s

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: BMI088 (SPI)
- **Barometer**: BMP388 (I2C, bus 4, internal)
- **Magnetometer**: IST8310 (I2C, bus 1, external); BMM150 (I2C, internal, variant V6S013), BMM150 (I2C, internal, variant V6S015), BMM350 (I2C, internal, other variants)

### Interfaces

- **PWM outputs**: 9 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - BMP388 (barometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 2
  - BMI088 (IMU)
- **CAN buses**: 1
- **USB**: TODO: confirm USB connector type
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "auterion/fmu-v6s",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 10,
  "fmu_servo_outputs": 9,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 2,
  "num_can_buses": 1,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "BMI088"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [
      "BMM150",
      "BMM350",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6s/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "auterion/fmu-v6s",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Output 9 does not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 10 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer3)
- Output 9 in group3 (Timer5)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/auterion_fmu-v6s/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "auterion/fmu-v6s",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART5",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/auterion_fmu-v6s/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "auterion/fmu-v6s",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS6",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6s/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "auterion/fmu-v6s",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6s board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6s/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "auterion/fmu-v6s",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | Yes |
| USART2 | /dev/ttyS1 | Debug Console | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | - |
| UART4 | /dev/ttyS3 |  | - |
| UART5 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 |  | - |
| UART8 | /dev/ttyS6 | GPS1 | - |
```

---

### auterion/fmu-v6x

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-42688P (SPI)
- **Barometer**: BMP388 (I2C, internal), BMP388 (I2C, external)
- **Magnetometer**: TODO: confirm which is installed — BMM150 (I2C, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMP388 (barometer, internal)
  - BMP388 (barometer, external)
  - BMM150 (magnetometer, internal)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - ICM-20602 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "auterion/fmu-v6x",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-42688P"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [
      "BMM150",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6x/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "auterion/fmu-v6x",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `PX4IO/RC` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/auterion_fmu-v6x/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "auterion/fmu-v6x",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC",
    "side": "IO"
  },
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/auterion_fmu-v6x/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "auterion/fmu-v6x",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6x/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "auterion/fmu-v6x",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6x board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6x/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "auterion/fmu-v6x",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO/RC | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

---

### av/x-v1

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F777 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)

### Sensors

- **IMU**: ICM-20948
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: LSM303AGR (SPI)

### Interfaces

- **PWM outputs**: 9 (FMU)
- **Serial ports**: 8
- **I2C ports**: 3
- **SPI buses**: 4
  - LSM303AGR (magnetometer)
- **CAN buses**: 1
- **USB**: TODO: confirm USB connector type
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "av/x-v1",
  "chip_model": "STM32F777",
  "has_io_board": false,
  "total_outputs": 9,
  "fmu_servo_outputs": 9,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ICM-20948"
    ],
    "baro": [],
    "mag": [
      "LSM303AGR"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x-v1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "av/x-v1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 9 outputs are in 6 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 5 in group2 (Timer2)
- Output 6 in group3 (Timer8)
- Output 7 in group4 (Timer11)
- Output 8 in group5 (Timer10)
- Output 9 in group6 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/av_x-v1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "av/x-v1",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/av_x-v1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "av/x-v1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS6",
        "uart": "UART7"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x-v1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "av/x-v1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x-v1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x-v1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "av/x-v1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | TELEM3 | - |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| UART5 | /dev/ttyS4 |  | - |
| USART6 | /dev/ttyS5 |  | - |
| UART7 | /dev/ttyS6 | GPS1 | - |
| UART8 | /dev/ttyS7 | Debug Console | - |
```

---

### bitcraze/crazyflie

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F405 (32-bit Arm® Cortex®-M4, 168 MHz, 1MB flash, 192KB RAM)

### Sensors

- **IMU**: MPU-9250
- **Barometer**: LPS25H (I2C, bus 3, internal)
- **Magnetometer**: AK8963 (I2C, bus 3, internal)

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: 2
  - LPS25H (barometer, internal, bus 3)
  - AK8963 (magnetometer, internal, bus 3)
- **SPI buses**: 1
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "bitcraze/crazyflie",
  "chip_model": "STM32F405",
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 1,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "MPU-9250"
    ],
    "baro": [
      "LPS25H"
    ],
    "mag": [
      "AK8963"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/crazyflie/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "bitcraze/crazyflie",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 4 outputs are in 2 groups:

- Outputs 1-3 in group1 (Timer2)
- Output 4 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/bitcraze_crazyflie/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "bitcraze/crazyflie",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/bitcraze_crazyflie/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "bitcraze/crazyflie",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/crazyflie/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "bitcraze/crazyflie",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: crazyflie board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/crazyflie/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "bitcraze/crazyflie",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### bitcraze/crazyflie21

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F405 (32-bit Arm® Cortex®-M4, 168 MHz, 1MB flash, 192KB RAM)

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: BMP388 (I2C, internal)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: 2
  - BMP388 (barometer, internal)
- **SPI buses**: 1
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "bitcraze/crazyflie21",
  "chip_model": "STM32F405",
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 1,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [],
    "baro": [
      "BMP388"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/crazyflie21/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "bitcraze/crazyflie21",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 4 outputs are in 2 groups:

- Outputs 1-3 in group1 (Timer2)
- Output 4 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/bitcraze_crazyflie21/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "bitcraze/crazyflie21",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/bitcraze_crazyflie21/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "bitcraze/crazyflie21",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/crazyflie21/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "bitcraze/crazyflie21",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: crazyflie21 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/crazyflie21/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "bitcraze/crazyflie21",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### corvon/743v1

**Doc:** [https://docs.px4.io/main/en/flight_controller/corvon_743v1](https://docs.px4.io/main/en/flight_controller/corvon_743v1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), BMI270 (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: IST8310 (I2C, internal)

### Interfaces

- **PWM outputs**: 10 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - IST8310 (magnetometer, internal)
- **SPI buses**: 1
  - BMI088 (IMU)
  - BMI270 (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "corvon/743v1",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 10,
  "fmu_servo_outputs": 10,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 1,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "BMI270"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/743v1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "corvon/743v1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10 support [DShot](../peripherals/dshot.md).
- Outputs 1-9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 10 supports Bidirectional DShot output only (no eRPM capture).

The 10 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-10 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/corvon_743v1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "corvon/743v1",
  "modules": {
    "rc_input": true,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/corvon_743v1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "corvon/743v1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "USART3"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/743v1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "corvon/743v1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: 743v1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/743v1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "corvon/743v1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | GPS1 | - |
| UART4 | /dev/ttyS3 | TELEM3 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | URT6 | - |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10 support [DShot](../peripherals/dshot.md).
- Outputs 1-9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 10 supports Bidirectional DShot output only (no eRPM capture).

The 10 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-10 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cuav/7-nano

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20948, IIM-42652 (SPI)
- **Barometer**: TODO: confirm which is installed — BMP581 (SPI), ICP-20100 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — IIS2MDC (I2C, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 7
- **I2C ports**: 4
  - ICP-20100 (barometer, internal)
  - IIS2MDC (magnetometer, internal)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - IIM-42652 (IMU)
  - BMI088 (IMU)
  - BMP581 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "cuav/7-nano",
  "chip_model": "STM32H753",
  "has_io_board": false,
  "total_outputs": 14,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20948",
      "IIM-42652"
    ],
    "baro": [
      "BMP581",
      "ICP-20100"
    ],
    "mag": [
      "IIS2MDC",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/7-nano/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cuav/7-nano",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "analog"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 14 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer1)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cuav_7-nano/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cuav/7-nano",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cuav_7-nano/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cuav/7-nano",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS6",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/7-nano/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cuav/7-nano",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: 7-nano board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/7-nano/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cuav/7-nano",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART5 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 |  | - |
| UART7 | /dev/ttyS5 | TELEM1 | Yes |
| UART8 | /dev/ttyS6 | GPS2 | - |
```

---

### cuav/fmu-v6x

**Doc:** [https://docs.px4.io/main/en/flight_controller/cuav_pixhawk_v6x](https://docs.px4.io/main/en/flight_controller/cuav_pixhawk_v6x)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20948, ICM-45686 (SPI), IIM-42652 (SPI)
- **Barometer**: TODO: confirm which is installed — BMP581 (I2C, bus 2, external), ICP-20100 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — RM3100 (I2C, bus 4, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMP581 (barometer, external, bus 2)
  - ICP-20100 (barometer, internal)
  - RM3100 (magnetometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - IIM-42652 (IMU)
  - BMI088 (IMU)
  - ICM-45686 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cuav/fmu-v6x",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20948",
      "ICM-45686",
      "IIM-42652"
    ],
    "baro": [
      "BMP581",
      "ICP-20100"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6x/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cuav/fmu-v6x",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cuav_pixhawk_v6x/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cuav/fmu-v6x",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cuav_pixhawk_v6x/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cuav/fmu-v6x",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6x/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cuav/fmu-v6x",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6x board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6x/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cuav/fmu-v6x",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cuav/nora

**Doc:** [https://docs.px4.io/main/en/flight_controller/cuav_nora](https://docs.px4.io/main/en/flight_controller/cuav_nora)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20649, ICM-20689 (SPI), ICM-20689 (SPI), ICM-20948, ICM-42688P (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: TODO: confirm which is installed — RM3100 (SPI), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 14 (FMU)
- **Serial ports**: 6
- **I2C ports**: 4
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ICM-20689 (IMU)
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - ICM-20689 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
  - RM3100 (magnetometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cuav/nora",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 14,
  "fmu_servo_outputs": 14,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20649",
      "ICM-20689",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/nora/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cuav/nora",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-12 support [DShot](../peripherals/dshot.md).
- Outputs 13-14 do not support DShot.
- Outputs 1-7, 9-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-12 in group3 (Timer1)
- Outputs 13-14 in group4 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cuav_nora/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cuav/nora",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cuav_nora/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cuav/nora",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/nora/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cuav/nora",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: nora board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/nora/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cuav/nora",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| UART4 | /dev/ttyS2 | GPS2 | - |
| USART6 | /dev/ttyS3 | TELEM2 | Yes |
| UART7 | /dev/ttyS4 | Debug Console | - |
| UART8 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-12 support [DShot](../peripherals/dshot.md).
- Outputs 13-14 do not support DShot.
- Outputs 1-7, 9-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-12 in group3 (Timer1)
- Outputs 13-14 in group4 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cuav/x25-evo

**Doc:** [https://docs.px4.io/main/en/flight_controller/cuav_x25-evo](https://docs.px4.io/main/en/flight_controller/cuav_x25-evo)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20948, IIM-42652 (SPI)
- **Barometer**: TODO: confirm which is installed — BMP581 (SPI), ICP-20100 (I2C, bus 4, internal)
- **Magnetometer**: TODO: confirm which is installed — RM3100 (SPI), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 4
  - ICP-20100 (barometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - IIM-42652 (IMU)
  - BMP581 (barometer)
  - RM3100 (magnetometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cuav/x25-evo",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 16,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20948",
      "IIM-42652"
    ],
    "baro": [
      "BMP581",
      "ICP-20100"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x25-evo/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cuav/x25-evo",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 16 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 16 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cuav_x25-evo/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cuav/x25-evo",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cuav_x25-evo/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cuav/x25-evo",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x25-evo/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cuav/x25-evo",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x25-evo board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x25-evo/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cuav/x25-evo",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | GPS2 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 |  | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 16 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 16 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cuav/x25-super

**Doc:** [https://docs.px4.io/main/en/flight_controller/cuav_x25-super](https://docs.px4.io/main/en/flight_controller/cuav_x25-super)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: IIM-42652 (SPI)
- **Barometer**: TODO: confirm which is installed — BMP581 (SPI), ICP-20100 (I2C, bus 4, internal)
- **Magnetometer**: TODO: confirm which is installed — RM3100 (SPI), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 4
  - ICP-20100 (barometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - IIM-42652 (IMU)
  - BMP581 (barometer)
  - RM3100 (magnetometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cuav/x25-super",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 16,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "IIM-42652"
    ],
    "baro": [
      "BMP581",
      "ICP-20100"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x25-super/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cuav/x25-super",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 16 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 16 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cuav_x25-super/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cuav/x25-super",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cuav_x25-super/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cuav/x25-super",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x25-super/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cuav/x25-super",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x25-super board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x25-super/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cuav/x25-super",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | GPS2 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 |  | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 16 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 16 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cuav/x7pro

**Doc:** [https://docs.px4.io/main/en/flight_controller/cuav_x7](https://docs.px4.io/main/en/flight_controller/cuav_x7)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ADIS16470 (SPI), BMI088 (SPI), ICM-20649, ICM-20689 (SPI), ICM-20948, ICM-42688P (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: TODO: confirm which is installed — RM3100 (SPI), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 14 (FMU)
- **Serial ports**: 6
- **I2C ports**: 4
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ADIS16470 (IMU)
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - ICM-20689 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
  - RM3100 (magnetometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cuav/x7pro",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 14,
  "fmu_servo_outputs": 14,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ADIS16470",
      "BMI088",
      "ICM-20649",
      "ICM-20689",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x7pro/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cuav/x7pro",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-12 support [DShot](../peripherals/dshot.md).
- Outputs 13-14 do not support DShot.
- Outputs 1-7, 9-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-12 in group3 (Timer1)
- Outputs 13-14 in group4 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cuav_x7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cuav/x7pro",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cuav_x7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cuav/x7pro",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x7pro/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cuav/x7pro",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x7pro board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x7pro/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cuav/x7pro",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| UART4 | /dev/ttyS2 | GPS2 | - |
| USART6 | /dev/ttyS3 | TELEM2 | Yes |
| UART7 | /dev/ttyS4 | Debug Console | - |
| UART8 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-12 support [DShot](../peripherals/dshot.md).
- Outputs 13-14 do not support DShot.
- Outputs 1-7, 9-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-12 in group3 (Timer1)
- Outputs 13-14 in group4 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cubepilot/cubeorange

**Doc:** [https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orange](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orange)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), ICM-20649 (SPI), ICM-20948 (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **Serial ports**: 6
- **I2C ports**: 2
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - ICM-20948 (IMU)
  - ICM-20649 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cubepilot/cubeorange",
  "chip_model": "STM32H743",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-20649",
      "ICM-20948"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/cubeorange/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cubepilot/cubeorange",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cubepilot_cube_orange/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cubepilot/cubeorange",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS3",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/cubepilot_cube_orange/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cubepilot/cubeorange",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS5",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/cubeorange/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cubepilot/cubeorange",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: cubeorange board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/cubeorange/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cubepilot/cubeorange",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 | PX4IO | - |
| UART7 | /dev/ttyS4 |  | - |
| UART8 | /dev/ttyS5 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cubepilot/cubeorangeplus

**Doc:** [https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orangeplus](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orangeplus)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H747 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20649 (SPI), ICM-20948, ICM-42688P (SPI), ICM-45686 (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: AK09916 (I2C, internal)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **Serial ports**: 6
- **I2C ports**: 3
  - AK09916 (magnetometer, internal)
- **SPI buses**: 3
  - ICM-42688P (IMU)
  - ICM-45686 (IMU)
  - ICM-20649 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "cubepilot/cubeorangeplus",
  "chip_model": "STM32H747",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 3,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20649",
      "ICM-20948",
      "ICM-42688P",
      "ICM-45686"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "AK09916"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/cubeorangeplus/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cubepilot/cubeorangeplus",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cubepilot_cube_orangeplus/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cubepilot/cubeorangeplus",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS3",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/cubepilot_cube_orangeplus/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cubepilot/cubeorangeplus",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS5",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/cubeorangeplus/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cubepilot/cubeorangeplus",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: cubeorangeplus board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/cubeorangeplus/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cubepilot/cubeorangeplus",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 | PX4IO | - |
| UART7 | /dev/ttyS4 |  | - |
| UART8 | /dev/ttyS5 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cubepilot/cubeyellow

**Doc:** [https://docs.px4.io/main/en/flight_controller/cubepilot_cube_yellow](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_yellow)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F777 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), ICM-20649 (SPI), ICM-20948 (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **Serial ports**: 6
- **I2C ports**: 2
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - ICM-20948 (IMU)
  - ICM-20649 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "cubepilot/cubeyellow",
  "chip_model": "STM32F777",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-20649",
      "ICM-20948"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/cubeyellow/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cubepilot/cubeyellow",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-6 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cubepilot_cube_yellow/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cubepilot/cubeyellow",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS3",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/cubepilot_cube_yellow/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cubepilot/cubeyellow",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS5",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/cubeyellow/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cubepilot/cubeyellow",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: cubeyellow board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/cubeyellow/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "cubepilot/cubeyellow",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 | PX4IO | - |
| UART7 | /dev/ttyS4 | TELEM3 | - |
| UART8 | /dev/ttyS5 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-6 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### cubepilot/io-v2

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F100

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "cubepilot/io-v2",
  "chip_model": "STM32F100",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/io-v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "cubepilot/io-v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-2 in group1 (Timer2)
- Outputs 3-4 in group2 (Timer4)
- Outputs 5-8 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/cubepilot_io-v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "cubepilot/io-v2",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/cubepilot_io-v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "cubepilot/io-v2",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/io-v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "cubepilot/io-v2",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "cubepilot/io-v2",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### diatone/mamba-f405-mk2

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F405 (32-bit Arm® Cortex®-M4, 168 MHz, 1MB flash, 192KB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, MPU-6000, MPU-9250, ICM-42688P (SPI)
- **Barometer**: BMP280 (I2C, bus 2, external)
- **Magnetometer**: TODO: confirm which is installed — AK8963 (I2C, bus 2, external), HMC5883L

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: 1
  - BMP280 (barometer, external, bus 2)
  - AK8963 (magnetometer, external, bus 2)
- **SPI buses**: 3
  - ICM-42688P (IMU)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "diatone/mamba-f405-mk2",
  "chip_model": "STM32F405",
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "MPU-6000",
      "MPU-9250",
      "ICM-42688P"
    ],
    "baro": [
      "BMP280"
    ],
    "mag": [
      "AK8963",
      "HMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/mamba-f405-mk2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "diatone/mamba-f405-mk2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 4 outputs are in 2 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/diatone_mamba-f405-mk2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "diatone/mamba-f405-mk2",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/diatone_mamba-f405-mk2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "diatone/mamba-f405-mk2",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/mamba-f405-mk2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "diatone/mamba-f405-mk2",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "diatone/mamba-f405-mk2",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### espressif/esp32

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: TODO: chip model

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "espressif/esp32",
  "chip_model": null,
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/esp32/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "espressif/esp32",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 4 outputs are in 1 groups:

- Outputs 1-4 in group1 (Timer0)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/espressif_esp32/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "espressif/esp32",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/espressif_esp32/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "espressif/esp32",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/esp32/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "espressif/esp32",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: esp32 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/esp32/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "espressif/esp32",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### flywoo/gn-f405

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F405 (32-bit Arm® Cortex®-M4, 168 MHz, 1MB flash, 192KB RAM)

### Sensors

- **IMU**: MPU-6000 (SPI)
- **Barometer**: BMP280 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: 1
  - BMP280 (barometer, external)
- **SPI buses**: 3
  - MPU-6000 (IMU)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "flywoo/gn-f405",
  "chip_model": "STM32F405",
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "MPU-6000"
    ],
    "baro": [
      "BMP280"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/gn-f405/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "flywoo/gn-f405",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 4 outputs are in 2 groups:

- Outputs 3-4 in group1 (Timer2)
- Outputs 1-2 in group2 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/flywoo_gn-f405/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "flywoo/gn-f405",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/flywoo_gn-f405/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "flywoo/gn-f405",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/gn-f405/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "flywoo/gn-f405",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "flywoo/gn-f405",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### gearup/airbrainh743

**Doc:** [https://docs.px4.io/main/en/flight_controller/gearup_airbrainh743](https://docs.px4.io/main/en/flight_controller/gearup_airbrainh743)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: ICM-42688P (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — HMC5883L, IST8310, LIS3MDL, QMC5883L, IIS2MDC (I2C, internal)

### Interfaces

- **PWM outputs**: 9 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - IIS2MDC (magnetometer, internal)
- **SPI buses**: 3
  - ICM-42688P (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "gearup/airbrainh743",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 9,
  "fmu_servo_outputs": 9,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "HMC5883L",
      "IST8310",
      "LIS3MDL",
      "QMC5883L",
      "IIS2MDC"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/airbrainh743/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "gearup/airbrainh743",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Output 9 does not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-8 in group3 (Timer2)
- Output 9 in group4 (Timer5)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/gearup_airbrainh743/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "gearup/airbrainh743",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS1",
    "uart": "USART2",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/gearup_airbrainh743/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "gearup/airbrainh743",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS6",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/airbrainh743/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "gearup/airbrainh743",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "gearup/airbrainh743",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | Debug Console | - |
| USART2 | /dev/ttyS1 | RC | - |
| USART3 | /dev/ttyS2 | TELEM4 | - |
| UART4 | /dev/ttyS3 | TELEM1 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| UART7 | /dev/ttyS5 | TELEM3 | - |
| UART8 | /dev/ttyS6 | GPS1 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Output 9 does not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-8 in group3 (Timer2)
- Output 9 in group4 (Timer5)

All outputs within the same group must use the same output protocol and rate.
-->

---

### hkust/nxt-dual

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: BMI088 (SPI), BMI088 (SPI)
- **Barometer**: SPL06 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 2
  - SPL06 (barometer, external)
- **SPI buses**: 4
  - BMI088 (IMU)
  - BMI088 (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "hkust/nxt-dual",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088"
    ],
    "baro": [
      "SPL06"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/nxt-dual/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "hkust/nxt-dual",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 7-8 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/hkust_nxt-dual/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "hkust/nxt-dual",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART5",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/hkust_nxt-dual/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "hkust/nxt-dual",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS2",
        "uart": "USART3"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/nxt-dual/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "hkust/nxt-dual",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: nxt-dual board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/nxt-dual/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "hkust/nxt-dual",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | - |
| USART3 | /dev/ttyS2 | GPS2 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | RC | - |
| USART6 | /dev/ttyS5 | Debug Console | - |
| UART7 | /dev/ttyS6 | TELEM3 | - |
| UART8 | /dev/ttyS7 | TELEM4 | - |
```

---

### hkust/nxt-v1

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602, ICM-42688P (SPI)
- **Barometer**: BMP388 (I2C, internal)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 2
  - BMP388 (barometer, internal)
- **SPI buses**: 3
  - ICM-42688P (IMU)
  - BMI088 (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "hkust/nxt-v1",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-42688P"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/nxt-v1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "hkust/nxt-v1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-5 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 6 supports Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Output 8 in group3 (Timer1)
- Output 7 in group4 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/hkust_nxt-v1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "hkust/nxt-v1",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART5",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/hkust_nxt-v1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "hkust/nxt-v1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/nxt-v1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "hkust/nxt-v1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: nxt-v1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/nxt-v1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "hkust/nxt-v1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | GPS2 | - |
| USART3 | /dev/ttyS2 | TELEM1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | TELEM3 | - |
```

---

### holybro/durandal-v1

**Doc:** [https://docs.px4.io/main/en/flight_controller/durandal](https://docs.px4.io/main/en/flight_controller/durandal)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-20689 (SPI); BMI088 (SPI, variant VD000000), ICM-20602 (SPI, variant VD000001)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: IST8310 (I2C, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 13 (5 FMU + 8 IO)
- **Serial ports**: 7
- **I2C ports**: 4
  - IST8310 (magnetometer, internal)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ICM-20689 (IMU)
  - MS5611 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "holybro/durandal-v1",
  "chip_model": "STM32H743",
  "has_io_board": true,
  "total_outputs": 10,
  "fmu_servo_outputs": 5,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20689",
      "ICM-20948"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/durandal-v1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/durandal-v1",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 10 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 5 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/durandal/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/durandal-v1",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS6",
    "uart": "UART8",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/durandal/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/durandal-v1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/durandal-v1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/durandal-v1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: durandal-v1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/durandal-v1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/durandal-v1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| USART6 | /dev/ttyS4 | TELEM3 | Yes |
| UART7 | /dev/ttyS5 | Debug Console | - |
| UART8 | /dev/ttyS6 | PX4IO | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 10 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 5 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/kakutef7

**Doc:** [https://docs.px4.io/main/en/flight_controller/kakutef7](https://docs.px4.io/main/en/flight_controller/kakutef7)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F745

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20689 (SPI), MPU-6000
- **Barometer**: BMP280 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)
- **OSD**: AT7456E

### Interfaces

- **PWM outputs**: 6 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
  - BMP280 (barometer, external)
- **SPI buses**: 3
  - ICM-20689 (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "holybro/kakutef7",
  "chip_model": "STM32F745",
  "has_io_board": false,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20689",
      "MPU-6000"
    ],
    "baro": [
      "BMP280"
    ],
    "mag": [],
    "osd": [
      "AT7456E"
    ]
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/kakutef7/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/kakutef7",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 3-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Outputs 1-2, 5-6 support Bidirectional DShot output only (no eRPM capture).

The 6 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer1)
- Output 5 in group3 (Timer8)
- Output 6 in group4 (Timer5)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/kakutef7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/kakutef7",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/kakutef7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/kakutef7",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/kakutef7/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/kakutef7",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "holybro/kakutef7",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 3-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Outputs 1-2, 5-6 support Bidirectional DShot output only (no eRPM capture).

The 6 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer1)
- Output 5 in group3 (Timer8)
- Output 6 in group4 (Timer5)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/kakuteh7

**Doc:** [https://docs.px4.io/main/en/flight_controller/kakuteh7](https://docs.px4.io/main/en/flight_controller/kakuteh7)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI270, ICM-42688P (SPI), MPU-6000
- **Barometer**: SPA06 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
  - SPA06 (barometer, external)
- **SPI buses**: 3
  - ICM-42688P (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "holybro/kakuteh7",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI270",
      "ICM-42688P",
      "MPU-6000"
    ],
    "baro": [
      "SPA06"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/kakuteh7/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/kakuteh7",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/kakuteh7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/kakuteh7",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/kakuteh7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/kakuteh7",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/kakuteh7/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/kakuteh7",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: kakuteh7 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/kakuteh7/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/kakuteh7",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/kakuteh7-wing

**Doc:** [https://docs.px4.io/main/en/flight_controller/kakuteh7-wing](https://docs.px4.io/main/en/flight_controller/kakuteh7-wing)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: ICM-42688P (SPI)
- **Barometer**: TODO: confirm which is installed — BMP280, SPA06 (I2C, bus 4, internal)
- **Magnetometer**: TODO: list magnetometer(s)
- **OSD**: AT7456E (SPI)

### Interfaces

- **PWM outputs**: 14 (FMU)
- **Serial ports**: 7
- **I2C ports**: 3
  - SPA06 (barometer, internal, bus 4)
- **SPI buses**: 3
  - ICM-42688P (IMU)
  - AT7456E (OSD)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "holybro/kakuteh7-wing",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 14,
  "fmu_servo_outputs": 14,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 3,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P"
    ],
    "baro": [
      "BMP280",
      "SPA06"
    ],
    "mag": [],
    "osd": [
      "AT7456E"
    ]
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/kakuteh7-wing/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/kakuteh7-wing",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "analog"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8, 11-14 support [DShot](../peripherals/dshot.md).
- Outputs 9-10 do not support DShot.
- Outputs 1-5, 7-8, 11-14 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 6 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 6 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer5)
- Outputs 9-10 in group4 (Timer15)
- Outputs 11-13 in group5 (Timer3)
- Output 14 in group6 (Timer2)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): PPM, SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/kakuteh7-wing/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/kakuteh7-wing",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": true
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/kakuteh7-wing/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/kakuteh7-wing",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/kakuteh7-wing/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/kakuteh7-wing",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: kakuteh7-wing board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/kakuteh7-wing/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/kakuteh7-wing",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | GPS2 | - |
| USART3 | /dev/ttyS2 | TELEM1 | - |
| UART5 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | TELEM3 | Yes |
| UART8 | /dev/ttyS6 | Debug Console | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8, 11-14 support [DShot](../peripherals/dshot.md).
- Outputs 9-10 do not support DShot.
- Outputs 1-5, 7-8, 11-14 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 6 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 6 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer5)
- Outputs 9-10 in group4 (Timer15)
- Outputs 11-13 in group5 (Timer3)
- Output 14 in group6 (Timer2)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/kakuteh7dualimu

**Doc:** [https://docs.px4.io/main/en/flight_controller/kakuteh7v2](https://docs.px4.io/main/en/flight_controller/kakuteh7v2)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-42688P (SPI), ICM-45686 (SPI)
- **Barometer**: ICP-20100 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)
- **OSD**: AT7456E

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
  - ICP-20100 (barometer, external)
- **SPI buses**: 3
  - ICM-45686 (IMU)
  - ICM-42688P (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "holybro/kakuteh7dualimu",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P",
      "ICM-45686"
    ],
    "baro": [
      "ICP-20100"
    ],
    "mag": [],
    "osd": [
      "AT7456E"
    ]
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/kakuteh7dualimu/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/kakuteh7dualimu",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): PPM, SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/kakuteh7v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/kakuteh7dualimu",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": true
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/kakuteh7v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/kakuteh7dualimu",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/kakuteh7dualimu/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/kakuteh7dualimu",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: kakuteh7dualimu board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/kakuteh7dualimu/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/kakuteh7dualimu",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | TELEM3 | - |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | TELEM4 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/kakuteh7mini

**Doc:** [https://docs.px4.io/main/en/flight_controller/kakuteh7mini](https://docs.px4.io/main/en/flight_controller/kakuteh7mini)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI270, ICM-42688P (SPI), MPU-6000
- **Barometer**: SPA06 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
  - SPA06 (barometer, external)
- **SPI buses**: 3
  - ICM-42688P (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "holybro/kakuteh7mini",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI270",
      "ICM-42688P",
      "MPU-6000"
    ],
    "baro": [
      "SPA06"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/kakuteh7mini/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/kakuteh7mini",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/kakuteh7mini/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/kakuteh7mini",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/kakuteh7mini/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/kakuteh7mini",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/kakuteh7mini/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/kakuteh7mini",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: kakuteh7mini board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/kakuteh7mini/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/kakuteh7mini",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/kakuteh7v2

**Doc:** [https://docs.px4.io/main/en/flight_controller/kakuteh7v2](https://docs.px4.io/main/en/flight_controller/kakuteh7v2)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI270, ICM-42688P (SPI), MPU-6000
- **Barometer**: SPA06 (I2C, external)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
  - SPA06 (barometer, external)
- **SPI buses**: 3
  - ICM-42688P (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "holybro/kakuteh7v2",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI270",
      "ICM-42688P",
      "MPU-6000"
    ],
    "baro": [
      "SPA06"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/kakuteh7v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/kakuteh7v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/kakuteh7v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/kakuteh7v2",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/kakuteh7v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/kakuteh7v2",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/kakuteh7v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/kakuteh7v2",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: kakuteh7v2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/kakuteh7v2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/kakuteh7v2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer5)
- Outputs 7-8 in group4 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### holybro/pix32v5

**Doc:** [https://docs.px4.io/main/en/flight_controller/holybro_pix32_v5](https://docs.px4.io/main/en/flight_controller/holybro_pix32_v5)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI055 (SPI), ICM-20602 (SPI), ICM-20689 (SPI), ICM-20948
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: IST8310 (I2C, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 7
- **I2C ports**: 4
  - IST8310 (magnetometer, internal)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ICM-20602 (IMU)
  - ICM-20689 (IMU)
  - BMI055 (IMU)
  - MS5611 (barometer)
- **CAN buses**: 3
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "holybro/pix32v5",
  "chip_model": "STM32F765",
  "has_io_board": true,
  "total_outputs": 11,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 3,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI055",
      "ICM-20602",
      "ICM-20689",
      "ICM-20948"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/pix32v5/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "holybro/pix32v5",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 11 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-8 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 11 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/holybro_pix32_v5/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "holybro/pix32v5",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS6",
    "uart": "UART8",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/holybro_pix32_v5/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "holybro/pix32v5",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/pix32v5/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "holybro/pix32v5",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: pix32v5 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/pix32v5/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "holybro/pix32v5",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| USART6 | /dev/ttyS4 | TELEM3 | Yes |
| UART7 | /dev/ttyS5 | Debug Console | - |
| UART8 | /dev/ttyS6 | PX4IO | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 11 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-8 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 11 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### matek/gnss-m9n-f4

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F405 (32-bit Arm® Cortex®-M4, 168 MHz, 1MB flash, 192KB RAM)

### Sensors

- **IMU**: ICM-20602 (SPI)
- **Barometer**: DPS310 (I2C, external)
- **Magnetometer**: RM3100 (SPI)

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: 1
  - DPS310 (barometer, external)
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - RM3100 (magnetometer)
- **CAN buses**: 1
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "matek/gnss-m9n-f4",
  "chip_model": "STM32F405",
  "has_io_board": false,
  "total_outputs": null,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "RM3100"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/gnss-m9n-f4/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "matek/gnss-m9n-f4",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 4 outputs are in 2 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/matek_gnss-m9n-f4/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "matek/gnss-m9n-f4",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/matek_gnss-m9n-f4/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "matek/gnss-m9n-f4",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/gnss-m9n-f4/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "matek/gnss-m9n-f4",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "matek/gnss-m9n-f4",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### matek/h743

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, ICM-42688P (SPI), MPU-6000
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: QMC5883L (I2C, external)

### Interfaces

- **PWM outputs**: 12 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - QMC5883L (magnetometer, external)
- **SPI buses**: 4
  - ICM-42688P (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "matek/h743",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 12,
  "fmu_servo_outputs": 12,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-42688P",
      "MPU-6000"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "matek/h743",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 12 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10 support [DShot](../peripherals/dshot.md).
- Outputs 11-12 do not support DShot.
- Outputs 1-9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 10 supports Bidirectional DShot output only (no eRPM capture).

The 12 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-6 in group2 (Timer5)
- Outputs 7-10 in group3 (Timer4)
- Outputs 11-12 in group4 (Timer15)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/matek_h743/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "matek/h743",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/matek_h743/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "matek/h743",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "matek/h743",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "matek/h743",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | Debug Console | - |
| USART2 | /dev/ttyS1 | GPS1 | - |
| USART3 | /dev/ttyS2 | TELEM1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 |  | - |
| UART7 | /dev/ttyS5 | TELEM3 | Yes |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

---

### matek/h743-mini

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), MPU-6000 (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: QMC5883L (I2C, external)

### Interfaces

- **PWM outputs**: 12 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - QMC5883L (magnetometer, external)
- **SPI buses**: 4
  - ICM-20602 (IMU)
  - MPU-6000 (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "matek/h743-mini",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 12,
  "fmu_servo_outputs": 12,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "MPU-6000"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743-mini/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "matek/h743-mini",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 12 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10 support [DShot](../peripherals/dshot.md).
- Outputs 11-12 do not support DShot.
- Outputs 1-9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 10 supports Bidirectional DShot output only (no eRPM capture).

The 12 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-6 in group2 (Timer5)
- Outputs 7-10 in group3 (Timer4)
- Outputs 11-12 in group4 (Timer15)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/matek_h743-mini/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "matek/h743-mini",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/matek_h743-mini/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "matek/h743-mini",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743-mini/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "matek/h743-mini",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743-mini board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743-mini/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "matek/h743-mini",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | Debug Console | - |
| USART2 | /dev/ttyS1 | GPS1 | - |
| USART3 | /dev/ttyS2 | TELEM1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 |  | - |
| UART7 | /dev/ttyS5 | TELEM3 | Yes |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

---

### matek/h743-slim

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), ICM-42688P, MPU-6000 (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: QMC5883L (I2C, external)

### Interfaces

- **PWM outputs**: 12 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - QMC5883L (magnetometer, external)
- **SPI buses**: 4
  - MPU-6000 (IMU)
  - ICM-20602 (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "matek/h743-slim",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 12,
  "fmu_servo_outputs": 12,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-42688P",
      "MPU-6000"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743-slim/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "matek/h743-slim",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 12 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10 support [DShot](../peripherals/dshot.md).
- Outputs 11-12 do not support DShot.
- Outputs 1-9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 10 supports Bidirectional DShot output only (no eRPM capture).

The 12 outputs are in 4 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-6 in group2 (Timer5)
- Outputs 7-10 in group3 (Timer4)
- Outputs 11-12 in group4 (Timer15)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/matek_h743-slim/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "matek/h743-slim",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/matek_h743-slim/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "matek/h743-slim",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743-slim/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "matek/h743-slim",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743-slim board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743-slim/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "matek/h743-slim",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | Debug Console | - |
| USART2 | /dev/ttyS1 | GPS1 | - |
| USART3 | /dev/ttyS2 | TELEM1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 |  | - |
| UART7 | /dev/ttyS5 | TELEM3 | Yes |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

---

### micoair/h743

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), BMI270 (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: IST8310 (I2C, internal)

### Interfaces

- **PWM outputs**: 10 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - IST8310 (magnetometer, internal)
- **SPI buses**: 1
  - BMI088 (IMU)
  - BMI270 (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "micoair/h743",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 10,
  "fmu_servo_outputs": 10,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 1,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "BMI270"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "micoair/h743",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10 support [DShot](../peripherals/dshot.md).
- Outputs 1-9 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 10 supports Bidirectional DShot output only (no eRPM capture).

The 10 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-10 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/micoair_h743/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "micoair/h743",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/micoair_h743/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "micoair/h743",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "USART3"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "micoair/h743",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "micoair/h743",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | GPS1 | - |
| UART4 | /dev/ttyS3 | TELEM3 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | URT6 | - |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

---

### micoair/h743-aio

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), BMI270 (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 9 (FMU)
- **Serial ports**: 7
- **I2C ports**: 2
  - DPS310 (barometer, internal)
- **SPI buses**: 1
  - BMI088 (IMU)
  - BMI270 (IMU)
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "micoair/h743-aio",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 9,
  "fmu_servo_outputs": 9,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 1,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "BMI270"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743-aio/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "micoair/h743-aio",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-9 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/micoair_h743-aio/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "micoair/h743-aio",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/micoair_h743-aio/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "micoair/h743-aio",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "USART3"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743-aio/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "micoair/h743-aio",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743-aio board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743-aio/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "micoair/h743-aio",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | TELEM2 | - |
| USART3 | /dev/ttyS2 | GPS1 | - |
| UART4 | /dev/ttyS3 | TELEM3 | - |
| USART6 | /dev/ttyS4 | RC | - |
| UART7 | /dev/ttyS5 | URT6 | - |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

---

### micoair/h743-lite

**Doc:** [https://docs.px4.io/main/en/flight_controller/micoair743-lite](https://docs.px4.io/main/en/flight_controller/micoair743-lite)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: ICM-45686 (SPI)
- **Barometer**: SPA06 (I2C, bus 2, external)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 14 (FMU)
- **Serial ports**: 8
- **I2C ports**: 2
  - SPA06 (barometer, external, bus 2)
- **SPI buses**: 1
  - ICM-45686 (IMU)
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "micoair/h743-lite",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 14,
  "fmu_servo_outputs": 14,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 1,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-45686"
    ],
    "baro": [
      "SPA06"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743-lite/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "micoair/h743-lite",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8, 11-12 support [DShot](../peripherals/dshot.md).
- Outputs 9-10, 13-14 do not support DShot.
- Outputs 1-8, 11-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 14 outputs are in 5 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer3)
- Outputs 11-12 in group3 (Timer4)
- Outputs 13-14 in group4 (Timer12)
- Outputs 9-10 in group5 (Timer15)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/micoair743-lite/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "micoair/h743-lite",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/micoair743-lite/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "micoair/h743-lite",
  "source": {
    "gps_ports": [
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      },
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "USART3"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743-lite/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "micoair/h743-lite",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743-lite board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743-lite/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "micoair/h743-lite",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | GPS2 | - |
| USART3 | /dev/ttyS2 | GPS1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | TELEM3 | - |
| USART6 | /dev/ttyS5 | RC | - |
| UART7 | /dev/ttyS6 | URT6 | - |
| UART8 | /dev/ttyS7 | TELEM4 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8, 11-12 support [DShot](../peripherals/dshot.md).
- Outputs 9-10, 13-14 do not support DShot.
- Outputs 1-8, 11-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 14 outputs are in 5 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer3)
- Outputs 11-12 in group3 (Timer4)
- Outputs 13-14 in group4 (Timer12)
- Outputs 9-10 in group5 (Timer15)

All outputs within the same group must use the same output protocol and rate.
-->

---

### micoair/h743-v2

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), BMI270 (SPI)
- **Barometer**: SPL06 (I2C, internal)
- **Magnetometer**: QMC5883L (I2C, internal)

### Interfaces

- **PWM outputs**: 10 (FMU)
- **Serial ports**: 8
- **I2C ports**: 2
  - SPL06 (barometer, internal)
  - QMC5883L (magnetometer, internal)
- **SPI buses**: 2
  - BMI088 (IMU)
  - BMI270 (IMU)
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "micoair/h743-v2",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 10,
  "fmu_servo_outputs": 10,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "BMI270"
    ],
    "baro": [
      "SPL06"
    ],
    "mag": [
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h743-v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "micoair/h743-v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 10 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 9-10 do not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 10 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer3)
- Outputs 7-8 in group3 (Timer4)
- Outputs 9-10 in group4 (Timer15)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/micoair_h743-v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "micoair/h743-v2",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/micoair_h743-v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "micoair/h743-v2",
  "source": {
    "gps_ports": [
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      },
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "USART3"
      }
    ],
    "has_pps_capture": true,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h743-v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "micoair/h743-v2",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h743-v2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h743-v2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "micoair/h743-v2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM1 | - |
| USART2 | /dev/ttyS1 | GPS2 | - |
| USART3 | /dev/ttyS2 | GPS1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | TELEM3 | - |
| USART6 | /dev/ttyS5 | RC | - |
| UART7 | /dev/ttyS6 | URT6 | - |
| UART8 | /dev/ttyS7 | TELEM4 | - |
```

---

### modalai/fc-v1

**Doc:** [https://docs.px4.io/main/en/flight_controller/modalai_fc_v1](https://docs.px4.io/main/en/flight_controller/modalai_fc_v1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948, ICM-42688P (SPI)
- **Barometer**: BMP388 (I2C, internal)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMP388 (barometer, internal)
- **SPI buses**: 4
  - ICM-20602 (IMU)
  - ICM-42688P (IMU)
  - BMI088 (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "modalai/fc-v1",
  "chip_model": "STM32F765",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fc-v1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "modalai/fc-v1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/modalai_fc_v1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "modalai/fc-v1",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/modalai_fc_v1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "modalai/fc-v1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fc-v1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "modalai/fc-v1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fc-v1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fc-v1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "modalai/fc-v1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| UART5 | /dev/ttyS4 | TELEM2 | Yes |
| USART6 | /dev/ttyS5 |  | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-7 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-8 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### modalai/fc-v2

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-42688P (SPI), ICM-42688P (SPI)
- **Barometer**: ICP-20100 (I2C, internal)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - ICP-20100 (barometer, internal)
- **SPI buses**: 4
  - ICM-42688P (IMU)
  - ICM-42688P (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "modalai/fc-v2",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P"
    ],
    "baro": [
      "ICP-20100"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fc-v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "modalai/fc-v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/modalai_fc-v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "modalai/fc-v2",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/modalai_fc-v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "modalai/fc-v2",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fc-v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "modalai/fc-v2",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fc-v2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fc-v2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "modalai/fc-v2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 |  | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 |  | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 |  | - |
```

---

### modalai/voxl2-io

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F100

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "modalai/voxl2-io",
  "chip_model": "STM32F100",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/voxl2-io/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "modalai/voxl2-io",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-2 in group1 (Timer2)
- Outputs 5-8 in group2 (Timer3)
- Outputs 3-4 in group3 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/modalai_voxl2-io/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "modalai/voxl2-io",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/modalai_voxl2-io/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "modalai/voxl2-io",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/voxl2-io/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "modalai/voxl2-io",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "modalai/voxl2-io",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### mro/ctrl-zero-classic

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 12 (FMU)
- **Serial ports**: 6
- **I2C ports**: 2
- **SPI buses**: 4
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/ctrl-zero-classic",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 12,
  "fmu_servo_outputs": 12,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ctrl-zero-classic/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/ctrl-zero-classic",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 12 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-10, 12 support [DShot](../peripherals/dshot.md).
- Output 11 does not support DShot.
- Outputs 1-6, 8-10, 12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 7 supports Bidirectional DShot output only (no eRPM capture).

The 12 outputs are in 5 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-7 in group2 (Timer4)
- Outputs 8-10 in group3 (Timer2)
- Output 11 in group4 (Timer15)
- Output 12 in group5 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_ctrl-zero-classic/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/ctrl-zero-classic",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/mro_ctrl-zero-classic/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/ctrl-zero-classic",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS4",
        "uart": "UART7"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ctrl-zero-classic/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/ctrl-zero-classic",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ctrl-zero-classic board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ctrl-zero-classic/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/ctrl-zero-classic",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM4 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | GPS1 | - |
| UART7 | /dev/ttyS4 | GPS2 | - |
| UART8 | /dev/ttyS5 | TELEM3 | - |
```

---

### mro/ctrl-zero-f7

**Doc:** [https://docs.px4.io/main/en/flight_controller/mro_control_zero_f7](https://docs.px4.io/main/en/flight_controller/mro_control_zero_f7)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F777 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/ctrl-zero-f7",
  "chip_model": "STM32F777",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ctrl-zero-f7/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/ctrl-zero-f7",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_control_zero_f7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/ctrl-zero-f7",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/mro_control_zero_f7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/ctrl-zero-f7",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ctrl-zero-f7/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/ctrl-zero-f7",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ctrl-zero-f7 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ctrl-zero-f7/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/ctrl-zero-f7",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 |  | - |
| UART7 | /dev/ttyS4 | Debug Console | - |
| UART8 | /dev/ttyS5 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### mro/ctrl-zero-f7-oem

**Doc:** [https://docs.px4.io/main/en/flight_controller/mro_control_zero_f7](https://docs.px4.io/main/en/flight_controller/mro_control_zero_f7)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F777 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 3
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/ctrl-zero-f7-oem",
  "chip_model": "STM32F777",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ctrl-zero-f7-oem/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/ctrl-zero-f7-oem",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_control_zero_f7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/ctrl-zero-f7-oem",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/mro_control_zero_f7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/ctrl-zero-f7-oem",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ctrl-zero-f7-oem/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/ctrl-zero-f7-oem",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ctrl-zero-f7-oem board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ctrl-zero-f7-oem/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/ctrl-zero-f7-oem",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 |  | - |
| UART7 | /dev/ttyS4 | TELEM3 | - |
| UART8 | /dev/ttyS5 | Debug Console | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### mro/ctrl-zero-h7

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 1
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/ctrl-zero-h7",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ctrl-zero-h7/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/ctrl-zero-h7",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_ctrl-zero-h7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/ctrl-zero-h7",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/mro_ctrl-zero-h7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/ctrl-zero-h7",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ctrl-zero-h7/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/ctrl-zero-h7",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ctrl-zero-h7 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ctrl-zero-h7/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/ctrl-zero-h7",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 |  | - |
| UART7 | /dev/ttyS4 | TELEM3 | - |
| UART8 | /dev/ttyS5 | Debug Console | - |
```

---

### mro/ctrl-zero-h7-oem

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 6
- **I2C ports**: 3
- **SPI buses**: 3
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/ctrl-zero-h7-oem",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ctrl-zero-h7-oem/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/ctrl-zero-h7-oem",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_ctrl-zero-h7-oem/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/ctrl-zero-h7-oem",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/mro_ctrl-zero-h7-oem/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/ctrl-zero-h7-oem",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ctrl-zero-h7-oem/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/ctrl-zero-h7-oem",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ctrl-zero-h7-oem board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ctrl-zero-h7-oem/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/ctrl-zero-h7-oem",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART2 | /dev/ttyS0 | TELEM1 | Yes |
| USART3 | /dev/ttyS1 | TELEM2 | Yes |
| UART4 | /dev/ttyS2 | GPS1 | - |
| USART6 | /dev/ttyS3 |  | - |
| UART7 | /dev/ttyS4 | TELEM3 | - |
| UART8 | /dev/ttyS5 | Debug Console | - |
```

---

### mro/pixracerpro

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixracer](https://docs.px4.io/main/en/flight_controller/pixracer)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948 (SPI)
- **Barometer**: DPS310 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 7
- **I2C ports**: 1
- **SPI buses**: 4
  - ICM-20602 (IMU)
  - BMI088 (IMU)
  - ICM-20948 (IMU)
  - DPS310 (barometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/pixracerpro",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 4,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/pixracerpro/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/pixracerpro",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixracer/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/pixracerpro",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/pixracer/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/pixracerpro",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS6",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/pixracerpro/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/pixracerpro",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: pixracerpro board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/pixracerpro/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/pixracerpro",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | TELEM3 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 |  | - |
| UART7 | /dev/ttyS5 | TELEM4 | - |
| UART8 | /dev/ttyS6 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### mro/x21

**Doc:** [https://docs.px4.io/main/en/flight_controller/mro_x2.1](https://docs.px4.io/main/en/flight_controller/mro_x2.1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), ICM-20948, MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **I2C ports**: 1
- **SPI buses**: 2
  - ICM-20602 (IMU)
  - MPU-9250 (IMU)
  - MS5611 (barometer)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/x21",
  "chip_model": "STM32F42",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-20948",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x21/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/x21",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_x2.1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/x21",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/mro_x2.1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/x21",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x21/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/x21",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x21 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x21/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/x21",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-6 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### mro/x21-777

**Doc:** [https://docs.px4.io/main/en/flight_controller/mro_x2.1](https://docs.px4.io/main/en/flight_controller/mro_x2.1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F777 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), ICM-20948, MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **Serial ports**: 7
- **I2C ports**: 1
- **SPI buses**: 2
  - ICM-20602 (IMU)
  - MPU-9250 (IMU)
  - MS5611 (barometer)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "mro/x21-777",
  "chip_model": "STM32F777",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-20948",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x21-777/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "mro/x21-777",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-6 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/mro_x2.1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "mro/x21-777",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/mro_x2.1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "mro/x21-777",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x21-777/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "mro/x21-777",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x21-777 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x21-777/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "mro/x21-777",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 |  | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | GPS1 | - |
| USART6 | /dev/ttyS4 | PX4IO | - |
| UART7 | /dev/ttyS5 | Debug Console | - |
| UART8 | /dev/ttyS6 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-6 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### narinfc/h7

**Doc:** [https://docs.px4.io/main/en/flight_controller/vololand_narinfc_h7](https://docs.px4.io/main/en/flight_controller/vololand_narinfc_h7)
**Documented:** No

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ADIS16470 (SPI), BMI088 (SPI), ICM-20649, ICM-20689 (SPI), ICM-20948, ICM-42688P (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: TODO: confirm which is installed — RM3100 (SPI), RM3100 (SPI), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 14 (FMU)
- **Serial ports**: 6
- **I2C ports**: 4
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ADIS16470 (IMU)
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - ICM-20689 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
  - RM3100 (magnetometer)
  - RM3100 (magnetometer)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "narinfc/h7",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 14,
  "fmu_servo_outputs": 14,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ADIS16470",
      "BMI088",
      "ICM-20649",
      "ICM-20689",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h7/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "narinfc/h7",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 14 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-12 support [DShot](../peripherals/dshot.md).
- Outputs 13-14 do not support DShot.
- Outputs 1-7, 9-12 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 8 supports Bidirectional DShot output only (no eRPM capture).

The 14 outputs are in 4 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-8 in group2 (Timer4)
- Outputs 9-12 in group3 (Timer1)
- Outputs 13-14 in group4 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/vololand_narinfc_h7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "narinfc/h7",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/vololand_narinfc_h7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "narinfc/h7",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS2",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h7/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "narinfc/h7",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h7 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h7/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "narinfc/h7",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| UART4 | /dev/ttyS2 | GPS2 | - |
| USART6 | /dev/ttyS3 | TELEM2 | Yes |
| UART7 | /dev/ttyS4 | Debug Console | - |
| UART8 | /dev/ttyS5 |  | - |
```

---

### nxp/fmuk66-e

**Doc:** [https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66](https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: TODO: chip model

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20948, ICM-42688P (SPI)
- **Barometer**: TODO: confirm which is installed — MPL3115A2, BMP280 (I2C, internal)
- **Magnetometer**: BMM150 (I2C, internal)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: TODO: number of I2C ports
  - BMP280 (barometer, internal)
  - BMM150 (magnetometer, internal)
- **SPI buses**: TODO: number of SPI buses
  - BMI088 (IMU)
  - ICM-42688P (IMU)
- **USB**: TODO: confirm USB connector type
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "nxp/fmuk66-e",
  "chip_model": null,
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "MPL3115A2",
      "BMP280"
    ],
    "mag": [
      "BMM150"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmuk66-e/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "nxp/fmuk66-e",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 2 groups:

- Outputs 1-6 in group1 (FTM0)
- Outputs 7-8 in group2 (FTM3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/nxp_rddrone_fmuk66/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "nxp/fmuk66-e",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/nxp_rddrone_fmuk66/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "nxp/fmuk66-e",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmuk66-e/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "nxp/fmuk66-e",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmuk66-e board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmuk66-e/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "nxp/fmuk66-e",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (FTM0)
- Outputs 5-6 in group2 (FTM3)

All outputs within the same group must use the same output protocol and rate.
-->

---

### nxp/fmuk66-v3

**Doc:** [https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66](https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: TODO: chip model

### Sensors

- **IMU**: ICM-20948
- **Barometer**: TODO: confirm which is installed — MPL3115A2 (I2C, internal), BMP280 (I2C, internal)
- **Magnetometer**: BMM150 (I2C, internal)

### Interfaces

- **PWM outputs**: 6 (FMU)
- **I2C ports**: TODO: number of I2C ports
  - BMP280 (barometer, internal)
  - MPL3115A2 (barometer, internal)
  - BMM150 (magnetometer, internal)
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "nxp/fmuk66-v3",
  "chip_model": null,
  "has_io_board": false,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ICM-20948"
    ],
    "baro": [
      "MPL3115A2",
      "BMP280"
    ],
    "mag": [
      "BMM150"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmuk66-v3/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "nxp/fmuk66-v3",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (FTM0)
- Outputs 5-6 in group2 (FTM3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/nxp_rddrone_fmuk66/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "nxp/fmuk66-v3",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/nxp_rddrone_fmuk66/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "nxp/fmuk66-v3",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmuk66-v3/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "nxp/fmuk66-v3",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmuk66-v3 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmuk66-v3/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "nxp/fmuk66-v3",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (FTM0)
- Outputs 5-6 in group2 (FTM3)

All outputs within the same group must use the same output protocol and rate.
-->

---

### nxp/mr-canhubk3

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: TODO: chip model

### Sensors

- **IMU**: ICM-42688P (variant MR-CANHUBK3-ADAP), ICM-20649 (variant MR-CANHUBK3-ADAP), ICM-20649 (other variants), ICM-42688P (other variants)
- **Barometer**: BMP388 (I2C, external)
- **Magnetometer**: LIS3MDL (I2C, bus 2, external); BMM150 (I2C, external, variant MR-CANHUBK3-ADAP), IST8310 (I2C, bus 1, external, variant MR-CANHUBK3-ADAP), BMM150 (I2C, external, other variants), IST8310 (I2C, bus 2, external, other variants)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: TODO: number of I2C ports
  - BMP388 (barometer, external)
  - LIS3MDL (magnetometer, external, bus 2)
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "nxp/mr-canhubk3",
  "chip_model": null,
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ICM-20649",
      "ICM-42688P"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [
      "BMM150",
      "IST8310",
      "LIS3MDL"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/mr-canhubk3/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "nxp/mr-canhubk3",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 8 groups:

- Output 1 in group1 (EMIOS0_Channel0)
- Output 2 in group2 (EMIOS0_Channel1)
- Output 3 in group3 (EMIOS0_Channel2)
- Output 4 in group4 (EMIOS0_Channel3)
- Output 5 in group5 (EMIOS0_Channel4)
- Output 6 in group6 (EMIOS0_Channel5)
- Output 7 in group7 (EMIOS0_Channel6)
- Output 8 in group8 (EMIOS0_Channel7)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/nxp_mr-canhubk3/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "nxp/mr-canhubk3",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/nxp_mr-canhubk3/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "nxp/mr-canhubk3",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/mr-canhubk3/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "nxp/mr-canhubk3",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: mr-canhubk3 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/mr-canhubk3/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "nxp/mr-canhubk3",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### nxp/mr-tropic

**Doc:** [https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176](https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: MIMXRT1064

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20948, ICM-42688P, ICM-45686 (SPI)
- **Barometer**: TODO: confirm which is installed — BMP388 (I2C, bus 4, internal), ICP-20100, MS5611
- **Magnetometer**: TODO: confirm which is installed — BMM350 (I2C, bus 4, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 7
- **I2C ports**: 3
  - BMP388 (barometer, internal, bus 4)
  - BMM350 (magnetometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 2
  - ICM-45686 (IMU)
  - BMI088 (IMU)
- **USB**: TODO: confirm USB connector type
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "nxp/mr-tropic",
  "chip_model": "MIMXRT1064",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20948",
      "ICM-42688P",
      "ICM-45686"
    ],
    "baro": [
      "BMP388",
      "ICP-20100",
      "MS5611"
    ],
    "mag": [
      "BMM350",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/mr-tropic/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "nxp/mr-tropic",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 2 groups:

- Outputs 1-4 in group1 (PWM2)
- Outputs 5-8 in group2 (PWM4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/nxp_mr_vmu_rt1176/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "nxp/mr-tropic",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS6",
    "uart": "LPUART8",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/nxp_mr_vmu_rt1176/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "nxp/mr-tropic",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS1",
        "uart": "LPUART3"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/mr-tropic/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "nxp/mr-tropic",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: mr-tropic board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/mr-tropic/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "nxp/mr-tropic",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| LPUART2 | /dev/ttyS0 |  | - |
| LPUART3 | /dev/ttyS1 | GPS1 | Yes |
| LPUART4 | /dev/ttyS2 | TELEM1 | Yes |
| LPUART5 | /dev/ttyS3 | TELEM3 | Yes |
| LPUART6 | /dev/ttyS4 | TELEM2 | - |
| LPUART7 | /dev/ttyS5 |  | - |
| LPUART8 | /dev/ttyS6 | RC | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-3, 5-6 support [DShot](../peripherals/dshot.md).
- Output 4 does not support DShot.
- Outputs 1-3, 5-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (PWM2)
- Outputs 5-6 in group2 (PWM4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### nxp/tropic-community

**Doc:** [https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176](https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: MIMXRT1062 (32-bit Arm® Cortex®-M7, 600 MHz, 8MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20948, ICM-42688P (SPI), ICM-45686
- **Barometer**: TODO: confirm which is installed — BMP388 (I2C, bus 4, internal), ICP-20100, MS5611
- **Magnetometer**: TODO: confirm which is installed — BMM150 (I2C, bus 4, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 6 (FMU)
- **Serial ports**: 5
- **I2C ports**: 2
  - BMP388 (barometer, internal, bus 4)
  - BMM150 (magnetometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 2
  - ICM-42688P (IMU)
  - BMI088 (IMU)
- **USB**: TODO: confirm USB connector type
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "nxp/tropic-community",
  "chip_model": "MIMXRT1062",
  "has_io_board": false,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20948",
      "ICM-42688P",
      "ICM-45686"
    ],
    "baro": [
      "BMP388",
      "ICP-20100",
      "MS5611"
    ],
    "mag": [
      "BMM150",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/tropic-community/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "nxp/tropic-community",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-3, 5-6 support [DShot](../peripherals/dshot.md).
- Output 4 does not support DShot.
- Outputs 1-3, 5-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (PWM2)
- Outputs 5-6 in group2 (PWM4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/nxp_mr_vmu_rt1176/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "nxp/tropic-community",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/nxp_mr_vmu_rt1176/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "nxp/tropic-community",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS1",
        "uart": "LPUART3"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/tropic-community/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "nxp/tropic-community",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: tropic-community board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/tropic-community/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "nxp/tropic-community",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| LPUART2 | /dev/ttyS0 |  | - |
| LPUART3 | /dev/ttyS1 | GPS1 | Yes |
| LPUART4 | /dev/ttyS2 | TELEM1 | - |
| LPUART5 | /dev/ttyS3 | TELEM2 | - |
| LPUART8 | /dev/ttyS4 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-3, 5-6 support [DShot](../peripherals/dshot.md).
- Output 4 does not support DShot.
- Outputs 1-3, 5-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (PWM2)
- Outputs 5-6 in group2 (PWM4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### nxp/ucans32k146

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: TODO: chip model

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: TODO: confirm which is installed — HMC5883L (I2C, external), IST8308 (I2C, external), IST8310 (I2C, external), IIS2MDC (I2C, external), LIS3MDL (I2C, external), QMC5883L (I2C, external), RM3100 (I2C, external)

### Interfaces

- **PWM outputs**: 2 (FMU)
- **I2C ports**: TODO: number of I2C ports
  - HMC5883L (magnetometer, external)
  - IST8308 (magnetometer, external)
  - IST8310 (magnetometer, external)
  - IIS2MDC (magnetometer, external)
  - LIS3MDL (magnetometer, external)
  - QMC5883L (magnetometer, external)
  - RM3100 (magnetometer, external)
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "nxp/ucans32k146",
  "chip_model": null,
  "has_io_board": false,
  "total_outputs": 2,
  "fmu_servo_outputs": 2,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [
      "HMC5883L",
      "IST8308",
      "IST8310",
      "IIS2MDC",
      "LIS3MDL",
      "QMC5883L",
      "RM3100"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ucans32k146/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "nxp/ucans32k146",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 2 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 2 outputs are in 2 groups:

- Output 2 in group1 (FTM1)
- Output 1 in group2 (FTM2)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/nxp_ucans32k146/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "nxp/ucans32k146",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/nxp_ucans32k146/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "nxp/ucans32k146",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ucans32k146/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "nxp/ucans32k146",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "nxp/ucans32k146",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### omnibus/f4sd

**Doc:** [https://docs.px4.io/main/en/flight_controller/omnibus_f4_sd](https://docs.px4.io/main/en/flight_controller/omnibus_f4_sd)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F405 (32-bit Arm® Cortex®-M4, 168 MHz, 1MB flash, 192KB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, MPU-6000
- **Barometer**: BMP280 (SPI)
- **Magnetometer**: HMC5883L

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: 1
- **SPI buses**: 3
  - BMP280 (barometer)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "omnibus/f4sd",
  "chip_model": "STM32F405",
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "MPU-6000"
    ],
    "baro": [
      "BMP280"
    ],
    "mag": [
      "HMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/f4sd/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "omnibus/f4sd",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 4 outputs are in 2 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/omnibus_f4_sd/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "omnibus/f4sd",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/omnibus_f4_sd/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "omnibus/f4sd",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/f4sd/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "omnibus/f4sd",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "omnibus/f4sd",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 4 outputs are in 2 groups:

- Outputs 1-2 in group1 (Timer3)
- Outputs 3-4 in group2 (Timer2)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v2

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk](https://docs.px4.io/main/en/flight_controller/pixhawk)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — MPU-6000 (SPI), MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: HMC5883L (I2C, internal), HMC5883L (SPI)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **I2C ports**: 2
  - HMC5883L (magnetometer, internal)
- **SPI buses**: 3
  - MPU-6000 (IMU)
  - MPU-9250 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
  - HMC5883L (magnetometer)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v2",
  "chip_model": "STM32F42",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "MPU-6000",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "HMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v2",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/pixhawk/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v2",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v2",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v3

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk-2](https://docs.px4.io/main/en/flight_controller/pixhawk-2)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20948, MPU-6000 (SPI), MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI), MS5611 (SPI)
- **Magnetometer**: HMC5883L (I2C, internal), HMC5883L (SPI)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **I2C ports**: 2
  - HMC5883L (magnetometer, internal)
- **SPI buses**: 3
  - MPU-6000 (IMU)
  - MPU-9250 (IMU)
  - MS5611 (barometer)
  - MS5611 (barometer)
  - HMC5883L (magnetometer)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v3",
  "chip_model": "STM32F42",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20948",
      "MPU-6000",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "HMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v3/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v3",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk-2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v3",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/pixhawk-2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v3",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v3/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v3",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v3 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v3/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v3",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) ([Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry) not supported).

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v4

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixracer](https://docs.px4.io/main/en/flight_controller/pixracer)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, ICM-20948, MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: LIS3MDL (SPI)

### Interfaces

- **I2C ports**: 1
- **SPI buses**: 3
  - MPU-9250 (IMU)
  - MS5611 (barometer)
  - LIS3MDL (magnetometer)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v4",
  "chip_model": "STM32F42",
  "has_io_board": false,
  "total_outputs": 6,
  "fmu_servo_outputs": 0,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-20948",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "LIS3MDL"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v4/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v4",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

PWM output information could not be determined from source.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixracer/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v4",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixracer/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v4",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v4/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v4",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v4 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v4/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v4",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v4pro

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk3_pro](https://docs.px4.io/main/en/flight_controller/pixhawk3_pro)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F46
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, ICM-20948, MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: LIS3MDL (SPI)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **I2C ports**: 2
- **SPI buses**: 4
  - MPU-9250 (IMU)
  - MS5611 (barometer)
  - LIS3MDL (magnetometer)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v4pro",
  "chip_model": "STM32F46",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-20948",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "LIS3MDL"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v4pro/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v4pro",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "analog"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk3_pro/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v4pro",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixhawk3_pro/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v4pro",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v4pro/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v4pro",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v4pro board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v4pro/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v4pro",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v5

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk4](https://docs.px4.io/main/en/flight_controller/pixhawk4)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-20689 (SPI), BMI055 (SPI); ICM-42688P (SPI, variant V5005002), ICM-42688P (SPI, variant V5006002), ICM-20602 (SPI, other variants)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: IST8310 (I2C, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 7
- **I2C ports**: 4
  - IST8310 (magnetometer, internal)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ICM-20689 (IMU)
  - BMI055 (IMU)
  - MS5611 (barometer)
- **CAN buses**: 3
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v5",
  "chip_model": "STM32F765",
  "has_io_board": true,
  "total_outputs": 11,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 3,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI055",
      "ICM-20602",
      "ICM-20689",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v5/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v5",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 11 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-8 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 11 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk4/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v5",
  "modules": {
    "rc_input": true,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS6",
    "uart": "UART8",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixhawk4/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v5",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v5/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v5",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v5 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v5/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v5",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | TELEM4 | - |
| USART6 | /dev/ttyS4 |  | Yes |
| UART7 | /dev/ttyS5 | Debug Console | - |
| UART8 | /dev/ttyS6 | PX4IO | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 11 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-4 support [DShot](../peripherals/dshot.md).
- Outputs 5-8 do not support DShot.
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 11 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v5x

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk5x](https://docs.px4.io/main/en/flight_controller/pixhawk5x)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-42688P (SPI), ICM-20602 (SPI); BMI088 (SPI, variant V5X000), BMI088 (SPI, variant V5X001), ICM-20649 (SPI, other variants)
- **Barometer**: BMP388 (I2C, internal, variant V5X000), BMP388 (I2C, bus 2, external, other variants)
- **Magnetometer**: BMM150 (I2C, internal), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMM150 (magnetometer, internal)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
  - ICM-42688P (IMU)
  - ICM-20602 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v5x",
  "chip_model": "STM32F765",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ADIS16507",
      "BMI088",
      "ICM-20602",
      "ICM-20649",
      "ICM-20948",
      "ICM-42688P",
      "IIM-42652"
    ],
    "baro": [
      "BMP388",
      "MS5611"
    ],
    "mag": [
      "BMM150",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v5x/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v5x",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `PX4IO/RC` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk5x/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v5x",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC",
    "side": "IO"
  },
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixhawk5x/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v5x",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v5x/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v5x",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v5x board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v5x/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v5x",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | Yes |
| USART6 | /dev/ttyS5 | PX4IO/RC | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v6c

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk6c](https://docs.px4.io/main/en/flight_controller/pixhawk6c)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-42688P (SPI); BMI088 (SPI, variant V6C000002), BMI088 (SPI, variant V6C002002), BMI055 (SPI, other variants)
- **Barometer**: MS5611 (I2C, bus 4, external)
- **Magnetometer**: IST8310 (I2C, bus 4, external), IST8310 (I2C, bus 1, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 7
- **I2C ports**: 3
  - MS5611 (barometer, external, bus 4)
  - IST8310 (magnetometer, external, bus 4)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 2
  - ICM-42688P (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v6c",
  "chip_model": "STM32H743",
  "has_io_board": true,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 3,
  "num_spi_buses": 2,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI055",
      "BMI088",
      "ICM-42688P"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6c/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v6c",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-5 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 6 supports Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer5)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk6c/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v6c",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/pixhawk6c/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v6c",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS6",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6c/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v6c",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6c board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6c/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v6c",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | - |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART5 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 | PX4IO | - |
| UART7 | /dev/ttyS5 | TELEM1 | Yes |
| UART8 | /dev/ttyS6 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-5 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Output 6 supports Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer5)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v6u

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk6x_pro](https://docs.px4.io/main/en/flight_controller/pixhawk6x_pro)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20602 (SPI), ICM-20948
- **Barometer**: TODO: confirm which is installed — BMP388 (I2C, internal), MS5611 (I2C, external)
- **Magnetometer**: BMM150 (I2C, internal)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMP388 (barometer, internal)
  - MS5611 (barometer, external)
  - BMM150 (magnetometer, internal)
- **SPI buses**: 5
  - ICM-20602 (IMU)
  - BMI088 (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v6u",
  "chip_model": "STM32H753",
  "has_io_board": false,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "ICM-20948"
    ],
    "baro": [
      "BMP388",
      "MS5611"
    ],
    "mag": [
      "BMM150"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6u/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v6u",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk6x_pro/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v6u",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixhawk6x_pro/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v6u",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6u/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v6u",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6u board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6u/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v6u",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 |  | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 |  | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (MAIN).

Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v6x

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk6x](https://docs.px4.io/main/en/flight_controller/pixhawk6x)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ADIS16470 (SPI, variant V6X006), IIM-42652 (SPI, variant V6X006), ICM-45686 (SPI, variant V6X006), ICM-45686 (SPI, other variants), BMI088 (SPI, other variants), ICM-42688P (SPI, other variants), ICM-20649 (SPI, other variants), ICM-20649 (SPI, variant V6X004), ICM-42670P (SPI, variant V6X004), BMI088 (SPI, variant V6X010), ICM-42688P (SPI, variant V6X010), ICM-42670P (SPI, variant V6X003)
- **Barometer**: ICP-20100 (I2C, internal, variant V6X006), BMP388 (I2C, internal, other variants), BMP388 (I2C, external, other variants), ICP-20100 (I2C, internal, variant V6X001), ICP-20100 (I2C, external, variant V6X001), ICP-20100 (I2C, internal, variant V6X008)
- **Magnetometer**: IST8310 (I2C, bus 1, external); BMM150 (I2C, internal, other variants), RM3100 (I2C, bus 4, internal, variant V6X001)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 5
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v6x",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ADIS16470",
      "BMI088",
      "ICM-20649",
      "ICM-42670P",
      "ICM-42688P",
      "ICM-45686",
      "IIM-42652"
    ],
    "baro": [
      "BMP388",
      "ICP-20100"
    ],
    "mag": [
      "RM3100",
      "BMM150",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6x/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v6x",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `PX4IO/RC` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk6x/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v6x",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC",
    "side": "IO"
  },
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixhawk6x/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v6x",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6x/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v6x",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6x board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6x/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v6x",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO/RC | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/fmu-v6xrt

**Doc:** [https://docs.px4.io/main/en/flight_controller/pixhawk6x-rt](https://docs.px4.io/main/en/flight_controller/pixhawk6x-rt)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: MIMXRT1176
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-42688P (SPI, variant V6XRT000), BMI088 (SPI, variant V6XRT000), ICM-42688P (SPI, variant V6XRT001), BMI088 (SPI, variant V6XRT001), ICM-45686 (SPI, variant V6XRT002), BMI088 (SPI, variant V6XRT002)
- **Barometer**: BMP388 (I2C, bus 3, internal), BMP388 (I2C, bus 2, external)
- **Magnetometer**: IST8310 (I2C, bus 1, external); BMM150 (I2C, internal, variant V6XRT000), BMM350 (I2C, internal, variant V6XRT001), BMM350 (I2C, internal, variant V6XRT002)

### Interfaces

- **PWM outputs**: 20 (12 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - BMP388 (barometer, internal, bus 3)
  - BMP388 (barometer, external, bus 2)
  - IST8310 (magnetometer, external, bus 1)
- **SPI buses**: 4
- **USB**: TODO: confirm USB connector type
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "px4/fmu-v6xrt",
  "chip_model": "MIMXRT1176",
  "has_io_board": true,
  "total_outputs": 12,
  "fmu_servo_outputs": 12,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 4,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ADIS16470",
      "BMI088",
      "ICM-20602",
      "ICM-20649",
      "ICM-20948",
      "ICM-42670P",
      "ICM-42688P",
      "ICM-45686",
      "IIM-42652"
    ],
    "baro": [
      "BMP388",
      "ICP-20100",
      "MS5611"
    ],
    "mag": [
      "BMM350",
      "LIS2MDL",
      "BMM150",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fmu-v6xrt/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/fmu-v6xrt",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 12 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 9-12 do not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 12 outputs are in 4 groups:

- Outputs 1-3 in group1 (PWM1)
- Outputs 4-7 in group2 (PWM2)
- Outputs 8-10 in group3 (PWM3)
- Outputs 11-12 in group4 (PWM4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `PX4IO/RC` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/pixhawk6x-rt/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/fmu-v6xrt",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "LPUART6",
    "label": "PX4IO/RC",
    "side": "IO"
  },
  "io_serial": {
    "device": "/dev/ttyS4",
    "uart": "LPUART6",
    "label": "PX4IO/RC"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/pixhawk6x-rt/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/fmu-v6xrt",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS1",
        "uart": "LPUART3"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS3",
        "uart": "LPUART5"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fmu-v6xrt/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/fmu-v6xrt",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: fmu-v6xrt board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/fmu-v6xrt/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "px4/fmu-v6xrt",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| LPUART1 | /dev/ttyS0 | Debug Console | - |
| LPUART3 | /dev/ttyS1 | GPS1 | - |
| LPUART4 | /dev/ttyS2 | TELEM1 | Yes |
| LPUART5 | /dev/ttyS3 | GPS2 | - |
| LPUART6 | /dev/ttyS4 | PX4IO/RC | - |
| LPUART8 | /dev/ttyS5 | TELEM2 | Yes |
| LPUART10 | /dev/ttyS6 | TELEM3 | Yes |
| LPUART11 | /dev/ttyS7 |  | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 12 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 9-12 do not support DShot.
- Outputs 1-8 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 12 outputs are in 4 groups:

- Outputs 1-3 in group1 (PWM1)
- Outputs 4-7 in group2 (PWM2)
- Outputs 8-10 in group3 (PWM3)
- Outputs 11-12 in group4 (PWM4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### px4/io-v2

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F100

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "px4/io-v2",
  "chip_model": "STM32F100",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/io-v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "px4/io-v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 8 outputs are in 3 groups:

- Outputs 1-2 in group1 (Timer2)
- Outputs 3-4 in group2 (Timer4)
- Outputs 5-8 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): TODO: list supported protocols
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/px4_io-v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "px4/io-v2",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/px4_io-v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "px4/io-v2",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/io-v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "px4/io-v2",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "px4/io-v2",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### radiolink/PIX6

**Doc:** [https://docs.px4.io/main/en/flight_controller/radiolink_pix6](https://docs.px4.io/main/en/flight_controller/radiolink_pix6)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-42688P (SPI), MPU-6000, MPU-9250
- **Barometer**: TODO: confirm which is installed — SPA06 (I2C, internal), MS5611
- **Magnetometer**: IST8310 (I2C, internal), IST8310 (I2C, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 6
- **I2C ports**: 2
  - SPA06 (barometer, internal)
  - IST8310 (magnetometer, internal)
  - IST8310 (magnetometer, external)
- **SPI buses**: 3
  - ICM-42688P (IMU)
  - BMI088 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "radiolink/PIX6",
  "chip_model": "STM32F765",
  "has_io_board": true,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-42688P",
      "MPU-6000",
      "MPU-9250"
    ],
    "baro": [
      "SPA06",
      "MS5611"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/PIX6/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "radiolink/PIX6",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Outputs 5-8 support Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer2)
- Outputs 7-8 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/radiolink_pix6/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "radiolink/PIX6",
  "modules": {
    "rc_input": true,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "UART8",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/radiolink_pix6/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "radiolink/PIX6",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/PIX6/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "radiolink/PIX6",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: PIX6 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/PIX6/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "radiolink/PIX6",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| USART3 | /dev/ttyS2 | TELEM2 | Yes |
| UART4 | /dev/ttyS3 | GPS2 | - |
| UART7 | /dev/ttyS4 | Debug Console | - |
| UART8 | /dev/ttyS5 | PX4IO | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-8 support [DShot](../peripherals/dshot.md).
- Outputs 1-4 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).
- Outputs 5-8 support Bidirectional DShot output only (no eRPM capture).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer2)
- Outputs 7-8 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
-->

---

### raspberrypi/pico

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: TODO: chip model

### Sensors

- **IMU**: MPU-9250
- **Barometer**: BMP280
- **Magnetometer**: HMC5883L

### Interfaces

- **PWM outputs**: 4 (FMU)
- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "raspberrypi/pico",
  "chip_model": null,
  "has_io_board": false,
  "total_outputs": 4,
  "fmu_servo_outputs": 4,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 0,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "MPU-9250"
    ],
    "baro": [
      "BMP280"
    ],
    "mag": [
      "HMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/pico/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "raspberrypi/pico",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 4 FMU PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 4 outputs are in 2 groups:

- Outputs 1-2 in group1 (Timer1)
- Outputs 3-4 in group2 (Timer2)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/raspberrypi_pico/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "raspberrypi/pico",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/raspberrypi_pico/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "raspberrypi/pico",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/pico/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "raspberrypi/pico",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card

This board does not have an SD card slot.

<!-- sd-source-data
{
  "board": "raspberrypi/pico",
  "source": {
    "has_sd_card": false
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### siyi/n7

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-20689 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: IST8310 (I2C, bus 1, external), IST8310 (I2C, bus 3, internal)

### Interfaces

- **PWM outputs**: 13 (5 FMU + 8 IO)
- **Serial ports**: 5
- **I2C ports**: 4
  - IST8310 (magnetometer, external, bus 1)
  - IST8310 (magnetometer, internal, bus 3)
- **SPI buses**: 3
  - BMI088 (IMU)
  - ICM-20689 (IMU)
  - MS5611 (barometer)
- **CAN buses**: 1
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "siyi/n7",
  "chip_model": "STM32H743",
  "has_io_board": true,
  "total_outputs": 5,
  "fmu_servo_outputs": 5,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 3,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20689"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/n7/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "siyi/n7",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina226"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 5 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 5 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 5 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/siyi_n7/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "siyi/n7",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART8",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/siyi_n7/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "siyi/n7",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/n7/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "siyi/n7",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: n7 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/n7/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "siyi/n7",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | Yes |
| UART4 | /dev/ttyS2 | TELEM4 | - |
| UART7 | /dev/ttyS3 | Debug Console | - |
| UART8 | /dev/ttyS4 | PX4IO | - |
```

---

### sky-drones/smartap-airlink

**Doc:** [https://docs.px4.io/main/en/flight_controller/airlink](https://docs.px4.io/main/en/flight_controller/airlink)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F765 (32-bit Arm® Cortex®-M7, 216 MHz, 2MB flash, 512KB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088, ICM-20602 (SPI), MPU-9250 (SPI), MPU-9250 (SPI)
- **Barometer**: TODO: confirm which is installed — MS5611 (I2C, external), BMP388 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — BMM150 (I2C, internal), HMC5883L (I2C, external), LIS3MDL (I2C, external)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - MS5611 (barometer, external)
  - BMP388 (barometer, internal)
  - BMM150 (magnetometer, internal)
  - HMC5883L (magnetometer, external)
  - LIS3MDL (magnetometer, external)
- **SPI buses**: 5
  - MPU-9250 (IMU)
  - ICM-20602 (IMU)
  - MPU-9250 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "sky-drones/smartap-airlink",
  "chip_model": "STM32F765",
  "has_io_board": true,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-20602",
      "MPU-9250"
    ],
    "baro": [
      "MS5611",
      "BMP388"
    ],
    "mag": [
      "BMM150",
      "HMC5883L",
      "LIS3MDL"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/smartap-airlink/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "sky-drones/smartap-airlink",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": "analog"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/airlink/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "sky-drones/smartap-airlink",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/airlink/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "sky-drones/smartap-airlink",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/smartap-airlink/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "sky-drones/smartap-airlink",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: smartap-airlink board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/smartap-airlink/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "sky-drones/smartap-airlink",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 |  | - |
| UART5 | /dev/ttyS4 | TELEM2 | Yes |
| USART6 | /dev/ttyS5 | PX4IO | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### spracing/h7extreme

**Doc:** [https://docs.px4.io/main/en/flight_controller/spracingh7extreme](https://docs.px4.io/main/en/flight_controller/spracingh7extreme)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, ICM-42688P (SPI), ICM-42688P (SPI), MPU-6000
- **Barometer**: BMP388 (I2C, internal)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 7
- **I2C ports**: 1
  - BMP388 (barometer, internal)
- **SPI buses**: 3
  - ICM-42688P (IMU)
  - ICM-42688P (IMU)
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "spracing/h7extreme",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "ICM-42688P",
      "MPU-6000"
    ],
    "baro": [
      "BMP388"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/h7extreme/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "spracing/h7extreme",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/spracingh7extreme/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "spracing/h7extreme",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/spracingh7extreme/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "spracing/h7extreme",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/h7extreme/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "spracing/h7extreme",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: h7extreme board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/h7extreme/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "spracing/h7extreme",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 |  | - |
| USART2 | /dev/ttyS1 |  | - |
| USART3 | /dev/ttyS2 |  | - |
| UART4 | /dev/ttyS3 |  | - |
| UART5 | /dev/ttyS4 |  | - |
| USART6 | /dev/ttyS5 |  | - |
| UART8 | /dev/ttyS6 | Debug Console | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer8)

All outputs within the same group must use the same output protocol and rate.
-->

---

### svehicle/e2

**Doc:** [https://docs.px4.io/main/en/flight_controller/svehicle_e2](https://docs.px4.io/main/en/flight_controller/svehicle_e2)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-42688P (SPI), ICM-20649 (SPI), BMI088 (SPI), ICM-45686
- **Barometer**: TODO: confirm which is installed — BMP388, MS5611, ICP-20100 (I2C, external), ICP-20100 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — QMC5883L, BMM150 (I2C, internal), RM3100 (I2C, internal)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - ICP-20100 (barometer, external)
  - ICP-20100 (barometer, internal)
  - RM3100 (magnetometer, internal)
  - BMM150 (magnetometer, internal)
- **SPI buses**: 5
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - ICM-20649 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus/CPPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "svehicle/e2",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P",
      "ICM-20649",
      "BMI088",
      "ICM-45686"
    ],
    "baro": [
      "BMP388",
      "MS5611",
      "ICP-20100"
    ],
    "mag": [
      "QMC5883L",
      "BMM150",
      "RM3100"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/e2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "svehicle/e2",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": false,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/svehicle_e2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "svehicle/e2",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/svehicle_e2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "svehicle/e2",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/e2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "svehicle/e2",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: e2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/e2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "svehicle/e2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
-->

---

### thepeach/k1

**Doc:** [https://docs.px4.io/main/en/flight_controller/thepeach_k1](https://docs.px4.io/main/en/flight_controller/thepeach_k1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 13 (5 FMU + 8 IO)
- **I2C ports**: 2
- **SPI buses**: 2
  - ICM-20602 (IMU)
  - MPU-9250 (IMU)
  - MS5611 (barometer)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "thepeach/k1",
  "chip_model": "STM32F42",
  "has_io_board": true,
  "total_outputs": 5,
  "fmu_servo_outputs": 5,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/k1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "thepeach/k1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 5 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 5 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 5 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/thepeach_k1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "thepeach/k1",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/thepeach_k1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "thepeach/k1",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/k1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "thepeach/k1",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: k1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/k1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "thepeach/k1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 5 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 5 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 5 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### thepeach/r1

**Doc:** [https://docs.px4.io/main/en/flight_controller/thepeach_r1](https://docs.px4.io/main/en/flight_controller/thepeach_r1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602 (SPI), MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 14 (6 FMU + 8 IO)
- **I2C ports**: 1
- **SPI buses**: 2
  - ICM-20602 (IMU)
  - MPU-9250 (IMU)
  - MS5611 (barometer)
- **USB**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "thepeach/r1",
  "chip_model": "STM32F42",
  "has_io_board": true,
  "total_outputs": 6,
  "fmu_servo_outputs": 6,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/r1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "thepeach/r1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, and SUMD receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/thepeach_r1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "thepeach/r1",
  "modules": {
    "rc_input": false,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/thepeach_r1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "thepeach/r1",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/r1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "thepeach/r1",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: r1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/r1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "thepeach/r1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 6 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

[DShot](../peripherals/dshot.md) is not supported.

The 6 outputs are in 2 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 5-6 in group2 (Timer4)

All outputs within the same group must use the same output protocol and rate.
-->

---

### uvify/core

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32F42

### Sensors

- **IMU**: TODO: confirm which is installed — ICM-20602, MPU-9250 (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: confirm which is installed — BMM150, IST8310, LIS3MDL (I2C, external)

### Interfaces

- **I2C ports**: 1
  - LIS3MDL (magnetometer, external)
- **SPI buses**: 2
  - MPU-9250 (IMU)
  - MS5611 (barometer)
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "uvify/core",
  "chip_model": "STM32F42",
  "has_io_board": false,
  "total_outputs": 6,
  "fmu_servo_outputs": 0,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 1,
  "num_spi_buses": 2,
  "num_can_buses": 0,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "ICM-20602",
      "MPU-9250"
    ],
    "baro": [
      "MS5611"
    ],
    "mag": [
      "BMM150",
      "IST8310",
      "LIS3MDL"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/core/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "uvify/core",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

PWM output information could not be determined from source.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers
- `TODO: PPM port label` (FMU): PPM

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/uvify_core/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "uvify/core",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `TODO: GPS port label` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/uvify_core/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "uvify/core",
  "source": {
    "gps_ports": [],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TODO: TELEM port label** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/core/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "uvify/core",
  "source": {
    "telem_ports": []
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: core board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/core/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "uvify/core",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

Serial port mapping could not be determined from source.
```

---

### x-mav/ap-h743r1

**Doc:** [https://docs.px4.io/main/en/flight_controller/x-mav_ap-h743r1](https://docs.px4.io/main/en/flight_controller/x-mav_ap-h743r1)
**Documented:** Yes

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI270 (SPI), BMI270 (SPI), ICM-42688P
- **Barometer**: TODO: confirm which is installed — DPS310, SPL06 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — IST8310, QMC5883L, QMC5883P (I2C, internal)

### Interfaces

- **PWM outputs**: 15 (7 FMU + 8 IO)
- **Serial ports**: 7
- **I2C ports**: 3
  - SPL06 (barometer, internal)
  - QMC5883P (magnetometer, internal)
- **SPI buses**: 3
  - BMI270 (IMU)
  - BMI270 (IMU)
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: DSM/SRXL2, S.Bus
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "x-mav/ap-h743r1",
  "chip_model": "STM32H743",
  "has_io_board": true,
  "total_outputs": 7,
  "fmu_servo_outputs": 7,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 3,
  "num_spi_buses": 3,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI270",
      "ICM-42688P"
    ],
    "baro": [
      "DPS310",
      "SPL06"
    ],
    "mag": [
      "IST8310",
      "QMC5883L",
      "QMC5883P"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ap-h743r1/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "x-mav/ap-h743r1",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 7 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 7 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 7 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/x-mav_ap-h743r1/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "x-mav/ap-h743r1",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS4",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/x-mav_ap-h743r1/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "x-mav/ap-h743r1",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS1",
        "uart": "USART2"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ap-h743r1/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "x-mav/ap-h743r1",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ap-h743r1 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ap-h743r1/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "x-mav/ap-h743r1",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | GPS2 | - |
| USART3 | /dev/ttyS2 | TELEM1 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| USART6 | /dev/ttyS4 | PX4IO | - |
| UART7 | /dev/ttyS5 | TELEM3 | - |
| UART8 | /dev/ttyS6 | TELEM4 | - |
```

**Existing doc section (for reference):**

<!--
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 7 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 7 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 7 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
-->

---

### x-mav/ap-h743v2

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-42688P (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — IST8310 (I2C, internal), QMC5883L
- **OSD**: AT7456E (SPI)

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - IST8310 (magnetometer, internal)
- **SPI buses**: 4
  - BMI088 (IMU)
  - ICM-42688P (IMU)
  - AT7456E (OSD)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "x-mav/ap-h743v2",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-42688P"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "IST8310",
      "QMC5883L"
    ],
    "osd": [
      "AT7456E"
    ]
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/ap-h743v2/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "x-mav/ap-h743v2",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 7-8 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/x-mav_ap-h743v2/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "x-mav/ap-h743v2",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART5",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/x-mav_ap-h743v2/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "x-mav/ap-h743v2",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/ap-h743v2/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "x-mav/ap-h743v2",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: ap-h743v2 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/ap-h743v2/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "x-mav/ap-h743v2",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | - |
| USART3 | /dev/ttyS2 | EXT2 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | RC | - |
| USART6 | /dev/ttyS5 | TELEM3 | - |
| UART7 | /dev/ttyS6 |  | - |
| UART8 | /dev/ttyS7 | TELEM4 | - |
```

---

### xc-fly/xc-slam

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-42688P (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — IST8310 (I2C, internal), QMC5883L

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - IST8310 (magnetometer, internal)
- **SPI buses**: 4
  - BMI088 (IMU)
  - ICM-42688P (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "xc-fly/xc-slam",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-42688P"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "IST8310",
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/xc-slam/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "xc-fly/xc-slam",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 7-8 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/xc-fly_xc-slam/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "xc-fly/xc-slam",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART5",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/xc-fly_xc-slam/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "xc-fly/xc-slam",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/xc-slam/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "xc-fly/xc-slam",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: xc-slam board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/xc-slam/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "xc-fly/xc-slam",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | - |
| USART3 | /dev/ttyS2 | EXT2 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | RC | - |
| USART6 | /dev/ttyS5 | TELEM3 | - |
| UART7 | /dev/ttyS6 |  | - |
| UART8 | /dev/ttyS7 | TELEM4 | - |
```

---

### xc-fly/xc-slim

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H743 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: TODO: confirm which is installed — BMI088 (SPI), ICM-42688P (SPI)
- **Barometer**: DPS310 (I2C, internal)
- **Magnetometer**: TODO: confirm which is installed — IST8310 (I2C, internal), QMC5883L

### Interfaces

- **PWM outputs**: 8 (FMU)
- **Serial ports**: 8
- **I2C ports**: 2
  - DPS310 (barometer, internal)
  - IST8310 (magnetometer, internal)
- **SPI buses**: 4
  - BMI088 (IMU)
  - ICM-42688P (IMU)
- **CAN buses**: 1
- **USB**: Yes
- **RC input**: Yes
- **Analog battery inputs**: 1
- **Additional analog inputs**: TODO: number of additional analog inputs

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes

<!-- overview-source-data
{
  "board": "xc-fly/xc-slim",
  "chip_model": "STM32H743",
  "has_io_board": false,
  "total_outputs": 8,
  "fmu_servo_outputs": 8,
  "io_outputs": 0,
  "has_sd_card": true,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 2,
  "num_spi_buses": 4,
  "num_can_buses": 1,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-42688P"
    ],
    "baro": [
      "DPS310"
    ],
    "mag": [
      "IST8310",
      "QMC5883L"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/xc-slim/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "xc-fly/xc-slim",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": false,
    "power_monitor_type": null
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 8 FMU PWM outputs (MAIN).

All outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 8 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Outputs 7-8 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `RC` (FMU): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/xc-fly_xc-slim/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "xc-fly/xc-slim",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": false,
    "has_ppm_pin": false,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS4",
    "uart": "UART5",
    "label": "RC",
    "side": "FMU"
  },
  "io_serial": null,
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

![GNSS Connection](../../assets/flight_controller/xc-fly_xc-slim/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "xc-fly/xc-slim",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3**, **TELEM4** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/xc-slim/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "xc-fly/xc-slim",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3",
      "TELEM4"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: xc-slim board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/xc-slim/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "xc-fly/xc-slim",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM1 | - |
| USART3 | /dev/ttyS2 | EXT2 | - |
| UART4 | /dev/ttyS3 | TELEM2 | - |
| UART5 | /dev/ttyS4 | RC | - |
| USART6 | /dev/ttyS5 | TELEM3 | - |
| UART7 | /dev/ttyS6 |  | - |
| UART8 | /dev/ttyS7 | TELEM4 | - |
```

---

### zeroone/x6

**Doc:** No flight controller doc found

**Proposed `specifications` section:**

```markdown
## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: ICM-45686 (SPI, variant ZeroOneX6000), BMI088 (SPI, variant ZeroOneX6000), BMI088 (SPI, variant ZeroOneX6001), BMI088 (SPI, variant //X6), BMI088 (SPI, variant PRO)
- **Barometer**: ICP-20100 (I2C, internal), ICP-20100 (I2C, external)
- **Magnetometer**: RM3100 (I2C, bus 4, internal), IST8310 (I2C, bus 1, external), IST8310 (I2C, internal)

### Interfaces

- **PWM outputs**: 16 (8 FMU + 8 IO)
- **Serial ports**: 8
- **I2C ports**: 4
  - ICP-20100 (barometer, internal)
  - ICP-20100 (barometer, external)
  - RM3100 (magnetometer, internal, bus 4)
  - IST8310 (magnetometer, external, bus 1)
  - IST8310 (magnetometer, internal)
- **SPI buses**: 5
- **CAN buses**: 2
- **USB**: Yes
- **RC input**: PPM
- **Analog battery inputs**: 2
- **Additional analog inputs**: TODO: number of additional analog inputs
- **Ethernet**: Yes

### Electrical Data

- **Input voltage**: TODO: supply voltage range

### Mechanical Data

- **Dimensions**: TODO: dimensions (mm)
- **Weight**: TODO: weight (g)

### Other

- **SD card**: Yes
- **Onboard heater**: Yes

<!-- overview-source-data
{
  "board": "zeroone/x6",
  "chip_model": "STM32H753",
  "has_io_board": true,
  "total_outputs": 9,
  "fmu_servo_outputs": 8,
  "io_outputs": 8,
  "has_sd_card": true,
  "has_ethernet": true,
  "has_heater": true,
  "num_i2c_buses": 4,
  "num_spi_buses": 5,
  "num_can_buses": 2,
  "has_usb": true,
  "sensor_drivers": {
    "imu": [
      "BMI088",
      "ICM-45686"
    ],
    "baro": [
      "ICP-20100"
    ],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->
```

**Proposed `power` section:**

```markdown
## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER 1 port label** or **TODO: POWER 2 port label** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER 1 port label`: TODO: connector type
- `TODO: POWER 2 port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/x6/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "zeroone/x6",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->
```

**Proposed `pwm_outputs` section:**

```markdown
## PWM Outputs {#pwm_outputs}

This flight controller supports up to 9 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

FMU Outputs:

- Outputs 1-6 support [DShot](../peripherals/dshot.md).
- Outputs 7-8 do not support DShot.
- Outputs 1-6 support [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 9 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer5)
- Outputs 5-6 in group2 (Timer4)
- Outputs 7-8 in group3 (Timer12)

All outputs within the same group must use the same output protocol and rate.
```

**Proposed `radio_control` section:**

```markdown
### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `TODO: RC port label` (IO): SBUS, DSM/DSMX, ST24, SUMD, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/zeroone_x6/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "zeroone/x6",
  "modules": {
    "rc_input": true,
    "common_rc": false,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": null,
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO"
  },
  "rc_ports_wizard": null
}
-->
```

**Proposed `gps_compass` section:**

```markdown
### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): 10-pin JST GH ([Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)) — GPS, compass (I2C), safety switch, buzzer, LED
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/zeroone_x6/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "zeroone/x6",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS7",
        "uart": "UART8"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": true,
    "has_buzzer": true
  },
  "gps_ports_wizard": null
}
-->
```

**Proposed `telemetry` section:**

```markdown
## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to a TELEM port — **TELEM1**, **TELEM2**, **TELEM3** (if connected to **TELEM1**, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/x6/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "zeroone/x6",
  "source": {
    "telem_ports": [
      "TELEM1",
      "TELEM2",
      "TELEM3"
    ]
  }
}
-->
```

**Proposed `sd_card` section:**

```markdown
## SD Card (Optional) {#sd_card}

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card into _TODO: x6 board name_ as shown below.

![SD Card Slot](../../assets/flight_controller/x6/sd_card_slot.jpg)

:::tip
For more information see [Basic Concepts > SD Cards (Removable Memory)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

<!-- checklist
- [ ] Confirm SD card slot location
- [ ] Add photo showing SD card insertion
-->

<!-- sd-source-data
{
  "board": "zeroone/x6",
  "source": {
    "has_sd_card": true
  }
}
-->
```

**Proposed `serial_ports` section:**

```markdown
## Serial Port Mapping

| UART | Device | Port | Flow Control |
|------|--------|------|:---:|
| USART1 | /dev/ttyS0 | GPS1 | - |
| USART2 | /dev/ttyS1 | TELEM3 | Yes |
| USART3 | /dev/ttyS2 | Debug Console | - |
| UART4 | /dev/ttyS3 | EXT2 | - |
| UART5 | /dev/ttyS4 | TELEM2 | - |
| USART6 | /dev/ttyS5 | PX4IO | - |
| UART7 | /dev/ttyS6 | TELEM1 | Yes |
| UART8 | /dev/ttyS7 | GPS2 | - |
```

---
