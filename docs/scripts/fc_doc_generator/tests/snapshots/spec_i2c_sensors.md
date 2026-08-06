## Specifications {#specifications}

- **Processor**
  - **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **Sensors**
  - **IMU**: TODO: list imu(s)
  - **Barometer**: TODO: list barometer(s)
  - **Magnetometer**: IST8310 (I2C, internal), IST8310 (I2C, bus 1, external)
- **Interfaces**
  - **I2C ports**: 4 (3 internal, 1 external)
    - TODO: label for I2C bus 1 (I2C1, external): IST8310 (magnetometer)
    - Internal: IST8310 (magnetometer)
  - **SPI buses**: TODO: number of SPI buses
  - **USB**: TODO: confirm USB connector type
  - **Analog battery inputs**: 1
  - **Additional analog inputs**: TODO: number of additional analog inputs
- **Electrical Data**
  - **Operating voltage**: TODO: supply voltage range
- **Mechanical Data**
  - **Dimensions**: TODO: dimensions (mm)
  - **Weight**: TODO: weight (g)

<!-- overview-source-data
{
  "board": "test/fixture",
  "chip_model": "STM32H753",
  "has_io_board": false,
  "total_outputs": null,
  "fmu_servo_outputs": 0,
  "io_outputs": 0,
  "has_sd_card": false,
  "has_ethernet": false,
  "has_heater": false,
  "num_i2c_buses": 4,
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": null
}
-->