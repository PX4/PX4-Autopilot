## Specifications {#specifications}

- **Processor**
  - **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **Sensors**
  - **IMU**: ICM-20649, ICM-20689, BMI088 <!-- TODO: Unnecessary driver(s) in firmware?: ICM-20948, ICM-42688P. -->
  - **Barometer**: TODO: list barometer(s)
  - **Magnetometer**: RM3100 <!-- TODO: Unnecessary driver(s) in firmware?: IST8310. -->
- **Interfaces**
  - **I2C ports**: TODO: number of I2C ports
  - **SPI buses**: 3
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
  "num_i2c_buses": 0,
  "num_spi_buses": 3,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ICM-20649",
      "ICM-20689",
      "BMI088",
      "ICM-20948",
      "ICM-42688P"
    ],
    "baro": [],
    "mag": [
      "RM3100",
      "IST8310"
    ],
    "osd": []
  },
  "overview_wizard": {
    "imu": [
      "ICM-20649",
      "ICM-20689",
      "BMI088"
    ],
    "mag": [
      "RM3100"
    ]
  }
}
-->