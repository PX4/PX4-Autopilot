## Specifications {#specifications}

- **Processor**
  - **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **Sensors**
  - **IMU**: TODO: list imu(s)
  - **Barometer**: TODO: list barometer(s)
  - **Magnetometer**: TODO: list magnetometer(s)
- **Interfaces**
  - **I2C ports**: 4 (3 external, 1 internal)
    - I2C1 (external, GPS1): IST8310 (magnetometer) — on GPS connector
    - I2C2 (external, POWER): INA226 (power monitor)
    - I2C3 (internal): BMP388 (barometer)
    - I2C4 (external): free (no sensor detected)
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
    "mag": [],
    "osd": []
  },
  "overview_wizard": null
}
-->