## Specifications {#specifications}

- **Processor**
  - **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **Sensors**
  - **IMU**: TODO: list imu(s)
  - **Barometer**: TODO: list barometer(s)
  - **Magnetometer**: TODO: list magnetometer(s)
- **Interfaces**
  - **I2C ports**: TODO: number of I2C ports
  - **SPI buses**: TODO: number of SPI buses
  - **USB**: TODO: confirm USB connector type
  - **Analog battery inputs**: 2
  - **Additional analog inputs**: TODO: number of additional analog inputs
- **Electrical Data**
  - **Operating voltage**: 7.0–55.0 (V)
  - **USB-C power input**: 4.75–5.25 (V)
  - **Servo rail**: High-voltage capable, up to 36 (V) (servo rail must be separately powered and does not power the autopilot)
  - **Power supply**: Triple redundant (POWER1, POWER2, USB-C)
- **Mechanical Data**
  - **Dimensions**: 46 × 64 × 22 (mm)
  - **Weight**: 75.0 (g)

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
  "num_spi_buses": 0,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [],
    "baro": [],
    "mag": [],
    "osd": []
  },
  "overview_wizard": {
    "min_voltage": "7.0",
    "max_voltage": "55.0",
    "usb_powers_fc": true,
    "usb_pwr_min_v": "4.75",
    "usb_pwr_max_v": "5.25",
    "has_servo_rail": true,
    "servo_rail_max_v": "36",
    "width_mm": "46",
    "length_mm": "64",
    "height_mm": "22",
    "weight_g": 75.0
  }
}
-->