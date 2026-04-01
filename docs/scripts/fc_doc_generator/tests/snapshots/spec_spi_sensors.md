## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)

### Sensors

- **IMU**: ICM-42688P (SPI)
- **Barometer**: MS5611 (SPI)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: 4
  - ICM-42688P (IMU)
  - MS5611 (barometer)
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
  "num_spi_buses": 4,
  "num_can_buses": 0,
  "has_usb": false,
  "sensor_drivers": {
    "imu": [
      "ICM-42688P"
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