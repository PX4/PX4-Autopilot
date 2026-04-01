## Specifications {#specifications}

### Processor

- **Main FMU Processor**: STM32H753 (32-bit Arm® Cortex®-M7, 480 MHz, 2MB flash, 1MB RAM)
- **IO Processor**: STM32F100 (32-bit Arm® Cortex®-M3, 24 MHz, 8KB SRAM)

### Sensors

- **IMU**: TODO: list imu(s)
- **Barometer**: TODO: list barometer(s)
- **Magnetometer**: TODO: list magnetometer(s)

### Interfaces

- **PWM outputs**: 8 (0 FMU + 8 IO)
- **I2C ports**: TODO: number of I2C ports
- **SPI buses**: TODO: number of SPI buses
- **USB**: TODO: confirm USB connector type
- **RC input**: SBUS, DSM/DSMX, ST24, SUMD, CRSF, GHST, PPM (via IO)
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
  "has_io_board": true,
  "total_outputs": null,
  "fmu_servo_outputs": 0,
  "io_outputs": 8,
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