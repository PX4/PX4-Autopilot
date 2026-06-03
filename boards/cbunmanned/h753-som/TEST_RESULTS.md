# CBUnmanned H753-SOM Test Results

## Working

- [x] Console (UART5, PC12/PD2)
- [x] ADC (battery voltage/current)
- [x] ICM42670P IMU #1 (SPI4, CS PE4) - correct orientation
- [x] ICM42670P IMU #2 (SPI4, CS PE15) - correct orientation
- [x] LPS22HB barometer (I2C1)
- [x] USB CDC ACM / QGC auto-connect

## Not Yet Tested

### Serial Ports
- [x] TEL1 - USART2 (/dev/ttyS0) PD5 TX / PA3 RX + flow control (PA0 CTS / PD4 RTS)
- [x] GPS1 - USART3 (/dev/ttyS1) PD8 TX / PD9 RX
- [x] GPS2 - UART4 (/dev/ttyS2) PC10 TX / PC11 RX
- [x] RC   - USART6 (/dev/ttyS4) PC6 TX / PC7 RX
- [x] TEL2 - UART7 (/dev/ttyS5) PE8 TX / PE7 RX + flow control (PE10 CTS / PE9 RTS)
- [x] TEL3 - UART8 (/dev/ttyS6) PE1 TX / PE0 RX + flow control (PD14 CTS / PD15 RTS)

### Other Peripherals
- [x] PWM outputs (9 channels: TIM1 CH2/3/4, TIM2 CH3/1, TIM3 CH1/3/4, TIM4 CH2)
- [x] CAN1 (PD0 RX / PD1 TX)
- [x] CAN2 (PB5 RX / PB6 TX)
- [x] I2C3 external (PA8 SCL / PC9 SDA) - GPS1 compass
- [x] I2C4 external (PD12 SCL / PD13 SDA) - GPS2 compass
- [x] SD card (SDMMC2)
- [ ] Ethernet (DP83825IRMQR PHY) - requires separate SOM, not installed
- [x] Flash parameter storage (sectors 14-15)
- [x] LED (PB2)
- [x] Bootloader

## Known Issues

- CAN transceivers require 5V power supply; USB-only power (4.5V after diode drop) is insufficient
- Ethernet PHY is on a separate SOM; not yet tested
