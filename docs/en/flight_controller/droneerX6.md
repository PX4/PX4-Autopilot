# DroneerX6 Flight Controller


The DroneerX6 is a series of flight controllers manufactured by Droneer, which is based on the open-source FMU v6X architecture and Pixhawk Autopilot Bus open source specifications.
![DroneerX6](https://github.com/Droneer-uav/Droneerx6/blob/main/DroneerX6_V1.png "DroneerX6")



## Where To Buy {#store}

[Droneer](https://www.droneer.com)

## Sensors

- IMU1-ICM45686(With vibration isolation)
- IMU2-ICM45686(With vibration isolation)
- IMU3-ICM45686(No vibration isolation)

## Microprocessor

- [STM32H743IIK6 MCU](https://www.st.com/en/microcontrollers-microprocessors/stm32h743ii.html)
  - 480MHz
  - 2MB Flash
  - 1MB RAM

## Other Features

- FRAM
- [Pixhawk Autopilot Bus (PAB) Form Factor](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- LED Indicators
- MicroSD Slot
- USA Built
- Designed with a 1W heater. Keeps sensors warm in extreme conditions

## Power Requirements

- 5V
- 500mA
  - 300ma for main system
  - 200ma for heater


## Pinout

![DroneerX6 Pinout](https://github.com/Droneer-uav/Droneerx6/blob/main/DroneerX6_Pinout.png  "DroneerX6")

## Serial Port Mapping

| UART   | Device     | Port          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | GPS           |
| USART2 | /dev/ttyS1 | TELEM3        |
| USART3 | /dev/ttyS2 | Debug Console |
| UART4  | /dev/ttyS3 | UART4 & I2C   |
| UART5  | /dev/ttyS4 | TELEM2        |
| USART6 | /dev/ttyS5 | PX4IO/RC      |
| UART7  | /dev/ttyS6 | TELEM1        |
| UART8  | /dev/ttyS7 | GPS2          |


## Building Firmware

```sh
make droneer_x6_default
```

