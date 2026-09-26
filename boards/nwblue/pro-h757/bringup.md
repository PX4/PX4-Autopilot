# NWBlue Pro H757 bring-up

30x30 mm FPV flight controller (NWBlue) carrying a CubePilot CubeNode H757
module. STM32H757, ICM45686 IMU and DPS368 baro on SPI3, IIS2MDC mag on I2C3,
microSD on SDMMC2, 9 DShot/PWM outputs, CAN1, 6 UARTs.

## Port map

| Connector | UART | PX4 device | Use |
| --- | --- | --- | --- |
| ESC J10 pin 4 | USART1 (RX only) | `/dev/ttyS0` (TEL3) | ESC telemetry |
| VTX J9 pin 5 | USART2 (RX only) | `/dev/ttyS1` | spare camera backchannel |
| RC J11 | UART4 | `/dev/ttyS2` (RC) | serial RC |
| VTX J9 pins 3,4 | USART6 | `/dev/ttyS3` (TEL2) | MSP DisplayPort OSD |
| TELEM1 J4 | UART7 | `/dev/ttyS4` (TEL1) | MAVLink, 8N1, no flow control |
| GPS J8 | UART8 | `/dev/ttyS5` (GPS1) | GPS, plus I2C4 for its compass |

## Console

There is no dedicated debug console; NSH runs over USB. During bring-up it can
be moved to a UART by setting `CONFIG_<port>_SERIAL_CONSOLE=y` and dropping
`# CONFIG_DEV_CONSOLE is not set` in `nuttx-config/nsh/defconfig`.

## Retest checklist

Broken or untested on the hand-built rev-A prototype, needs re-verifying on
production hardware:

- [x] **DShot, all four groups** — TIM1 (CH1-4), TIM4 (CH5-6), TIM3 (CH7-8),
      TIM2 (CH9). Bidirectional works everywhere except FMU_CH6, which is
      TIM4_CH4: the H7 DMAMUX has request lines for TIM4_CH1-CH3 only, so there
      is no capture DMA for it and `dshot.c` excludes it. Silicon limitation,
      not fixable by remapping. Use CH6 for plain DShot or PWM.
- [x] **GPS (UART8)** — both directions, after ruling out a bad GPS module.
- [x] **DPS368 baro** — stable; the rev-A intermittency was a solder joint.
- [x] **Battery voltage** — `BAT1_V_DIV 21.0` (200k/10k) against a meter on VSYS.
- [x] **Battery current** — `ADC3_INP0` on PC2 tracks an injected voltage
      correctly. `BAT1_A_PER_V` is left at ArduPilot's 24.0 as a placeholder;
      the real value depends on whichever current sensor is wired to the ESC
      connector and can only come from a measurement.
- [x] **IMU orientation** — no rotation. The module is aligned with the carrier;
      the ArduPilot hwdef's `ROTATION_ROLL_180` does not match this hardware.
- [x] **Internal mag orientation** — IIS2MDC unrotated, confirmed by compass
      calibration.
- [x] **RC input (UART4)** — SBUS receiving on J11.
- [x] **DroneCAN (CAN1)** — needs the uavcan clock moved off TIM5, see
      `CONFIG_BOARD_UAVCAN_TIMER_OVERRIDE` in `default.px4board`.
- [ ] **ESC telemetry RX (USART1)** — driver side verified: instrumenting
      DShot.cpp shows PX4 setting the telemetry request bit at the expected
      rate with all preconditions met, and the Telem pin routes to PB7 per the
      schematic and the pinmap. Not testable on the unit used for bring-up -
      its Telem net is shorted to ground (0 ohm, connector through to the
      CubeNode pad). The same net is clean on another board, so this is a fault
      on that one unit, not the design. Retest on undamaged hardware.
- [x] **TELEM1 (UART7)** — MAVLink both directions. Rev A had the flow control
      pins miswired; J4 pins 4 and 5 are not connected on the production board
      and the port runs 8N1.
- [ ] **Buzzer** — unconfirmed. It plays the right melody but very quietly, in
      every combination tried: two buzzers (active and passive), two boards, 5V
      and 12V, and both the PWM and GPIO tone alarm backends. Whether this is a
      driver-side or a hardware limit was never established. Left on the PWM
      backend, which is the more capable of the two.
- [ ] **MSP DisplayPort OSD** — untested; needs an HD VTX on J9. There is no
      OSD chip on the board, so analog video cannot be overlaid: the VTX has to
      render it. Not enabled by default - set `MSP_OSD_CONFIG` to 102 (TEL2).
- [x] **RGB LEDs** — blue blinks the arming state, green solid when armed, red
      on overload, via `BOARD_ARMED_LED`/`BOARD_ARMED_STATE_LED` in commander.
      There is no colour status indication (no amber on low battery and so on):
      that goes through the `led_control` topic, which only the rgbled chip
      drivers consume, and these are plain GPIOs with no such driver.
- [ ] **Status LED (PC0)** — driven by the bootloader only. Decide whether the
      application should use it.

Confirmed working: USB CDC ACM, microSD, internal and external mag, ICM45686,
ADC3 rail sensing (VSYS/5V/3V3/VREG).

## Related links

- Product page: https://nwblue.com/products/pro-h757-fpv-flight-controller
- Manufacturer documentation: https://docs.nwblue.com/nw-blue/products/autopilots/proh757/overview
- CubeNode pin descriptions: https://docs.cubepilot.org/user-guides/cubenode/pin-descriptions
- ArduPilot board support: https://github.com/ArduPilot/ardupilot/pull/33349
