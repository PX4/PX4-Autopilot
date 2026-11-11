# ESC Configuration over USB (AM32, Bluejay, BLHeli)

PX4 can act as the link between an ESC configurator and the ESCs, so that you can read and change ESC settings and flash ESC firmware without disconnecting the ESCs from the flight controller.
This works with the browser-based configurators [am32.ca](https://am32.ca) (AM32 ESCs) and [esc-configurator.com](https://esc-configurator.com) (Bluejay and BLHeli_S ESCs).

The flight controller is connected to the computer with its USB cable, and the configurator talks to the ESCs through the ESC signal wires.
While a configuration session is active, the normal motor outputs (DShot and PWM) are off.

::: warning
Remove the propellers before you connect a configurator.
:::

## Supported Boards

- [AGAM FMU-v6XRT](../flight_controller/agam_v6xrt.md) (`agam_fmu-v6xrt`)
- [ARK FMU-v6S](../flight_controller/ark_v6s.md) (`ark_fmu-v6s`)
- [ARK FMU-v6X](../flight_controller/ark_v6x.md) (`ark_fmu-v6x`)
- [ARK FMU-v6X RT](../flight_controller/ark_v6xrt.md) (`ark_fmu-v6xrt`)
- [ARK FPV](../flight_controller/ark_fpv.md) (`ark_fpv`)
- [ARK Pi6X Flow](../flight_controller/ark_pi6x.md) (`ark_pi6x`)
- [NXP MR-NAVQ95](../flight_controller/nxp_mr_navq95.md) (`nxp_mr-navq95`)
- `nxp_mr-tropic`
- [Pixhawk 6X-RT](../flight_controller/pixhawk6x-rt.md) (`px4_fmu-v6xrt`)

Other STM32F7/H7 and NXP i.MX RT boards can be added with a build configuration change, see [Enabling on Other Boards](#enabling-on-other-boards).

Only ESCs connected to the flight controller's own motor outputs can be configured; outputs on an I/O board (PX4IO) are not supported.
The server only uses outputs that are assigned to a motor in [Actuator Configuration](../config/actuators.md), among the first eight outputs.

The configurator numbers the ESCs by motor: ESC 1 is the output assigned to Motor 1, ESC 2 is Motor 2, and so on.

## Configure the ESCs

1. Set the parameter [SYS_USB_AUTO](../advanced_config/parameter_reference.md#SYS_USB_AUTO) to `1` (_Auto-detect_) and reboot the flight controller.
1. Plug in the USB cable and power the ESCs (connect the battery).
1. Before any other program opens the USB port, open [am32.ca](https://am32.ca) or [esc-configurator.com](https://esc-configurator.com) in a browser that supports WebSerial (Chrome or Edge).
1. Click **Connect** and select the flight controller's USB port.
   The flight controller recognises the configurator and starts the ESC configuration server; the motor outputs are switched off at that moment.
1. Read, change and write the ESC settings, or flash new ESC firmware.
1. Click **Disconnect** when you are done.
   The flight controller reboots, so the USB connection drops.
   After that reboot the motor outputs work normally again.

The USB port is given to the first program that talks to it after it is plugged in.
If _QGroundControl_ connected first, unplug and replug the USB cable or reboot the flight controller.

If you unplug the USB cable or close the configurator without disconnecting, the flight controller also reboots.

Any program that first sends a valid MSP request to the USB port starts the ESC configuration server, and the motor outputs then stay off until the flight controller reboots.

## Command Line Tools

### Host Tool

`Tools/esc_4way.py` reads and writes one ESC from a computer through the ESC configuration server (or any flight controller that supports MSP and the 4way interface):

```sh
Tools/esc_4way.py info                               # probe ESC 0
Tools/esc_4way.py settings                           # AM32 settings, raw and decoded
Tools/esc_4way.py direction toggle                   # or: direction set 0|1
Tools/esc_4way.py dump esc0_flash.bin
Tools/esc_4way.py flash AM32_firmware.hex            # Intel hex or bin
Tools/esc_4way.py read 0xF800 184 -o settings.bin
Tools/esc_4way.py write 0xF800 settings.bin
Tools/esc_4way.py check /dev/ttyACM0                 # protocol self-check
```

When the port sends MAVLink heartbeats, the tool first sets `SYS_USB_AUTO` to `1` over MAVLink and reboots the flight controller, which frees the port for the ESC configuration server.
The parameter stays set.
`--setup` does that without listening for heartbeats first.
Use `-d` to select the serial device (default `/dev/ttyACM0`), `--target` for the ESC (default 0) and `--no-exit` to keep the session open.
The tool needs `pymavlink` for the switch over MAVLink.

### Using Another Serial Port

The ESC configuration server does not have to run on the USB port.
`esc_4way start -d <device>` starts it on any serial port, which is useful for boards without USB, or for a UART connected to a companion computer:

```sh
esc_4way start -d /dev/ttyS3
```

Use the device node of the port, for example `/dev/ttyS3` for a UART or `/dev/ttyACM0` for USB.
The port must not be used by another module (for example MAVLink), so free it first.
Then connect the host tool or configurator to the other end of that serial link, for example with `Tools/esc_4way.py -d <host serial device> info`.
The server is started manually in this case, so USB auto-detection is not involved.
Stop it with `esc_4way stop`, or reboot the flight controller.

## USB Vendor ID

am32.ca only shows USB devices from a list of known flight controller vendor IDs, so use a version of the configurator whose list includes the vendor ID of your board's USB port.
esc-configurator.com has an override for this and shows the port regardless of its vendor ID.

## Enabling on Other Boards

The ESC configuration server drives the ESC signal pins as a half-duplex UART.
That backend exists for STM32F7/H7 boards and for the NXP i.MX RT1064/RT1176 and i.MX95 boards.
To enable the server, add this key to the board's `default.px4board`:

```text
CONFIG_DRIVERS_ESC_4WAY=y
```

USB auto-detection needs the board's USB port to be managed by `cdcacm_autostart` (`CONFIG_DRIVERS_CDCACM_AUTOSTART=y`); without it, start the server manually with `esc_4way start -d <device>`.

On STM32 boards the UART bit timing comes from a spare hardware timer, `CONFIG_UART_BITBANG_TIMER` (default `13`).
That timer must be enabled in the board's NuttX configuration (`CONFIG_STM32H7_TIM13=y` or `CONFIG_STM32F7_TIM13=y` in `nuttx-config/nsh/defconfig`), and nothing else on the board may use it: not the HRT, the tone alarm, an LED, or a PWM/DShot output timer.

## See Also

- [Serial Passthrough (MAVLink SERIAL_CONTROL)](../uart/serial_passthrough.md): raw access to the ESC signal pins over MAVLink, for example from a companion computer.
- [DShot ESCs](dshot.md)
