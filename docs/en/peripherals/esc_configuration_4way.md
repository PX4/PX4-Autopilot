# ESC Configuration over USB (AM32, Bluejay, BLHeli)

PX4 can act as the link between an ESC configurator and the ESCs, so that you can read and change ESC settings and flash ESC firmware without disconnecting the ESCs from the flight controller.
This works with the browser-based configurators [am32.ca](https://am32.ca) (AM32 ESCs) and [esc-configurator.com](https://esc-configurator.com) (Bluejay and BLHeli_S ESCs).

The flight controller is connected to the computer with its USB cable, and the configurator talks to the ESCs through the ESC signal wires.
While a configuration session is active, the normal motor outputs (DShot and PWM) are off.

::: warning
Remove the propellers before you connect a configurator.
:::

## Supported Boards

- [px4_fmu-v6xrt](../flight_controller/pixhawk6x-rt.md)
- `ark_fmu-v6xrt`
- `nxp_mr-tropic`

Only ESCs connected to the flight controller's own motor outputs can be configured; outputs on an I/O board (PX4IO) are not supported.

## Configure the ESCs

1. Connect the flight controller to QGroundControl and set the parameter [PASSTHRU_EN](../advanced_config/parameter_reference.md#PASSTHRU_EN) to `2` (_USB ESC configuration server_).
1. Reboot the flight controller.
   On this boot the motor outputs stay off and the USB port is used by the ESC configuration server instead of MAVLink, so QGroundControl will not connect.
   `PASSTHRU_EN` resets itself to `0` during this boot.
1. Power the ESCs (connect the battery).
1. Close QGroundControl (or any other program that holds the USB port), then open [am32.ca](https://am32.ca) or [esc-configurator.com](https://esc-configurator.com) in a browser that supports WebSerial (Chrome or Edge).
1. Click **Connect** and select the flight controller's USB port.
1. Read, change and write the ESC settings, or flash new ESC firmware.
1. Click **Disconnect** when you are done.
   The flight controller reboots, so the USB connection drops.
   After that reboot the motor outputs work normally again and QGroundControl can connect.

If you unplug the USB cable during a session, the flight controller also reboots.
Server mode lasts for a single boot: any reboot or power cycle, also in the middle of a session, returns the flight controller to normal operation.

## Alternative: Plug In and Connect

If [SYS_USB_AUTO](../advanced_config/parameter_reference.md#SYS_USB_AUTO) is set to `1` (auto-detect), you do not need to set `PASSTHRU_EN`:

1. Plug in the USB cable and power the ESCs.
1. Before any other program opens the USB port, open the configurator and click **Connect**.
   The flight controller recognises the configurator and starts the ESC configuration server; the motor outputs are switched off at that moment.
1. Click **Disconnect** when you are done; the flight controller reboots as above.

This only works while nothing else has claimed the USB port since it was plugged in.
If QGroundControl connected first, unplug and replug the USB cable.

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

When the port sends MAVLink heartbeats, the tool first sets `PASSTHRU_EN` to `2` over MAVLink and reboots the flight controller into the ESC configuration server, and waits for MAVLink to return after the session ends.
`--setup` does that switch without listening for heartbeats first.
Use `-d` to select the serial device (default `/dev/ttyACM0`), `--target` for the ESC (default 0) and `--no-exit` to keep the session open.
The tool needs `pymavlink` for the switch over MAVLink.

## USB Vendor ID

am32.ca only shows USB devices from a list of known flight controller vendor IDs, so use a version of the configurator whose list includes the vendor ID of your board's USB port.
esc-configurator.com has an override for this and shows the port regardless of its vendor ID.

## See Also

- [Serial Passthrough (MAVLink SERIAL_CONTROL)](../uart/serial_passthrough.md): raw access to the ESC signal pins over MAVLink, for example from a companion computer.
- [DShot ESCs](dshot.md)
