# DroneCAN Shell

The DroneCAN Shell is an [NuttShell (NSH)](https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410) console that can be accessed over a DroneCAN/UAVCAN bus, via the `uavcan.protocol.AccessCommandShell` service.

It is intended for peripheral nodes that only have a CAN connection and no accessible serial debug port.

:::tip
This is a separate NSH instance from the [System Console](../debug/system_console.md) and the [MAVLink Shell](../debug/mavlink_shell.md) — see [System Console vs. Shells](../debug/consoles.md#console_vs_shell) for how the three compare.
If the system does not start properly you should instead use the [System Console](../debug/system_console.md).
:::

## Preconditons

PX4 firmware [must be built with](../hardware/porting_guide_config.md) `CONFIG_UAVCANNODE_COMMAND_SHELL` enabled.

## Opening the Shell

### dronecan_shell.py

Access the shell from a terminal using the **dronecan_shell.py** script.

#### Dependencies

```sh
pip3 install --user dronecan
```

#### Command

The script uses `termios` for terminal handling, so it only runs on Linux and macOS (not Windows).

```sh
./Tools/dronecan_shell.py <device> [--baudrate BAUDRATE] [--bitrate BITRATE] [--allocator]
```

| Argument      | Default   | Description                                         |
| ------------- | --------- | --------------------------------------------------- |
| `device`      | required  | Serial port of the CAN adapter, e.g. `/dev/ttyACM0` |
| `--baudrate`  | `115200`  | Serial baudrate to the adapter                      |
| `--bitrate`   | `1000000` | CAN bus bitrate                                     |
| `--allocator` | disabled  | Run a dynamic node ID allocator on this script      |

The script listens on the bus for a few seconds, lists every node it finds, and asks which one to connect to. With only one node found, it connects automatically.

#### Scenarios

FMU connected to the bus, already allocating dynamic node IDs:

```sh
./Tools/dronecan_shell.py /dev/ttyACM0
```

No other allocator on the bus, so the script must allocate an ID for the target node:

```sh
./Tools/dronecan_shell.py /dev/ttyACM0 --allocator
```

Don't use `--allocator` if the FMU or anything else is already allocating IDs on the same bus. Two allocators racing can assign conflicting IDs.

## Using the DroneCAN Shell

For information see: [PX4 Consoles/Shells > Using Consoles/Shells](../debug/consoles.md#using_the_console).

## Limitations

- The output buffer is just the pipe's current buffer, not a persistent per-command output store.
- `FLAG_CLEAR_OUTPUT_BUFFERS`, `FLAG_READ_STDERR`, `FLAG_READ_STDOUT`, `FLAG_RUNNING`, `FLAG_HAS_PENDING_STDERR` are not implemented.
- Stderr is not connected to a pipe; only stdin and stdout are.
