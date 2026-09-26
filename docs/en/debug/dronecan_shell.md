# DroneCAN Shell

The DroneCAN Shell is a [NuttShell (NSH)](https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410) console running on a DroneCAN peripheral node (a CAN node running PX4 firmware), which you access over the CAN bus via the `uavcan.protocol.AccessCommandShell` service.

It is intended for peripheral nodes that only have a CAN connection and no accessible serial debug port.

:::tip
This gives you a shell on the peripheral node, not on the flight controller.
To access the flight controller, use the [MAVLink Shell](../debug/mavlink_shell.md) or the [System Console](../debug/system_console.md) — see [System Console vs. Shells](../debug/consoles.md#console_vs_shell) for how they compare.
If the node does not boot, the DroneCAN Shell is not available: use the node's [System Console](../debug/system_console.md) instead, if it has an accessible debug port.
:::

## Preconditions

PX4 firmware [must be built with](../hardware/porting_guide_config.md) `CONFIG_UAVCANNODE_COMMAND_SHELL` enabled.
This option is part of the `uavcannode` driver (`CONFIG_DRIVERS_UAVCANNODE`), so it is only available in DroneCAN peripheral-node firmware (such as `ark_can-gps`), not on flight controllers.
It is not enabled by default on any board.

## Opening the Shell

### dronecan_shell.py

Access the shell from a terminal using the `dronecan_shell.py` script.

#### Dependencies

```sh
pip3 install --user dronecan pyserial
```

#### Command

The script uses `termios` for terminal handling, so it only runs on Linux and macOS (not Windows).

```sh
./Tools/dronecan_shell.py <device> [--node-id NODE_ID] [--baudrate BAUDRATE] [--bitrate BITRATE] [--allocator]
```

| Argument      | Default   | Description                                         |
| ------------- | --------- | --------------------------------------------------- |
| `device`      | required  | Serial port of the CAN adapter, e.g. `/dev/ttyACM0` |
| `--node-id`   | `100`     | DroneCAN node ID used by the script itself          |
| `--baudrate`  | `115200`  | Serial baudrate to the adapter                      |
| `--bitrate`   | `1000000` | CAN bus bitrate                                     |
| `--allocator` | disabled  | Act as a dynamic node ID allocator                  |

The script listens on the bus for 5 seconds, lists the nodes it finds, and asks for the ID of the one to connect to.
The list includes every node on the bus, such as the flight controller and ESCs, but only PX4 peripheral nodes built with `CONFIG_UAVCANNODE_COMMAND_SHELL` provide a shell.

In the shell, **Ctrl+C** resets the shell (it starts a new NSH session rather than interrupting the running command), and **Ctrl+D** returns to the node list.
Arrow keys, command history and tab completion are not supported.

#### Scenarios

FMU connected to the bus, already allocating dynamic node IDs:

```sh
./Tools/dronecan_shell.py /dev/ttyACM0
```

No other allocator on the bus, so the script must allocate an ID for the target node:

```sh
./Tools/dronecan_shell.py /dev/ttyACM0 --allocator
```

:::warning
Don't use `--allocator` if the FMU or anything else is already allocating IDs on the same bus.
Two allocators racing can assign conflicting IDs.
:::

## Using the DroneCAN Shell

For information see: [PX4 Consoles/Shells > Using Consoles/Shells](../debug/consoles.md#using_the_console).

## Limitations

- Only one shell session per node: a request from another client node closes the current shell and starts a new one.
- The output buffer is just the pipe's current buffer, not a persistent per-command output store.
- `FLAG_CLEAR_OUTPUT_BUFFERS`, `FLAG_READ_STDERR`, `FLAG_READ_STDOUT`, `FLAG_RUNNING`, `FLAG_HAS_PENDING_STDERR` are not implemented.
- `last_exit_status` is not reported (always 0).
- There is no separate stderr stream: stderr is redirected to stdout.
