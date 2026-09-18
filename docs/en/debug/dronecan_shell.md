# DroneCAN Shell

The DroneCAN Shell is an _NSH console_ that can be accessed over a DroneCAN/UAVCAN bus, via the `uavcan.protocol.AccessCommandShell` service.

It is intended for peripheral nodes that only have a CAN connection and no accessible serial debug port.

## Preconditons
The node must be built with `CONFIG_UAVCANNODE_COMMAND_SHELL` enabled.


## Opening the Shell

### dronecan_shell.py

Access the shell from a terminal using the **dronecan_shell.py** script.

#### Dependencies

```sh
pip3 install --user dronecan
```

#### Command

```sh
./Tools/dronecan_shell.py <device> [--baudrate BAUDRATE] [--bitrate BITRATE] [--allocator]
```

| Argument      | Default   | Description                                             |
| ------------- | --------- | -------------------------------------------------------- |
| `device`      | required  | Serial port of the CAN adapter, e.g. `/dev/ttyACM0`       |
| `--baudrate`  | `115200`  | Serial baudrate to the adapter                           |
| `--bitrate`   | `1000000` | CAN bus bitrate                                           |
| `--allocator` | disabled  | Run a dynamic node ID allocator on this script            |

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
- The output buffer is just the pipe's current buffer, not a persistent per-command output store
- _FLAG_CLEAR_OUTPUT_BUFFERS_, _FLAG_READ_STDERR_, _FLAG_READ_STDOUT_, _FLAG_RUNNING_, _FLAG_HAS_PENDING_STDERR_ are not implemented
- Stderr is not connected to a pipe; only stdin and stdout are
