# DroneCAN over MAVLink: test matrix

What CAN frame forwarding over MAVLink (`MAV_CMD_CAN_FORWARD`, `CAN_FRAME`, `CANFD_FRAME`,
`CAN_FILTER_MODIFY`) has to be tested for. The main use case is updating DroneCAN node firmware
from a GCS (`dronecan_gui_tool` with a `mavcan:` interface, Mission Planner) through the autopilot.

The rows marked U, D, L or M are covered by gtests in this repository. Rows marked HW are the
specification for the hardware test library outside of PX4.

## Layers

| Layer | What | Where |
|-------|------|-------|
| U  | Unit tests of the pure logic | `src/lib/can_forward/CanForwardFilterTest.cpp`, `CanForwardSessionTest.cpp`, `CanFrameCodecTest.cpp` |
| D  | `ForwardingCanDriver` on the real libuavcan `CanIOManager`, with a fake CAN driver whose TX mailboxes can be made full | `src/lib/can_forward/ForwardingCanDriverTest.cpp` |
| L  | Three real libuavcan dispatchers in one process: GCS node 127, autopilot node 10 behind `ForwardingCanDriver`, target node 42 on an in-memory bus. The MAVLink link is modelled by the frame sink and `pumpToBus()`. Real DSDL signatures and multi-frame transfers | `src/lib/can_forward/CanForwardTransferTest.cpp` |
| M  | `MavlinkCanForward` (MAVLink side) with real uORB topics | `src/modules/mavlink/MavlinkCanForwardTest.cpp` |
| HW | External library: autopilot on USB (and on a telemetry radio), a USB-SLCAN adapter on the autopilot's CAN bus running an emulated DroneCAN node (pydronecan on `slcan:`), the GCS side using pydronecan `mavcan:` | outside of PX4 |

Run the in-tree tests with:

```bash
make tests TESTFILTER="Can[FD]"
```

## Wire conventions under test (compatible with ArduPilot and existing clients)

- `MAV_CMD_CAN_FORWARD.param1` is the 1-based bus, 0 stops forwarding
- `CAN_FILTER_MODIFY.bus` is 1-based, `CAN_FRAME.bus` / `CANFD_FRAME.bus` are 0-based
- `id` bit 31 marks an extended frame; `len` is a byte count
- the client resends `MAV_CMD_CAN_FORWARD` about once per second, the session ends 5 s after the last one
- filter IDs are DroneCAN data type IDs: anonymous frames are 0, services `(id >> 16) & 0xFF`, messages `(id >> 8) & 0xFFFF`

## C: Frame codec and validation

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| C1 | Extended ID, len 0..8 | Frame copied, bytes past `len` zero | U, M |
| C2 | `CAN_FRAME` with len 9, 12, 64, 255 | Dropped, counted as invalid | U, M |
| C3 | Standard ID up to 0x7FF | Accepted | U, M |
| C4 | Standard ID above 0x7FF | Dropped | U, M |
| C5 | RTR or error flag set | Dropped, a GCS cannot put error frames on the bus | U, M |
| C6 | Frame for a bus other than the session bus | Dropped | M, D |
| C7 | `CANFD_FRAME` with len <= 8 | Accepted as a classic frame | M |
| C8 | `CANFD_FRAME` with len 12..64 | Dropped and counted (no CAN FD in libuavcan v0) | M |
| C9 | Frames sent to the GCS | `CAN_FRAME`, 0-based bus, EFF bit kept, target = requesting system/component | M, HW |

## F: Filter (`CAN_FILTER_MODIFY`)

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| F1 | No filter | Everything forwarded | U, HW |
| F2 | REPLACE with 0 IDs | Filter cleared, everything forwarded | U |
| F3 | REPLACE / ADD with duplicates and unsorted IDs | Sorted set without duplicates | U |
| F4 | REMOVE of an absent ID, REMOVE down to empty | No-op; empty set forwards everything | U |
| F5 | 17 IDs, unknown operation, bus 0 | Rejected, filter unchanged | U, M |
| F6 | ADD beyond 64 IDs | Rejected as a whole | U |
| F7 | Type ID extraction for anonymous frames, GetNodeInfo (1), BeginFirmwareUpdate (40), file.GetInfo (45), file.Read (48), NodeStatus (341), Allocation (1), request and response | Exact IDs | U |
| F8 | Filter sent right after the request, before the ack | Applied | M |
| F9 | Minimal firmware update filter {0, 1, 40, 45, 48, 341} | Complete update works, other traffic is not forwarded | U, L, HW |
| F10 | `dronecan_gui_tool` "low bandwidth" filter (15 IDs) | Only these types reach the GCS | HW |

## S: Session

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| S1 | Request bus 1 | ACCEPTED, session active, MAVLink instance bound | U, M, HW |
| S2 | Request bus 0 | ACCEPTED, forwarding stops, instance unbound | U, M, HW |
| S3 | Bus above the number of interfaces, 255, -1, 1.5, NaN, infinity | DENIED, nothing bound, a running session continues | U, M |
| S3a | Bus whose CAN interface the board does not enable | DENIED | HW |
| S4 | Keepalive at 1 Hz for a long time | Session never times out | U, HW |
| S5 | No keepalive, also on a silent bus | Session ends 5 s after the last keepalive; MAVLink unbinds on the status | U, M, HW |
| S6 | Keepalive just before 5 s | Session continues | U |
| S7 | Request a different bus | Forwarding moves to that bus | U, HW |
| S8 | Request from another client (system, component or link) | Takes the session over, the old client gets no more frames and its frames are dropped | U, M, HW |
| S9 | Request as `COMMAND_LONG` and as `COMMAND_INT` | Both work | HW |
| S10 | Request addressed to another system | Ignored | HW |
| S11 | Clock going backwards | No timeout | U |
| S12 | Exactly one `COMMAND_ACK` per keepalive (Commander does not answer as well) | One ack | HW |
| S13 | Keepalive while the uavcan driver saves node parameters | Still answered | HW |
| S14 | `mavlink stop` on the bound instance | No crash, the session times out | HW |

## A: Arming safety

GCS frames must never reach the bus while the vehicle is armed or prearmed.

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| A1 | New session while armed, and while only prearmed | TEMPORARILY_REJECTED | U, HW |
| A2 | Arming or prearming during a session | Monitoring only: bus traffic still reaches the GCS, GCS frames are dropped and counted | U, D, HW |
| A3 | Disarming again | Frames from the GCS are sent again without a new request | U |
| A4 | GCS frames waiting for a TX mailbox when the vehicle arms | Dropped; nothing reaches the bus after arming | D, L, HW |
| A5 | Arming in the middle of a firmware update | Node stays in its bootloader, no frame on the bus after arming; the update can be retried after disarming | L, HW |
| A6 | PX4's own firmware update server (`UAVCAN_ENABLE=3`) while a GCS forwards | Does not start updates during the session; checks the nodes again after it | HW |
| A7 | `COM_PREARM_MODE` = Always | No GCS frames can be sent (documented) | HW |

## D: Data paths in the driver

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| D1 | Frame received from the bus | Forwarded once | D |
| D2 | Frame the autopilot sends itself (NodeStatus) | Forwarded once, only after it was transmitted | D, HW |
| D3 | Loopback frame (time sync master) | Not forwarded twice | D |
| D4 | GCS frame | On the bus once, not echoed to the GCS, private flag never reaches the platform driver | D, HW |
| D5 | GCS request to the autopilot's node, GCS broadcast, anonymous GCS frame | Also fed to the autopilot's node; the answer reaches the GCS | D, L, HW |
| D6 | GCS frame for another node (file.Read response) | Not fed to the autopilot's node | D |
| D7 | `select()` with only fed frames pending | Read bit only for requested interfaces, no blocking | D |
| D8 | TX mailbox full in the middle of a transfer | Order kept, at most one forwarded frame queued in libuavcan | D, L |
| D9 | 150 frame burst (4 pipelined 256 byte read responses) with one free mailbox per cycle | All sent in order, nothing dropped | D, L |
| D10 | Frame not sent within its deadline (no ACK on the bus) | Dropped by libuavcan, counted, no memory leak | D, HW |
| D11 | No session | Plain pass-through; GCS frames dropped | D |
| D12 | Frames on another interface than the session bus | Not forwarded | D |
| D13 | `can_interface_status` counters | Forwarded frames count as TX, frames fed to the node as RX (documented) | D |
| D14 | `uavcan status` on a SocketCAN board | Bus state printed, no bad cast through the decorator | HW |
| D15 | Overflow of the 16 entry queue to the autopilot's node | Counted, bus side unaffected | D |

## M: MAVLink side

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| M1 | Frames from a system/component that does not own the session | Dropped, counted | M |
| M2 | Frames before any session | Dropped, counted | M |
| M3 | Frames with target component 0 (pydronecan) and with the autopilot's component | Accepted | HW |
| M4 | TX buffer full in the middle of a batch | Continues at the same frame, no gaps or duplicates | M |
| M5 | More than 8 batches between two sends (`can_forward_rx` overrun) | Counted | M |
| M6 | Request on two MAVLink instances | Only the latest one gets the frames | M |
| M7 | Signing active, unsigned `CAN_FRAME` | Rejected by the signing check | HW |
| M8 | A forwarding link (`-f`) | `CAN_*` messages for this vehicle are not forwarded to other links | HW |
| M9 | Frames are published once per receive buffer, full batches at once | No added latency, order kept | M |
| M10 | Frames from before the binding | Not sent to the new client | M |

## E: End to end, firmware update

| ID | Case | Expected | Layer |
|----|------|----------|-------|
| E1 | Discovery: the node's NodeStatus and GetNodeInfo | GCS shows name, unique ID and version | L, HW |
| E2 | GetNodeInfo to the autopilot's own node | Answered | L, HW |
| E3 | Read, set and save node parameters | Persist on the node | HW |
| E4 | Read responses of 0, 1, 6, 7, 8, 13, 14, 255, 256, 258, 300 bytes; full images of 0 B, 1 B, 255 B, 256 B, 257 B, 4 KiB, 1 MiB | Byte exact image on the node (CRC check), node reboots into the new version | L (transfers), HW (images) |
| E5 | 1 and 4 reads in flight | Both work, throughput recorded | L, HW |
| E6 | Over a 57600 baud telemetry radio and over USB | Works, no corruption | HW |
| E7 | GCS paused for 6 s during an update, then resumed | Session ends; update resumes or fails cleanly | HW |
| E8 | MAVLink link lost during an update | Autopilot keeps running; a retry works | L, HW |
| E9 | Node power cycled during an update | Update restarts and succeeds | HW |
| E10 | Other nodes on the bus during an update (e.g. a GNSS used by the estimator) | Unaffected, PX4 keeps using them | HW |
| E11 | Bus flood at 1 kHz without a filter | Drops counted, autopilot CPU load and work queue latency within limits | HW |
| E12 | Clients: pydronecan `mavcan`, `dronecan_gui_tool`, Mission Planner | All find and update the node | HW |
| E13 | 1 h session | No memory growth, consistent counters | HW |
| E14 | Autopilot reboot during a session | Client recovers by sending a new request | HW |
| E15 | Bootloader comes up anonymous or with another node ID | Its frames are forwarded, the update completes | L, HW |
| E16 | 1 MiB image over USB | Duration recorded, all drop counters 0, no read retries on the node | HW |
| E17 | Node rejects the image (wrong board, bad CRC) | Error shown on the GCS, autopilot unaffected | HW |
| E18 | Same image through MAVLink FTP to `ufw/` with `UAVCAN_ENABLE=3` | Node updated by PX4's own server (comparison path) | HW |
| E19 | Update attempt while prearmed | Rejected, node stays in its application | HW |

## Hardware setup for the external library

- Autopilot with a CAN port and `UAVCAN_ENABLE` >= 1, connected over USB; for E6 also over a telemetry radio.
- A USB-SLCAN adapter on the same CAN bus (with termination). On the host it runs the emulated node:
  `dronecan.make_node('/dev/ttyACMx', node_id=42, bitrate=1000000)` (or `slcand` + SocketCAN).
- Emulated node: NodeStatus, GetNodeInfo with a known unique ID and version, param GetSet and
  ExecuteOpcode, BeginFirmwareUpdate server and a bootloader-like `file.Read` client with a
  configurable number of reads in flight, timeouts and retries. Fault injection: drop or delay
  frames, reboot during the transfer, come up anonymous, reject the image.
- GCS side: pydronecan `dronecan.make_node('mavcan:...', node_id=127)`, keepalive and filter as
  `dronecan_gui_tool` does them.
- A second SLCAN adapter (or the same one, read only) as a bus sniffer for the "nothing on the bus"
  checks (A4, A5, D4).
- Assertions on the autopilot use `uavcan status`, the `can_forward_status` topic (`listener`
  or the ULog) and `top` / `work_queue status`.
