# Manual Control Input via UDP

This module listens to manual control messages on a UDP port and republishes them to uORB topic `manual_control_input`. The UDP port (default: `51324`) can be set via parameter `p`.

## Message Format

The message has the following structure that has been derived from the `manual_control_setpoint_s` message:
```
| 0x0F (u8) | roll (f32) | pitch (f32) | yaw (f32) | throttle (f32) | N (u8) | aux_1 ... aux_N (N x f32) | CRC-32 (u32) |
```

The message starts with a byte (8bit) for the header `0x0F`, followed by 4 float values (32bit) for roll, pitch, yaw and throttle (range [-1,+1]). The AUX channels are encoded as a variable-length array: The following byte (8bit) stores the number of AUX channels (range: [0,6]) which is followed by the variable amount of float values (32bit) for each of the optionally 6 AUX channels (range: [0,1]). Finally, the last 32bit store the CRC-32 computed over the entire message. The byte order is big-endian. Float values are represented according to the IEEE 754 standard.

## Example Client

```Python
#!/usr/bin/env python3

# File "mc_client.py". Call with list of 4 to 10 channels:
# ./mc_client.py 0.34 -0.56 0 0.6 0.3 0.5

import argparse
import socket
import struct
import zlib

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("values", type=float, nargs="+",
                        help="channel values in the order roll, pitch, yaw, throttle and optionally up to 6 AUX channels")
    args = parser.parse_args()

    # format message
    v = args.values
    naux = len(v) - 4

    msg = struct.pack("!BffffB" + naux * 'f', 0X0F, *v[0:4], naux, *v[4:])
    msg += struct.pack("!L", zlib.crc32(msg))

    # send message
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg, ("localhost", 51324))
```
