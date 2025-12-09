Take a look at our latest commit. Also take a look at the latest commit in src/drivers/gps/devices. Please briefly summarize what we're doing to a claude_summary.md so that I can verify that you understand the context.

We need to write a python ulog parser for post processing the gps_dump data so that I can verify all of the RTCM messages are there, uncorrupted, and at the correct rate. This allows me to verify the full chain from F9P --> gps driver --> uorb --> CANnode publisher --> FC uavcan receiver --> uorb --> logger.

Please analyze our changes, generate the summary, and write a parser and save it to Tools/gps_dump_analyzer.py. The python tool should do the parsing/analysis and generate a report. We are looking for uncorrupted data. There should be no bytes unassociated to a message, no partial messages (except at the end of the log) and each message of interest should arrive at regular 5Hz intervals. The report presentationis up to you, both textual and visual are nice. It helps to identify when/where in the log the dropouts are occuring (if at all).

---

# RTCM3 Message Parsing Reference

## GpsDump uORB Message

RTCM data is logged in 79-byte chunks via `GpsDump.msg`. A single MSM7 message will span multiple entries.

| Field | Size | Description |
|-------|------|-------------|
| timestamp | uint64 | Time since system start (microseconds) |
| instance | uint8 | GNSS receiver instance |
| len | uint8 | Length of data (MSB indicates direction) |
| data | uint8[79] | Raw RTCM bytes |

## RTCM3 Frame Structure

| Field | Size | Description |
|-------|------|-------------|
| Preamble | 1 byte | Always `0xD3` |
| Reserved + Length | 2 bytes | 6 reserved bits + 10-bit payload length |
| Payload | N bytes | Message data (up to 1023 bytes) |
| CRC | 3 bytes | CRC-24Q |

## Extracting Message Type

Message type is in the first 12 bits of payload:

```c
uint16_t msg_type = (payload[0] << 4) | (payload[1] >> 4);
```

## Target MSM7 Messages

| Message | Constellation | Typical Size |
|---------|---------------|--------------|
| 1077 | GPS MSM7 | 200-400 bytes |
| 1087 | GLONASS MSM7 | 200-350 bytes |
| 1097 | Galileo MSM7 | 150-300 bytes |
| 1127 | BeiDou MSM7 | 200-400 bytes |
| 1005 | Base position | ~19 bytes |
| 1230 | GLONASS biases | ~10-15 bytes |

## Parser Steps

1. Reassemble stream from sequential `GpsDump` entries
2. Scan for `0xD3` preamble
3. Extract length from bytes 1-2 (lower 10 bits)
4. Read payload + 3-byte CRC
5. Verify CRC-24Q
6. Extract message type from first 12 bits of payload
7. Validate message count and rate per type
