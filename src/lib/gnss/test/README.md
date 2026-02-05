# RTCM Library Tests

Unit tests for the RTCM3 parser library.

## Running Tests

```bash
# Build and test
make tests TESTFILTER=Rtcm
```

## Test Descriptions

### RtcmCrcTest.cpp (2 tests)

| Test | Description |
|------|-------------|
| `CRC24Q_EmptyData` | Verifies CRC function handles empty/null input without crashing |
| `CRC24Q_KnownValue` | Verifies CRC produces a valid 24-bit value |

### RtcmParserTest.cpp (11 tests)

| Test | Description |
|------|-------------|
| `PayloadLength_MasksReservedBits` | Verifies reserved bits are ignored when extracting length |
| `MessageType_Extraction` | Verifies 12-bit message type is correctly extracted from payload |
| `SingleMinimalFrame` | Parses smallest valid frame (2-byte payload) |
| `SingleMaximalFrame` | Parses largest valid frame (1023-byte payload) |
| `MultipleFramesInSequence` | Parses 3 back-to-back valid frames |
| `FrameArrivesInChunks` | Parses a frame arriving one byte at a time |
| `GarbageBeforeValidFrame` | Skips 50 bytes of garbage, then parses valid frame |
| `ValidFrameBadFrameValid` | Verifies recovery after CRC error between valid frames |
| `BufferOverflowProtection` | Verifies buffer rejects data beyond capacity |
| `EmptyPayloadFrame` | Parses frame with 0-byte payload |
| `GetNextOnEmptyBuffer` | Verifies empty buffer returns nullptr |

### RtcmBustedSenderTest.cpp (5 tests)

Tests for handling completely invalid data streams.

| Test | Description |
|------|-------------|
| `BustedSender_AllZeros` | 1000 bytes of 0x00 - all discarded |
| `BustedSender_AllPreambles` | 500 bytes of 0xD3 - parser waits for incomplete frame |
| `BustedSender_RandomNoise` | 500 random bytes - most discarded or cause CRC errors |
| `BustedSender_ValidHeaderBadCrc` | Well-formed frame with wrong CRC |
| `BustedSender_RepeatedBadCrcFrames` | 100 consecutive bad-CRC frames |

### RtcmStressTest.cpp (3 tests)

Performance and stress tests.

| Test | Description |
|------|-------------|
| `Stress_ManySmallValidFrames` | 1000 valid frames - verifies no data loss |
| `Stress_LongGarbageStreamThenValid` | 10KB garbage then 1 valid frame - exercises `discardBytes()` |
| `Stress_InterleavedGarbageAndValid` | 100 cycles of garbage + valid frame - simulates flaky sender |
