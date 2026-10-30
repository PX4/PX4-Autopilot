# Linux IIO IMU Interface Requirements

This document defines the kernel IIO IMU interface required by PX4 `iio_imu`.
The kernel handles hardware configuration, sampling, and FIFO operation. PX4
decodes and publishes data using IIO metadata. Existing attributes follow the
Linux IIO ABI; `*_range_raw` attributes are extensions defined by this interface.

## Device and Channels

The driver must expose a `/dev/iio:deviceN` character device and its corresponding
`/sys/bus/iio/devices/iio:deviceN/` attribute directory, including a `name` attribute.
The same device node must provide the following scan channels:

| Channel | Requirement | Data |
| --- | --- | --- |
| `in_accel_x`, `in_accel_y`, `in_accel_z` | Required | Three-axis raw acceleration counts, including gravity |
| `in_anglvel_x`, `in_anglvel_y`, `in_anglvel_z` | Required | Three-axis raw angular velocity counts |
| `in_temp` | Optional | Raw IMU temperature counts |
| `in_timestamp` | Required | Sampling time of this scan, in ns |

Each scan contains a corresponding set of acceleration, angular velocity, and
timestamp values. When available, temperature is included in the same scan.
Combining six-axis data from separate device nodes is not supported. Without a
temperature channel, PX4 reports temperature as NaN.

## Scan Format and Layout

Each scan channel must expose the following attributes under `scan_elements/`:

| Attribute | Meaning |
| --- | --- |
| `<channel>_en` | Read/write control selecting whether the channel is included in the scan |
| `<channel>_index` | Unique, nonnegative ordering index; gaps are allowed; this is not a byte offset |
| `<channel>_type` | Byte order, signedness, valid bit count, storage bit count, and right shift |

The type format is `[le|be]:[s|u]<bits>/<storagebits>[X1][>>shift]`. An omitted shift
is zero. Each channel must contain a single value and satisfy
`bits + shift <= storagebits`.

| Channel | Signedness | Valid bits | Storage bits |
| --- | --- | --- | --- |
| Acceleration and angular velocity | Signed `s` | 2–16 | 8, 16, or 32 |
| Temperature | Signed `s` or unsigned `u` | 1–32 | 8, 16, or 32 |
| Timestamp | Signed `s` | 64 | 64; shift must be zero |

All three axes of a sensor must have the same valid bit count. Floating-point
formats and repeated multi-value channels are not supported.

Channels are ordered by ascending index. Each channel is naturally aligned to
its storage size in bytes, and the total scan size is aligned to the largest
storage size. The timestamp index must be greater than every other data channel
index, placing the timestamp at the end of the scan on an 8-byte boundary.
Channel offsets and total scan size are determined from metadata, not fixed values.

## Scale, Offset, and Raw Range

The following attributes reside in the device's sysfs directory and must reflect
the actual current configuration:

| Attribute | Requirement | Unit or meaning |
| --- | --- | --- |
| `in_accel_scale` | Required, finite, and greater than zero | m/s² per raw count |
| `in_anglvel_scale` | Required, finite, and greater than zero | rad/s per raw count |
| `in_accel_range_raw` | Should be provided; may be absent | Positive magnitude of the raw acceleration range |
| `in_anglvel_range_raw` | Should be provided; may be absent | Positive magnitude of the raw angular velocity range |
| `in_temp_scale` | Required when a temperature channel exists; finite and greater than zero | m°C per raw count |
| `in_temp_offset` | Optional, defaults to zero; finite and may be nonzero | Offset in raw counts, applied before temperature scaling |

Scale, offset, and range_raw apply to decoded counts after right shifting,
masking, and sign extension. For example, attributes for `le:s12/16>>4` apply to
the decoded 12-bit count, not the left-aligned 16-bit storage value.

`range_raw` must be a positive integer no greater than `2^(bits - 1)`, where
`bits` is the valid bit count. This upper limit represents the nominal magnitude
of a signed encoding. `range_raw = 1000` describes a raw measurement range of
approximately −1000 to +1000, regardless of the storage width.

```text
If range_raw exists: raw_limit = range_raw
If range_raw is absent: raw_limit = 2^(bits - 1)
Physical range magnitude = raw_limit × scale
Acceleration or angular velocity = decoded_raw × scale
Temperature (°C) = (decoded_raw + in_temp_offset) × in_temp_scale / 1000
```

The raw range may be inferred only when the attribute is absent (ENOENT).
Read failures, empty values, invalid numbers, and out-of-range values are
interface errors. When range_raw is absent, the inferred value is used as the
saturation boundary. The kernel should expose range_raw to describe the actual
saturation boundary accurately. Temperature does not require a range_raw attribute.

A scan may contain the most recent temperature measurement; the temperature
hardware update rate need not match the six-axis sampling rate. Temperature
scale and offset attributes are not required when no temperature scan channel exists.

## Sampling Rate and Timestamps

The driver must expose one of the following sampling rate descriptions, in Hz:

- Device-wide `sampling_frequency`; or
- Both `in_accel_sampling_frequency` and `in_anglvel_sampling_frequency`.

Acceleration and angular velocity must have the same actual output sampling rate,
within 1–32000 Hz. If both device-wide and sensor-specific attributes exist,
their values must agree.

`current_timestamp_clock` must be present. It must either report `monotonic` or
allow that clock to be selected and read back while the buffer is disabled.
Scan timestamps must use the same CLOCK_MONOTONIC clock domain, be expressed in
nanoseconds, and be positive and strictly increasing.

Timestamps represent sampling times. For hardware FIFO batches, each scan must
carry its corresponding sampling time. A single interrupt time or userspace
read time must not be substituted for the timestamps of an entire batch.

## Buffer and Configuration Lifecycle

The kernel must provide read/write `buffer/enable` and `buffer/length`
attributes, support `poll()` and nonblocking `read()`, and return complete scans
from each read. `buffer/watermark` is optional; when present, it must be writable.

The kernel must support the combination of all required channels and the optional
temperature channel, and allow other scan channels to be disabled. A buffer has
one exclusive consumer. Its initial enabled state is unrestricted: PX4 disables
it before configuration, enables it after configuration, and disables it on
normal shutdown.

The channel selection, scan format, scales, offsets, raw ranges, sampling rate,
and timestamp clock must remain unchanged during acquisition. Changing these
settings requires acquisition to stop and metadata to be read again before
acquisition resumes.

## References

- [Linux IIO scan layout and buffers](https://docs.kernel.org/iio/iio_devbuf.html)
- [Linux IIO sysfs ABI](https://github.com/torvalds/linux/blob/master/Documentation/ABI/testing/sysfs-bus-iio)
