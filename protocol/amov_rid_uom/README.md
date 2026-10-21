# AMOV RID / UOM protocol v1

This directory is mirrored byte-for-byte in PX4-Autopilot, ArduRemoteID and
QGroundControl. `amov.xml` is the common message definition. Change all mirrors
together and run `test_protocol.py` before releasing a matched set of builds.
The generator merges the extension into a **build-directory copy** of the
repository's pinned MAVLink dialect. It never edits an upstream submodule and
includes the project messages in the ordinary MAVLink CRC/dispatch tables.

## MAVLink contract

MAVLink 2 is required. These project-local message IDs are checked for conflicts
against the upstream XML set at generation time.

| Message | ID | Payload bytes | CRC extra |
| --- | ---: | ---: | ---: |
| `AMOV_RID_IDENTITY_REQUEST` | 61000 | 16 | 26 |
| `AMOV_RID_IDENTITY` | 61001 | 224 | 250 |
| `AMOV_UOM_STATUS` | 61002 | 72 | 206 |

Field definitions and enum values are in `amov.xml`. Every message carries
`protocol_version=1`; a different version is unsupported, not a partial success.
Fixed-width strings can fill the whole field and need not end in NUL. Consumers
must use bounded copies. Identity strings use non-space ASCII (33–126), with unused
bytes zero-filled. Empty complete registration means unknown; the broadcast
suffix must never be substituted for it.

The ground station generates a nonzero random `session_id` for a connection
generation and a nonzero `request_id` for each query. PX4 returns its own nonzero
random `boot_id`, the RID's nonzero `rid_boot_id`, and the selected identity.
An unactivated RID remains queryable: `result=OK`, `activation_state=UNACTIVATED`,
`identity_revision=0` and an empty identity are a valid diagnostic response.
They never establish reporting or arming readiness. `pending_revision` is
informational; an uploaded package has not become effective yet.

Reporting status is bound to the source system/component, both sessions, RID
boot, identity revision and digest. PX4 invalidates the binding on device loss,
replacement, restart or identity change. Late network callbacks and old MAVLink
status must not restore a previous binding. A vehicle has one reporting owner;
a different live owner must not silently replace it. An expired owner can be
replaced through a new identity handshake.

Send reporting status on changes and at 1 Hz, including failures. Receive age is
measured with PX4's local monotonic clock; the heartbeat expires after 3 seconds.
In the v1 mock contract, a successful provider acknowledgement expires after
5 seconds. `last_ack_age_ms=UINT32_MAX` means no acknowledgement. Continued
heartbeats cannot renew an old platform acknowledgement. Sequence comparison is
uint32 wrap-aware; duplicate and older sequences do not renew state.

Ready requires READY or REPORTING, all four prerequisite flags, a fresh accepted
provider acknowledgement, and independent PX4 checks of the same RID identity,
activation and ArmStatus. Production firmware rejects MOCK. Only a distinct
explicit test build accepts mock readiness. These messages do not authorize
mode changes: an isolated UOM failure in flight produces warnings and logs.
The existing RID failure action remains independent.

## DroneCAN read-only identity snapshot

Use standard `uavcan.protocol.param.GetSet`. Integer values use the existing
integer union member; strings use the string member. Each key fits the existing
16-character parameter-name bound. The ordinary numerical parameter bridge is
unchanged.

| Key | Value |
| --- | --- |
| `RID_ID_SCHEMA` | Integer 1 |
| `RID_BOOT_ID` | Nonzero uint32, regenerated on every boot |
| `RID_ID_REV` | Effective uint32 signed revision; 0 before activation |
| `RID_ACT_STATE` | 0 unactivated, 1 active, 2 invalid |
| `RID_UAS_ID` | Exactly 20 non-space ASCII characters when active |
| `RID_REG_ID` | Complete registration: empty or 8–64 non-space ASCII characters |
| `RID_UIN` | Optional broadcast suffix, 0 or 8 non-space ASCII characters |
| `RID_CLASS` | Integer 0–4 |
| `RID_UID` | Twelve uppercase hex digits from the stable factory MAC |
| `RID_ID_HASH` | 64 uppercase hex digits encoding BLAKE2b-256; empty before activation |
| `RID_PENDING` | Pending uint32 revision; 0 if none |
| `RID_ACT_ERR` | Activation transaction result code; separate from effective activation |

Read boot and revision, read all fields, then read revision and boot again.
Discard and retry a bounded number of times if either changes. Responses must
come from the selected node and match the requested key, type and length.
Select and retain the node's GetNodeInfo hardware UID as well as its current
node ID. ArmStatus from another node cannot satisfy this identity's health check.

When complete registration is present, the signed UIN must explicitly equal its
last eight characters. When registration is unknown, UIN may independently be
empty or eight characters. This is the project identity-consistency rule, not a
claim to validate an official registration-number format. The issuer may derive
the suffix before signing; the RID must not silently fill signed fields afterward.

## Identity digest

Hash exactly these 115 bytes using BLAKE2b with a 32-byte output:

| Bytes | Encoding |
| ---: | --- |
| 10 | ASCII `RID-ID-V1` followed by NUL |
| 4 | Effective revision, little-endian uint32 |
| 6 | Stable factory MAC, raw bytes in displayed order |
| 1 | Aircraft class |
| 20 | Aircraft product identifier, ASCII |
| 1 | Complete registration length |
| 64 | Complete registration, NUL-padded |
| 1 | Broadcast suffix length, 0 or 8 |
| 8 | Broadcast suffix, NUL-padded |

The digest does not include boot IDs, pending state or activation progress.
It is hex text on GetSet and raw 32 bytes in MAVLink. The authorization signature
is verified by the RID; this digest is for identity consistency, not a second
signature scheme.

## Generation and tests

Use the repository's pinned pymavlink package (its parent directory is passed
as `--pymavlink-root`):

```sh
python3 protocol/amov_rid_uom/generate.py \
  --definitions src/modules/mavlink/mavlink/message_definitions/v1.0 \
  --pymavlink-root src/modules/mavlink/mavlink \
  --dialect common --output /tmp/amov-c --work-dir /tmp/amov-xml
```

For a bench client, add `--language Python3` and set `--output` to the desired
Python module filename. PX4 and QGC application builds run the C generator
automatically. CMake sets `PYTHONHASHSEED=0`; generated headers omit the build
date so repeated builds use stable generated metadata.

`test_protocol.py --help` lists inputs for the two upstream definition trees.
Use `--pymavlink-root` for PX4 and `--qgc-pymavlink-root` for QGC when their
pinned generators differ; a newer XML schema must not be tested with the older
generator merely because the custom message fields match. The test compiles a
C sender/parser against each generated dialect, exchanges
all three messages in both directions, rejects corrupted frames, checks the
frozen wire lengths/CRC extras, verifies mirrors and rejects ID collisions.
It tests software wire compatibility; it is not a physical CAN or radio test.
