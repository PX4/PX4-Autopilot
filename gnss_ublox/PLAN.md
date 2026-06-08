# PLAN — In-tree u-blox GNSS driver + generic module firmware-update-over-CAN

Tracker for the work on branch `wip/ublox-gnss-driver` (cut off `014098f62b`, the
Teseo fw-proxy prototype). Two intertwined goals:

1. **De-submodule the u-blox driver** — stand up a dedicated, self-contained driver at `src/drivers/gnss/ublox/`, dropping the `PX4-GPSDrivers` submodule for this driver. Used by the ARK RTK GPS cannode (`ark_can-rtk-gps`).
2. **Generic firmware-update-over-CAN** — let any CAN node pull a firmware image for its attached peripheral from the flight controller's DroneCAN file server, version-gated, and write it to the peripheral. u-blox is the first consumer; the flashing protocol itself is stubbed for now.

This started as the Teseo X-Loader proxy prototype (`src/drivers/teseo_fw_proxy/`). The Teseo module is a custom out-of-tree part; this work re-targets the same concept at the in-repo u-blox driver and generalizes the transport so Teseo/Septentrio/etc. can reuse it.

---

## Decisions locked

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | **New parallel driver** `src/drivers/gnss/ublox/`. The generic `gps` driver + `PX4-GPSDrivers` submodule stay intact for all other boards/receivers; the cannode opts in. | Lowest blast radius; ships incrementally; mirrors how `gnss/septentrio` already coexists with `gps`. Repo-wide replacement would drop MTK/Ashtech/Emlid/Femtomes/NMEA/Unicore support. |
| D2 | **Vendor + prune the UBX parser, rewrite the shell.** Bring `ubx.cpp/.h`, `gps_helper.{h,cpp}`, `base_station.h`, `rtcm.{cpp,h}`, `definitions.h` in-tree; delete non-u-blox parsers; rewrite the driver shell (`gps.cpp` → `ublox.cpp`) cleanly, ublox-only. | Keep the battle-tested UBX protocol code (board detect, proto versions, moving base, survey-in, F9P/X20). A clean-room parser rewrite is a 10× effort + regression risk on safety-critical nav parsing. |
| D3 | **FW bytes cross DroneCAN → u-blox via uORB** (byte buffer + queue), not a flash scratchpad. | The cannode has no spare flash to stage a ~1 MB+ image (448 KB total flash, ~78% used); flash staging adds erase stalls + wear for zero benefit. The existing RTCM injection path already proves uORB-in-RAM for this exact shape. |
| D4 | **Two generic layers + one device seam.** (a) generic "DroneCAN file read over uORB" client in `uavcannode`; (b) shared driver-side firmware-update helper in `src/lib/`; (c) device-specific `writeChunk()` (u-blox stub now). | The only non-generic code is the actual flashing. Any cannode driver reuses (a) and (b). |
| D5 | **Generic file-read topic framing.** `uavcannode` has no "firmware" concept — it just proxies `file.GetInfo`/`file.Read` over uORB. | Maximally reusable (config/calibration pulls too); keeps `uavcannode` policy-free. |
| D6 | **Version-aware updating via the request path.** Path is **part-keyed with board override** and encodes the node's **current** version; the FC serves only if a newer image exists. Sentinels: `@0` (unknown / first-flash / recovery → newest), `@force` (newest even if not strictly newer → re-flash/downgrade). | Version gating, force, and boot-time auto-check fall out of one GetInfo. Part-keyed because a chip's firmware (e.g. F9P) is the same across carrier boards; board override gives a per-board approval knob. |
| D7 | **Identifier = descriptive part (+ optional board name), not `hw_version_minor`.** | The cannode supplies the path explicitly, so the FC needs no bus-identity matching; a bare minor int is opaque and collides across vendors, whereas the part/board names are unique and human-readable when dropping files on the SD card. |

**Driver-shape sub-decisions:** single-instance (drop the Main/Secondary dual-spawn from `gps.cpp`); drop SPI and the `__PX4_LINUX` ifdef; module `MAIN ublox`.

---

## Topology — who runs where

| Component | Location | Thread / WQ | Owns | Links libuavcan? |
|-----------|----------|-------------|------|------------------|
| `ublox` driver | `src/drivers/gnss/ublox/ublox.cpp` + vendored parser | own task thread (like `gps` today) | the UART + u-blox lifecycle; FW-update **session master** | **No** |
| `ModuleFirmwareUpdater` helper | `src/lib/module_fw_update/` | runs on the driver thread | GetInfo→Read→write choreography (session/offset/watchdog/retry) | No |
| `UbloxFirmwareUpdater` | `src/drivers/gnss/ublox/` | driver thread | device-specific `begin/writeChunk/finish/abort` (**stub**) | No |
| Generic file-read client | `uavcannode` (new endpoint) | `px4::wq_configurations::uavcan` | `ServiceClient<file::GetInfo/Read>`; uORB ↔ DroneCAN bridge | Yes (already does) |
| File server + version resolver | FC `src/drivers/uavcan/uavcan_servers.cpp` | FC | serve / version-gate `module/…` paths | Yes |

The driver staying libuavcan-free is the whole point of the uORB seam, and it keeps all blocking serial I/O on the driver thread — never on the uavcan WQ — so DroneCAN heartbeats/NodeStatus are protected by construction.

---

## Precedents reused (don't reinvent)

- **RTCM inbound path = the data-flow precedent.** `Subscribers/RTCMStream.hpp` receives `uavcan.equipment.gnss.RTCMStream` on the uavcan WQ, copies bytes into `gps_inject_data_s`, publishes the `gps_inject_data` topic (`uint8[300] data`, `ORB_QUEUE_LENGTH = 8`). The `gps` driver drains it in `handleInjectDataTopic()` (`src/drivers/gps/gps.cpp:557`) and writes to the UART. Our file-read client is the *reverse* of this (a uORB-sub that drives a DroneCAN service call), and `Publishers/GnssFix2.hpp` already shows "uORB-sub → DroneCAN action".
- **`src/drivers/gnss/septentrio/` = the structural precedent** for a self-contained `gnss/` driver with no submodule (own decoder, own rtcm, `module.yaml`/`Kconfig`, per-board opt-in).
- **`src/drivers/teseo_fw_proxy/` = the pull-side prototype.** `ServiceClient<uavcan::protocol::file::Read>` bound to `UavcanNode::instance()->get_node()`, sharing the uavcan WQ so the response callback and `Run()` never overlap. This logic migrates into the `uavcannode` file-read client (and gets simpler — no CMake flag replication, direct `get_node()`, no `instance()` hack).

**Two distinct RTCM helpers — keep both, don't conflate:** `src/lib/gnss/rtcm.{h,cpp}` (`gnss::Rtcm3Parser`, used by the *shell* to reassemble injected RTCM frames) vs the vendored `devices/src/rtcm.{h,cpp}` (`RTCMParsing`, used by *ubx.cpp* for base-station RTCM output).

---

## Layer 1 — generic DroneCAN file-read over uORB (in `uavcannode`)

Two vendor-neutral topics; `uavcannode` subscribes the request topic and issues the service call, and the `ServiceClient` response callback (same uavcan WQ as RTCMStream → no lock) publishes the response topic. Gated by a Kconfig like the other endpoints (e.g. `UAVCANNODE_FILE_CLIENT`).

```
# request  (driver → uavcannode)
uint32  session_id
uint8   cmd            # GETINFO | READ | ABORT
char[N] path           # e.g. "module/ublox-f9p@1.32"
uint32  offset
uint8   server_node_id # FC file server; 0 = auto/discover

# response (uavcannode → driver)
uint32  session_id
uint8   cmd
uint32  offset
uint16  len
uint8   eof
uint64  file_size      # from GETINFO
int16   error          # 0 = ok; distinct value = "no update needed"
uint8[256] data        # 256 = file.Read v0 payload cap
```

GetInfo + Read are both available: `…/dsdlc_generated/uavcan/protocol/file/GetInfo.hpp` and `Read.hpp`; the FC backend implements both (`uavcan_drivers/posix/include/uavcan_posix/basic_file_server_backend.hpp:350` `getInfo()`).

## Layer 2 — shared driver-side helper (`src/lib/module_fw_update/`)

Owns the choreography so every device driver reuses it: GetInfo → Read loop → offset / session / watchdog / retry → done/fail. Device drivers implement a small virtual interface — the **only** per-device code:

```
begin(total_size)
writeChunk(offset, data, len)
finish()
abort()
```

u-blox provides `UbloxFirmwareUpdater` (stubbed now: log + checksum, like Teseo's `mock_xloader_write`). Because the helper runs on the driver thread, `writeChunk` may block freely without touching the WQ.

---

## Path grammar & version gating

The request path is a **version-gated query**, not a literal filename:

```
module/<part>@<current-version>                 # canonical, part-keyed
module/<board-name>/<part>@<current-version>    # board override (resolver prefers this when present)

e.g.  module/ublox-f9p@1.32
      module/org.ark.can-rtk-gps/ublox-f9p@1.32
```

- `<part>` from the driver (u-blox: `_board` enum → `ublox-f9p` / `ublox-x20` / …).
- `<current-version>` from the driver (u-blox: the `FWVER=` token in MON-VER, which `payloadRxAddMonVer()` in `ubx.cpp` already parses — currently just logged).
- `<board-name>` from a param/default (the cannode's identity, e.g. `org.ark.can-rtk-gps`).

**FC resolver** — a path-rewriting shim wrapping `BasicFileServerBackend` (NOT a fork). On `getInfo`/`read` for a `module/…` path it parses `(part, board?, requested_version)`, scans for the highest available (board-specific dir preferred, else part-keyed), and:

- highest **>** requested → rewrite to that real file and serve normally. Read is stateless/offset-keyed, so the resolver maps deterministically to the same file each call while the dir is stable.
- highest **≤** requested → return the distinct **"no update"** error. The helper maps that to *up-to-date, success, no flash*.

Binaries on the SD card carry their version: `…/ublox-f9p@1.40.bin` under `UAVCAN_FIRMWARE_PATH` (`/fs/microsd/ufw`, `uavcan_module.hpp:53`), placed the same way as cannode `.uavcan` files.

**Sentinels:** `@0` → unknown / first-flash / recovery (blank or bricked module that can't report MON-VER) → serve newest unconditionally. `@force` → serve newest even if not strictly newer (re-flash same / downgrade / repair).

**Boot-time auto-check falls out for free:** at startup (or on a timer) the driver issues one GetInfo for its current-version path; bytes back → flash, "no update" → carry on. Drop a newer binary on the SD card and nodes pick it up on next boot.

---

## FW-update session protocol (lossless lockstep)

One chunk in flight by construction → lossless and in-order with zero staging. The driver is the session master; the `uavcannode` client is a reactive byte fetcher.

1. **Trigger** (`ublox fw_update …` command for now) → driver leaves the `receive()` nav loop, `session_id++`, `updater.begin()` (stub).
2. Driver requests **GETINFO** for the version-gated path. On "no update" → done (already current). On OK → record `file_size`, proceed.
3. Driver requests **READ** at `offset`. `uavcannode` issues `file.Read`; the response callback (uavcan WQ, no lock) publishes the chunk — or `error` (libuavcan's 500 ms request timeout surfaces here).
4. Driver gets the chunk (matching `session_id` + `offset`), `updater.writeChunk()` (stub, blocking on its own thread), advances `offset`; loops to step 3, or on `eof` → `updater.finish()` → resume nav.
5. **Watchdog:** no chunk for the current offset within T → re-request (idempotent), N retries → abort + resume nav.

Per-chunk cost = two thread handoffs + a CAN RTT; ~4096 chunks for a 1 MB image (256 B v0 cap) is tens of seconds — fine for firmware. Pipelining (2 outstanding) is a later optimization; strict lockstep for the stub.

---

## Staging

### Stage 1 — de-submodule + u-blox-only driver (behavior-preserving, no FW)
1. Create `src/drivers/gnss/ublox/`; vendor `ubx.cpp/.h`, `gps_helper.{h,cpp}`, `base_station.h`, `rtcm.{cpp,h}` (the `RTCMParsing` one), `definitions.h`. Drop `crc.*` (ubx doesn't use it) and all non-ublox parsers.
2. Write `ublox.cpp` = `gps.cpp` pruned to UBX-only: remove `gps_driver_mode_t`, the auto-detect loop, the `#ifndef CONSTRAINED_FLASH` device branches; keep the `callback()` glue, RTCM injection (`gnss::Rtcm3Parser`, `DEPENDS gnss`), publishers, all `GPS_UBX_*` params, board detection, reset. Single-instance; module `MAIN ublox`.
3. `CMakeLists.txt` / `Kconfig` (`DRIVERS_GNSS_UBLOX`) / `module.yaml` / `params.yaml` modeled on `gnss/septentrio/`. No `px4_add_git_submodule`.
4. Cannode wiring: `boards/ark/can-rtk-gps/default.px4board` `DRIVERS_GPS` → `DRIVERS_GNSS_UBLOX`; `init/rc.board_sensors` `gps start -d /dev/ttyS0 -p ubx` → `ublox start -d /dev/ttyS0`.
5. Checkpoint: `make ark_can-rtk-gps_default`; verify nav parity + flash size.

### Stage 2a — generic file-read pipe + helper + u-blox stub (literal path)
1. New uORB msgs for the Layer-1 request/response topics (register in `msg/CMakeLists.txt`).
2. Generic file-read client endpoint in `uavcannode` + Kconfig gate. Migrate the Teseo `ServiceClient` logic here (simplified). GetInfo + Read.
3. `src/lib/module_fw_update/` helper (choreography + virtual interface).
4. `UbloxFirmwareUpdater` stub; wire the driver's `fw_update` command + FW-mode loop driving the helper.
5. Checkpoint: drop a dummy binary on the FC at a literal path, run the trigger, watch GetInfo→Read→stub-write→DONE end to end.

### Stage 2b — version gating
1. FC resolver shim over `BasicFileServerBackend` (part-keyed + board override; highest-version match; "no update" error).
2. Sentinels `@0` / `@force`; driver builds the version-gated path from MON-VER.
3. Boot-time auto-check (one GetInfo).
4. Remove the Teseo prototype (`src/drivers/teseo_fw_proxy/`, its board config + Kconfig).

### Stage 3 — productionization (later)
Real u-blox bootloader protocol in `UbloxFirmwareUpdater` (safeboot/flash/per-block ACK/baud handshake/image verify); production trigger (param or a DroneCAN `BeginFirmwareUpdate` flow) instead of the shell command; FC-side `.bin` delivery UX + GCS progress; `FW.db`/version-DB integration for auto-resolve; robustness (resume-from-offset, abort, CRC gating, brown-out safety). Teseo/Septentrio adopt the helper.

---

## Open items

- **O1 — version token format.** FWVER strings are messy (`HPG 1.32`, `SPG 5.10`, `EXT CORE 1.00 (…)`); define one comparable dotted-numeric token + comparison rule for all devices. *(Owner: Jake — to define after reviewing how u-blox versions its modules.)*
- **O2 — "no update" error code.** Pick a specific GetInfo error value, distinct from real errors (malformed path / IO).
- **O3 — trigger + path-override param/command names.**
- **O4 — `server_node_id` discovery** (how the cannode learns the FC file-server node id; param vs observe-the-bus vs `0=auto`).
- **O5 — param ownership.** `ublox/params.yaml` redefines the `GPS_UBX_*` / `GPS_1_GNSS` set under the same names; a board runs `gps` **or** `ublox`, not both (duplicate keys collide). Revisit if a shared params yaml is wanted.

---

## Planned file inventory

**Add**
- `src/drivers/gnss/ublox/` — `ublox.cpp`, vendored `ubx.cpp/.h`, `gps_helper.{h,cpp}`, `base_station.h`, `rtcm.{cpp,h}`, `definitions.h`, `UbloxFirmwareUpdater.{h,cpp}`, `CMakeLists.txt`, `Kconfig`, `module.yaml`, `params.yaml`.
- `src/lib/module_fw_update/` — helper + `CMakeLists.txt`.
- `msg/` — Layer-1 file-read request/response messages (+ `msg/CMakeLists.txt`).

**Modify**
- `src/drivers/uavcannode/` — new file-read client endpoint, `Kconfig`, `CMakeLists.txt`, `UavcanNode.cpp` registration.
- `src/drivers/uavcan/uavcan_servers.cpp` (+ a resolver shim) — version-gated `module/…` serving (Stage 2b).
- `boards/ark/can-rtk-gps/default.px4board`, `init/rc.board_sensors` — swap `gps`→`ublox`, swap proxy Kconfig.

**Remove (Stage 2b)**
- `src/drivers/teseo_fw_proxy/` + its board config/Kconfig.

---

## Notes

- **Security (inherited, conscious choice):** the FC UAVCAN v0 file server is unauthenticated — any node can read any file under `ufw`. The version resolver doesn't change that. Worth a code comment.
- **Anchors:** board identity `boards/ark/can-rtk-gps/uavcan_board_identity` (`org.ark.can-rtk-gps`, hw 0.82); `gps.cpp` callback/inject (`:415`, `:557`); `GpsInjectData.msg`; `UavcanNode` on `wq_configurations::uavcan`.
