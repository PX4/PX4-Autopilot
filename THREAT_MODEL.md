# PX4 Threat Model

This document states what PX4 defends against, what it does not, and how an
incoming security report is judged. It is normative: where it and a triage
opinion disagree, this document wins. It describes `main`; code references are
symbols, not line numbers, and were verified at `09ce800969b`.

A bug that lets an attacker cross one of the boundaries described here is a
security bug. A bug that becomes reachable only after another boundary was
already crossed is a weakness, not a vulnerability. PX4 hardens against
weaknesses, but a weakness alone is not a security bug.

## What PX4 protects

PX4's asset is **control authority over a physical vehicle**: who or what
decides where it flies and whether it stops flying. Confidentiality, logs, and
parameters matter only as paths to that authority. The worst outcome is not
disclosure of data; it is an attacker flying, crashing, or disabling an
aircraft. PX4 is a safety system, so a single security failure can become a
physical hazard, and the threat model is written in those terms.

PX4 cannot promise a vehicle will not crash. It cannot promise a sensor is
telling the truth about the physical world. It does not encrypt telemetry.
Where the text below says "guarantee," it means a property PX4's own code
enforces, not an assurance about the outcome of a flight.

## What PX4 assumes

Like any system, PX4 trusts the layer beneath it. It assumes:

- **Sensors report within specification.** PX4 cannot distinguish a real
  rangefinder return, GPS fix, or gyro reading from a spoofed one. Defending
  the physics is outside firmware.
- **Peripherals are the ones the operator installed.** No bus (UART, I2C, SPI,
  CAN, SMBus) is authenticated. A device on a bus is trusted as whatever driver
  the operator enabled for it.
- **The link is the integrator's to secure.** MAVLink is an open protocol.
  Reaching a MAVLink link and commanding the vehicle is the design, not a
  defect. Link security (radio choice, RF range, network segmentation, a VPN)
  lives below PX4. Message signing is the mechanism PX4 ships for integrators
  who need to close that boundary.
- **The SD card is code.** Boot scripts, the signing key, some boards'
  parameter stores, and staged peripheral firmware all live on it. Physical
  control of the card is trusted.
- **The companion computer is inside the boundary.** uXRCE-DDS and Zenoh
  publishers get direct uORB access with no external-origin tag
  (`from_external` is set in `mavlink_receiver` and nowhere in
  `uxrce_dds_client` or `zenoh`). A companion that can reach those transports is
  the operator.
- **The onboard shell and the wired bootloader are administrative.** A peer that
  can open the NSH shell or reflash over USB is trusted as an administrator.

These assumptions are the boundary. A report that an attacker who already holds
one of them can then do harm is describing the assumption, not breaking it.

## What PX4 guarantees, by configuration

PX4 does not have one guarantee set. It has a configuration, and the guarantees
are a function of where the deployment sits on the ladder below. A report is
judged against the tier the reported system is actually in.

### Tier 0 — Open (no signing key, shipped default)

No authentication on any control interface. A peer within reach of any
configured link holds the authority of the operator: arm, disarm, mode change,
mission and geofence upload, `PARAM_SET`, `SERIAL_CONTROL` shell, MAVLink FTP,
flight termination. This is the documented default. At Tier 0, PX4 guarantees
exactly two things:

1. **Parser memory safety.** Malformed input on any link or bus does not
   corrupt memory or execute code. This is the class PX4 ships fixes for
   (crsf_rc, tattu_can, lightware, `MavlinkLogHandler`).
2. **A complete capability list.** The set of effects a link peer can reach is
   exactly the set published in the [security hardening
   guide](docs/en/mavlink/security_hardening.md). A reachable effect that is not
   on that list is a bug.

Everything else at Tier 0 is published capability, not a defense.

### Tier 1 — Authenticated (signing key provisioned, v1.17+)

With a key on the SD card, PX4 acts on no inbound MAVLink message other than the
unsigned allowlist (`HEARTBEAT`, `RADIO_STATUS`, `ADSB_VEHICLE`, `COLLISION`)
unless the frame carries a valid signature. Operator and attacker become
distinct principals, and three properties become real:

- The operator can take back control; an unauthenticated peer cannot take it.
- Failsafes react to genuine conditions, not forged ones.
- Irreversible actions (terminate, disarm, signing changes) require
  authentication, and signing changes are refused while armed.

Tier 1 has named limits: verification runs in the pinned MAVLink submodule;
signing fails **open** if the key cannot be read at startup; the allowlist above
is always accepted and includes an actuating message (`ADSB_VEHICLE`); and
signing is authentication, not encryption. Telemetry is still plaintext.

### Tier 2 — Authenticated and offboard-isolated

Tier 1 only holds if the adversary cannot reach the uXRCE-DDS or Zenoh
transport. Those publishers drive `/fmu/in/actuator_motors`,
`/fmu/in/actuator_servos`, and `/fmu/in/vehicle_command` with no external-origin
check, so a companion-LAN peer bypasses signing entirely. **Signing on a vehicle
whose offboard transport is reachable buys nothing.** Isolating that transport
is a precondition of Tier 1 being real, not an optional extra.

### Tier 3 — Verified boot and locked configuration

Secure boot with an integrator-provided key, a read-only parameter set, and
wired reflash disabled. Physical and wired access partially enter scope here.
No in-tree board reaches this tier without integrator work, and the one board
with secure-boot defaults ships a **publicly committed test key** that must be
replaced.

Each tier names what is missing to climb to the next. The gaps are the roadmap.

## What does not constitute a security bug

Concrete classes, pre-decided so a report is matched to a rule instead of an
argument. These carve out of the guarantees above.

- **Using an unauthenticated link as designed.** Commanding the vehicle over an
  unsigned MAVLink link is the documented default (see Tier 0). Securing the
  link is the integrator's job. This covers the recurring reports:
  `SERIAL_CONTROL` shell access, MAVLink FTP file operations, `PARAM_SET` of
  safety parameters, mission and geofence overwrite, remote arm/disarm, and
  flight termination, when the only precondition is "reached the link." The
  exceptions below remain in scope even here.
- **Memory corruption is always in scope.** A crafted frame that corrupts
  memory or executes code beyond the protocol's semantics is a bug on any link,
  signed or not. Openness of the link never excuses it.
- **A capability outside the published set is in scope.** A reachable effect not
  in the capability list is a bug until the list is updated, regardless of tier.
- **A pivot to another bus is in scope.** Using the flight controller to reach a
  bus that was not otherwise reachable (for example `TUNNEL` or
  `SERIAL_CONTROL` passthrough to an ESC, GPS, or CAN node bootloader) is a bug.
  The link's openness grants the link, not the peripheral behind the controller.
- **Persistence is in scope.** An artifact planted over a link that survives the
  attacker leaving it (a boot script, flashed peripheral firmware, an
  overwritten parameter store) is a persistence primitive, judged on its own,
  not dismissed as transient link access.
- **Spoofed physical inputs are out of scope.** GPS spoofing, acoustic or EMI
  gyro injection, and magnetic interference are attacks on the physics PX4
  assumes. They are in scope only if PX4 mishandles the resulting bytes (a
  parser bug), not the physical value.
- **Physical access is out of scope,** except on a board explicitly configured
  with a mechanism whose purpose is to defend against it. SD card extraction or
  modification, debug ports, and wired bootloader reflash are assumed. On a
  secure-boot board with a replaced key, wired reflash re-enters scope.
- **In-tree board defaults are PX4's responsibility.** A board configuration
  shipped in this repository is PX4's, not the vendor's. "The integrator should
  have changed it" does not apply to a default PX4 ships.
- **SITL is a deployment, not a reproducer.** A crash reproducible only in SITL
  or simulation is a bug, not a vulnerability, unless it also reproduces on a
  shipping flight target. SITL is excluded from severity, not accepted as proof.
- **A parameter precondition demotes only if the same link cannot set it.**
  "Requires a non-default parameter" is not a mitigation when `PARAM_SET` over
  the reporter's link sets that parameter for free.
- **Unsupported versions are out of scope.** Only the branches listed in
  [SECURITY.md](SECURITY.md) are in scope. A bug must be shown on a supported
  branch.

## Severity is the flight outcome

CVSS asks how an attacker reaches a bug. For a vehicle, the question is what the
bug does to the aircraft. Severity is assigned by outcome, then demoted by the
carve-outs above. The GitHub advisory severity field is set from the mapping in
parentheses.

1. **Catastrophic (Critical).** Control authority or code execution on a vehicle
   that is airborne or can be commanded to fly: actuator control, deliberate
   crash, RCE.
2. **Hazardous (High).** Forged input outside the published set that steers an
   armed vehicle or forces an unsafe failsafe: estimator or navigation
   corruption, spoofed conditions that trip a failsafe.
3. **Persistent (High).** Compromise that survives reboot or the attacker
   leaving: boot-script injection, peripheral or bootloader reflash, parameter
   store overwrite.
4. **Degraded (Moderate).** Denial of service, crash to a safe failsafe,
   disclosure of telemetry or logs.
5. **Weakness (Low).** Bypass of a mitigation with no demonstrated effect, or a
   violation reachable only after another was already achieved.

A safety mechanism that happens to limit an attacker (a geofence bounding a
hijacked vehicle) is not claimed as a security control and does not reduce
severity. It is a safety feature that an attacker with control authority can
reconfigure.
