# PX4 Security Scope

This document says where PX4's security boundary sits, so a reporter can tell before filing whether a finding is a vulnerability, and a maintainer can say why it is or isn't.

It is not a threat model.
It does not enumerate threats, rate risks, or prescribe mitigations.
It describes the boundary the code implements today.

To use it: find your deployment under [Configurations](#configurations), then check the finding against [Always in scope](#always-in-scope) and [Out of scope](#out-of-scope).
Those two lists cover the recurring cases, not every case.
A finding that fits none of them is a judgement call, and maintainers make it on the report.

Report through the GitHub Security tab as described in [SECURITY.md](SECURITY.md), which also lists the supported branches.

## What PX4 protects

PX4 protects one thing: who gets to fly the aircraft, and who gets to make it stop.

Everything else matters because of what it leads to, not in itself.
Logs, parameters and telemetry are worth protecting because of what someone can do with them next.
Reading a flight log is a far smaller problem than flying away with the vehicle.

Two things PX4 cannot promise: that a vehicle will not crash, and that a sensor is telling the truth about the world.

## The boundary

```
                 integrator's responsibility
        ..............................................
        :  telemetry radio    RC link    companion   :
        :        |              |         network    :
        :        |              |            |       :
        ''''''''''''''''''''''''''''''''''''''''''''''
                 |              |            |
              MAVLink          RC      uXRCE-DDS/Zenoh
                 |              |            |
        +--------+--------------+------------+--------+
        |             flight controller               |
        +---+---------------+---------------+---------+
            |               |               |
        SD card       USB / NSH shell    UART/I2C/SPI/CAN
                                          peripherals
```

Everything above the dotted line is the integrator's to secure, and PX4 assumes nothing about who is on it.
Everything below is administrative: a peer with the SD card, the USB port or a bus is trusted as the operator.

PX4's job is what happens at the boxes in the middle: whatever arrives on those interfaces must not do more than the interface is documented to do.

## What PX4 assumes

- **Sensors report within specification.**
  PX4 cannot reliably distinguish a real GPS fix, rangefinder return or gyro reading from a spoofed one.
  Some spoofing detection exists and is best-effort.
- **Peripherals are the ones the operator installed.**
  No bus (UART, I2C, SPI, CAN, SMBus) is authenticated.
  A device on a bus is trusted as whatever driver the operator enabled for it.
- **Physical access is administrative.**
  The SD card holds logs, the MAVLink signing key, some boards' parameters or parameter backups and staged peripheral firmware, and it can contain boot scripts that are run at startup.
  Debug ports and the bootloader allow a full reflash.
- **The onboard shell is administrative.**
  A peer that can open the NSH shell is an administrator.
  NuttX supports a console password; no PX4 board enables it.
- **The offboard transports are inside the boundary.**
  uXRCE-DDS and Zenoh publishers reach uORB directly, including `/fmu/in/actuator_motors`, `/fmu/in/actuator_servos` and `/fmu/in/vehicle_command`.
  They carry no external-origin marking, so the command guards that apply to MAVLink do not apply to them.
  A peer that can reach those transports is the operator, which is why keeping that network to trusted parties is the integrator's job and not an optional extra.
- **Securing the links is the integrator's job.**
  See [MAVLink Security Hardening](docs/en/mavlink/security_hardening.md).

These assumptions are the boundary.
If a report starts from one of them already being true, that the attacker has the SD card, or is on the companion network, then it is describing how PX4 works rather than finding a hole in it.

## Configurations

### As shipped

Every control interface is unauthenticated, unsigned and unencrypted.
Anyone who can reach a link can do anything the operator can do: arm, disarm, change mode, upload missions and geofences, set parameters, open a shell over `SERIAL_CONTROL`, read and write files over MAVLink FTP, terminate flight.
[MAVLink Security Hardening](docs/en/mavlink/security_hardening.md) lists these capabilities in full.

This is the documented default and it is not a defect.
In this configuration PX4 guarantees only what the [Always in scope](#always-in-scope) list names.

### Hardened

An integrator can raise that, and the options are:

- **Secure the link below PX4.**
  An encrypted radio, a VPN or IPsec gives confidentiality and authentication using standard, reviewed cryptography.
  This is the strongest option and PX4 is not involved in it.
- **Isolate the offboard transports.**
  uXRCE-DDS and Zenoh bypass every MAVLink control, so if an adversary can reach them, the rest of this list buys nothing.
- **Enable [MAVLink message signing](docs/en/mavlink/message_signing.md).**
  This authenticates MAVLink frames but does not encrypt them.
  The protocol was audited when it was drafted; PX4's implementation of it has not been.
  A small allowlist (`HEARTBEAT`, `RADIO_STATUS`, `ADSB_VEHICLE`, `COLLISION`) is always accepted unsigned.
  Signing is inactive if the key file is absent, which is deliberate: removing the card is the recovery path when a key is lost.
- **Lock the configuration.**
  Read-only parameters, [secure boot](docs/en/advanced_config/bootloader_secure_boot.md) with a replaced key, and no wired reflash.
  The in-tree secure boot variant ships a publicly committed test key that an integrator must replace.

What each of these buys is described where it is documented.
PX4 makes no guarantee that they combine into a secure deployment; that assessment belongs to the integrator.

## Always in scope

Regardless of configuration, and regardless of how open the link is:

- **Memory corruption.**
  Input on any link or bus that corrupts memory or executes code.
- **Hangs and races.**
  Input that stalls a control path, spins a work queue, or corrupts state through timing or ordering rather than through malformed content.
- **Any effect beyond the documented capability set.**
  If a link peer can reach an effect that is not documented as reachable, that is a bug until the documentation is corrected.
  MAVLink FTP escaping the directory it advertises is this class.
- **Pivoting to a bus that was not otherwise reachable.**
  Using the flight controller to reach an ESC, GPS or CAN node bootloader behind it, for example through `TUNNEL` or `SERIAL_CONTROL` passthrough.
  Reaching the link grants the link, not the peripherals behind the controller.
- **Persistence.**
  Anything planted over a link that outlives the attacker's access to it: a boot script, flashed peripheral firmware, an overwritten parameter store.

An in-tree board configuration is PX4's responsibility, not the integrator's.
"The integrator should have changed it" does not apply to a default that PX4 ships.

## Out of scope

- **Using an unauthenticated link as it is documented to work.**
  Commanding the vehicle over an unsigned link, when the only precondition is reaching the link.
  The list above still applies.
- **Physical access**, except where a board is configured with a mechanism whose purpose is to resist it.
  On a secure boot board with a replaced key, wired reflash comes back into scope.
- **Spoofed physical inputs.**
  GPS spoofing, acoustic or EMI injection, magnetic interference.
  In scope only if PX4 mishandles the resulting bytes.
- **Plaintext telemetry.**
  PX4 does not encrypt telemetry, and eavesdropping on a link is not a finding on its own.
- **Code that only ever runs in simulation.**
  A finding in a simulator-only code path is a bug, not a vulnerability.
  This is about where the affected code runs, not where you reproduced it: if the code also runs on a flight target, a SITL reproducer is fine, and [SECURITY.md](SECURITY.md) asks for one.
- **Unsupported branches.**
  See [SECURITY.md](SECURITY.md).

A parameter precondition does not put a finding out of scope if the same link can set that parameter.

## Severity

Decide scope first using the two lists above.
A finding that is out of scope is closed, not scored.

What remains is scored with CVSS, because that is what GitHub advisories carry.
When choosing the impact metrics, rate the effect on the aircraft rather than on the data, in roughly this order:

1. Taking control of a vehicle that is flying or could be told to fly, and running your own code on it.
2. Forcing an unsafe state: corrupting the estimator, or spoofing a condition that trips a failsafe.
3. Persistence across reboot.
4. Loss of control or telemetry links, and disclosure.

A safety feature that happens to limit an attacker, such as a geofence around a hijacked vehicle, is not a security control and does not reduce severity.
Someone who is already flying the vehicle can reconfigure it.
