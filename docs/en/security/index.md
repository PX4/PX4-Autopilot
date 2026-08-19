# Security

This section covers PX4 platform security: what PX4 protects, how to harden a
deployment for production, and how to report a vulnerability.

PX4 is flight-control firmware used by manufacturers and integrators to build
real vehicles. Security here means protecting **control authority** over the
vehicle from a deliberate attacker. This is distinct from
[Safety](../config/safety_intro.md), which protects against component failure
and operator error. The two overlap (a geofence limits a runaway *and* a
hijacked vehicle), but they defend against different things.

::: warning
MAVLink is an open protocol and is **unauthenticated by default**. Any device
that can reach a MAVLink link can command the vehicle, including shell access,
file transfer, parameter changes, and flight termination. Securing the link is
the integrator's responsibility. See [Security Hardening](../mavlink/security_hardening.md).
:::

## Securing a Deployment

For anyone taking PX4 to production:

- [Security Hardening](../mavlink/security_hardening.md) — the production
  hardening checklist. Start here.
- [Message Signing](../mavlink/message_signing.md) — cryptographic
  authentication for MAVLink. The mechanism PX4 ships for closing the open-link
  boundary.
- [Bootloader Secure Boot](../advanced_config/bootloader_secure_boot.md) —
  verified firmware boot on supported hardware.
- [Log Encryption](../dev_log/log_encryption.md) — encrypting flight logs at
  rest.
- [Read-Only Parameters](../advanced/parameters_and_configurations.md#read-only-parameters) —
  locking down safety-critical or product-defining parameters so end users
  cannot change them.

## Reporting and Scope

The security process documents live in the PX4-Autopilot repository, not on this
site, so there is one authoritative copy that does not fork per release or per
translation:

- [Reporting a Vulnerability](https://github.com/PX4/PX4-Autopilot/blob/main/SECURITY.md) —
  how to report, supported versions, and the response process.
- [Threat Model](https://github.com/PX4/PX4-Autopilot/blob/main/THREAT_MODEL.md) —
  what PX4 defends against, what it does not, and how a report is judged to be a
  security bug or not. Read this before reporting.
