# Security

Securing PX4 for a production vehicle is the integrator's responsibility.

::: tip
PX4 is open by default: every PX4 control interface is unauthenticated, unsigned and unencrypted, and any peer that can reach a link can command the vehicle.
This is intended!
:::

This section provides links to security documentation for integrators and security researchers.

## Integrators

- [MAVLink Security Hardening](../mavlink/security_hardening.md) — what an unauthenticated link exposes, and the checklist for a production deployment.
- [MAVLink Message Signing](../mavlink/message_signing.md) — authenticating MAVLink frames, and what signing does not cover.
- [Bootloader Secure Boot](../advanced_config/bootloader_secure_boot.md) — verifying firmware at boot, and replacing the committed test key.
- [Log Encryption](../dev_log/log_encryption.md) — encrypting flight logs at rest.
- [Read-Only Parameters](../advanced/parameters_and_configurations.md#read-only-parameters) — locking down settings that end users should not change.

Securing the link itself sits below PX4.
An encrypted radio, a VPN or IPsec uses standard cryptography and protects every interface at once, including the ones MAVLink signing does not cover.

## Security Researchers

- [Security Policy](https://github.com/PX4/PX4-Autopilot/blob/main/SECURITY.md) — supported versions, how to report a vulnerability, and the rules for AI-assisted findings.
- [Security Scope](https://github.com/PX4/PX4-Autopilot/blob/main/SECURITY_SCOPE.md) — where PX4's security boundary sits, and what is in scope for a report.
