# Drivers

- Verify device configuration once, in `Configure()`. Never re-read or re-write registers from the run loop: no periodic `RegisterCheck`.
- Keep one scale for the life of the driver. Every sample in a FIFO batch shares a scale, so pick the wider range in `Configure()` and never switch it at runtime.

Older drivers still do both; change that in a PR about it, not as a side effect.
