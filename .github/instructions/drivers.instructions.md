---
applyTo: "src/drivers/**,src/modules/cyphal/**"
---

# Drivers/CAN Review Guidelines

In addition to the core code review guidelines:

- CAN bus devices behave differently from serial/SPI; check driver assumptions
- ESC index mapping: telemetry index != channel when motors are disabled
- ESC hardware quirks: 4-in-1 ESCs may report current on only one channel
- Verify device_id correctness and I2CSPIDriver patterns
- Time representation: prefer `hrt_abstime` over iteration counts
- Code shared by several drivers of one type goes in `src/lib/<topic>/` (e.g. `src/lib/rc/`); `src/drivers/<type>/` holds only per-driver directories, no loose `.cpp`/`.h`
