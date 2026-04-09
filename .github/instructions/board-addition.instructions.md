---
applyTo: "boards/**"
---

# Board Addition Review Guidelines

In addition to the core code review guidelines, when reviewing new board additions:

- **Flight logs**: require a link to https://logs.px4.io demonstrating basic operation for the vehicle type (hover for multicopters, stable flight for fixed-wing, driving for rovers, etc.); short bench-only logs are insufficient
- **Documentation**: require a docs page in `docs/en/flight_controller/` with pinout, where-to-buy, connector types, version badge, and manufacturer-supported notice block
- **USB VID/PID**: must not reuse another manufacturer's Vendor ID; manufacturer must use their own
- **Board naming**: directory is `boards/{manufacturer}/{board}/`, both lowercase, hyphens for board name
- **Unique board_id**: registered in `boards/boards.json`, no collisions
- **Copied code cleanup**: check for leftover files, configs, or comments from the template board. Ask "Is this real or leftover?"
- **RC configuration**: prefer `CONFIG_DRIVERS_COMMON_RC` over legacy `CONFIG_DRIVERS_RC_INPUT`
- **No board-specific custom modules**: reject copy-pasted drivers (e.g., custom heater) when existing infrastructure works
- **Bootloader**: expect a bootloader defconfig (`nuttx-config/bootloader/defconfig`) or explanation of shared bootloader
- **CI integration**: board must be added to CI compile workflows so it builds on every PR
- **Flash constraints**: verify enabled modules fit in flash; we are running low across all board targets
- **Port labels**: serial port labels must match what is physically printed on the board
- **Hardware availability**: for unknown manufacturers, verify the product exists and is purchasable (no vaporware)
