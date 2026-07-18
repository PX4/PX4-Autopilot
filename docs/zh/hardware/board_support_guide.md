# Manufacturer's PX4 Board Support Guide

This guide explains how to get your hardware supported by PX4: added to the PX4 codebase, listed on the PX4 website, and available to users in QGroundControl.
If you are a hardware manufacturer looking to add a flight controller or peripheral to PX4, or to port PX4 to a new board, start here.

The process is **open and driven through GitHub**.
You do not need permission or a private agreement to start.
When your hardware works with PX4 and you can prove it, you open a pull request, and the maintainers review it in the open.
There is no application form and no waiting on an assignment before you can begin.

The path depends on what you are bringing to PX4:

- [Adding a Flight Controller](#flight-controllers) explains the steps for a **new board that runs PX4**.
- [Adding a Component](#components) explains how to get support for a **peripheral or component** (GPS/RTK receiver, compass, data link, distance sensor, etc.).

:::info
If you only want to know _what_ each support level commits the PX4 team to (versus what you maintain yourself), see [Support Categories](#support-categories) at the end.
:::

## Adding a Flight Controller {#flight-controllers}

### 1. Build your own Firmware Target

The recommended approach for any new flight controller is to **maintain your own board build target based on PX4**, rather than trying to reuse an existing FMU target.

This applies even if your board is electrically close to an FMUv5X or FMUv6X design.
As soon as you change the sensor set, connectors, or other peripherals, you should have your own target.
It gives you a stable place to express your hardware's configuration and keeps you in control of your own board.

Start by reading the porting guides:

- [飞控移植指南](../hardware/porting_guide.md)
- [NuttX Board Port (config & pin mapping)](../hardware/porting_guide_config.md)
- [PX4 Nuttx Porting Guide > Bootloader](../hardware/porting_guide_nuttx.md#bootloader)

Good real-world examples of manufacturer board targets to model yours on:

- [ARK V6X](https://github.com/PX4/PX4-Autopilot/pull/25715)
- [ZeroOne 6X](https://github.com/PX4/PX4-Autopilot/pull/23623)

Your board must use the [PX4 bootloader protocol](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/nuttx/src/bootloader).

### 2. Reserve a Board ID

Every flight controller needs a unique **board ID** for bootloader and firmware selection in QGroundControl.
This ID lives in your board's `firmware.prototype` file.

You reserve it by **opening a pull request against [PX4/PX4-Bootloader](https://github.com/PX4/PX4-Bootloader)** proposing your value.
You pick the value; the maintainers coordinate to make sure it does not collide with an existing board. Examples to follow:

- [PX4-Bootloader#260](https://github.com/PX4/PX4-Bootloader/pull/260)
- [PX4-Bootloader#262](https://github.com/PX4/PX4-Bootloader/pull/262)

:::info
**If your board will also run ArduPilot, reserve the _same_ board ID in both bootloaders.**
Open a matching PR against [ArduPilot](https://github.com/ArduPilot/ardupilot) (board IDs live in the `hwdef` definitions under [`libraries/AP_HAL_ChibiOS/hwdef`](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef)) and use the identical board ID value there.
Keeping the two aligned avoids a class of confusing flashing and identification problems where the same physical board reports different IDs depending on which firmware it is running.
:::

### 3. Sort Out your USB VID/PID

If your board exposes USB, it needs a USB Vendor ID (VID) and Product ID (PID).

The VID/PID pair is how QGroundControl recognizes your hardware over USB.
It is what lets QGC match the connected board to the right firmware and present it correctly to the user.
The pair must be unique to your product.

:::warning
**Do not copy another manufacturer's VID/PID.** If your board reports a VID/PID that belongs to someone else's hardware, QGroundControl will misidentify it, associate it with the wrong board, and may offer or flash the wrong firmware.
Each board needs its own identity.
:::

There are several ways to get a valid pair, listed in order of preference:

1. **Buy your own VID from USB-IF (preferred).** As a serious hardware manufacturer, the cleanest option is to purchase your own Vendor ID directly from the [USB Implementers Forum](https://www.usb.org/getting-vendor-id).
   The VID/PID then unambiguously identify _your_ company and product.
2. **Obtain a free VID/PID.** If purchasing your own is not an option, you can get a free VID/PID pair from one of several community registries or chip-vendor programs, for example [pid.codes](https://pid.codes/) (VID `0x1209`) or the [Openmoko registry](https://github.com/openmoko/openmoko-usb-oui) (VID `0x1d50`) for open-source hardware, or a sub-PID from your MCU/USB-bridge vendor (e.g. Microchip, FTDI).
   PX4 takes no position on how you source a free pair and assumes no responsibility for it.
   The only requirement is that your VID/PID pair is unique and does not conflict with anything already in the PX4 codebase.
3. **Use the Dronecode VID.** [Dronecode Foundation members](https://dronecode.org/membership/) can request a PID allocated under the Dronecode USB VID.
   This is one of the membership benefits, alongside access to the Pixhawk FMU reference schematics.

:::info
Membership is **not** required to get your board supported in PX4.
The Dronecode VID is a convenience for members; any unique, non-conflicting VID/PID pair from the options above is equally acceptable for a board support pull request.
:::

### 4. Fly it and Capture Logs

**Bench testing is not enough.** Before a flight controller is accepted, you must demonstrate stable flight on the current PX4 release with your hardware.

Capture flight logs from that testing and upload them to [logs.px4.io](https://logs.px4.io).
You will link these in your pull request so the maintainers can review the flight data alongside your code.

### 5. Open the Pull Request

Open a pull request against [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) containing:

- Your board support code (the build target from step 1).
- Board documentation: a public pinout mapping PX4 pin definitions to the microcontroller pins and physical connectors, plus a block diagram or schematic of the main components (sensors, power supply) sufficient to understand boot order and software requirements.
- **Links to your flight logs** from step 4, in the PR description, as proof the hardware has actually been flown.

If you have never contributed via GitHub, follow the [documentation contribution guide](../contribute/docs.md) for the mechanics of forking, branching, and opening a PR.

:::tip
A clean, reviewable commit history makes a real difference.
Split your work into logical commits rather than one large dump, and engage early on Discord if you have questions.
The most common reasons first-time board PRs stall are incomplete documentation and missing or insufficient flight-test evidence.
:::

### 6. Maintain It {#maintain}

Once your board is merged, you own it.
That means:

- Handling support questions from your own customers.
- Keeping your board target building and working as PX4 evolves across releases.
- Responding to issues and regressions specific to your hardware.

You are not expected to fix unrelated PX4 bugs, but you are expected to maintain what you contribute.
Boards that fall out of maintenance and stop building may be moved to [experimental](#support-categories) or removed.

## Adding a Component {#components}

For peripherals that work _with_ a flight controller, GPS/RTK receivers, compasses, data links, distance sensors, and similar, the path is lighter.
There is no board ID or firmware target involved.

1. **Validate** that your product works with PX4.
   A few simple flights are enough.
2. **Edit the relevant documentation page** to add your product, for example the [GPS & Compass](../gps_compass/index.md) or [RTK GPS](../gps_compass/rtk_gps.md) page for a GNSS product.
3. **Open a pull request** against [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) with your documentation changes.
4. **Attach proof** of step 1 (flight logs, linked or uploaded to [logs.px4.io](https://logs.px4.io)) in the PR description, so the maintainers can review it alongside your edits.

New to GitHub? The [documentation contribution guide](../contribute/docs.md) covers the whole flow.

## Support Categories {#support-categories}

These categories describe **who is responsible for supporting a board**, the PX4 team or the manufacturer, and whether it is listed on the PX4 website.
They do not change the steps above.
The boards in each category are listed at [px4.io/autopilots](https://px4.io/autopilots/).

### Pixhawk Standard

Boards that conform to the [Pixhawk standards](https://pixhawk.org/standards/).
These are fully supported and maintained by the PX4 development and test teams.
Qualifying requires passing the electrical tests mandated by the standard and signing the Pixhawk adopter and trademark agreement.
PX4 generally supports standards released within the last few years, since those are what is commercially available.

For examples, see [Pixhawk Standard Autopilots](../flight_controller/autopilot_pixhawk_standard.md).

### Manufacturer Supported

Boards supported by the manufacturer, the category most new flight controllers fall into.
The manufacturer owns support and keeps the board working across PX4 releases, as described in [Maintain it](#maintain) above.
These boards can be as well supported as Pixhawk boards.
There is no formal support or test commitment from the PX4 team, but a close working relationship between the manufacturer and PX4 teams benefits everyone.

For examples, see [Manufacturer-Supported Autopilots](../flight_controller/autopilot_manufacturer_supported.md).

### Experimental

Boards that work with at least one PX4 release for a defined vehicle type, but not necessarily the latest, and are not actively maintained to either of the above standards.
Previously supported boards that fall out of active maintenance land here.

For examples, see [Experimental Autopilots](../flight_controller/autopilot_experimental.md).

### Unsupported

Boards that meet none of the above.
Typically: boards that would take minimal effort to reach "experimental" but that nobody, manufacturer or dev team, is currently pursuing; hardware whose owner has violated the [Code of Conduct](https://discuss.px4.io/t/px4-community-code-of-conduct/13655); designs whose required tools/libraries/drivers are blocked by incompatible licensing; or boards that do not meet the general requirements above.
Unsupported boards are not listed on the PX4 website and may be removed from the codebase.

### Hardware revision sensing (VER/REV ID) {#ver_rev_id}

:::warning
**This path is no longer recommended.** New manufacturers should build their own firmware target ([step 1](#1-build-your-own-firmware-target)) instead.
This section is kept for context and for the few boards that still depend on the mechanism.
:::

VER/REV ID is a sensing mechanism on FMUv5X and later.
The board encodes its version and revision either through resistor dividers read on dedicated sensing pins, or through values stored in an onboard EEPROM.
PX4 reads these at boot and uses them, together with the standard configuration, to determine the expected device and power-supply layout for that FMU version.

It was originally designed to allow **cross-vendor compatibility between baseboards and FMU modules**: the idea being that a standard FMU module from one vendor could drop into a standard baseboard from another and run the same binary, with the VER/REV ID telling the firmware which hardware combination it was on.
The VER/REV ID values were coordinated centrally with the PX4 board maintainers (historically by a PR against the [DS-018 Pixhawk standard](https://github.com/pixhawk/Pixhawk-Standards)) so that assignments stayed unique across vendors.

In practice the ecosystem has **deviated from that model**.
Modern boards differ enough in sensors, connectors, and peripherals that the shared-binary, drop-in-module assumption rarely holds, and maintaining your own firmware target has become the cleaner and more reliable approach.
For that reason:

- We **no longer recommend** that manufacturers go through the VER/REV ID path.
  Build your own firmware target ([step 1](#1-build-your-own-firmware-target)) instead.
  It applies to nearly all new hardware, and is required as soon as you deviate from a published FMU sensor set in any way.
- The Dronecode team is **not currently offering validation services** to certify that a board meets the FMU standard for the purpose of obtaining a default VER/REV ID assignment.

## 获取帮助

The board support process improves over time, and contributions to the process itself are welcome.

- Ask the community on the PX4 [Discord](https://discord.com/invite/dronecode) under the `Hardware` channels, or on the [discuss forum](https://discuss.px4.io/).
- Join the regular hardware call.
- Consultancy options are listed at [px4.io/community/consultants](https://px4.io/community/consultants/).

If you need direct coordination that cannot happen in a pull request, for example sorting out a board ID conflict, you can reach the maintainers at [boards@px4.io](mailto:boards@px4.io). Use this as a last resort: anything that can happen in the open on GitHub should.
