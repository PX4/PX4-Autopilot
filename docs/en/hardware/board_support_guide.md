# Manufacturer's PX4 Board Support Guide

The PX4 development and test teams fully support and maintain boards that are compliant with the [Pixhawk Standard](https://pixhawk.org/standards/).
Manufacturers who wish to deviate from the standard or create completely new boards can do so, but will need to support any resulting compatibility differences.

This guide outlines the [general requirements](#general_requirements) for board support, along with the additional requirements for the different [board support categories](#board-support-categories).

::: info
Boards that are not compliant with the requirements are [unsupported](#unsupported); they will not be listed on the PX4 website hardware list and will be removed from the codebase.
:::

<a id="general_requirements"></a>

## General Requirements

The general requirements for all supported boards are:

1. The hardware must be available in the market.
1. The boards may not have blocking hardware bugs or unacceptable quality that make it impossible or dangerous to use the board with PX4 on a UAV.
   Board needs to pass acceptance criteria to ensure quality of parts and assembly.
1. A clear and easy way to contact customer support for customers.
   One or more of the following is accepted:

   - PX4 Discord server presence
   - Support email
   - Phone number

1. Point of contact (PoC) for the PX4 maintainers (direct email or available in Slack/Forum/Github)
1. The board needs to use the [PX4 bootloader protocol](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/nuttx/src/bootloader).
   For more information on bootloaders see: [PX4 Nuttx Porting Guide > Bootloader](../hardware/porting_guide_nuttx.md#bootloader).
1. Adequate documentation, which includes, but is not limited to:

   - A complete pinout made available publicly that maps PX4 pin definitions to:
     1. Microcontroller pins
     2. Physical external connectors
   - A block diagram or full schematic of the main components (sensors, power supply, etc.) that allows to infer software requirements and boot order
   - A manual of the finished product detailing its use

1. There must be a dedicated webpage for the board with PX4, which lists the features and limitations for usage with PX4, and includes or links to the above described documentation.

## Board Support Categories

The board support categories are listed below. The autopilot boards in each category are listed at: [https://px4.io/autopilots/.](https://px4.io/autopilots/)

::: info
Manufacturer supported boards may be as well/better supported than Pixhawk boards (for example through economies of scale).
:::

## Pixhawk Standard

A Pixhawk board is one that conforms to the Pixhawk standards. These standards are laid out on [http://pixhawk.org](http://pixhawk.org/), but at high-level require that the board passes electrical tests mandated by the standard and the manufacturer has signed the Pixhawk adopter and trademark agreement.

PX4 generally only supports boards that are commercially available, which typically means that board standards released within the last five years are supported.

<a id="ver_rev_id"></a>

### VER and REV ID (Hardware Revision and Version Sensing)

FMUv5 and onwards have an electrical sensing mechanism.
This sensing coupled with optional configuration data will be used to define hardware’s configuration with respect to a mandatory device and power supply configuration. Manufacturers must obtain the VER and REV ID from PX4 board maintainers by issuing a PR to ammend the [DS-018 Pixhawk standard](https://github.com/pixhawk/Pixhawk-Standards) for board versions and revisions.

Because these boards are 100% compliant with the Pixhawk standard, the values assigned for VER and REV ID are the defaults for that FMU Version.

## Manufacturer Supported

These boards are supported by the manufacturer.
To qualify for this category the board must work with the latest stable PX4 release within 4 months of that release.

- Manufacture owns the support
- Manufacturer must supply at least 2 boards to the core-dev team (for use on test rack and by test team)

:::tip
While there is no commitment from the PX4 maintainers and the flight test team to support and test boards in this category, we strongly recommended PX4 and manufacturer teams build close working relationships.
This will result in a better result for all parties.
:::

::: info
These boards will be assigned [VER and REV ID](#ver_rev_id) based on compatibility.
A special assignment will be made by PX4 if the board is a variant of an FMU specification and capable of running the same binary, with minor differences supported by the manufacturer.
Contact the PX4 maintainer at [boards@px4.io](mailto:boards@px4.io) to request more information.
:::

## Experimental

These boards are all boards that don't fall in the above categories, or don't fall in those categories _anymore_.
The following requirements apply:

- The board must be working with at least one PX4 release for a defined vehicle type, but not necessarily the latest release.

::: info
Experimental boards that were _previously_ Pixhawk or Manufacturer supported will have/retain their original IDs.
_New_ experimental boards are allocated [VER and REV IDs](#ver_rev_id) based on compatibility, in the same way as Manufacturer Supported boards.
:::

## Unsupported

This category includes all boards that aren't supported by the PX4 project or a manufacturer, and that fall outside the"experimental" support.

- Board is somewhat compatible on paper with something we already support, and it would take minimal effort to raise it to "experimental", but neither the dev-team or the manufacturer are currently pursuing this
- Manufacturer/Owner of hardware violates our [Code of Conduct](https://discuss.px4.io/t/code-of-conduct/13655)
- Closed source, where any of the necessary tools/libs/drivers/etc needed to add support for a board is deemed incompatible due to licensing restrictions
- Board doesn't meet minimum requirements outlined in the General requirements

::: info
Unsupported boards will NOT be assigned [VER and REV ID](#ver_rev_id) (and cannot run PX4 FMUvX firmware).
:::

## Release Process

It is assumed that when a manufacturer declares that a board falls in a certain category, that the board is compliant with the requirements for that category and the general requirements.

When a new board is brought to market that falls into the manufacturer supported or experimental category, the manufacturer is responsible for updating the PX4 documentation and doing the board release process in PX4. We recommend the following steps:

Contact PX4 board maintainers at [boards@px4.io](mailto:boards@px4.io) and request the following:

1. The assignment of a _board id_ for bootloader and firmware selection in QGC.
2. The assignment of REV and VER ID resistor values.
3. If the board supports USB: Either request the assignment of a USB VID and PID or provide the USB VID and PID.

Integrate the board according to the board porting release process described in the [porting guide](../hardware/porting_guide.md)

:::warning
The board support process may be changed and improved over time.
Hardware manufacturers are encouraged to contribute to this process through the regular hardware call, the Discuss forum or Discord.
:::

## Support

If parts of the board support guide/process are not clear:

- Ask the community for help on Discord channels under `Hardware` category, or on the discuss forum
- Attend the regular hardware call
- Consultancy options are listed here: [https://px4.io/community/consultants/](https://px4.io/community/consultants/)
