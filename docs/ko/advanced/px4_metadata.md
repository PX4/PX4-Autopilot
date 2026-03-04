# PX4 Metadata

PX4 uses and generates data that has associated human- and machine- readable metadata:

- [Parameters](../advanced_config/parameters.md) configure PX4 behaviour.
  - A parameter is represented by an ID string that maps to a value stored in PX4.
  - The associated metadata includes a description of the setting, its possible values, information about how the value might be presented (say for bitmasks).
- [Events](../concept/events_interface.md) provide notification of events, such as reasons for a failsafe, low battery warnings, end of calibration, and so on.
  - An event is represented by an id, and is sent with a log level and arguments.
  - The associated metadata includes a message, a description and a list of arguments (including their type) of each event.
- [Actuators](../config/actuators.md) configuration customizes the specific geometry of the vehicle, assigns actuators and motors to flight controller outputs, and tests the actuator and motor response.
  - The metadata contains information about supported vehicle geometries, a list of output drivers, and how to configure them.
  - _QGroundControl_ uses that information to dynamically build a configuration UI.

The metadata and metadata translations are shared with external systems, such as QGroundControl, allowing them to display information about parameters and events, and to configure vehicle geometry and actuator output mappings.

This topic explains how you can define metadata and help translate strings (and "just for your information", how it all works).

## Metadata Translation

Translations for PX4 metadata are done using Crowdin in the [PX4-Metadata-Translations](https://crowdin.com/project/px4-metadata-translations) project.
For more information about working with PX4 and Crowdin see [Translation](../contribute/translation.md).

## Defining Metadata

PX4 metadata is defined in PX4 source code alongside its associated data.
This is done either in a C/C++ comment with special markup to indicate metadata fields and their values, or using YAML files.

For more information see the topics for each data type:

- [Parameters & Configurations > Creating/Defining Parameters](../advanced/parameters_and_configurations.md#creating-defining-parameters)
- [Events Interface](../concept/events_interface.md)
- [Actuator Metadata](#actuator-metadata) (below)

## Metadata Toolchain

The process for handling metadata is the same for all metadata types.

Metadata is collected into JSON files every time PX4 is built.

For most flight controllers (as most have enough FLASH available), the JSON file is xz-compressed and stored within the generated binary.
The file is then shared to ground stations using the MAVLink [Component Metadata Protocol](https://mavlink.io/en/services/component_information.html).
Using the component metadata protocol ensures that the recipient can always fetch up-to-date metadata for the code running on the vehicle.
Events metadata is also added to the log files, allowing log analysis tools (such as Flight Review) to use the correct metadata to display events.

Binaries for flight controller targets with constrained memory do not store the parameter metadata in the binary, but instead reference the same data stored on `px4-travis.s3.amazonaws.com`.
This applies, for example, to the [Omnibus F4 SD](../flight_controller/omnibus_f4_sd.md).
The metadata is uploaded via [github CI](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/metadata.yml) for all build targets (and hence will only be available once parameters have been merged into main).

:::info
You can identify memory constrained boards because they specify `CONFIG_BOARD_CONSTRAINED_FLASH=y` in their [px4board definition file](https://github.com/PX4/PX4-Autopilot/blob/main/boards/omnibus/f4sd/default.px4board).

If doing custom development on a FLASH-constrained board you can adjust the URL [here](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/component_information/CMakeLists.txt#L41) to point to another server.
:::

The metadata on `px4-travis.s3.amazonaws.com` is used if parameter metadata is not present on the vehicle.
It may also be used as a fallback to avoid a very slow download over a low-rate telemetry link.

The metadata JSON files for CI builds of `main` are also copied to the github repo: [PX4/PX4-Metadata-Translations](https://github.com/PX4/PX4-Metadata-Translations/).
This integrates with Crowdin to get translations, which are stored in the [translated](https://github.com/PX4/PX4-Metadata-Translations/tree/main/translated) folder as xz-compressed translation files for each language.
These are referenced by the vehicle component metadata, and are downloaded when needed.
For more information see [PX4-Metadata-Translations](https://github.com/PX4/PX4-Metadata-Translations/) and [Component Metadata Protocol > Translation](https://mavlink.io/en/services/component_information.html#translation).

:::info
The parameter XML file of the main branch is copied into the QGC source tree via CI and is used as a fallback in cases where no metadata is available via the component metadata protocol (this approach predates the existence of the component metadata protocol).
:::

### Actuator Metadata

The following diagram shows how actuator metadata is assembled from the source code and used by QGroundControl:

![Actuators Metadata](../../assets/diagrams/actuator_metadata_processing.svg)

<!-- Source: https://docs.google.com/drawings/d/1hMQmIijdFjr21rREcXj50qz0C1b47JW0OEa6p5P231k/edit -->

- **Left**: the metadata is defined in `module.yml` files in different modules.
  The `control_allocator` modules defines the geometries, while each output driver defines its set of channels and configuration parameters.
  [The schema file](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml) documents the structure of these yaml files.
- **Middle**: At build time, the `module.yml` files for all enabled modules for the currently built target are parsed and turned into an `actuators.json` file using the [Tools/module_config/generate_actuators_metadata.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/module_config/generate_actuators_metadata.py) script.
  There is also [schema file](https://github.com/mavlink/mavlink/blob/master/component_metadata/actuators.schema.json) for this.
- **Right**: At runtime, the JSON file is requested by QGroundControl via MAVLink Component Metadata API (described above).

## 추가 정보

- [Parameters & Configurations](../advanced/parameters_and_configurations.md)
- [Events Interface](../concept/events_interface.md)
- [Translation](../contribute/translation.md)
- [Component Metadata Protocol](https://mavlink.io/en/services/component_information.html) (mavlink.io)
- [PX4-Metadata-Translations](https://github.com/PX4/PX4-Metadata-Translations/) (Github)
