# Events Interface

<Badge type="tip" text="PX4 v1.13" />

The _Events Interface_ provides a system-wide API for notification of events, which are published to GCSs via the _MAVLink Events Service_ (to GCSs and other components) and also stored in [system logs](../dev_log/logging.md).

The interface can be used for publishing events for state changes or any other type of occurrence, including things like arming readiness, calibration completion, and reaching the target takeoff height.

::: info
The events interface will replace the use of `mavlink_log_*` calls in PX4 code, (and `STATUS_TEXT` messages in MAVLink) for event notification in PX4 v1.13 and later.
There will be an intermediate period where [both approaches are supported](#backward-compatibility).
:::

## Usage

### Basic

To use the API, add this include:

```cpp
#include <px4_platform_common/events.h>
```

And then define and send the event from the desired code location:

```cpp
events::send(events::ID("mymodule_test"), events::Log::Info, "Test Message");
```

#### Backward compatibility

For older GCS versions without events interface support, PX4 currently sends out all events also as `mavlink_log_*` `STATUSTEXT` message.
In addition, the message must be tagged with an appended tab (`\t`) so that newer GCS's can ignore that and only show the event.

So whenever adding an event, be sure to also add a `mavlink_log_` call. For example:

```cpp
mavlink_log_info(mavlink_log_pub, "Test Message\t");
events::send(events::ID("mymodule_test"), events::Log::Info, "Test Message");
```

All such `mavlink_log_` calls will be removed after the next release.

### Detailed

The above is a minimal example, this is a more extensive one:

```cpp
uint8_t arg1 = 0;
float arg2 = -1.f;
/* EVENT
 * @description
 * This is the detailed event description.
 *
 * - value of arg1: {1}
 * - value of arg2: {2:.1}
 *
 * <profile name="dev">
 * (This paragraph is only meant to be shown to developers).
 * This behavior can be configured with the parameter <param>COM_EXAMPLE</param>.
 * </profile>
 *
 * Link to documentation: <a>https://docs.px4.io</a>
 */
events::send<uint8_t, float>(events::ID("event_name"),
	{events::Log::Error, events::LogInternal::Info}, "Event Message", arg1, arg2);
```

Explanations and requirements:

- `/* EVENT`: This tag indicates that a comment defines metadata for the following event.
- **event_name**: the event name (`events::ID(event_name)`).
  - must be unique within the whole source code of PX4.
    As a general convention, prefix it with the module name, or the source file for larger modules.
  - must be a valid variable name, i.e. must not contain spaces, colons, etc.
  - from that name, a 24 bit event ID is derived using a hash function.
    This means as long as the event name stays the same, so will the ID.
- **Log Level**:

  - valid log levels are the same as used in the MAVLink [MAV_SEVERITY](https://mavlink.io/en/messages/common.html#MAV_SEVERITY) enum.
    In order of descending importance these are:

    ```plain
    Emergency,
    Alert,
    Critical,
    Error,
    Warning,
    Notice,
    Info,
    Debug,
    Disabled,
    ```

  ```
  - Above we specify a separate external and internal log level, which are the levels displayed to GCS users and in the log file, respectively: `{events::Log::Error, events::LogInternal::Info}`.
    For the majority of cases you can pass a single log level, and this will be used for both exernal and internal cases.
  There are cases it makes sense to have two different log levels.
  For example an RTL failsafe action: the user should see it as Warning/Error, whereas in the log, it is an expected system response, so it can be set to `Info`.
  ```

- **Event Message**:
  - Single-line, short message of the event.
    It may contain template placeholders for arguments (e.g. `{1}`). For more information see below.
- **Event Description**:
  - Detailed, optional event description.
  - Can be multiple lines/paragraphs.
  - It may contain template placeholders for arguments (e.g. `{2}`) and supported tags (see below)

#### Arguments and Enums

Events can have a fixed set of arguments that can be inserted into the message or description using template placeholders (e.g. `{2:.1m}` - see next section).

Valid types: `uint8_t`, `int8_t`, `uint16_t`, `int16_t`, `uint32_t`, `int32_t`, `uint64_t`, `int64_t` and `float`.

You can also use enumerations as arguments:

- PX4-specific/custom enumerations for events should be defined in [src/lib/events/enums.json](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/events/enums.json), and can then be used as event argument in the form of `events::send<events::px4::enums::my_enum_t>(...)`.
- MAVLink "common" events are defined in [mavlink/libevents/events/common.json](https://github.com/mavlink/libevents/blob/master/events/common.json) and can be used as event argument in the form of `events::send<events::common::enums::my_enum_t>(...)`.

#### Text format

Text format for event message description:

- characters can be escaped with \\

  These have to be escaped: '\\\\', '\\<', '\\{'.

- supported tags:

  - Profiles: `<profile name="[!]NAME">CONTENT</profile>`

    `CONTENT` will only be shown if the name matches the configured profile.
    This can be used for example to hide developer information from end-users.

  - URLs: `<a [href="URL"]>CONTENT</a>`.
    If `href` is not set, use `CONTENT` as `URL` (i.e.`<a>https://docs.px4.io</a>` is interpreted as `<a href="https://docs.px4.io">https://docs.px4.io</a>`)
  - Parameters: `<param>PARAM_NAME</param>`
  - no nested tags of the same type are allowed

- arguments: template placeholders that follow python syntax, with 1-based indexing (instead of 0)

  - general form: `{ARG_IDX[:.NUM_DECIMAL_DIGITS][UNIT]}`

    UNIT:

    - m: horizontal distance in meters
    - m_v: vertical distance in meters
    - m^2: area in m^2
    - m/s: speed in m/s
    - C: temperature in degrees celsius

  - `NUM_DECIMAL_DIGITS` only makes sense for real number arguments.

## Logging

Events are logged according to the internal log level, and [Flight Review](../log/flight_review.md) displays events.

::: info
Flight review downloads metadata based on PX4 master, so if a definition is not yet on master, it will only be able to display the event ID.
:::

## Implementation

During PX4 build, only the code is added directly to the binary by the compiler (i.e. the event ID, log level(s) and any arguments).

The metadata for all events is built into a separate JSON metadata file (using a python script that scans the whole source code for event calls).

### Publishing Event Metadata to a GCS

The event metadata JSON file is compiled into firmware (and/or hosted on the Internet), and made available to ground stations via the [MAVLink Component Metadata service](https://mavlink.io/en/services/component_information.html).
This ensures that metadata is always up-to-date with the code running on the vehicle.

This process is the same as for [parameter metadata](../advanced/parameters_and_configurations.md#publishing-parameter-metadata-to-a-gcs).
For more information see [PX4 Metadata (Translation & Publication)](../advanced/px4_metadata.md)
