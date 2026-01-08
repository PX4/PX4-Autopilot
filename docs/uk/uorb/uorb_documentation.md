# uORB Documentation Standard

This topic demonstrates and explains how to document uORB messages.

:::info
At time of writing many topics have not been updated.
:::

## Загальний огляд

The [AirspeedValidated](../msg_docs/AirspeedValidated.md) message shown below is a good example of a uORB topic that has been documented to the current standard.

```py
# Validated airspeed
#
# Provides information about airspeed (indicated, true, calibrated) and the source of the data.
# Used by controllers, estimators and for airspeed reporting to operator.

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start

float32 indicated_airspeed_m_s # [m/s] [@invalid NaN] Indicated airspeed (IAS)
float32 calibrated_airspeed_m_s # [m/s] [@invalid NaN] Calibrated airspeed (CAS)
float32 true_airspeed_m_s # [m/s] [@invalid NaN] True airspeed (TAS)

int8 airspeed_source # [@enum SOURCE] Source of currently published airspeed values
int8 SOURCE_DISABLED = -1 # Disabled
int8 SOURCE_GROUND_MINUS_WIND = 0 # Ground speed minus wind
int8 SOURCE_SENSOR_1 = 1 # Sensor 1
int8 SOURCE_SENSOR_2 = 2 # Sensor 2
int8 SOURCE_SENSOR_3 = 3 # Sensor 3
int8 SOURCE_SYNTHETIC = 4 # Synthetic airspeed

float32 calibrated_ground_minus_wind_m_s # [m/s] [@invalid NaN] CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption
float32 calibraded_airspeed_synth_m_s # [m/s] [@invalid NaN] Synthetic airspeed
float32 airspeed_derivative_filtered # [m/s^2] Filtered indicated airspeed derivative
float32 throttle_filtered # [-] Filtered fixed-wing throttle
float32 pitch_filtered # [rad] Filtered pitch
```

The main things to note are:

- Documentation is added using formatted uORB comments.
  Any text on a line after the `#` character is a comment, except for lines that start with the text `# TOPIC` (which indicates a multi-topic message).
- The message starts with a comment block consisting of short description (mandatory), followed by a longer description and then a space.
- Field and constants almost all have comments.
  The comments are added on the same line as the field/constant, separated by one space.
- Fields:
  - Comments are all on the same line as the field (extra lines become internal comments).
  - Comments start with metadata, such as the units (`[m/s]`, `[rad/s]`) or allowed values (`[@enum SOURCE]`), and can also list invalid values (`[@invalid NaN]`) and allowed ranges (`[@range min, max]`).
  - Units are required except for boolean fields or for fields with an enum value.
    `[-]` is used to indicate unitless fields.
  - Comments follow the metadata after a space.
    The line should not be terminated in a full stop.
- Constants:
  - Don't have metadata: the description follows the comment marker after one space.
  - Some constants, such as `MESSAGE_VERSION`, don't need documentation because they are standardized.
  - Constants with the same name prefix are grouped together as enums after the associated field.

The following sections expand on the allowed formats.

## Message Description

Every message should start with a comment block that describes the message:

```py
# Short description (mandatory)
#
# Longer description for the message if needed.
# Can be multiline, and should have punctuation.
# Should be followed by an empty line.
```

This consists of a mandatory short description, optionally followed by an empty comment line, and then a longer description.

Short description (mandatory):

- A succinct explanation for the purpose of the message.
- Usually just one line without a terminating full stop.
- Minimally it may just mirror the message name.
- For example, [`AirspeedValidated`](../msg_docs/AirspeedValidated.md) above has the short description `Validated airspeed`.

Long description (Optional):

- Additional context required to understand how the message is used.
- In particular this should be anything that can't be inferred from the name, fields or constants, such as the publishers and expected consumers.
  It might also cover whether the message is only used for a particular frame type or mode.
- The message is often multiline and contains punctuation.
- May include comment lines that are empty, in order to indicate paragraphs.

Both short and long descriptions may be multi-line.
Single line descriptions should not include a terminating full stop, but multiline comments should do so.

The message description block ends at the first non-comment line, which should be an empty line, but might be a field or constant.
Any subsequent comment lines are considered "internal comments".

### Fields

A typical field comment looks like this:

```py
float32 indicated_airspeed_m_s # [m/s] [@invalid NaN] Indicated airspeed (IAS)
```

Field comments must all be on the same line as the field, and consist of optional metadata followed by a description:

- `metadata` (Optional)
  - Information about the field units and allowed values:
    - `[<unit>]`
      - The unit of measurement inside square brackets (note, no `@` delineator indicates a unit), such as `[m]` for metres.
        - Allowed units include: `m`, `m/s`, `m/s^2`, `rad`, `rad/s`, `rpm`, `V`, `A`, `mA`, `mAh`, `W`, `dBm`, `s`, `ms`, `us`, `Ohm`, `MB`, `Kb/s`, `degC`, `Pa`.
        - Units are required unless clearly invalid, such as when the field is a boolean, or is an enum value.
        - Unitless values should be specified as `[-]`.
          Note though that units are not required for boolean fields or enum fields.
    - `[@enum <enum_name>]`
      - The `enum_name` gives the prefix of constant values in the message that can be assigned to the field.
        Note that enums in uORB are just a naming convention: they are not explicitly declared.
        Multiple enum names allowed for a field indicates a possible error in the field design.
    - `[@range <lower_value>, <upper_value>]`
      - The allowed range of the field, specified as a `lower_value` and/or an `upper_value`.
        Either value can be omitted to indicate an unbounded upper or lower value.
        For example `[@range 0, 3]`, `[@range 5.3, ]`, `[@range , 3]`.
    - `[@invalid <value> <description>]`
      - The `value` to set the field to indicate that the field doesn't contain valid data, such as `[@invalid NaN]`.
        The `description` is optional, and might be used to indicate the conditions under which data is invalid.
    - `[@frame <value>]`
      - The `frame` in which the field is set, such as `[@frame NED]` or `[@frame Body]`.
- `description`
  - A concise description of the purpose of the field, and including any important information that can't be inferred from the name!
    Use a capital first letter, and omit the full stop if the description is a single sentence.
    Multiple sentences may also omit the final full stop.

### Constants

Constants follow the documentation conventions as fields except they only have a description (no metadata).
Documentation for a constant might look like this:

```py
int8 SOURCE_GROUND_MINUS_WIND = 0 # Ground speed minus wind
```

Constants are often grouped together following a field as enum values.
Note below how the prefix `SOURCE` for the values is specified as an enum against the _field_.

```py
int8 airspeed_source # [@enum SOURCE] Source of currently published airspeed values
int8 SOURCE_DISABLED = -1 # Disabled
int8 SOURCE_GROUND_MINUS_WIND = 0 # Ground speed minus wind
...
```

A small number of constants have a standardised meaning and do not require documentation.
These are:

- `ORB_QUEUE_LENGTH`
- `MESSAGE_VERSION`

### `# TOPICS`

The prefix `# TOPICS` is used to indicate topic names for multi-topic messages.
For example, the [VehicleGlobalPosition.msg](../msg_docs/VehicleGlobalPosition.md) message definition is used to define the topic ids as shown:

```text
# TOPICS vehicle_global_position vehicle_global_position_groundtruth external_ins_global_position
# TOPICS estimator_global_position
# TOPICS aux_global_position
```

At time of writing there is no format for documenting these.
