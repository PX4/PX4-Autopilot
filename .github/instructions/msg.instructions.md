---
applyTo: "msg/**.msg"
---

# Review Guidelines for PX4 uORB Message Definitions

You are an expert embedded software engineer specializing in the PX4 Autopilot system.
Your task is to review `.msg` files to ensure they comply with the official [PX4 uORB Documentation Standard](../../docs/en/uorb/uorb_documentation).

## 1. File Structure & Header

- **Mandatory Short Description:** Every message must start with a `#` comment providing a succinct explanation (e.g., `# Validated airspeed`). 
  - No terminating full stop for single-line descriptions.
- **Long Description:** Optional but encouraged for complex messages. Should be separated from the short description by an empty comment line (`#`). Use full punctuation for multi-line descriptions.
- **Timestamp:** All messages **must** include `uint64 timestamp # [us] Time since system start`.
- **Versioning:** If the message is in `msg/versioned/`, it **must** include `uint32 MESSAGE_VERSION`.

## 2. Field Documentation Standards

- **Inline Comments:** Fields should have a comment on the same line, separated by exactly one space from the field name. This should be followed first by metadata tags (described below) if required, and then descriptive text.
- **Metadata Tags:**
  - Metadata tags appear first in the field comments, and are delineated by brackets.
  - The type of metdata is indicated by a value following the @ symbol, except for unit metadata, which is assumed to be units if there is no value immediately following the @ symbol.
  - The metadata types are:
    - **Units:** Required for all non-boolean/non-enum fields.
      - Format: `[unit]` (e.g., `[m/s]`, `[rad/s]`, `[m/s^2]`, `[deg]`).
      - Unitless fields must use `[-]`.
    - **Ranges:** Use `[@range min, max]` to define valid bounds.
      Values for either `min` and `max` may be omitted if there is no lower or upper bound.
    - **Enums:** Use `[@enum NAME]` to link a field to a set of constants defined in the file.
      - The `NAME` should be the prefix of a set of constants defined in the file.
    - **Invalid Values:** Use `[@invalid NaN]` to indicate that a `NaN` value represents "no data".
      If another value represents "no data" that would replace `NaN` as the invalid value. A short description can follow the value by one space.
- **Field descriptions:** Field descriptions come after any metadata, if present. 
  - The description should start with a capital letter. If the description is a single sentence it should omit the final full stop.
  - The description may contain unit or other metadata, but this metadata must also be captured in the metadata tags.
- **Example**. 
  The following example shows a field of type `float32` with a name `true_airspeed`.
   The comment starts one space after the field name, has `unit` metadata and `invalid` metadata, followed by the descriptive text `True airspeed (TAS)`.
     
     ```
     float32 true_airspeed # [m/s] [@invalid NaN] True airspeed (TAS)
     ```

## 3. Constants & Enums

- **Naming:** Constants related to a specific field should share a common prefix in their name that matches the associated field (except in terms of case)
- **Position in message:** Constants should appear immediately after the field in which they are used, ordered by their value.
- **Documentation:** 
  - Constants may have a comment, separated by exactly one space from the contant name.
  - The comment may contain a description of the purpose of the constant. The description should start with a capital letter. If the description is a single sentence it should omit the final full stop.
  - The description/comment is optional if the purpose of the constant is obvious from its name.
- **Standard Constants:** Standardized fields like `MESSAGE_VERSION` or `ORB_QUEUE_LENGTH` do not require individual documentation.
- **Example:** The following code shows a field airspeed_source followed by the constants that can be used with it. 
  These all share the prefix `AIRSPEED_SOURCE` matching the field name, and this is listed in the field metadata as the enum name.
  The contants are listed immediately after the field, and ordered by their value in descending order.

   ```
   int8 airspeed_source # [@enum AIRSPEED_SOURCE] Source of currently published airspeed values
   int8 AIRSPEED_SOURCE_DISABLED = -1 # Disabled
   int8 AIRSPEED_SOURCE_GROUND_MINUS_WIND = 0 # Ground speed minus wind
   int8 AIRSPEED_SOURCE_SENSOR_1 = 1 # Sensor 1
   int8 AIRSPEED_SOURCE_SENSOR_2 = 2 # Sensor 2
   int8 AIRSPEED_SOURCE_SENSOR_3 = 3 # Sensor 3
   int8 AIRSPEED_SOURCE_SYNTHETIC = 4 # Synthetic airspeed
   ```

## 4. Multi-Topic Messages

- If a single `.msg` file defines multiple topics (e.g., `actuator_controls_0`, `actuator_controls_1`), the file must end with a `# TOPICS` line followed by the space-separated topic names.

## 5. Field Changes

- For files in `./msg/versioned/` or one of its subfolders:
  - The file must have a field named `MESSAGE_VERSION`.
  - If a field name or type changes, or if a field is deleted or added, the value of the `MESSAGE_VERSION` field must be incremented by one.

## Review Checklist

1. Does it have a mandatory short description at the top?
2. Is the `uint64 timestamp` field present?
3. Do all numeric fields have units in square brackets?
4. Are enums linked to fields using the `[@enum Name]` tag?
5. Are single-line comments lacking a trailing period?
6. If the message is nested in `msg/versioned` (or a subfolder), is `MESSAGE_VERSION` included, and has the value been iterated.