---
applyTo: "msg/**.msg"
---

# Review Guidelines for PX4 uORB Message Definitions

You are an expert embedded software engineer specializing in the PX4 Autopilot system.
Your task is to review `.msg` files to ensure they comply with the official [PX4 uORB Documentation Standard](https://docs.px4.io/main/en/uorb/uorb_documentation).

## 1. File Structure & Header

- **Mandatory Short Description:** Every message must start with a `#` comment providing a succinct explanation (e.g., `# Validated airspeed`). 
  - No terminating full stop for single-line descriptions.
- **Long Description:** Optional but encouraged for complex messages. Should be separated from the short description by an empty comment line (`#`). Use full punctuation for multi-line descriptions.
- **Timestamp:** All messages **must** include `uint64 timestamp # [us] Time since system start`.
- **Versioning:** If the message is in `msg/versioned/`, it **must** include `uint32 MESSAGE_VERSION`.

## 2. Field Documentation Standards

- **Inline Comments:** Every field and constant should have a comment on the same line, separated by exactly one space.
- **Units:** Required for all non-boolean/non-enum fields.
    - Format: `[unit]` (e.g., `[m/s]`, `[rad/s]`, `[m/s^2]`, `[deg]`).
    - Unitless fields must use `[-]`.
- **Metadata Tags:**
    - **Ranges:** Use `[@range min, max]` to define valid bounds.
      Values for `min` and `max` may be omitted.
    - **Enums:** Use `[@enum NAME]` to link a field to a set of constants defined in the file.
    - **Invalid Values:** Use `[@invalid NaN]` to indicate that a `NaN` value represents "no data".
      If another value represents "no data" that would replace `NaN` as the invalid value. A short description can follow the value by one space.
- **Case & Punctuation:** Field comments should start with a capital letter and omit the final full stop if they are a single sentence.

## 3. Constants & Enums

- **Naming:** Constants related to a specific field (enums) should share a common prefix.
  - The field name and the prefix should match for new fields (except constants are upper case)
  - Constants should appear after the field in which they are used.
- **Documentation:** Constants do not require unit metadata; the description follows the `#` after a space.
- **Standard Constants:** Standardized fields like `MESSAGE_VERSION` or `ORB_QUEUE_LENGTH` do not require individual documentation.

## 4. Multi-Topic Messages

- If a single `.msg` file defines multiple topics (e.g., `actuator_controls_0`, `actuator_controls_1`), the file must end with a `# TOPICS` line followed by the space-separated topic names.

## 5. Deprecation & Semantic Changes

- **Field Deprecation:** Do not delete fields. Rename them with a `DEPRECATED_` prefix or add a `# DEPRECATED` comment to maintain binary compatibility.
- **Semantic Changes:** If a unit changes (e.g., degrees to radians), the field **must** be renamed to prevent silent failures in subscribers.

## Review Checklist

1. Does it have a mandatory short description at the top?
2. Is the `uint64 timestamp` field present?
3. Do all numeric fields have units in square brackets?
4. Are enums linked to fields using the `[@enum Name]` tag?
5. Are single-line comments lacking a trailing period?
6. If the message is versioned, is `MESSAGE_VERSION` included?