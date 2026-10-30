---
applyTo: "src/**,ROMFS/**,boards/**"
---

# Parameter Guidelines

In addition to the core code review guidelines:

- Each parameter has one owner: the module that owns the concept. Name and describe it by what that module does with it.
- A module that needs another module's result consumes it on uORB instead of reading that module's thresholds and recomputing it. Shared vehicle configuration read by several modules (e.g. `NAV_ACC_RAD`) is existing practice; the rule targets coupling to another module's internal decisions.
- No two parameters with the same meaning in different modules. Two concepts that shared one parameter are not duplicates: when logic moves between modules, split the parameter along the concepts, each named and described for its own role, rather than replacing one side with a hardcoded constant.
- `src/lib/parameters/param_translation.cpp` is for parameters whose name or meaning differs from the last release. Renames and changes within the unreleased development window need no translation. When a released parameter splits into two, keep it and copy its value to the new one on import, so tuned setups keep both behaviours.

## Descriptions

Descriptions ship xz-compressed in ROMFS on boards without `CONSTRAINED_FLASH`, so every word costs flash.

- `long` holds only what `short`, `unit`, `min`/`max`, `default`, `reboot_required`, `requires_ethernet` and the option labels don't already say. Omit it if nothing is left. Change `short` only when it is wrong or repeats the type.
- A few words of option detail go in the option's label, in parentheses. More goes in `long`, one `Label: effect.` line per option. Never re-list the options in `long`.
- Keep every fact that affects configuration: behaviour, conditions, dependencies, limits, warnings. Cut restatements, "This parameter…", "Note that…", examples that repeat the rule, and rationale that doesn't change the value picked.
- Wording already used in other descriptions costs almost nothing after compression; new wording costs full price. Use the stock phrases "Set to 0 to disable.", "Set to -1 to disable.", "Only used when X is enabled." and "e.g.", and refer to parameters by name.
- Plain words in full sentences. No invented abbreviations or symbols as words (`<=`, `=`, `/` for "or").
- Prose is one paragraph in a plain or quoted YAML scalar. Newlines reach QGC unchanged, so a `|` block wrapped mid-sentence shows broken lines. Use newlines only between per-option lines.
