---
applyTo: "src/**,ROMFS/**,boards/**"
---

# Parameter Guidelines

In addition to the core code review guidelines:

- Each parameter has one owner: the module that owns the concept. Name and describe it by what that module does with it.
- A module that needs another module's result consumes it on uORB instead of reading that module's thresholds and recomputing it. Shared vehicle configuration read by several modules (e.g. `NAV_ACC_RAD`) is existing practice; the rule targets coupling to another module's internal decisions.
- No two parameters with the same meaning in different modules. Two concepts that shared one parameter are not duplicates: when logic moves between modules, split the parameter along the concepts, each named and described for its own role, rather than replacing one side with a hardcoded constant.
- `src/lib/parameters/param_translation.cpp` is for parameters whose name or meaning differs from the last release. Renames and changes within the unreleased development window need no translation. When a released parameter splits into two, keep it and copy its value to the new one on import, so tuned setups keep both behaviours.
