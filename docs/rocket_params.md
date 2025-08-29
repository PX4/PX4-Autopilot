# Rocket Mode Manager Parameters

This document lists the configurable parameters exposed by the Rocket Mode Manager module (`src/modules/rocket_mode_manager/module.yaml`). Use these parameters to tune launch detection, boost/coast timing, and control allocation behavior.

> Note: parameter names are the canonical PX4 parameter names and can be modified via QGroundControl or `param set`.

| Parameter | Type | Unit | Default | Short Description |
|-----------|------|------:|---------:|------------------|
| RKT_DEPLOY_V | float | m/s | 5.0 | Wing deployment velocity threshold (body X-axis) |
| RKT_ALT_THRESH | float | m | 3.0 | Altitude threshold for wing deployment (meters descended from max) |
| RKT_LAUNCH_A | float | m/s^2 | 15.0 | Launch detection acceleration threshold |
| RKT_LAUNCH_V | float | m/s | 3.0 | Launch detection velocity threshold |
| RKT_BOOST_A | float | m/s^2 | 5.0 | Boost end acceleration threshold |
| RKT_BOOST_T | float | s | 10.0 | Maximum boost phase duration |
| RKT_COAST_T | float | s | 30.0 | Maximum coast phase duration |
| RKT_NAV_MASK | int32 | - | -2147483648 | Navigation mask used to restrict available flight modes during rocket phase |

## Control Allocation and Control Surface Parameters

The module exposes a set of parameters used to configure the control allocator for rocket and fixed-wing phases.

- Rocket phase torque matrix elements: `RKT_CA_R0` ... `RKT_CA_R17` (18 floats, 6 servos × 3 components). Default vector: [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- Fixed-wing phase torque matrix elements: `RKT_CA_F0` ... `RKT_CA_F17` (18 floats, 6 servos × 3 components). Default vector: [0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0]

- Rocket-phase control surface types: `RKT_CS_R0` ... `RKT_CS_R5` (6 ints) — default: [12, 12, 12, 12, 0, 0]

- Fixed-wing control surface types: `RKT_CS_F0` ... `RKT_CS_F5` (6 ints) — default: [3, 4, 3, 4, 1, 2]

## Usage Notes

- `RKT_NAV_MASK` is used by the module to restrict available navigation modes during rocket flight. The module sets this to `-2147483648` during rocket phase (to enforce Rocket Roll-only) and restores fixed-wing modes after wing deployment.

- `RKT_ALT_THRESH` is used as the altitude descent threshold from the maximum altitude to trigger wing deployment (backup to velocity-based detection).

- The torque matrix parameters (`RKT_CA_*`) and control surface types are used to programmatically set the `CA_SV_CS#_*` parameters during phase transitions. Changes to the RKT_* CA parameters will be applied at runtime when the module configures control allocation.

---

Generated from `src/modules/rocket_mode_manager/module.yaml` on 2025-08-19.
