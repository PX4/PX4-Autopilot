# Firmware Manifest & Metadata

Each PX4 NuttX build produces a **manifest** — a JSON metadata object that describes the firmware target, board hardware, and build variant.
This manifest is embedded in the `.px4` firmware file and aggregated into a unified release manifest consumed by ground stations like QGroundControl for firmware discovery and selection.

## Manifest Schema

```json
{
  "name": "px4_fmu-v6x",
  "target": "px4_fmu-v6x_multicopter",
  "label_pretty": "Multicopter",
  "firmware_category": "vehicle",
  "manufacturer": "Holybro",
  "hardware": {
    "architecture": "cortex-m7",
    "vendor_id": "0x1234",
    "product_id": "0x5678",
    "chip": "stm32h753ii",
    "productstr": "Pixhawk 6X"
  }
}
```

### Field Descriptions

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Board name without variant label (e.g. `px4_fmu-v6x`) |
| `target` | string | Full build target including variant (e.g. `px4_fmu-v6x_multicopter`) |
| `label_pretty` | string | Human-readable variant name shown in ground stations (e.g. "Multicopter") |
| `firmware_category` | string | Build classification: `vehicle`, `peripheral`, `dev`, or `bootloader` |
| `manufacturer` | string | Board manufacturer name |
| `hardware` | object | Hardware details (architecture, USB IDs, chip, product string) |

## Firmware Categories

The `firmware_category` field classifies each build for ground station filtering:

| Value | Description | Examples | Ground Station Behavior |
|-------|-------------|----------|------------------------|
| `vehicle` | Production firmware for a vehicle type | multicopter, fixedwing, vtol, rover, uuv, spacecraft | Shown to users (primary) |
| `peripheral` | Firmware for CAN sensor nodes and peripherals | ark/can-gps, holybro/can-gps-v1, ark/can-flow, ark/mag | Shown in a dedicated peripheral/sensor section |
| `dev` | Developer/engineering builds | default, zenoh, mavlink-dev, flash-analysis, performance-test | Hidden by default, advanced mode only |
| `bootloader` | Bootloader binaries | bootloader, canbootloader | Never shown to end users |

### Auto-Detection

The firmware category is automatically inferred from the build label (the last segment of the target string after the final `_`):

- Labels `multicopter`, `fixedwing`, `vtol`, `rover`, `uuv`, `spacecraft` → `vehicle`
- Labels `bootloader`, `canbootloader` → `bootloader`
- Boards with `CONFIG_BOARD_ROMFSROOT="cannode"` (CAN sensor peripherals) → `peripheral`
- Everything else (`default`, `zenoh`, `mavlink-dev`, etc.) → `dev`

The known vehicle labels are maintained in `_VEHICLE_LABELS` in `Tools/manifest/gen_board_manifest_from_defconfig.py`.
Peripheral detection uses the `cannode` ROMFS root, which is shared by all CAN sensor boards (ARK, Holybro, CUAV, Freefly, Matek, NXP, etc.).
A build-time warning is emitted to stderr when an unrecognized label (other than `default`) falls through to `dev`, so that new vehicle types are not silently hidden from end users.

### Adding a New Vehicle Type

If you are adding a new vehicle type (e.g. `balloon`), you must do **one** of the following — otherwise the build is silently classified as `dev` and hidden from end users in QGroundControl:

1. **Add the label to `_VEHICLE_LABELS`** in `Tools/manifest/gen_board_manifest_from_defconfig.py` — this is the preferred approach when the vehicle type applies across multiple boards.
2. **Set `CONFIG_BOARD_FIRMWARE_CATEGORY="vehicle"`** in the `.px4board` file — use this for one-off overrides or when a board variant doesn't follow the standard naming.

### Overriding Auto-Detection

If a board variant needs a non-standard classification for any reason, set `CONFIG_BOARD_FIRMWARE_CATEGORY` in the `.px4board` file:

```
CONFIG_BOARD_FIRMWARE_CATEGORY="vehicle"
```

This override takes precedence over auto-detection.

## Pretty Labels (`label_pretty`)

The `label_pretty` field provides a human-readable name for each build variant.
Ground stations display this instead of raw target strings like `px4_fmu-v6x_default`.

### Setting `label_pretty`

Set `CONFIG_BOARD_LABEL_PRETTY` in each `.px4board` file:

```
CONFIG_BOARD_LABEL_PRETTY="Multicopter"
```

### Kconfig Inheritance Caveat

Non-base variants (e.g. `multicopter.px4board`, `rover.px4board`) are merged on top of `default.px4board`.
If `default.px4board` sets `CONFIG_BOARD_LABEL_PRETTY="Default"` and a variant file does not override it, the variant inherits the "Default" label — which is incorrect.

**Every variant file must set its own `CONFIG_BOARD_LABEL_PRETTY`.**

See [PX4 Board Configuration > Kconfig Label Inheritance](porting_guide_config.md#px4-kconfig-label-inheritance) for more details on how variant configs inherit from defaults.

## Build Pipeline

The manifest flows through three stages:

1. **`gen_board_manifest_from_defconfig.py`** — Generates per-build `manifest.json` from CMake variables and defconfig values, including auto-detection of `firmware_category`
2. **`px_mkfw.py`** — Bundles the manifest into the `.px4` firmware file
3. **`update_firmware_manifest.py`** — Aggregates individual `.px4` manifests into a unified release manifest for ground station consumption

## Backward Compatibility

- Old `.px4` files without `label_pretty` / `firmware_category`: ground stations fall back to raw label and filename matching (existing behavior)
- These are additive fields — no `format_version` bump is needed
