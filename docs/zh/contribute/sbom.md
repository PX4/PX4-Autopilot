# Software Bill of Materials (SBOM)

PX4 generates a [Software Bill of Materials](https://ntia.gov/SBOM) for every firmware build in [SPDX 2.3](https://spdx.github.io/spdx-spec/v2.3/) JSON format.

## Why SBOM?

- **Regulatory compliance**: The EU Cyber Resilience Act (CRA) requires SBOMs for products with digital elements (reporting obligations begin in September 2026).
- **Supply chain transparency**: SBOMs enumerate every component compiled into firmware, enabling users and integrators to audit dependencies.
- **NTIA minimum elements**: Each SBOM satisfies all seven [NTIA required fields](https://www.ntia.gov/report/2021/minimum-elements-software-bill-materials-sbom): supplier, component name, version, unique identifier, dependency relationship, author, and timestamp.

## Format

PX4 uses SPDX 2.3 JSON.
SPDX is the Linux Foundation's own standard (ISO/IEC 5962), aligning with PX4's position as a Dronecode/LF project.
Zephyr RTOS also uses SPDX.

Each SBOM contains:

- **Primary package**: The PX4 firmware for a specific board target, marked with `primaryPackagePurpose: FIRMWARE`.
- **Git submodules**: All third-party libraries included via git submodules (~33 packages), with SPDX license identifiers and commit hashes.
- **Python build dependencies**: Packages from `Tools/setup/requirements.txt` marked as `BUILD_DEPENDENCY_OF` the firmware.
- **Board-specific modules**: Internal PX4 modules compiled for the target board.
- **Compiler**: The C compiler used for the build.

Typical SBOM size: 70-100 packages, ~500 lines, ~20 KB JSON.

## Generation

SBOMs are generated automatically as part of every CMake build.
The output file is:

```txt
build/<target>/<target>.sbom.spdx.json
```

例如：

```txt
build/px4_fmu-v6x_default/px4_fmu-v6x_default.sbom.spdx.json
```

The generator script is `Tools/ci/generate_sbom.py`.
It requires PyYAML (`pyyaml`) for loading license overrides.

### CMake Integration

The `sbom` CMake target is included in the default `ALL` target.
The relevant CMake module is `cmake/sbom.cmake`.

### Disabling SBOM Generation

Set the environment variable before building.
This is checked at CMake configure time, so a clean build or reconfigure is required:

```sh
PX4_SBOM_DISABLE=1 make px4_fmu-v6x_default
```

If the build directory already exists, force a reconfigure:

```sh
PX4_SBOM_DISABLE=1 cmake -B build/px4_fmu-v6x_default .
```

### Manual Generation

You can also run the generator directly:

```sh
python3 Tools/ci/generate_sbom.py \
    --source-dir . \
    --board px4_fmu-v6x_default \
    --modules-file build/px4_fmu-v6x_default/config_module_list.txt \
    --compiler arm-none-eabi-gcc \
    --output build/px4_fmu-v6x_default/px4_fmu-v6x_default.sbom.spdx.json
```

## Artifacts

SBOMs are available in:

| Location        | Path                                     |
| --------------- | ---------------------------------------- |
| Build directory | `build/<target>/<target>.sbom.spdx.json` |
| GitHub Releases | Alongside `.px4` firmware files          |
| S3              | Same directory as firmware artifacts     |

## Validation

Validate an SBOM against the SPDX JSON schema:

```sh
python3 -c "
import json
doc = json.load(open('build/px4_sitl_default/px4_sitl_default.sbom.spdx.json'))
assert doc['spdxVersion'] == 'SPDX-2.3'
assert doc['dataLicense'] == 'CC0-1.0'
assert len(doc['packages']) > 0
print(f'Valid: {len(doc[\"packages\"])} packages')
"
```

For full schema validation, use the [SPDX online validator](https://tools.spdx.org/app/validate/) or the `spdx-tools` CLI.

## License Detection

Submodule licenses are identified through a combination of auto-detection and manual overrides.

### Auto-Detection

The generator reads the first 100 lines of each submodule's LICENSE or COPYING file
and matches keywords against known patterns.
Copyleft licenses (GPL, LGPL, AGPL) are checked before permissive ones
to prevent false positives.

Supported patterns include:

| SPDX Identifier               | Matched Keywords                                                   |
| ----------------------------- | ------------------------------------------------------------------ |
| GPL-3.0-only  | "GNU GENERAL PUBLIC LICENSE", "Version 3"                          |
| GPL-2.0-only  | "GNU GENERAL PUBLIC LICENSE", "Version 2"                          |
| LGPL-3.0-only | "GNU LESSER GENERAL PUBLIC LICENSE", "Version 3"                   |
| LGPL-2.1-only | "GNU Lesser General Public License", "Version 2.1" |
| AGPL-3.0-only | "GNU AFFERO GENERAL PUBLIC LICENSE", "Version 3"                   |
| Apache-2.0    | "Apache License", "Version 2.0"                    |
| MIT                           | "Permission is hereby granted"                                     |
| BSD-3-Clause                  | "Redistribution and use", "Neither the name"                       |
| BSD-2-Clause                  | "Redistribution and use", "THIS SOFTWARE IS PROVIDED"              |
| ISC                           | "Permission to use, copy, modify, and/or distribute"               |
| EPL-2.0       | "Eclipse Public License", "2.0"                    |
| Unlicense                     | "The Unlicense", "unlicense.org"                   |

If no pattern matches, the license is set to `NOASSERTION`.

### Override File

When auto-detection fails or returns the wrong result,
add an entry to `Tools/ci/license-overrides.yaml`:

```yaml
overrides:
  src/lib/crypto/libtomcrypt:
    license: "Unlicense"
    comment: "Public domain dedication. Functionally equivalent to Unlicense."
```

Each entry maps a submodule path to its correct SPDX license identifier.
The optional `comment` field is emitted as `licenseComments` in the SBOM,
providing context for auditors reviewing complex licensing situations
(dual licenses, composite LICENSE files, public domain dedications).

### Copyleft Guardrail

The `--verify-licenses` command flags submodules with copyleft licenses
(GPL, LGPL, AGPL) in a dedicated warning section.
This is informational only and does not cause a failure.
It helps maintainers track copyleft obligations when adding new submodules.

### Platform Filtering

Submodules under `platforms/nuttx/` are excluded from POSIX and QURT SBOMs.
The `--platform` argument (set automatically by CMake via `${PX4_PLATFORM}`)
controls which platform-specific submodules are included.
This ensures SITL builds do not list NuttX RTOS packages.

### 验证

Run the verify command to check detection for all submodules:

```sh
python3 Tools/ci/generate_sbom.py --verify-licenses --source-dir .
```

This prints each submodule with its detected license, any override, and the final value.
It exits non-zero if any checked-out submodule resolves to `NOASSERTION` without an override.
Copyleft warnings are printed after the main table.

### Adding a New Submodule

1. Add the submodule normally.
2. Run `--verify-licenses` to confirm the license is detected.
3. If detection fails, add an override to `Tools/ci/license-overrides.yaml`.
4. If the license is not in the SPDX list, use `LicenseRef-<name>`.

### EU CRA Compliance

The EU Cyber Resilience Act requires SBOMs for products with digital elements.
The goal is zero `NOASSERTION` licenses in shipped firmware SBOMs.
Every submodule should have either a detected or overridden license.
The `--verify-licenses` check enforces this in CI.

## What's in an SBOM

This section is for integrators, compliance teams, and anyone reviewing SBOM artifacts.

### Where to Find SBOMs

| Location        | Path                                     |
| --------------- | ---------------------------------------- |
| Build directory | `build/<target>/<target>.sbom.spdx.json` |
| GitHub Releases | Alongside `.px4` firmware files          |
| S3              | Same directory as firmware artifacts     |

### Reading the JSON

Each SBOM is a single JSON document following SPDX 2.3.
Key fields:

- **`packages`**: Array of all components. Each has `name`, `versionInfo`, `licenseConcluded`, and `SPDXID`.
- **`relationships`**: How packages relate. `CONTAINS` means a submodule is compiled into firmware. `BUILD_DEPENDENCY_OF` means a tool used only during build.
- **`licenseConcluded`**: The SPDX license identifier determined for that package.
- **`licenseComments`**: Free-text explanation for complex cases (dual licenses, composite files, public domain).
- **`externalRefs`**: Package URLs (purls) linking to GitHub repos or PyPI.

### Understanding NOASSERTION

`NOASSERTION` means no license could be determined.
For submodules, this happens when:

- The submodule is not checked out (common in CI shallow clones).
- No LICENSE/COPYING file exists.
- The LICENSE file does not match any known pattern and no override is configured.

For shipped firmware, `NOASSERTION` should be resolved by adding an override.
For build-only dependencies (Python packages), `NOASSERTION` is acceptable
since these are not compiled into the firmware binary.
