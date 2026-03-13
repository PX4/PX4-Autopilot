# Software Bill of Materials (SBOM)

PX4 generates a [Software Bill of Materials](https://ntia.gov/SBOM) for every firmware build in [SPDX 2.3](https://spdx.github.io/spdx-spec/v2.3/) JSON format.

## Why SBOM?

- **Regulatory compliance**: The EU Cyber Resilience Act (CRA) requires SBOMs for products with digital elements (reporting obligations begin September 2026).
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

```
build/<target>/<target>.sbom.spdx.json
```

For example:
```
build/px4_fmu-v6x_default/px4_fmu-v6x_default.sbom.spdx.json
```

The generator script is `Tools/ci/generate_sbom.py`.
It uses only Python standard library (no pip dependencies).

### CMake Integration

The `sbom` CMake target is included in the default `ALL` target.
The relevant cmake module is `cmake/sbom.cmake`.

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

| Location | Path |
|----------|------|
| Build directory | `build/<target>/<target>.sbom.spdx.json` |
| GitHub Releases | Alongside `.px4` firmware files |
| S3 | Same directory as firmware artifacts |

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

## Updating License Mappings

Submodule licenses are maintained in a hardcoded map in `Tools/ci/generate_sbom.py` (`SUBMODULE_LICENSES`).
When adding or changing submodules:

1. Identify the license of the new submodule (check its LICENSE file or repository).
2. Add the mapping to `SUBMODULE_LICENSES` using the [SPDX license identifier](https://spdx.org/licenses/).
3. If the license is not in the SPDX list, use `LicenseRef-<name>`.

Unmapped submodules will appear as `NOASSERTION` in the SBOM.
