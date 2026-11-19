# Software Bill of Materials (SBOM)

PX4 generates a [Software Bill of Materials](https://www.ntia.gov/SBOM) for every firmware build in [SPDX 3.0.1](https://spdx.github.io/spdx-spec/v3.0.1/) JSON-LD format.

## Why SBOM?

- **Regulatory compliance**: The EU Cyber Resilience Act (CRA) requires SBOMs for products with digital elements (reporting obligations begin in September 2026).
- **Supply chain transparency**: SBOMs enumerate every component compiled into firmware, enabling users and integrators to audit dependencies.
- **NTIA minimum elements**: Each SBOM satisfies all seven [NTIA required fields](https://www.ntia.gov/report/2021/minimum-elements-software-bill-materials-sbom): supplier, component name, version, unique identifier, dependency relationship, author, and timestamp.

## Format

PX4 uses SPDX 3.0.1, serialized as JSON-LD.
SPDX is the Linux Foundation's own standard (ISO/IEC 5962), aligning with PX4's position as a Dronecode/LF project.
Zephyr RTOS also uses SPDX.

Earlier PX4 releases produced SPDX 2.3 JSON, with the same file name.

Each SBOM contains:

- **Primary package**: The PX4 firmware for a specific board target, with `software_primaryPurpose: firmware`. It is the document's root element.
- **Firmware file**: The `.px4` file the build distributes, with its SHA-256 hash. Only builds that produce a `.px4` (NuttX boards) include it.
- **Git submodules**: The third-party libraries included via git submodules that the board actually builds, with SPDX license expressions, commit hashes and, for PX4-hosted forks, the upstream project.
- **Board-specific modules**: Internal PX4 modules compiled for the target board.
- **Python build dependencies**: Packages from `Tools/setup/requirements.txt`, at the version installed in the build environment, which the firmware depends on at build time.
- **Build-only submodules**: Submodules used only to generate code during the build (such as the DroneCAN DSDL compiler), recorded as build dependencies rather than firmware contents.
- **Compiler**: The C compiler used for the build, with its version, recorded as a build tool.

A typical NuttX board SBOM has around 280 packages and is about 700 KB.

## Generation

SBOMs are generated automatically as part of every CMake build.
The output file is:

```txt
build/<target>/<target>.sbom.spdx.json
```

For example:

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
    --board-config build/px4_fmu-v6x_default/boardconfig \
    --compiler arm-none-eabi-gcc \
    --platform nuttx \
    --firmware-file build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4 \
    --output build/px4_fmu-v6x_default/px4_fmu-v6x_default.sbom.spdx.json
```

`--board-config` and `--firmware-file` are optional.
Without `--board-config`, submodules that depend on a Kconfig option (see `requires` below) are always included.
Without `--firmware-file`, the SBOM has no firmware file element.

## Artifacts

SBOMs are available in:

| Location        | Path                                     |
| --------------- | ---------------------------------------- |
| Build directory | `build/<target>/<target>.sbom.spdx.json` |
| GitHub Releases | Alongside `.px4` firmware files          |
| S3              | Same directory as firmware artifacts     |

## Validation

Validate an SBOM against the SPDX 3.0.1 JSON schema and SHACL model with [spdx3-validate](https://pypi.org/project/spdx3-validate/):

```sh
pip install spdx3-validate
spdx3-validate --json build/px4_sitl_default/px4_sitl_default.sbom.spdx.json
```

The validator downloads the schema and model from spdx.org, so it needs network access.

To summarize an SBOM or check the NTIA minimum elements, use `Tools/ci/inspect_sbom.py`:

```sh
python3 Tools/ci/inspect_sbom.py build/px4_sitl_default/px4_sitl_default.sbom.spdx.json
python3 Tools/ci/inspect_sbom.py build/px4_sitl_default/px4_sitl_default.sbom.spdx.json ntia
```

## License Detection

Submodule licenses are identified through a combination of auto-detection and manual overrides.

### Auto-Detection

The generator reads the first 100 lines of each submodule's LICENSE or COPYING file
and matches keywords against known patterns.
Copyleft licenses (GPL, LGPL, AGPL) are checked before permissive ones
to prevent false positives.

Supported patterns include:

| SPDX Identifier | Matched Keywords                                      |
| --------------- | ----------------------------------------------------- |
| GPL-3.0-only    | "GNU GENERAL PUBLIC LICENSE", "Version 3"             |
| GPL-2.0-only    | "GNU GENERAL PUBLIC LICENSE", "Version 2"             |
| LGPL-3.0-only   | "GNU LESSER GENERAL PUBLIC LICENSE", "Version 3"      |
| LGPL-2.1-only   | "GNU Lesser General Public License", "Version 2.1"    |
| AGPL-3.0-only   | "GNU AFFERO GENERAL PUBLIC LICENSE", "Version 3"      |
| Apache-2.0      | "Apache License", "Version 2.0"                       |
| MIT             | "Permission is hereby granted"                        |
| BSD-3-Clause    | "Redistribution and use", "Neither the name"          |
| BSD-2-Clause    | "Redistribution and use", "THIS SOFTWARE IS PROVIDED" |
| ISC             | "Permission to use, copy, modify, and/or distribute"  |
| EPL-2.0         | "Eclipse Public License", "2.0"                       |
| Unlicense       | "The Unlicense", "unlicense.org"                      |

If no pattern matches, the license is `NOASSERTION`: the submodule's license relationships point to the SPDX `NoAssertionLicense` value, which states explicitly that the license is unknown.

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
The optional `comment` field is emitted as the `comment` of the package's `hasConcludedLicense` relationship,
providing context for auditors reviewing complex licensing situations
(dual licenses, composite LICENSE files, public domain dedications).

Despite its name, the file holds all per-submodule SBOM metadata.
Every key is optional:

| Key          | Effect                                                                                                                                   |
| ------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `license`    | SPDX license expression, overriding auto-detection.                                                                                      |
| `comment`    | Why the license override exists.                                                                                                         |
| `copyright`  | Copyright text for the package. Without it the SBOM records `NOASSERTION`.                                                               |
| `originator` | Upstream organization of a fork hosted under the PX4 GitHub organization, recorded as `originatedBy`.                                    |
| `requires`   | Kconfig symbol the board must enable (`CONFIG_X`), or must not enable (`!CONFIG_X`), for the submodule to be listed.                     |
| `scope`      | `build` for submodules used only to build the firmware (code generators). They are recorded as build dependencies, not firmware contents. |

For example, the two Micro XRCE-DDS Client versions are listed only when the board selects them:

```yaml
overrides:
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client-v3:
    originator: "eProsima"
    requires: "CONFIG_UXRCE_DDS_CLIENT_USE_DDS_V3"
```

The top-level `organizations` map gives the supplier name for a git host organization:

```yaml
organizations:
  PX4: "Dronecode Foundation"
  mavlink: "MAVLink"
```

Organizations not listed appear under their git host name.

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

Other submodules are included when a module the board builds lives inside them or contains them,
and when the board configuration (passed by CMake via `--board-config`) satisfies their `requires` entry.

### Verification

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

Each SBOM is a single SPDX 3.0.1 JSON-LD document.
Everything in it is an element in the top-level `@graph` array, identified by its `spdxId` and distinguished by its `type`:

- **`SpdxDocument`**: The document itself. `rootElement` points to the firmware package.
- **`software_Package`**: One per component, with `name`, `software_packageVersion`, `software_downloadLocation`, `software_packageUrl` (a purl linking to the GitHub repo, PyPI or the PX4 source path), `software_copyrightText` and, where known, `suppliedBy`. Forks hosted by PX4 also have `originatedBy`, the upstream project.
- **`software_contentIdentifier`**: A [Software Heritage ID](https://www.softwareheritage.org/software-hash-identifiers/) of the exact source: `swh:1:rev:` (the commit) for submodules, `swh:1:dir:` (the git tree) for PX4 modules. Modules with uncommitted changes have none.
- **`software_File`**: The `.px4` firmware file, with a SHA-256 hash in `verifiedUsing`.
- **`Relationship`**: How elements relate. The firmware `contains` its submodules and modules, and `hasDistributionArtifact` the firmware file.
- **`LifecycleScopedRelationship`**: A relationship limited to a lifecycle stage. With `scope: build`, the firmware `dependsOn` the Python packages and build-only submodules, and `usesTool` the compiler, only while it is being built.
- **`simplelicensing_LicenseExpression`**: An SPDX license expression. Each package links to one through a `hasDeclaredLicense` and a `hasConcludedLicense` relationship. The `comment` on `hasConcludedLicense` explains complex cases (dual licenses, composite files, public domain).

### Understanding Unknown Values

The SBOM marks unknown information explicitly instead of leaving it out:

- A license that could not be determined links to `expandedlicensing_NoAssertionLicense`. `inspect_sbom.py` reports these packages as `NOASSERTION`.
- Unknown copyright text is `NOASSERTION`.

Python packages and the compiler always have an unknown license.
For submodules, the license is unknown when:

- The submodule is not checked out (common in CI shallow clones).
- No LICENSE/COPYING file exists.
- The LICENSE file does not match any known pattern and no override is configured.

For shipped firmware, `NOASSERTION` should be resolved by adding an override.
For build-only dependencies (Python packages, the compiler), `NOASSERTION` is acceptable
since these are not compiled into the firmware binary.

### Verifying Release Artifacts

Release builds (version tags) publish signed [build provenance attestations](https://docs.github.com/en/actions/security-for-github-actions/using-artifact-attestations/using-artifact-attestations-to-establish-provenance-for-builds) for every firmware file and SBOM.
The attestations are stored by GitHub and recorded in the public [Sigstore](https://www.sigstore.dev/) transparency log; the files themselves are unchanged.

To check that a file was built by the PX4 release workflow, use the GitHub CLI:

```sh
gh attestation verify px4_fmu-v6x_default.px4 --repo PX4/PX4-Autopilot
gh attestation verify px4_fmu-v6x_default.sbom.spdx.json --repo PX4/PX4-Autopilot
```

The SBOM records the SHA-256 of its `.px4`, so a verified SBOM whose hash matches the firmware file describes that exact firmware:

```sh
sha256sum px4_fmu-v6x_default.px4
```

Builds from branches (including `main`) are not attested.
