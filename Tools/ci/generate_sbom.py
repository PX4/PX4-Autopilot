#!/usr/bin/env python3
"""Generate SPDX 2.3 JSON SBOM for a PX4 firmware build.

Produces one SBOM per board target containing:
- PX4 firmware as the primary package
- Git submodules as CONTAINS dependencies
- Python build requirements as BUILD_DEPENDENCY_OF packages
- Board-specific modules as CONTAINS packages

Requires PyYAML (pyyaml) for loading license overrides.
"""
import argparse
import configparser
import json
import re
import subprocess
import uuid
from datetime import datetime, timezone
from pathlib import Path

import yaml

# Ordered most-specific first: all keywords must appear for a match.
LICENSE_PATTERNS = [
    # Copyleft licenses first (more specific keywords prevent false matches)
    ("GPL-3.0-only",    ["GNU GENERAL PUBLIC LICENSE", "Version 3"]),
    ("GPL-2.0-only",    ["GNU GENERAL PUBLIC LICENSE", "Version 2"]),
    ("LGPL-3.0-only",   ["GNU LESSER GENERAL PUBLIC LICENSE", "Version 3"]),
    ("LGPL-2.1-only",   ["GNU Lesser General Public License", "Version 2.1"]),
    ("AGPL-3.0-only",   ["GNU AFFERO GENERAL PUBLIC LICENSE", "Version 3"]),
    # Permissive licenses
    ("Apache-2.0",      ["Apache License", "Version 2.0"]),
    ("MIT",             ["Permission is hereby granted"]),
    ("BSD-3-Clause",    ["Redistribution and use", "Neither the name"]),
    ("BSD-2-Clause",    ["Redistribution and use", "THIS SOFTWARE IS PROVIDED"]),
    ("ISC",             ["Permission to use, copy, modify, and/or distribute"]),
    ("EPL-2.0",         ["Eclipse Public License", "2.0"]),
    ("Unlicense",       ["The Unlicense", "unlicense.org"]),
]

COPYLEFT_LICENSES = {
    "GPL-2.0-only", "GPL-3.0-only",
    "LGPL-2.1-only", "LGPL-3.0-only",
    "AGPL-3.0-only",
}

def load_license_overrides(source_dir):
    """Load license overrides and comments from YAML config file.

    Returns (overrides, comments) dicts mapping submodule path to values.
    Falls back to empty dicts if the file is missing.
    """
    yaml_path = source_dir / "Tools" / "ci" / "license-overrides.yaml"
    if not yaml_path.exists():
        return {}, {}

    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    overrides = {}
    comments = {}
    for path, entry in (data.get("overrides") or {}).items():
        overrides[path] = entry["license"]
        if "comment" in entry:
            comments[path] = entry["comment"]

    return overrides, comments

LICENSE_FILENAMES = ["LICENSE", "LICENSE.md", "LICENSE.txt", "LICENCE", "LICENCE.md", "COPYING", "COPYING.md"]


def detect_license(submodule_dir):
    """Auto-detect SPDX license ID from LICENSE/COPYING file in a directory.

    Reads the first 100 lines of the first license file found and matches
    keywords against LICENSE_PATTERNS. Returns 'NOASSERTION' if no file
    is found or no pattern matches.
    """
    for fname in LICENSE_FILENAMES:
        license_file = submodule_dir / fname
        if license_file.is_file():
            try:
                lines = license_file.read_text(errors="replace").splitlines()[:100]
                text = "\n".join(lines)
            except OSError:
                continue

            text_upper = text.upper()
            for spdx_id_val, keywords in LICENSE_PATTERNS:
                if all(kw.upper() in text_upper for kw in keywords):
                    return spdx_id_val

            return "NOASSERTION"

    return "NOASSERTION"


def get_submodule_license(source_dir, sub_path, license_overrides):
    """Return the SPDX license for a submodule: override > auto-detect."""
    if sub_path in license_overrides:
        return license_overrides[sub_path]
    return detect_license(source_dir / sub_path)


def spdx_id(name: str) -> str:
    """Convert a name to a valid SPDX identifier (letters, digits, dots, hyphens)."""
    return re.sub(r"[^a-zA-Z0-9.\-]", "-", name)


def parse_gitmodules(source_dir):
    """Parse .gitmodules and return list of {name, path, url}."""
    gitmodules_path = source_dir / ".gitmodules"
    if not gitmodules_path.exists():
        return []

    config = configparser.ConfigParser()
    config.read(str(gitmodules_path))

    submodules = []
    for section in config.sections():
        if section.startswith("submodule "):
            name = section.split('"')[1] if '"' in section else section.split(" ", 1)[1]
            path = config.get(section, "path", fallback="")
            url = config.get(section, "url", fallback="")
            submodules.append({"name": name, "path": path, "url": url})

    return submodules


def get_submodule_commits(source_dir):
    """Get commit hashes for all submodules via git ls-tree -r (works without init)."""
    try:
        result = subprocess.run(
            ["git", "ls-tree", "-r", "HEAD"],
            cwd=str(source_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return {}

    commits = {}
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[1] == "commit":
            commits[parts[3]] = parts[2]

    return commits


def get_git_info(source_dir: Path) -> dict:
    """Get PX4 git version and hash."""
    info = {"version": "unknown", "hash": "unknown"}
    try:
        result = subprocess.run(
            ["git", "describe", "--always", "--tags", "--dirty"],
            cwd=str(source_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        info["version"] = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(source_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        info["hash"] = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    return info


def parse_requirements(requirements_path):
    """Parse pip requirements.txt into list of {name, version_spec}."""
    if not requirements_path.exists():
        return []

    deps = []
    for line in requirements_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("-"):
            continue
        # Split on version specifiers
        match = re.match(r"^([a-zA-Z0-9_\-]+)(.*)?$", line)
        if match:
            deps.append({
                "name": match.group(1),
                "version_spec": match.group(2).strip() if match.group(2) else "",
            })
    return deps


def read_module_list(modules_file, source_dir):
    """Read board-specific module list from file.

    Paths may be absolute; they are converted to relative paths under src/.
    Duplicates are removed while preserving order.
    """
    if not modules_file or not modules_file.exists():
        return []

    seen = set()
    modules = []
    source_str = str(source_dir.resolve()) + "/"

    for line in modules_file.read_text().splitlines():
        path = line.strip()
        if not path or path.startswith("#"):
            continue
        # Convert absolute path to relative
        if path.startswith(source_str):
            path = path[len(source_str):]
        if path not in seen:
            seen.add(path)
            modules.append(path)

    return modules


def make_purl(pkg_type: str, namespace: str, name: str, version: str = "") -> str:
    """Construct a Package URL (purl)."""
    purl = f"pkg:{pkg_type}/{namespace}/{name}"
    if version:
        purl += f"@{version}"
    return purl


def extract_git_host_org_repo(url):
    """Extract host type, org, and repo from a git URL.

    Returns (host, org, repo) where host is 'github', 'gitlab', or ''.
    """
    match = re.search(r"github\.com[:/]([^/]+)/([^/]+?)(?:\.git)?$", url)
    if match:
        return "github", match.group(1), match.group(2)
    match = re.search(r"gitlab\.com[:/](.+?)/([^/]+?)(?:\.git)?$", url)
    if match:
        return "gitlab", match.group(1), match.group(2)
    return "", "", ""


def generate_sbom(source_dir, board, modules_file, compiler, platform=""):
    """Generate a complete SPDX 2.3 JSON document."""
    license_overrides, license_comments = load_license_overrides(source_dir)
    git_info = get_git_info(source_dir)
    timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    # Deterministic namespace using UUID5 from git hash + board
    ns_seed = f"{git_info['hash']}:{board}"
    doc_namespace = f"https://spdx.org/spdxdocs/{board}-{uuid.uuid5(uuid.NAMESPACE_URL, ns_seed)}"

    doc = {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"PX4 Firmware SBOM for {board}",
        "documentNamespace": doc_namespace,
        "creationInfo": {
            "created": timestamp,
            "creators": [
                "Tool: px4-generate-sbom",
                "Organization: Dronecode Foundation",
            ],
            "licenseListVersion": "3.22",
        },
        "packages": [],
        "relationships": [],
    }

    # Primary package: PX4 firmware
    primary_spdx_id = f"SPDXRef-PX4-{spdx_id(board)}"
    doc["packages"].append({
        "SPDXID": primary_spdx_id,
        "name": board,
        "versionInfo": git_info["version"],
        "packageFileName": f"{board}.px4",
        "supplier": "Organization: Dronecode Foundation",
        "downloadLocation": "https://github.com/PX4/PX4-Autopilot",
        "filesAnalyzed": False,
        "primaryPackagePurpose": "FIRMWARE",
        "licenseConcluded": "BSD-3-Clause",
        "licenseDeclared": "BSD-3-Clause",
        "copyrightText": "Copyright (c) PX4 Development Team",
        "externalRefs": [
            {
                "referenceCategory": "PACKAGE-MANAGER",
                "referenceType": "purl",
                "referenceLocator": make_purl(
                    "github", "PX4", "PX4-Autopilot", git_info["version"]
                ),
            }
        ],
    })

    doc["relationships"].append({
        "spdxElementId": "SPDXRef-DOCUMENT",
        "relationshipType": "DESCRIBES",
        "relatedSpdxElement": primary_spdx_id,
    })

    # Git submodules (filtered to those relevant to this board's modules)
    submodules = parse_gitmodules(source_dir)
    submodule_commits = get_submodule_commits(source_dir)
    modules = read_module_list(modules_file, source_dir)

    def submodule_is_relevant(sub_path):
        """A submodule is relevant if any board module path overlaps with it."""
        # NuttX platform submodules are only relevant for NuttX builds
        if sub_path.startswith("platforms/nuttx/"):
            return platform in ("nuttx", "")
        if not modules:
            return True  # no module list means include all
        # Other platform submodules are always relevant
        if sub_path.startswith("platforms/"):
            return True
        for mod in modules:
            # Module is under this submodule, or submodule is under a module
            if mod.startswith(sub_path + "/") or sub_path.startswith(mod + "/"):
                return True
        return False

    for sub in submodules:
        if not submodule_is_relevant(sub["path"]):
            continue
        sub_path = sub["path"]
        sub_path_id = sub_path.replace("/", "-")
        sub_spdx_id = f"SPDXRef-Submodule-{spdx_id(sub_path_id)}"
        commit = submodule_commits.get(sub_path, "unknown")
        license_id = get_submodule_license(source_dir, sub_path, license_overrides)

        host, org, repo = extract_git_host_org_repo(sub["url"])
        download = sub["url"] if sub["url"] else "NOASSERTION"

        # Use repo name from URL for human-readable name, fall back to last path component
        display_name = repo if repo else sub_path.rsplit("/", 1)[-1]

        pkg = {
            "SPDXID": sub_spdx_id,
            "name": display_name,
            "versionInfo": commit,
            "supplier": f"Organization: {org}" if org else "NOASSERTION",
            "downloadLocation": download,
            "filesAnalyzed": False,
            "licenseConcluded": license_id,
            "licenseDeclared": license_id,
            "copyrightText": "NOASSERTION",
        }

        comment = license_comments.get(sub_path)
        if comment:
            pkg["licenseComments"] = comment

        if host and org and repo:
            pkg["externalRefs"] = [
                {
                    "referenceCategory": "PACKAGE-MANAGER",
                    "referenceType": "purl",
                    "referenceLocator": make_purl(host, org, repo, commit),
                }
            ]

        doc["packages"].append(pkg)
        doc["relationships"].append({
            "spdxElementId": primary_spdx_id,
            "relationshipType": "CONTAINS",
            "relatedSpdxElement": sub_spdx_id,
        })

    # Python build dependencies
    requirements_path = source_dir / "Tools" / "setup" / "requirements.txt"
    py_deps = parse_requirements(requirements_path)

    for dep in py_deps:
        dep_name = dep["name"]
        dep_spdx_id = f"SPDXRef-PyDep-{spdx_id(dep_name)}"
        version_str = dep["version_spec"] if dep["version_spec"] else "NOASSERTION"

        doc["packages"].append({
            "SPDXID": dep_spdx_id,
            "name": dep_name,
            "versionInfo": version_str,
            "supplier": "NOASSERTION",
            "downloadLocation": f"https://pypi.org/project/{dep_name}/",
            "filesAnalyzed": False,
            "primaryPackagePurpose": "APPLICATION",
            "licenseConcluded": "NOASSERTION",
            "licenseDeclared": "NOASSERTION",
            "copyrightText": "NOASSERTION",
            "externalRefs": [
                {
                    "referenceCategory": "PACKAGE-MANAGER",
                    "referenceType": "purl",
                    "referenceLocator": f"pkg:pypi/{dep_name}",
                }
            ],
        })
        doc["relationships"].append({
            "spdxElementId": dep_spdx_id,
            "relationshipType": "BUILD_DEPENDENCY_OF",
            "relatedSpdxElement": primary_spdx_id,
        })

    # Board-specific modules (already read above for submodule filtering)
    for mod in modules:
        mod_path_id = mod.replace("/", "-")
        mod_spdx_id = f"SPDXRef-Module-{spdx_id(mod_path_id)}"

        # Derive short name: strip leading src/ for readability
        display_name = mod
        if display_name.startswith("src/"):
            display_name = display_name[4:]

        doc["packages"].append({
            "SPDXID": mod_spdx_id,
            "name": display_name,
            "versionInfo": git_info["version"],
            "supplier": "Organization: Dronecode Foundation",
            "downloadLocation": "https://github.com/PX4/PX4-Autopilot",
            "filesAnalyzed": False,
            "licenseConcluded": "BSD-3-Clause",
            "licenseDeclared": "BSD-3-Clause",
            "copyrightText": "NOASSERTION",
        })
        doc["relationships"].append({
            "spdxElementId": primary_spdx_id,
            "relationshipType": "CONTAINS",
            "relatedSpdxElement": mod_spdx_id,
        })

    # Compiler as a build tool
    if compiler:
        compiler_spdx_id = f"SPDXRef-Compiler-{spdx_id(compiler)}"
        doc["packages"].append({
            "SPDXID": compiler_spdx_id,
            "name": compiler,
            "versionInfo": "NOASSERTION",
            "supplier": "NOASSERTION",
            "downloadLocation": "NOASSERTION",
            "filesAnalyzed": False,
            "primaryPackagePurpose": "APPLICATION",
            "licenseConcluded": "NOASSERTION",
            "licenseDeclared": "NOASSERTION",
            "copyrightText": "NOASSERTION",
        })
        doc["relationships"].append({
            "spdxElementId": compiler_spdx_id,
            "relationshipType": "BUILD_TOOL_OF",
            "relatedSpdxElement": primary_spdx_id,
        })

    return doc


def verify_licenses(source_dir):
    """Verify license detection for all submodules. Returns exit code."""
    license_overrides, _ = load_license_overrides(source_dir)
    submodules = parse_gitmodules(source_dir)
    if not submodules:
        print("No submodules found in .gitmodules")
        return 1

    has_noassertion = False
    print(f"{'Submodule Path':<65} {'Detected':<16} {'Override':<16} {'Final'}")
    print("-" * 115)

    for sub in submodules:
        sub_path = sub["path"]
        sub_dir = source_dir / sub_path

        checked_out = sub_dir.is_dir() and any(sub_dir.iterdir())
        if not checked_out:
            detected = "(not checked out)"
            override = license_overrides.get(sub_path, "")
            final = override if override else "NOASSERTION"
        else:
            detected = detect_license(sub_dir)
            override = license_overrides.get(sub_path, "")
            final = override if override else detected

        if final == "NOASSERTION" and checked_out:
            has_noassertion = True
            marker = " <-- NOASSERTION"
        elif final == "NOASSERTION" and not checked_out:
            marker = " (skipped)"
        else:
            marker = ""

        print(f"{sub_path:<65} {str(detected):<16} {str(override) if override else '':<16} {final}{marker}")

    # Copyleft warning (informational, not a failure)
    copyleft_found = []
    for sub in submodules:
        sub_path = sub["path"]
        sub_dir = source_dir / sub_path
        checked_out = sub_dir.is_dir() and any(sub_dir.iterdir())
        override = license_overrides.get(sub_path, "")
        if checked_out:
            final_lic = override if override else detect_license(sub_dir)
        else:
            final_lic = override if override else "NOASSERTION"
        for cl in COPYLEFT_LICENSES:
            if cl in final_lic:
                copyleft_found.append((sub_path, final_lic))
                break

    print()
    if copyleft_found:
        print("Copyleft licenses detected (informational):")
        for path, lic in copyleft_found:
            print(f"  {path}: {lic}")
        print()

    if has_noassertion:
        print("FAIL: Some submodules resolved to NOASSERTION. "
              "Add an entry to Tools/ci/license-overrides.yaml or check the LICENSE file.")
        return 1

    print("OK: All submodules have a resolved license.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Generate SPDX 2.3 JSON SBOM for PX4 firmware"
    )
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=Path.cwd(),
        help="PX4 source directory (default: cwd)",
    )
    parser.add_argument(
        "--verify-licenses",
        action="store_true",
        help="Verify license detection for all submodules and exit",
    )
    parser.add_argument(
        "--board",
        default=None,
        help="Board target name (e.g. px4_fmu-v5x_default)",
    )
    parser.add_argument(
        "--modules-file",
        type=Path,
        default=None,
        help="Path to config_module_list.txt",
    )
    parser.add_argument(
        "--compiler",
        default="",
        help="Compiler identifier (e.g. arm-none-eabi-gcc)",
    )
    parser.add_argument(
        "--platform",
        default="",
        help="PX4 platform (nuttx, posix, qurt). Filters platform-specific submodules.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output SBOM file path",
    )

    args = parser.parse_args()

    if args.verify_licenses:
        raise SystemExit(verify_licenses(args.source_dir))

    if not args.board:
        parser.error("--board is required when not using --verify-licenses")
    if not args.output:
        parser.error("--output is required when not using --verify-licenses")

    sbom = generate_sbom(
        source_dir=args.source_dir,
        board=args.board,
        modules_file=args.modules_file,
        compiler=args.compiler,
        platform=args.platform,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(sbom, f, indent=2)
        f.write("\n")

    pkg_count = len(sbom["packages"])
    print(f"SBOM generated: {args.output} ({pkg_count} packages)")


if __name__ == "__main__":
    main()
