#!/usr/bin/env python3
"""Generate an SPDX 3.0.1 JSON-LD SBOM for a PX4 firmware build.

Produces one SBOM per board target containing:
- PX4 firmware as the primary package (the document's root element)
- Git submodules and board-specific modules, which the firmware contains
- Python build requirements, which the firmware depends on at build time
- The compiler, which the firmware uses as a build tool

Licenses are expressed as LicenseExpression elements linked to each package
by hasDeclaredLicense and hasConcludedLicense relationships. A package whose
license is unknown is linked to the NoAssertionLicense individual, so the
SBOM states explicitly that the license is unknown. Unknown copyright text is
recorded as NOASSERTION for the same reason.

The output uses only the standard library plus PyYAML (for the overrides
file), so it runs unchanged in every build container.
"""
import argparse
import configparser
import hashlib
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

SPDX_SPEC_VERSION = "3.0.1"
SPDX_CONTEXT = "https://spdx.org/rdf/3.0.1/spdx-context.jsonld"
SPDX_LICENSE_LIST_VERSION = "3.22.0"
CREATION_INFO_ID = "_:creationinfo"
NOASSERTION = "NOASSERTION"
NOASSERTION_LICENSE = "expandedlicensing_NoAssertionLicense"

PX4_REPO_URL = "https://github.com/PX4/PX4-Autopilot"
PX4_COPYRIGHT = "Copyright (c) PX4 Development Team"
PX4_ORGANIZATION = "Dronecode Foundation"
PYPI_ORGANIZATION = "Python Package Index"

# Compiler vendors recognized from the first line of `<compiler> --version`.
# Unrecognized compilers get no supplier rather than a guessed one.
COMPILER_VENDORS = [
    ("Arm GNU Toolchain", "Arm Limited"),
    ("GNU Arm Embedded Toolchain", "Arm Limited"),
    ("Apple clang", "Apple Inc."),
    ("Ubuntu", "Canonical Ltd."),
]


def load_overrides(source_dir):
    """Load per-submodule SBOM metadata from Tools/ci/license-overrides.yaml.

    Returns (overrides, organizations): overrides maps a submodule path to its
    entry (license, comment, copyright, originator, requires, scope), and
    organizations maps a git host organization to a display name. Both are
    empty if the file is missing.
    """
    yaml_path = source_dir / "Tools" / "ci" / "license-overrides.yaml"
    if not yaml_path.exists():
        return {}, {}

    with open(yaml_path) as f:
        data = yaml.safe_load(f) or {}

    organizations = {k.lower(): v for k, v in (data.get("organizations") or {}).items()}
    return data.get("overrides") or {}, organizations


def license_overrides(overrides):
    """Map submodule path to overridden license, for entries that set one."""
    return {path: entry["license"] for path, entry in overrides.items() if "license" in entry}


def load_board_config(board_config):
    """Read a Kconfig boardconfig file into a dict of enabled symbols."""
    if not board_config or not board_config.exists():
        return None
    config = {}
    for line in board_config.read_text().splitlines():
        match = re.match(r"^(CONFIG_[A-Za-z0-9_]+)=(.*)$", line.strip())
        if match and match.group(2) != "n":
            config[match.group(1)] = match.group(2)
    return config


def requirement_met(requires, board_config):
    """Evaluate a `requires` entry ("CONFIG_X" or "!CONFIG_X") against the board config.

    Without a board config the requirement cannot be checked and is treated as met.
    """
    if not requires or board_config is None:
        return True
    negate = requires.startswith("!")
    return (requires.lstrip("!") in board_config) != negate

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


def get_submodule_license(source_dir, sub_path, overridden):
    """Return the SPDX license for a submodule: override > auto-detect."""
    if sub_path in overridden:
        return overridden[sub_path]
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


def run_command(args, cwd=None):
    """Run a command and return its stdout, or None if it fails or is missing."""
    try:
        result = subprocess.run(
            args,
            cwd=str(cwd) if cwd else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
        )
    except (subprocess.CalledProcessError, OSError):
        return None
    return result.stdout


def get_git_objects(source_dir):
    """Map paths in HEAD to (object type, hash) via git ls-tree (works without init).

    Covers submodule commits and directory trees, which give the submodule
    versions and the content identifiers of PX4 modules.
    """
    output = run_command(["git", "ls-tree", "-r", "-t", "HEAD"], source_dir) or ""
    objects = {}
    for line in output.splitlines():
        meta, _, path = line.partition("\t")
        parts = meta.split()
        if len(parts) == 3 and parts[1] in ("commit", "tree"):
            objects[path] = (parts[1], parts[2])
    return objects


def get_dirty_paths(source_dir):
    """Return the paths that differ from HEAD (modified, staged or untracked)."""
    output = run_command(["git", "status", "--porcelain", "--ignore-submodules=all"], source_dir) or ""
    return [line[3:].split(" -> ")[-1].strip('"') for line in output.splitlines()]


def get_git_info(source_dir: Path) -> dict:
    """Get the PX4 version (git describe), commit hash and exact tag, if any."""
    describe = run_command(["git", "describe", "--always", "--tags", "--dirty"], source_dir)
    commit = run_command(["git", "rev-parse", "HEAD"], source_dir)
    tag = run_command(["git", "describe", "--exact-match", "--tags", "HEAD"], source_dir)
    commit = commit.strip() if commit else "unknown"
    return {
        "version": describe.strip() if describe else "unknown",
        "hash": commit,
        # purl versions must be a tag or a commit, never a describe string
        "purl_version": tag.strip() if tag else commit,
    }


def get_python_package_version(name):
    """Return the installed version of a Python distribution, or None.

    Uses the interpreter running the generator, which is the one the build
    uses for its Python tooling.
    """
    try:
        from importlib import metadata
    except ImportError:
        return None
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


def get_compiler_info(compiler):
    """Return (name, version, vendor) for a compiler path; unknown parts are None."""
    name = Path(compiler).name
    version = run_command([compiler, "-dumpfullversion"]) or run_command([compiler, "-dumpversion"])
    banner = (run_command([compiler, "--version"]) or "").splitlines()
    first_line = banner[0] if banner else ""
    vendor = next((v for marker, v in COMPILER_VENDORS if marker in first_line), None)
    return name, version.strip() if version else None, vendor


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def swhid(object_type, object_hash):
    """Software Heritage ID for a git object: commits are revisions, trees are directories."""
    kind = {"commit": "rev", "tree": "dir"}[object_type]
    return {
        "type": "software_ContentIdentifier",
        "software_contentIdentifierType": "swhid",
        "software_contentIdentifierValue": f"swh:1:{kind}:{object_hash}",
    }


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


def make_purl(pkg_type: str, namespace: str, name: str, version: str = "", subpath: str = "") -> str:
    """Construct a Package URL (purl)."""
    purl = f"pkg:{pkg_type}/{namespace}/{name}" if namespace else f"pkg:{pkg_type}/{name}"
    if version:
        purl += f"@{version}"
    if subpath:
        purl += f"#{subpath}"
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


class SpdxGraph:
    """Collects the elements of an SPDX 3 JSON-LD document.

    Every element gets an IRI under the document namespace and refers to one
    shared CreationInfo blank node. Organizations and license expressions are
    created once and reused by every element that refers to them.
    """

    def __init__(self, namespace, created):
        self.namespace = namespace
        self.creation_info = {
            "type": "CreationInfo",
            "@id": CREATION_INFO_ID,
            "specVersion": SPDX_SPEC_VERSION,
            "created": created,
            "createdBy": [],
            "createdUsing": [],
        }
        self.elements = []
        self._organizations = {}
        self._licenses = {}
        self._relationship_count = 0

    def iri(self, local_id):
        return f"{self.namespace}#{spdx_id(local_id)}"

    def add(self, element_type, local_id, props):
        """Add an element, dropping properties whose value is None."""
        element = {
            "type": element_type,
            "spdxId": self.iri(local_id),
            "creationInfo": CREATION_INFO_ID,
        }
        element.update({k: v for k, v in props.items() if v is not None})
        self.elements.append(element)
        return element["spdxId"]

    def organization(self, name):
        if name not in self._organizations:
            self._organizations[name] = self.add(
                "Organization", f"Organization-{name}", {"name": name})
        return self._organizations[name]

    def license(self, expression):
        if expression not in self._licenses:
            self._licenses[expression] = self.add(
                "simplelicensing_LicenseExpression", f"License-{expression}", {
                    "simplelicensing_licenseExpression": expression,
                    "simplelicensing_licenseListVersion": SPDX_LICENSE_LIST_VERSION,
                })
        return self._licenses[expression]

    def relate(self, from_id, relationship_type, to_ids, scope=None, comment=None):
        """Add a Relationship, or a LifecycleScopedRelationship when scoped."""
        self._relationship_count += 1
        return self.add(
            "LifecycleScopedRelationship" if scope else "Relationship",
            f"Relationship-{self._relationship_count}", {
                "from": from_id,
                "relationshipType": relationship_type,
                "to": list(to_ids),
                "scope": scope,
                "comment": comment,
            })

    def set_license(self, package_id, license_id, comment=None):
        """Declare and conclude a license; NOASSERTION links to NoAssertionLicense."""
        if license_id == NOASSERTION:
            license_ref = NOASSERTION_LICENSE
        else:
            license_ref = self.license(license_id)
        self.relate(package_id, "hasDeclaredLicense", [license_ref])
        self.relate(package_id, "hasConcludedLicense", [license_ref], comment=comment)


def generate_sbom(source_dir, board, modules_file, compiler, platform="",
                  board_config=None, firmware_file=None):
    """Generate a complete SPDX 3.0.1 JSON-LD document."""
    overrides, org_names = load_overrides(source_dir)
    overridden = license_overrides(overrides)
    config = load_board_config(board_config)
    git_info = get_git_info(source_dir)
    git_objects = get_git_objects(source_dir)
    dirty_paths = get_dirty_paths(source_dir)
    timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    # Deterministic namespace using UUID5 from git hash + board
    ns_seed = f"{git_info['hash']}:{board}"
    doc_namespace = f"https://spdx.org/spdxdocs/{board}-{uuid.uuid5(uuid.NAMESPACE_URL, ns_seed)}"

    graph = SpdxGraph(doc_namespace, timestamp)
    dronecode = graph.organization(PX4_ORGANIZATION)

    def supplier(org):
        """Organization element for a git host organization, by display name."""
        return graph.organization(org_names.get(org.lower(), org)) if org else None

    def px4_purl(subpath=""):
        return make_purl("github", "PX4", "PX4-Autopilot", git_info["purl_version"], subpath)

    def content_id(path):
        """SWHID of a path in HEAD, unless the working tree changed it."""
        if path not in git_objects or any(d == path or d.startswith(path + "/") for d in dirty_paths):
            return None
        return [swhid(*git_objects[path])]

    tool = graph.add("Tool", "Tool-px4-generate-sbom", {
        "name": f"px4-generate-sbom {git_info['version']}",
        "externalIdentifier": [{
            "type": "ExternalIdentifier",
            "externalIdentifierType": "packageUrl",
            "identifier": px4_purl("Tools/ci/generate_sbom.py"),
        }],
    })
    graph.creation_info["createdBy"].append(dronecode)
    graph.creation_info["createdUsing"].append(tool)

    # Primary package: PX4 firmware
    primary = graph.add("software_Package", f"PX4-{board}", {
        "name": board,
        "software_packageVersion": git_info["version"],
        "software_downloadLocation": PX4_REPO_URL,
        "software_packageUrl": px4_purl(),
        "software_primaryPurpose": "firmware",
        "software_copyrightText": PX4_COPYRIGHT,
        "suppliedBy": dronecode,
    })
    graph.set_license(primary, "BSD-3-Clause")

    # The distributed firmware file, only for builds that produce one
    if firmware_file and firmware_file.is_file():
        firmware = graph.add("software_File", f"File-{firmware_file.name}", {
            "name": firmware_file.name,
            "software_fileKind": "file",
            "software_primaryPurpose": "firmware",
            "verifiedUsing": [{
                "type": "Hash",
                "algorithm": "sha256",
                "hashValue": sha256_file(firmware_file),
            }],
        })
        graph.relate(primary, "hasDistributionArtifact", [firmware])

    # Git submodules (filtered to those relevant to this board's modules)
    submodules = parse_gitmodules(source_dir)
    modules = read_module_list(modules_file, source_dir)

    def submodule_is_relevant(sub_path):
        """A submodule is relevant if any board module path overlaps with it."""
        if not requirement_met(overrides.get(sub_path, {}).get("requires"), config):
            return False
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

    contained = []
    build_deps = []

    for sub in submodules:
        if not submodule_is_relevant(sub["path"]):
            continue
        sub_path = sub["path"]
        entry = overrides.get(sub_path, {})
        commit = git_objects.get(sub_path, (None, None))[1]
        license_id = get_submodule_license(source_dir, sub_path, overridden)

        host, org, repo = extract_git_host_org_repo(sub["url"])

        # Use repo name from URL for human-readable name, fall back to last path component
        display_name = repo if repo else sub_path.rsplit("/", 1)[-1]

        sub_id = graph.add("software_Package", f"Submodule-{sub_path.replace('/', '-')}", {
            "name": display_name,
            "software_packageVersion": commit,
            "software_downloadLocation": sub["url"] or None,
            "software_packageUrl": make_purl(host, org, repo, commit) if host and org and repo else None,
            "software_copyrightText": entry.get("copyright", NOASSERTION),
            "software_contentIdentifier": [swhid("commit", commit)] if commit else None,
            "suppliedBy": supplier(org),
            "originatedBy": [graph.organization(entry["originator"])] if "originator" in entry else None,
        })
        graph.set_license(sub_id, license_id, comment=entry.get("comment"))
        (build_deps if entry.get("scope") == "build" else contained).append(sub_id)

    # Board-specific modules (already read above for submodule filtering)
    for mod in modules:
        # Derive short name: strip leading src/ for readability
        display_name = mod
        if display_name.startswith("src/"):
            display_name = display_name[4:]

        mod_id = graph.add("software_Package", f"Module-{mod.replace('/', '-')}", {
            "name": display_name,
            "software_packageVersion": git_info["version"],
            "software_downloadLocation": PX4_REPO_URL,
            "software_packageUrl": px4_purl(mod),
            "software_copyrightText": PX4_COPYRIGHT,
            "software_contentIdentifier": content_id(mod),
            "suppliedBy": dronecode,
        })
        graph.set_license(mod_id, "BSD-3-Clause")
        contained.append(mod_id)

    if contained:
        graph.relate(primary, "contains", contained)

    # Python build dependencies, at the version installed in the build environment
    requirements_path = source_dir / "Tools" / "setup" / "requirements.txt"
    pypi = graph.organization(PYPI_ORGANIZATION)

    for dep in parse_requirements(requirements_path):
        dep_name = dep["name"]
        version = get_python_package_version(dep_name)
        dep_id = graph.add("software_Package", f"PyDep-{dep_name}", {
            "name": dep_name,
            "software_packageVersion": version,
            "software_downloadLocation": f"https://pypi.org/project/{dep_name}/",
            "software_packageUrl": make_purl("pypi", "", dep_name.lower().replace("_", "-"), version),
            "software_primaryPurpose": "application",
            "software_copyrightText": NOASSERTION,
            "suppliedBy": pypi,
        })
        graph.set_license(dep_id, NOASSERTION)
        build_deps.append(dep_id)

    if build_deps:
        graph.relate(primary, "dependsOn", build_deps, scope="build")

    # Compiler as a build tool
    if compiler:
        name, version, vendor = get_compiler_info(compiler)
        compiler_id = graph.add("software_Package", f"Compiler-{name}", {
            "name": name,
            "software_packageVersion": version,
            "software_packageUrl": make_purl("generic", "", name, version),
            "software_primaryPurpose": "application",
            "software_copyrightText": NOASSERTION,
            "suppliedBy": graph.organization(vendor) if vendor else None,
        })
        graph.set_license(compiler_id, NOASSERTION)
        graph.relate(primary, "usesTool", [compiler_id], scope="build")

    document = {
        "type": "SpdxDocument",
        "spdxId": graph.iri("SpdxDocument"),
        "creationInfo": CREATION_INFO_ID,
        "name": f"PX4 Firmware SBOM for {board}",
        "dataLicense": graph.license("CC0-1.0"),
        "profileConformance": ["core", "software", "simpleLicensing"],
        "rootElement": [primary],
        "element": [e["spdxId"] for e in graph.elements],
    }

    return {
        "@context": SPDX_CONTEXT,
        "@graph": [graph.creation_info, document] + graph.elements,
    }


def verify_licenses(source_dir):
    """Verify license detection for all submodules. Returns exit code."""
    overridden = license_overrides(load_overrides(source_dir)[0])
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
        has_explicit_override = sub_path in overridden
        if not checked_out:
            detected = "(not checked out)"
            override = overridden.get(sub_path, "")
            final = override if override else "NOASSERTION"
        else:
            detected = detect_license(sub_dir)
            override = overridden.get(sub_path, "")
            final = override if override else detected

        if final == "NOASSERTION" and has_explicit_override:
            # Explicitly acknowledged in overrides file — not a failure
            marker = " (acknowledged)"
        elif final == "NOASSERTION" and checked_out:
            has_noassertion = True
            marker = " <-- UNRESOLVED"
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
        override = overridden.get(sub_path, "")
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
        print("FAIL: Some submodules have unresolved licenses. "
              "Add an entry to Tools/ci/license-overrides.yaml or check the LICENSE file.")
        return 1

    print("OK: All submodules have a resolved license.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Generate SPDX 3.0.1 JSON-LD SBOM for PX4 firmware"
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
        "--board-config",
        type=Path,
        default=None,
        help="Kconfig boardconfig file, used to evaluate `requires` entries in the overrides file",
    )
    parser.add_argument(
        "--firmware-file",
        type=Path,
        default=None,
        help="Firmware file the build distributes (e.g. the .px4), recorded with its SHA-256",
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
        board_config=args.board_config,
        firmware_file=args.firmware_file,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(sbom, f, indent=2)
        f.write("\n")

    pkg_count = sum(1 for e in sbom["@graph"] if e["type"] == "software_Package")
    print(f"SBOM generated: {args.output} ({pkg_count} packages)")


if __name__ == "__main__":
    main()
