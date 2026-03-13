#!/usr/bin/env python3
"""Inspect a PX4 SPDX SBOM file.

Usage:
    inspect_sbom.py <sbom.spdx.json>                  # full summary
    inspect_sbom.py <sbom.spdx.json> search <term>    # search packages by name
    inspect_sbom.py <sbom.spdx.json> ntia             # NTIA minimum elements check
    inspect_sbom.py <sbom.spdx.json> licenses         # license summary
    inspect_sbom.py <sbom.spdx.json> list <type>      # list packages (Submodule|PyDep|Module|all)
"""

import json
import sys
from collections import Counter
from pathlib import Path


def load(path):
    return json.loads(Path(path).read_text())


def pkg_type(pkg):
    spdx_id = pkg["SPDXID"]
    for prefix in ("Submodule", "PyDep", "Module", "Compiler", "PX4"):
        if f"-{prefix}-" in spdx_id or spdx_id.startswith(f"SPDXRef-{prefix}"):
            return prefix
    return "Other"


def summary(doc):
    print(f"spdxVersion:  {doc['spdxVersion']}")
    print(f"name:         {doc['name']}")
    print(f"namespace:    {doc['documentNamespace']}")
    print(f"created:      {doc['creationInfo']['created']}")
    print(f"creators:     {', '.join(doc['creationInfo']['creators'])}")
    print()

    types = Counter(pkg_type(p) for p in doc["packages"])
    print(f"Packages: {len(doc['packages'])}")
    for t, c in types.most_common():
        print(f"  {t}: {c}")
    print()

    rc = Counter(r["relationshipType"] for r in doc["relationships"])
    print(f"Relationships: {len(doc['relationships'])}")
    for t, n in rc.most_common():
        print(f"  {t}: {n}")
    print()

    primary = doc["packages"][0]
    print(f"Primary package:")
    print(f"  name:    {primary['name']}")
    print(f"  version: {primary['versionInfo']}")
    print(f"  purpose: {primary.get('primaryPackagePurpose', 'N/A')}")
    print(f"  license: {primary['licenseDeclared']}")
    print()

    noassert = [
        p["name"]
        for p in doc["packages"]
        if pkg_type(p) == "Submodule" and p["licenseDeclared"] == "NOASSERTION"
    ]
    if noassert:
        print(f"WARNING: {len(noassert)} submodules with NOASSERTION license:")
        for n in noassert:
            print(f"  - {n}")
    else:
        print("All submodule licenses mapped")

    print(f"\nFile size: {Path(sys.argv[1]).stat().st_size // 1024}KB")


def search(doc, term):
    term = term.lower()
    found = [p for p in doc["packages"] if term in p["name"].lower()]
    if not found:
        print(f"No packages matching '{term}'")
        return
    print(f"Found {len(found)} packages matching '{term}':\n")
    for p in found:
        print(json.dumps(p, indent=2))
        print()


def ntia_check(doc):
    required = ["SPDXID", "name", "versionInfo", "supplier", "downloadLocation"]
    missing = []
    for p in doc["packages"]:
        for f in required:
            if f not in p or p[f] in ("", None):
                missing.append((p["name"], f))

    if missing:
        print(f"FAIL: {len(missing)} missing fields:")
        for name, field in missing:
            print(f"  {name}: missing {field}")
    else:
        print(f"PASS: All {len(doc['packages'])} packages have required fields")

    print(f"\nCreators:  {doc['creationInfo']['creators']}")
    print(f"Timestamp: {doc['creationInfo']['created']}")

    rels = [r for r in doc["relationships"] if r["relationshipType"] == "DESCRIBES"]
    print(f"DESCRIBES relationships: {len(rels)}")

    return len(missing) == 0


def licenses(doc):
    by_license = {}
    for p in doc["packages"]:
        lic = p.get("licenseDeclared", "NOASSERTION")
        by_license.setdefault(lic, []).append(p["name"])

    for lic in sorted(by_license.keys()):
        names = by_license[lic]
        print(f"\n{lic} ({len(names)}):")
        for n in sorted(names):
            print(f"  {n}")


def list_packages(doc, filter_type):
    filter_type = filter_type.lower()
    for p in sorted(doc["packages"], key=lambda x: x["name"]):
        t = pkg_type(p)
        if filter_type != "all" and t.lower() != filter_type:
            continue
        lic = p.get("licenseDeclared", "?")
        ver = p["versionInfo"][:20] if len(p["versionInfo"]) > 20 else p["versionInfo"]
        print(f"  {t:10s}  {p['name']:50s}  {ver:20s}  {lic}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    doc = load(sys.argv[1])
    cmd = sys.argv[2] if len(sys.argv) > 2 else "summary"

    if cmd == "summary":
        summary(doc)
    elif cmd == "search":
        if len(sys.argv) < 4:
            print("Usage: inspect_sbom.py <file> search <term>")
            sys.exit(1)
        search(doc, sys.argv[3])
    elif cmd == "ntia":
        if not ntia_check(doc):
            sys.exit(1)
    elif cmd == "licenses":
        licenses(doc)
    elif cmd == "list":
        filter_type = sys.argv[3] if len(sys.argv) > 3 else "all"
        list_packages(doc, filter_type)
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
