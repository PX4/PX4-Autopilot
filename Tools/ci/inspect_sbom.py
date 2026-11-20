#!/usr/bin/env python3
"""Inspect a PX4 SPDX 3 SBOM file.

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

RELATIONSHIP_TYPES = ("Relationship", "LifecycleScopedRelationship")
NOASSERTION = "NOASSERTION"
# SPDX individuals that license relationships may point to instead of an element
LICENSE_INDIVIDUALS = {
    "expandedlicensing_NoAssertionLicense": NOASSERTION,
    "expandedlicensing_NoneLicense": "NONE",
}


class Sbom:
    """Index over the @graph of a PX4 SPDX 3 JSON-LD document."""

    def __init__(self, doc):
        graph = doc["@graph"]
        self.by_id = {e["spdxId"]: e for e in graph if "spdxId" in e}
        self.creation_info = next(e for e in graph if e["type"] == "CreationInfo")
        self.document = next(e for e in graph if e["type"] == "SpdxDocument")
        self.packages = [e for e in graph if e["type"] == "software_Package"]
        self.relationships = [e for e in graph if e["type"] in RELATIONSHIP_TYPES]
        self.root = self.by_id[self.document["rootElement"][0]]

        self.declared = {}
        self.contained = set()
        for r in self.relationships:
            if r["relationshipType"] == "hasDeclaredLicense":
                to = r["to"][0]
                self.declared[r["from"]] = (
                    LICENSE_INDIVIDUALS[to] if to in LICENSE_INDIVIDUALS
                    else self.by_id[to]["simplelicensing_licenseExpression"])
            elif r["relationshipType"] == "contains" and r["from"] == self.root["spdxId"]:
                self.contained.update(r["to"])

    def license(self, pkg):
        return self.declared.get(pkg["spdxId"], NOASSERTION)

    def supplier(self, pkg):
        return self.by_id[pkg["suppliedBy"]]["name"] if "suppliedBy" in pkg else None

    def originator(self, pkg):
        return self.names(pkg["originatedBy"]) if "originatedBy" in pkg else None

    def names(self, ids):
        return ", ".join(self.by_id[i]["name"] for i in ids)


def pkg_type(pkg):
    local_id = pkg["spdxId"].split("#", 1)[-1]
    for prefix in ("Submodule", "PyDep", "Module", "Compiler", "PX4"):
        if local_id.startswith(f"{prefix}-"):
            return prefix
    return "Other"


def summary(sbom):
    print(f"specVersion:  {sbom.creation_info['specVersion']}")
    print(f"name:         {sbom.document['name']}")
    print(f"spdxId:       {sbom.document['spdxId']}")
    print(f"created:      {sbom.creation_info['created']}")
    print(f"createdBy:    {sbom.names(sbom.creation_info['createdBy'])}")
    print(f"createdUsing: {sbom.names(sbom.creation_info.get('createdUsing', []))}")
    print()

    types = Counter(pkg_type(p) for p in sbom.packages)
    print(f"Packages: {len(sbom.packages)}")
    for t, c in types.most_common():
        print(f"  {t}: {c}")
    print()

    rc = Counter()
    for r in sbom.relationships:
        rc[r["relationshipType"]] += len(r["to"])
    print(f"Relationship edges: {sum(rc.values())} ({len(sbom.relationships)} relationship elements)")
    for t, n in rc.most_common():
        print(f"  {t}: {n}")
    print()

    primary = sbom.root
    print("Primary package:")
    print(f"  name:    {primary['name']}")
    print(f"  version: {primary.get('software_packageVersion', 'N/A')}")
    print(f"  purpose: {primary.get('software_primaryPurpose', 'N/A')}")
    print(f"  license: {sbom.license(primary)}")
    print()

    for f in (e for e in sbom.by_id.values() if e["type"] == "software_File"):
        hashes = ", ".join(f"{h['algorithm']}:{h['hashValue']}" for h in f.get("verifiedUsing", []))
        print(f"Firmware file: {f['name']} ({hashes or 'no hash'})")
        print()

    noassert = [
        p["name"]
        for p in sbom.packages
        if p["spdxId"] in sbom.contained and sbom.license(p) == NOASSERTION
    ]
    if noassert:
        print(f"WARNING: {len(noassert)} firmware components with no license assertion:")
        for n in noassert:
            print(f"  - {n}")
    else:
        print("All firmware component licenses mapped")

    print(f"\nFile size: {Path(sys.argv[1]).stat().st_size // 1024}KB")


def search(sbom, term):
    term = term.lower()
    found = [p for p in sbom.packages if term in p["name"].lower()]
    if not found:
        print(f"No packages matching '{term}'")
        return
    print(f"Found {len(found)} packages matching '{term}':\n")
    for p in found:
        print(json.dumps(p, indent=2))
        print(f"license:  {sbom.license(p)}")
        print(f"supplier: {sbom.supplier(p)}")
        if sbom.originator(p):
            print(f"upstream: {sbom.originator(p)}")
        print()


def ntia_check(sbom):
    """Check the NTIA minimum elements for every component in the firmware.

    Components are the root package and everything it contains. Build-time
    dependencies and tools are listed for information only: they are not
    part of the shipped binary.
    """
    components = [sbom.root] + [sbom.by_id[i] for i in sorted(sbom.contained)]
    required = {
        "name": lambda p: p.get("name"),
        "version": lambda p: p.get("software_packageVersion"),
        "supplier": sbom.supplier,
    }

    missing = [
        (p["name"], field)
        for p in components
        for field, get in required.items()
        if not get(p)
    ]

    if missing:
        print(f"FAIL: {len(missing)} missing fields:")
        for name, field in missing:
            print(f"  {name}: missing {field}")
    else:
        print(f"PASS: All {len(components)} firmware components have name, version and supplier")

    build_only = [p for p in sbom.packages if p is not sbom.root and p["spdxId"] not in sbom.contained]
    incomplete = [p["name"] for p in build_only if not all(get(p) for get in required.values())]
    if incomplete:
        print(f"INFO: {len(incomplete)} of {len(build_only)} build-time packages lack a version or supplier")

    print(f"\nAuthor:    {sbom.names(sbom.creation_info['createdBy'])}")
    print(f"Timestamp: {sbom.creation_info['created']}")
    print(f"Root element: {sbom.root['name']}, contains {len(sbom.contained)} components")

    return not missing


def licenses(sbom):
    by_license = {}
    for p in sbom.packages:
        by_license.setdefault(sbom.license(p), []).append(p["name"])

    for lic in sorted(by_license.keys()):
        names = by_license[lic]
        print(f"\n{lic} ({len(names)}):")
        for n in sorted(names):
            print(f"  {n}")


def list_packages(sbom, filter_type):
    filter_type = filter_type.lower()
    for p in sorted(sbom.packages, key=lambda x: x["name"]):
        t = pkg_type(p)
        if filter_type != "all" and t.lower() != filter_type:
            continue
        ver = p.get("software_packageVersion", NOASSERTION)[:20]
        print(f"  {t:10s}  {p['name']:50s}  {ver:20s}  {sbom.license(p)}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    doc = json.loads(Path(sys.argv[1]).read_text())
    if "@graph" not in doc:
        print(f"Not an SPDX 3 JSON-LD document (spdxVersion: {doc.get('spdxVersion', 'unknown')}).")
        print("SPDX 2.3 SBOMs from older PX4 releases need inspect_sbom.py from that release.")
        sys.exit(1)
    sbom = Sbom(doc)
    cmd = sys.argv[2] if len(sys.argv) > 2 else "summary"

    if cmd == "summary":
        summary(sbom)
    elif cmd == "search":
        if len(sys.argv) < 4:
            print("Usage: inspect_sbom.py <file> search <term>")
            sys.exit(1)
        search(sbom, sys.argv[3])
    elif cmd == "ntia":
        if not ntia_check(sbom):
            sys.exit(1)
    elif cmd == "licenses":
        licenses(sbom)
    elif cmd == "list":
        filter_type = sys.argv[3] if len(sys.argv) > 3 else "all"
        list_packages(sbom, filter_type)
    else:
        print(f"Unknown command: {cmd}")
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
