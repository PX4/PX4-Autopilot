#!/usr/bin/env python3
"""Collect what build_all publishes from build/ into artifacts/.

Run from the root of the checkout once the targets of a build job are built.
Everything in artifacts/ becomes the job's build artifact, is synced to S3 on
main, stable, beta and tags, and is filtered into the GitHub Release on tags
(see .github/workflows/build_all_targets.yml).

Published per target, where the build produced them:

  <target>.px4          firmware for QGroundControl and px_uploader
  <target>.uavcan.bin   CAN node application, published instead of the .px4
  <target>.bin          raw image of a bootloader (for SWD) or of the px4io
                        firmware (flashed to the IO coprocessor by the FMU)
  <target>.elf, *.deb
  <target>/             component metadata and the SBOM

_general/ holds the metadata of px4_sitl_default, for consumers that are not
tied to a board (Flight Review, pyulog, the firmware's fallback URIs).
"""

import shutil
import sys
from fnmatch import fnmatch
from pathlib import Path

BUILD = Path('build')
ARTIFACTS = Path('artifacts')

METADATA = ('parameters.xml', 'parameters.json', 'parameters.json.xz',
            'actuators.json', 'actuators.json.xz')
EVENTS = 'all_events.json.xz'
GENERAL_SOURCE = 'px4_sitl_default'


def has_raw_image(target):
    """Targets flashed from <target>.bin rather than from a .px4."""
    return target.endswith(('_bootloader', '_canbootloader')) or fnmatch(target, '*_io-v*_default')


def has_metadata_directory(target):
    """Bootloaders have no metadata, and their SBOM is not a recovery artifact.

    *_bootloader_* variants (bootloader_secureboot) are baked with the test
    keys kept in the tree.
    """
    return not (target.endswith(('_bootloader', '_canbootloader')) or '_bootloader_' in target)


def publish(source, destination):
    """Copy source to the destination file or directory, if the build produced it."""
    if not source.is_file():
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(source, destination)
    return True


def package_target(build_dir, artifacts):
    target = build_dir.name

    for uavcan_bin in sorted(build_dir.glob('*.uavcan.bin')):
        publish(uavcan_bin, artifacts / f'{target}.uavcan.bin')

    if has_raw_image(target):
        publish(build_dir / f'{target}.bin', artifacts / f'{target}.bin')

    # A CAN node is flashed from the .uavcan.bin. A bootloader .px4 is unused:
    # SWD takes the raw .bin, and USB flashing would write it into the
    # application slot.
    if not (artifacts / f'{target}.uavcan.bin').is_file() and 'bootloader' not in target:
        for px4 in sorted(build_dir.glob('*.px4')):
            publish(px4, artifacts / px4.name)

    for pattern in ('*.elf', '*.deb'):
        for image in sorted(build_dir.glob(pattern)):
            publish(image, artifacts / image.name)

    if not has_metadata_directory(target):
        return

    directory = artifacts / target
    directory.mkdir(exist_ok=True)
    # NuttX writes airframes.xml to the build root, SITL to docs/
    if not publish(build_dir / 'airframes.xml', directory / 'airframes.xml'):
        publish(build_dir / 'docs' / 'airframes.xml', directory / 'airframes.xml')
    for name in METADATA:
        publish(build_dir / name, directory / name)
    # The firmware advertises the events URI without the events/ subdirectory
    # (src/lib/component_information/CMakeLists.txt comp_metadata_events_uri_board).
    publish(build_dir / 'events' / EVENTS, directory / 'events' / EVENTS)
    publish(build_dir / 'events' / EVENTS, directory / EVENTS)
    for sbom in sorted(build_dir.glob('*.sbom.spdx.json')):
        publish(sbom, directory / sbom.name)


def package_general(artifacts):
    """Copy the px4_sitl_default metadata to _general/, when this job built it."""
    source = artifacts / GENERAL_SOURCE
    if not source.is_dir():
        return
    general = artifacts / '_general'
    for name in ('airframes.xml', *METADATA):
        published = publish(source / name, general / name)
        if name == 'airframes.xml' and not published:
            sys.exit(f"Error: expected '{source / name}' not found.")
    if not publish(source / 'events' / EVENTS, general / EVENTS):
        sys.exit(f"Error: expected '{source / 'events' / EVENTS}' not found.")


def package(build=BUILD, artifacts=ARTIFACTS):
    artifacts.mkdir(exist_ok=True)
    for build_dir in sorted(build.iterdir()):
        if build_dir.is_dir() and not build_dir.name.startswith('.'):
            package_target(build_dir, artifacts)
    package_general(artifacts)
    for path in sorted(artifacts.rglob('*')):
        if path.is_file():
            print(path.relative_to(artifacts))


if __name__ == '__main__':
    package()
