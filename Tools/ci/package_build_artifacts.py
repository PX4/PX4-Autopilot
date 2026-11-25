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
                        firmware (flashed to the IO coprocessor by the FMU),
                        published instead of the .px4
  <target>.elf, *.deb
  <target>/             the SBOM, and component metadata if the target has
                        MAVLink to advertise it

_general/ holds the metadata of px4_sitl_default, for consumers that are not
tied to a board (Flight Review, pyulog, the firmware's fallback URIs).

Targets listed under "unpublished" in build_all_config.yml are built for CI
only and nothing from them is published.
"""

import shutil
import sys
from fnmatch import fnmatch
from pathlib import Path

import yaml

CONFIG = Path(__file__).with_name('build_all_config.yml')
BOARDS = Path(__file__).resolve().parents[2] / 'boards'
BUILD = Path('build')
ARTIFACTS = Path('artifacts')

METADATA = ('parameters.xml', 'parameters.json', 'parameters.json.xz',
            'actuators.json', 'actuators.json.xz')
EVENTS = 'all_events.json.xz'
GENERAL_SOURCE = 'px4_sitl_default'
# build/px4io holds the px4io firmware that targets with an IO build for their
# own ROMFS (platforms/nuttx/cmake/px4io_firmware_build.cmake). The published
# IO image comes from the *_io-v*_default targets.
NOT_TARGETS = ('px4io',)


def target_labels(boards=BOARDS):
    """Label of every target in the tree, by target name (<vendor>_<board>_<label>).

    Read from the tree because a board name can contain an underscore itself.
    """
    return {f'{board.parent.parent.name}_{board.parent.name}_{board.stem}': board.stem
            for board in boards.glob('*/*/*.px4board')}


def unpublished_reason(target, labels, unpublished):
    """Why nothing from this target is published, or None if it is published."""
    return (unpublished.get('targets', {}).get(target)
            or unpublished.get('labels', {}).get(labels.get(target)))


def has_raw_image(target):
    """Targets flashed from <target>.bin rather than from a .px4."""
    return target.endswith(('_bootloader', '_canbootloader')) or fnmatch(target, '*_io-v*_default')


def has_metadata_directory(target):
    """Bootloaders have no metadata, and their SBOM is not a recovery artifact."""
    return not (target.endswith(('_bootloader', '_canbootloader')) or '_bootloader_' in target)


def has_mavlink(build_dir):
    """Component metadata is only fetched from the URIs a target advertises over MAVLink.

    A build directory without a boardconfig counts as having it, so that
    metadata is never dropped by accident.
    """
    boardconfig = build_dir / 'boardconfig'
    return not boardconfig.is_file() or 'CONFIG_MODULES_MAVLINK=y' in boardconfig.read_text().splitlines()


def publish(source, destination):
    """Copy source to the destination file, if the build produced it."""
    if not source.is_file():
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(source, destination)
    return True


def package_target(build_dir, artifacts):
    target = build_dir.name

    uavcan = [publish(image, artifacts / f'{target}.uavcan.bin') for image in sorted(build_dir.glob('*.uavcan.bin'))]
    raw = has_raw_image(target) and publish(build_dir / f'{target}.bin', artifacts / f'{target}.bin')

    # The .px4 is not published for a target that is flashed from another image.
    # A bootloader .px4 would also be written into the application slot by USB flashing.
    if not (any(uavcan) or raw or 'bootloader' in target):
        for px4 in sorted(build_dir.glob('*.px4')):
            publish(px4, artifacts / px4.name)

    for pattern in ('*.elf', '*.deb'):
        for image in sorted(build_dir.glob(pattern)):
            # toc.elf is an intermediate link product of the secure boot table of contents
            if image.name != 'toc.elf':
                publish(image, artifacts / image.name)

    if not has_metadata_directory(target):
        return

    directory = artifacts / target
    directory.mkdir(exist_ok=True)
    if has_mavlink(build_dir):
        # NuttX writes airframes.xml to the build root, SITL to docs/
        if not publish(build_dir / 'airframes.xml', directory / 'airframes.xml'):
            publish(build_dir / 'docs' / 'airframes.xml', directory / 'airframes.xml')
        for name in METADATA:
            publish(build_dir / name, directory / name)
        # The firmware advertises the events URI without the events/ subdirectory
        # (src/lib/component_information/CMakeLists.txt comp_metadata_events_uri_board).
        publish(build_dir / 'events' / EVENTS, directory / EVENTS)
    for sbom in sorted(build_dir.glob('*.sbom.spdx.json')):
        publish(sbom, directory / sbom.name)


def package_general(artifacts):
    """Copy the px4_sitl_default metadata to _general/, when this job built it."""
    source = artifacts / GENERAL_SOURCE
    if not source.is_dir():
        return
    general = artifacts / '_general'
    for name in ('airframes.xml', *METADATA, EVENTS):
        published = publish(source / name, general / name)
        if name in ('airframes.xml', EVENTS) and not published:
            sys.exit(f"Error: expected '{source / name}' not found.")


def package(build=BUILD, artifacts=ARTIFACTS, labels=None, unpublished=None):
    if labels is None:
        labels = target_labels()
    if unpublished is None:
        unpublished = yaml.safe_load(CONFIG.read_text())['unpublished']

    artifacts.mkdir(exist_ok=True)
    for build_dir in sorted(build.iterdir()):
        if not build_dir.is_dir() or build_dir.name.startswith('.') or build_dir.name in NOT_TARGETS:
            continue
        reason = unpublished_reason(build_dir.name, labels, unpublished)
        if reason:
            print(f'not published: {build_dir.name} ({reason})')
        else:
            package_target(build_dir, artifacts)
    package_general(artifacts)
    for path in sorted(artifacts.rglob('*')):
        if path.is_file():
            print(path.relative_to(artifacts))


if __name__ == '__main__':
    package()
