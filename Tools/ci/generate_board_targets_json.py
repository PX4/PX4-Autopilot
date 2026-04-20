#!/usr/bin/env python3
""" Script to generate a JSON config with all build targets (for CI) """

import argparse
import os
import sys
import json
import re
from kconfiglib import Kconfig

kconf = Kconfig()

# Supress warning output
kconf.warn_assign_undef = False
kconf.warn_assign_override = False
kconf.warn_assign_redun = False

source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
boards_dir = os.path.join(source_dir, '..', 'boards')

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')
parser.add_argument('-g', '--groups', dest='group', action='store_true',
                    help='Groups targets')
parser.add_argument('-f', '--filter', dest='filter', help='comma separated list of build target name prefixes to include instead of all e.g. "px4_fmu-v5_"')
parser.add_argument('-s', '--seeders', dest='seeders', action='store_true',
                    help='Output seeder matrix JSON (one entry per chip family)')

args = parser.parse_args()
verbose = args.verbose

target_filter = []
if args.filter:
    for target in args.filter.split(','):
        target_filter.append(target)

# Load CI configuration from YAML
import yaml
ci_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build_all_config.yml')
with open(ci_config_path) as f:
    ci_config = yaml.safe_load(f)

default_container = ci_config['containers']['default']
voxl2_container = ci_config['containers']['voxl2']
build_configs = []
grouped_targets = {}
excluded_boards = ['px4_ros2', 'espressif_esp32']  # TODO: fix and enable
excluded_manufacturers = ['atlflight']
excluded_platforms = []

# Container overrides for platforms/boards that need a non-default container
platform_container_overrides = {
    'qurt': voxl2_container,
}
board_container_overrides = {
    'modalai_voxl2': voxl2_container,
}
excluded_labels = [
    'stackcheck',
    'nolockstep', 'replay', 'test',
    'uavcanv1', # TODO: fix and enable
    ]

# Labels that mark isolated/special builds (poor cache reuse with normal builds)
special_labels = ci_config.get('special_labels', ['lto', 'protected'])

def detect_chip_family(manufacturer_name, board_name, label):
    """Detect the chip family for a board by reading its NuttX defconfig.

    Returns a chip family string used for cache grouping:
      stm32h7, stm32f7, stm32f4, stm32f1, imxrt, kinetis, s32k, rp2040, native, special
    """
    # Special labels get their own group regardless of chip
    if label in special_labels:
        return 'special'

    board_path = os.path.join(boards_dir, manufacturer_name, board_name)
    nsh_defconfig = os.path.join(board_path, 'nuttx-config', 'nsh', 'defconfig')

    if not os.path.exists(nsh_defconfig):
        # Try bootloader defconfig as fallback
        bl_defconfig = os.path.join(board_path, 'nuttx-config', 'bootloader', 'defconfig')
        if os.path.exists(bl_defconfig):
            nsh_defconfig = bl_defconfig
        else:
            return 'native'

    arch_chip = None
    specific_chip = None

    with open(nsh_defconfig) as f:
        for line in f:
            line = line.strip()
            if line.startswith('CONFIG_ARCH_CHIP='):
                arch_chip = line.split('=')[1].strip('"')
            elif line.startswith('CONFIG_ARCH_CHIP_STM32F') and line.endswith('=y'):
                specific_chip = line.split('=')[0].replace('CONFIG_ARCH_CHIP_', '')

    if arch_chip is None:
        return 'native'

    # Direct matches for chips that have unique CONFIG_ARCH_CHIP values
    if arch_chip == 'stm32h7':
        return 'stm32h7'
    elif arch_chip == 'stm32f7':
        return 'stm32f7'
    elif arch_chip == 'imxrt':
        return 'imxrt'
    elif arch_chip == 'kinetis':
        return 'kinetis'
    elif arch_chip.startswith('s32k'):
        return 's32k'
    elif arch_chip == 'rp2040':
        return 'rp2040'
    elif arch_chip == 'stm32':
        # Disambiguate STM32 sub-families using specific chip define
        if specific_chip:
            if specific_chip.startswith('STM32F1'):
                return 'stm32f1'
            elif specific_chip.startswith('STM32F4'):
                return 'stm32f4'
            else:
                return 'stm32f4'  # Default STM32 to F4
        return 'stm32f4'
    else:
        return 'native'

target_chip_families = {}  # target_name -> chip_family mapping
github_action_config = { 'include': build_configs }
extra_args = {}
if args.pretty:
    extra_args['indent'] = 2

def chunks(arr, size):
    # splits array into parts
    for i in range(0, len(arr), size):
        yield arr[i:i + size]

MERGE_BACK_THRESHOLD = 5

def chunks_merged(arr, size):
    """Split array into chunks, merging the last chunk back if it's too small."""
    result = list(chunks(arr, size))
    if len(result) > 1 and len(result[-1]) < MERGE_BACK_THRESHOLD:
        result[-2] = result[-2] + result[-1]
        result.pop()
    return result

def comma_targets(targets):
    # turns array of targets into a comma split string
    return ",".join(targets)

def process_target(px4board_file, target_name, manufacturer_name=None, board_dir_name=None, label=None):
    # reads through the board file and grabs
    # useful information for building
    ret = None
    platform = None
    toolchain = None
    group = None

    if px4board_file.endswith("default.px4board") or \
        px4board_file.endswith("performance-test.px4board") or \
        px4board_file.endswith("bootloader.px4board"):
        kconf.load_config(px4board_file, replace=True)
    else: # Merge config with default.px4board
        default_kconfig = re.sub(r'[a-zA-Z\d_-]+\.px4board', 'default.px4board', px4board_file)
        kconf.load_config(default_kconfig, replace=True)
        kconf.load_config(px4board_file, replace=False)

    if "BOARD_TOOLCHAIN" in kconf.syms:
        toolchain = kconf.syms["BOARD_TOOLCHAIN"].str_value

    if "BOARD_PLATFORM" in kconf.syms:
        platform = kconf.syms["BOARD_PLATFORM"].str_value

    assert platform, f"PLATFORM not found in {px4board_file}"

    if platform not in excluded_platforms:
        container = default_container

        # Extract board name (manufacturer_board) from target name
        board_name = '_'.join(target_name.split('_')[:2])

        # Apply container overrides for specific platforms or boards
        if platform in platform_container_overrides:
            container = platform_container_overrides[platform]
        if board_name in board_container_overrides:
            container = board_container_overrides[board_name]

        # Detect chip family for cache grouping
        chip_family = 'native'
        if manufacturer_name and board_dir_name:
            if platform == 'nuttx':
                chip_family = detect_chip_family(manufacturer_name, board_dir_name, label or '')
            elif board_name in board_container_overrides or platform in platform_container_overrides:
                chip_family = 'native'  # voxl2/qurt targets
            else:
                chip_family = 'native'

        # Boards with container overrides get their own group
        if board_name in board_container_overrides or platform in platform_container_overrides:
            group = 'voxl2'
        elif platform == 'posix':
            group = 'base'
            if toolchain:
                if toolchain.startswith('aarch64'):
                    group = 'aarch64'
                elif toolchain == 'arm-linux-gnueabihf':
                    group = 'armhf'
                else:
                    if verbose: print(f'unmatched toolchain: {toolchain}')
        elif platform == 'nuttx':
            group = 'nuttx'
        else:
            if verbose: print(f'unmatched platform: {platform}')

        ret = {'target': target_name, 'container': container, 'chip_family': chip_family}
        if(args.group):
            ret['arch'] = group

    return ret

# Look for board targets in the ./boards directory
if(verbose):
    print("=======================")
    print("= scanning for boards =")
    print("=======================")

# We also need to build metadata
# includes:
# - Airframe
# - Parameters
# - Events
metadata_targets = ['airframe_metadata', 'parameters_metadata', 'extract_events']
grouped_targets['base'] = {}
grouped_targets['base']['container'] = default_container
grouped_targets['base']['manufacturers'] = {}
grouped_targets['base']['manufacturers']['px4'] = []
grouped_targets['base']['manufacturers']['px4'] += metadata_targets
for mt in metadata_targets:
    target_chip_families[mt] = 'native'

for manufacturer in sorted(os.scandir(os.path.join(source_dir, '../boards')), key=lambda e: e.name):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        if verbose: print(f'excluding manufacturer {manufacturer.name}')
        continue

    for board in sorted(os.scandir(manufacturer.path), key=lambda e: e.name):
        if not board.is_dir():
            continue

        for files in sorted(os.scandir(board.path), key=lambda e: e.name):
            if files.is_file() and files.name.endswith('.px4board'):

                board_name = manufacturer.name + '_' + board.name
                label = files.name[:-9]
                target_name = manufacturer.name + '_' + board.name + '_' + label

                if target_filter and not any(target_name.startswith(f) for f in target_filter):
                    if verbose: print(f'excluding board {board_name} ({target_name})')
                    continue

                if board_name in excluded_boards:
                    if verbose: print(f'excluding board {board_name} ({target_name})')
                    continue

                if label in excluded_labels:
                    if verbose: print(f'excluding label {label} ({target_name})')
                    continue
                target = process_target(files.path, target_name,
                                       manufacturer_name=manufacturer.name,
                                       board_dir_name=board.name,
                                       label=label)
                if (args.group and target is not None):
                    if (target['arch'] not in grouped_targets):
                        grouped_targets[target['arch']] = {}
                        grouped_targets[target['arch']]['container'] = target['container']
                        grouped_targets[target['arch']]['manufacturers'] = {}
                    if(manufacturer.name not in grouped_targets[target['arch']]['manufacturers']):
                        grouped_targets[target['arch']]['manufacturers'][manufacturer.name] = []
                    grouped_targets[target['arch']]['manufacturers'][manufacturer.name].append(target_name)
                    target_chip_families[target_name] = target['chip_family']
                if target is not None:
                    build_configs.append(target)

# Remove companion targets from CI groups (parent target builds them via Make prerequisite)
for manufacturer in sorted(os.scandir(os.path.join(source_dir, '../boards')), key=lambda e: e.name):
    if not manufacturer.is_dir():
        continue
    for board in sorted(os.scandir(manufacturer.path), key=lambda e: e.name):
        if not board.is_dir():
            continue
        companion_file = os.path.join(board.path, 'companion_targets')
        if os.path.exists(companion_file):
            with open(companion_file) as f:
                companions = {l.strip() for l in f if l.strip() and not l.startswith('#')}
            for arch in grouped_targets:
                for man in grouped_targets[arch]['manufacturers']:
                    grouped_targets[arch]['manufacturers'][man] = [
                        t for t in grouped_targets[arch]['manufacturers'][man]
                        if t not in companions
                    ]

# Append _deb targets for boards that have cmake/package.cmake
for manufacturer in sorted(os.scandir(os.path.join(source_dir, '../boards')), key=lambda e: e.name):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        continue
    for board in sorted(os.scandir(manufacturer.path), key=lambda e: e.name):
        if not board.is_dir():
            continue
        board_name = manufacturer.name + '_' + board.name
        if board_name in excluded_boards:
            continue
        package_cmake = os.path.join(board.path, 'cmake', 'package.cmake')
        if os.path.exists(package_cmake):
            deb_target = board_name + '_deb'
            if target_filter and not any(deb_target.startswith(f) for f in target_filter):
                continue
            # Determine the container and group for this board
            container = default_container
            if board_name in board_container_overrides:
                container = board_container_overrides[board_name]
            target_entry = {'target': deb_target, 'container': container}
            if args.group:
                # Find the group where this board's _default target already lives
                default_target = board_name + '_default'
                group = None
                for g in grouped_targets:
                    targets_in_group = grouped_targets[g].get('manufacturers', {}).get(manufacturer.name, [])
                    if default_target in targets_in_group:
                        group = g
                        break
                if group is None:
                    group = 'base'
                target_entry['arch'] = group
                if group not in grouped_targets:
                    grouped_targets[group] = {'container': container, 'manufacturers': {}}
                if manufacturer.name not in grouped_targets[group]['manufacturers']:
                    grouped_targets[group]['manufacturers'][manufacturer.name] = []
                grouped_targets[group]['manufacturers'][manufacturer.name].append(deb_target)
                # Inherit chip_family from the default target
                default_chip = target_chip_families.get(default_target, 'native')
                target_chip_families[deb_target] = default_chip
            build_configs.append(target_entry)

if(verbose):
    import pprint
    print("============================")
    print("= Boards found in ./boards =")
    print("============================")
    pprint.pp(grouped_targets)

if(verbose):
    print("===================")
    print("= Generating JSON =")
    print("===================")

if (args.group):
    # Group targets by chip family for better ccache reuse.
    # Targets sharing the same MCU family (e.g. stm32h7) benefit from
    # a shared ccache seed since they compile the same NuttX kernel and HAL.
    #
    # Grouping strategy:
    #   1. Collect all targets per (arch, chip_family, manufacturer)
    #   2. Within each chip_family, large manufacturers get their own groups
    #      named "{manufacturer}-{chip_family}[-N]"
    #   3. Small manufacturers are merged into "misc-{chip_family}[-N]"
    #   4. Special groups: "special" (lto/protected/allyes), "io" (stm32f1),
    #      "voxl2-0" (unchanged)
    #   5. Non-NuttX groups: "base-N", "aarch64-N", "armhf-N" (unchanged)
    final_groups = []
    # Load grouping and cache config
    grouping_config = ci_config.get('grouping', {})
    CHIP_SPLIT_LIMITS = grouping_config.get('chip_split_limits', {})
    DEFAULT_SPLIT_LIMIT = grouping_config.get('default_split_limit', 12)
    LOWER_LIMIT = grouping_config.get('lower_limit', 3)

    cache_config = ci_config.get('cache', {})
    DEFAULT_CACHE_SIZE = cache_config.get('default_size', '400M')
    CHIP_CACHE_SIZES = cache_config.get('chip_sizes', {})

    if(verbose):
        print(f'=:Architectures: [{grouped_targets.keys()}]')

    for arch in grouped_targets:
        runner = 'x64'
        # armhf and aarch64 Linux boards need the arm64 container image
        # which ships the arm-linux-gnueabihf and aarch64-linux-gnu cross compilers
        # (the x64 container image does not include them)
        if arch in ('armhf', 'aarch64'):
            runner = 'arm64'
        if(verbose):
            print(f'=:Processing: [{arch}]')

        if arch == 'nuttx':
            # Re-bucket NuttX targets by chip_family then manufacturer
            chip_man_buckets = {}  # (chip_family, manufacturer) -> [target_names]
            for man in grouped_targets[arch]['manufacturers']:
                for target in grouped_targets[arch]['manufacturers'][man]:
                    chip = target_chip_families.get(target, 'native')
                    key = (chip, man)
                    if key not in chip_man_buckets:
                        chip_man_buckets[key] = []
                    chip_man_buckets[key].append(target)

            # Collect all chip families present
            chip_families_seen = sorted(set(k[0] for k in chip_man_buckets.keys()))

            for chip in chip_families_seen:
                SPLIT_LIMIT = CHIP_SPLIT_LIMITS.get(chip, DEFAULT_SPLIT_LIMIT)
                # Special naming for certain chip families
                if chip == 'special':
                    chip_label = 'special'
                elif chip == 'stm32f1':
                    chip_label = 'io'
                elif chip == 'rp2040':
                    chip_label = 'special'  # rp2040 goes into special group
                else:
                    chip_label = chip

                # Gather all (manufacturer -> targets) for this chip family
                # NXP chip families (imxrt, kinetis, s32k) pool all manufacturers
                # under "nxp" since all boards use NXP silicon regardless of
                # which directory they live in (e.g., px4/fmu-v6xrt is imxrt).
                nxp_chips = tuple(ci_config.get('nxp_chip_families', ['imxrt', 'kinetis', 's32k']))
                man_targets = {}
                for (c, m), targets in chip_man_buckets.items():
                    if c == chip:
                        man_key = 'nxp' if chip in nxp_chips else m
                        if man_key not in man_targets:
                            man_targets[man_key] = []
                        man_targets[man_key].extend(targets)

                # Merge rp2040 targets into a flat list for the special group
                if chip in ('special', 'rp2040'):
                    all_targets = []
                    for m in sorted(man_targets.keys()):
                        all_targets.extend(man_targets[m])
                    # These get added to the special bucket below
                    # We'll handle after the chip loop
                    continue

                if(verbose):
                    print(f'=:Processing chip_family: [{chip}] ({chip_label})')

                # Split into large-manufacturer groups and misc groups
                # For NXP-exclusive chip families, always use the nxp name
                # regardless of target count (there's no other manufacturer to pool with)
                force_named = chip in nxp_chips
                temp_group = []  # small manufacturers pooled here
                for man in sorted(man_targets.keys()):
                    man_len = len(man_targets[man])
                    if (force_named or man_len > LOWER_LIMIT) and man_len <= SPLIT_LIMIT:
                        group_name = f"{man}-{chip_label}"
                        if(verbose):
                            print(f'=:  [{man}][{man_len}] -> {group_name}')
                        final_groups.append({
                            "container": grouped_targets[arch]['container'],
                            "targets": comma_targets(man_targets[man]),
                            "arch": arch,
                            "chip_family": chip,
                            "runner": runner,
                            "group": group_name,
                            "len": man_len,
                        })
                    elif man_len > SPLIT_LIMIT:
                        chunk_counter = 0
                        for chunk in chunks_merged(man_targets[man], SPLIT_LIMIT):
                            group_name = f"{man}-{chip_label}-{chunk_counter}"
                            if(verbose):
                                print(f'=:  [{man}][{man_len}] -> {group_name} ({len(chunk)})')
                            final_groups.append({
                                "container": grouped_targets[arch]['container'],
                                "targets": comma_targets(chunk),
                                "arch": arch,
                                "chip_family": chip,
                                "runner": runner,
                                "group": group_name,
                                "len": len(chunk),
                            })
                            chunk_counter += 1
                    else:
                        if(verbose):
                            print(f'=:  [{man}][{man_len}] -> misc pool')
                        temp_group.extend(man_targets[man])

                # Emit misc groups for small manufacturers
                if temp_group:
                    misc_chunks = chunks_merged(temp_group, SPLIT_LIMIT)
                    num_misc_chunks = len(misc_chunks)
                    chunk_counter = 0
                    for chunk in misc_chunks:
                        if num_misc_chunks == 1:
                            group_name = f"misc-{chip_label}"
                        else:
                            group_name = f"misc-{chip_label}-{chunk_counter}"
                        if(verbose):
                            print(f'=:  [misc][{len(chunk)}] -> {group_name}')
                        final_groups.append({
                            "container": grouped_targets[arch]['container'],
                            "targets": comma_targets(chunk),
                            "arch": arch,
                            "chip_family": chip,
                            "runner": runner,
                            "group": group_name,
                            "len": len(chunk),
                        })
                        chunk_counter += 1

            # Now handle special + rp2040 targets
            SPLIT_LIMIT = CHIP_SPLIT_LIMITS.get('special', DEFAULT_SPLIT_LIMIT)
            special_targets = []
            for (c, m), targets in chip_man_buckets.items():
                if c in ('special', 'rp2040'):
                    special_targets.extend(targets)
            if special_targets:
                chunk_counter = 0
                for chunk in chunks_merged(special_targets, SPLIT_LIMIT):
                    if len(special_targets) <= SPLIT_LIMIT:
                        group_name = 'special'
                    else:
                        group_name = f'special-{chunk_counter}'
                    if(verbose):
                        print(f'=:  [special][{len(chunk)}] -> {group_name}')
                    final_groups.append({
                        "container": grouped_targets[arch]['container'],
                        "targets": comma_targets(chunk),
                        "arch": arch,
                        "chip_family": "special",
                        "runner": runner,
                        "group": group_name,
                        "len": len(chunk),
                    })
                    chunk_counter += 1

        elif arch == 'voxl2':
            # VOXL2 stays as its own group
            all_targets = []
            for man in grouped_targets[arch]['manufacturers']:
                all_targets.extend(grouped_targets[arch]['manufacturers'][man])
            if all_targets:
                final_groups.append({
                    "container": grouped_targets[arch]['container'],
                    "targets": comma_targets(all_targets),
                    "arch": arch,
                    "chip_family": "native",
                    "runner": runner,
                    "group": "voxl2-0",
                    "len": len(all_targets),
                })

        else:
            # Non-NuttX groups (base, aarch64, armhf) - keep simple grouping
            SPLIT_LIMIT = CHIP_SPLIT_LIMITS.get('native', DEFAULT_SPLIT_LIMIT)
            all_targets = []
            for man in grouped_targets[arch]['manufacturers']:
                all_targets.extend(grouped_targets[arch]['manufacturers'][man])
            if all_targets:
                chunk_counter = 0
                for chunk in chunks_merged(all_targets, SPLIT_LIMIT):
                    if len(all_targets) <= SPLIT_LIMIT:
                        group_name = f"{arch}-0"
                    else:
                        group_name = f"{arch}-{chunk_counter}"
                    final_groups.append({
                        "container": grouped_targets[arch]['container'],
                        "targets": comma_targets(chunk),
                        "arch": arch,
                        "chip_family": "native",
                        "runner": runner,
                        "group": group_name,
                        "len": len(chunk),
                    })
                    chunk_counter += 1

    # Add cache_size to each group based on chip family
    for g in final_groups:
        g['cache_size'] = CHIP_CACHE_SIZES.get(g['chip_family'], DEFAULT_CACHE_SIZE)

    if(verbose):
        import pprint
        print("================")
        print("= final_groups =")
        print("================")
        pprint.pp(final_groups)

        print("===============")
        print("= JSON output =")
        print("===============")

    if args.seeders:
        # Generate one seeder entry per chip family present in the groups.
        # Each seeder builds a representative target to warm the ccache for
        # all groups sharing that chip family.
        seeder_targets = ci_config.get('seeders', {})
        seeder_containers = {
            'native': default_container,
        }
        # Determine which chip families actually have groups
        active_families = set()
        for g in final_groups:
            cf = g['chip_family']
            active_families.add(cf)
            # voxl2 gets its own seeder with a different container
            if g['group'].startswith('voxl2'):
                active_families.add('voxl2')

        seeders = []
        for cf in sorted(active_families):
            if cf == 'special':
                continue  # special group seeds from stm32h7
            if cf == 'voxl2':
                seeders.append({
                    'chip_family': 'voxl2',
                    'target': 'modalai_voxl2_default',
                    'container': voxl2_container,
                    'runner': 'x64',
                })
            elif cf == 'native':
                # One seeder per runner arch that has native groups (exclude voxl2
                # which has its own seeder with a different container)
                native_runners = set()
                for g in final_groups:
                    if g['chip_family'] == 'native' and not g['group'].startswith('voxl2'):
                        native_runners.add(g['runner'])
                for r in sorted(native_runners):
                    seeders.append({
                        'chip_family': 'native',
                        'target': seeder_targets['native'],
                        'container': default_container,
                        'runner': r,
                    })
            else:
                seeders.append({
                    'chip_family': cf,
                    'target': seeder_targets.get(cf, seeder_targets['stm32h7']),
                    'container': seeder_containers.get(cf, default_container),
                    'runner': 'x64',
                })

        print(json.dumps({ "include": seeders }, **extra_args))
    else:
        print(json.dumps({ "include": final_groups }, **extra_args))
else:
    print(json.dumps(github_action_config, **extra_args))
