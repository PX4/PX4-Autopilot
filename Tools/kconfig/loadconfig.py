"""Shared helpers for loading PX4 board configs and the Kconfig tree.

The single supported way to parse the PX4 Kconfig tree outside of CMake.
src/modules/zenoh/Kconfig sources a generated topic catalog through the
ZENOH_KCONFIG_TOPICS environment variable; CMake generates the catalog and
exports the variable (cmake/kconfig.cmake), so tools that parse Kconfig
without a CMake build must go through ensure_env()/load_kconfig() to get
the same environment.
"""

import atexit
import glob
import os
import shutil
import subprocess
import sys
import tempfile
from collections import namedtuple

import kconfiglib

PX4_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
BOARDS_DIR = os.path.join(PX4_ROOT, 'boards')

_ZENOH_GENERATOR = os.path.join(PX4_ROOT, 'Tools', 'zenoh', 'px_generate_zenoh_topic_files.py')
_ZENOH_TEMPLATES = os.path.join(PX4_ROOT, 'Tools', 'zenoh', 'templates', 'zenoh')

# Labels that load standalone; any other label overlays default.px4board
STANDALONE_LABELS = ('default', 'performance-test', 'bootloader', 'canbootloader')

Board = namedtuple('Board', ['manufacturer', 'board', 'path'])
BoardTarget = namedtuple('BoardTarget', ['manufacturer', 'board', 'label', 'path', 'name'])


def ensure_env():
    """Make the Kconfig tree parseable in this process.

    Honors an existing ZENOH_KCONFIG_TOPICS (the CMake case); otherwise
    generates the topic catalog into a temporary directory, exactly as
    cmake/kconfig.cmake does, and exports the variable so child processes
    and later Kconfig constructions inherit it.
    """
    if os.environ.get('ZENOH_KCONFIG_TOPICS'):
        return
    out_dir = tempfile.mkdtemp(prefix='px4_zenoh_kconfig_')
    atexit.register(shutil.rmtree, out_dir, ignore_errors=True)
    msg_files = sorted(glob.glob(os.path.join(PX4_ROOT, 'msg', '*.msg')) +
                       glob.glob(os.path.join(PX4_ROOT, 'msg', 'versioned', '*.msg')))
    # stdout must stay clean: several callers print machine-parsed output
    subprocess.run([sys.executable, _ZENOH_GENERATOR, '--zenoh-config',
                    '-f'] + msg_files + ['-o', out_dir, '-e', _ZENOH_TEMPLATES],
                   check=True, stdout=subprocess.DEVNULL)
    os.environ['ZENOH_KCONFIG_TOPICS'] = os.path.join(out_dir, 'Kconfig.topics')


def load_kconfig(suppress_warnings=False):
    """Load the PX4 Kconfig tree.

    Must run with the PX4 source root as working directory: relative
    `source` statements resolve against it.
    """
    ensure_env()
    if not os.path.isfile('Kconfig'):
        sys.exit('loadconfig: run from the PX4 source root (no Kconfig in {})'.format(os.getcwd()))
    kconf = kconfiglib.Kconfig()
    if suppress_warnings:
        kconf.warn_assign_undef = False
        kconf.warn_assign_override = False
        kconf.warn_assign_redun = False
    return kconf


def enumerate_boards():
    """Yield a Board for every boards/<manufacturer>/<board> directory, sorted."""
    for manufacturer in sorted(os.scandir(BOARDS_DIR), key=lambda e: e.name):
        if not manufacturer.is_dir():
            continue
        for board in sorted(os.scandir(manufacturer.path), key=lambda e: e.name):
            if board.is_dir():
                yield Board(manufacturer.name, board.name, board.path)


def enumerate_targets():
    """Yield a BoardTarget for every *.px4board file, sorted."""
    for board in enumerate_boards():
        for entry in sorted(os.scandir(board.path), key=lambda e: e.name):
            if entry.is_file() and entry.name.endswith('.px4board'):
                label = entry.name[:-len('.px4board')]
                yield BoardTarget(board.manufacturer, board.board, label, entry.path,
                                  '_'.join((board.manufacturer, board.board, label)))


def load_target_config(kconf, px4board_path):
    """Load a target's config into kconf, replacing any previous state."""
    label = os.path.basename(px4board_path)[:-len('.px4board')]
    if label in STANDALONE_LABELS:
        kconf.load_config(px4board_path, replace=True)
    else:
        default_config = os.path.join(os.path.dirname(px4board_path), 'default.px4board')
        kconf.load_config(default_config, replace=True)
        kconf.load_config(px4board_path, replace=False)


def chip_family(board_path):
    """Chip family of a board, from its NuttX defconfig.

    Returns one of: stm32h7, stm32f7, stm32f4, stm32f1, imxrt, kinetis,
    s32k, rp2040, native.
    """
    nsh_defconfig = os.path.join(board_path, 'nuttx-config', 'nsh', 'defconfig')
    if not os.path.exists(nsh_defconfig):
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

    if arch_chip in ('stm32h7', 'stm32f7', 'imxrt', 'kinetis', 'rp2040'):
        return arch_chip
    elif arch_chip.startswith('s32k'):
        return 's32k'
    elif arch_chip == 'stm32':
        if specific_chip and specific_chip.startswith('STM32F1'):
            return 'stm32f1'
        return 'stm32f4'
    return 'native'
