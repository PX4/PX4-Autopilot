#!/usr/bin/env python3
"""Build a board package: the recipe behind `make <vendor>_<board>_deb`.

With boards/<vendor>/<board>/package.yaml, every component target is built
through the top-level Makefile, the packaging project in this directory is
configured into build/<vendor>_<board>_package and CPack produces the
package there. Without a descriptor the legacy path runs: the _default
target is built and CPack runs inside its build directory using the board's
cmake/package.cmake overrides. That path is deprecated.

    build_package.py <vendor>_<board> [--format deb] [--no-build] [--version V]
"""

import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import package_descriptor  # noqa: E402

FORMATS = {'deb': 'DEB'}


def run(command, cwd):
    print('+', ' '.join(str(c) for c in command), flush=True)
    subprocess.run([str(c) for c in command], cwd=cwd, check=True)


def build_legacy(board, args, source_dir):
    build_dir = source_dir / 'build' / f'{board}_default'
    print(f'warning: boards/{board.replace("_", "/", 1)} has no package.yaml; using the deprecated '
          'cmake/package.cmake path. See docs/en/hardware/board_packaging.md.', file=sys.stderr)
    if not args.no_build:
        run([args.make, f'{board}_default'], source_dir)
    run(['cpack', '-G', FORMATS[args.format]], build_dir)
    return build_dir


def build_package(descriptor, args, source_dir):
    build_root = source_dir / 'build'
    package_dir = build_root / f'{descriptor.board}_package'
    if not args.no_build:
        for component in descriptor.components:
            run([args.make, component['target']], source_dir)
    configure = ['cmake', '-S', package_descriptor.PACKAGING_DIR, '-B', package_dir,
                 f'-DPX4_PACKAGE_BOARD={descriptor.board}', f'-DPX4_BUILD_ROOT={build_root}',
                 f'-DPython3_EXECUTABLE={sys.executable}']
    if args.version:
        configure.append(f'-DPX4_PACKAGE_VERSION={args.version}')
    run(configure, source_dir)
    run(['cpack', '-G', FORMATS[args.format]], package_dir)
    return package_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('board', help='<vendor>_<board>')
    parser.add_argument('--format', default='deb', choices=sorted(FORMATS),
                        help='package format (default: deb)')
    parser.add_argument('--make', default=os.environ.get('MAKE', 'make'),
                        help='make program for the component builds (default: $MAKE or make)')
    parser.add_argument('--no-build', action='store_true', help='package what build/ already holds')
    parser.add_argument('--version', help='override the Debian version derived from git describe')
    args = parser.parse_args(argv)

    source_dir = package_descriptor.PX4_ROOT
    board_dir = package_descriptor.board_dir(source_dir, args.board)
    if not board_dir.is_dir():
        sys.exit(f'error: no board directory {board_dir}')

    if (board_dir / 'package.yaml').is_file():
        descriptor = package_descriptor.Descriptor(board_dir / 'package.yaml').validate()
        output_dir = build_package(descriptor, args, source_dir)
    else:
        output_dir = build_legacy(args.board, args, source_dir)

    packages = sorted(output_dir.glob(f'*.{args.format}'))
    if not packages:
        sys.exit(f'error: no .{args.format} produced in {output_dir}')
    for package in packages:
        print(package.relative_to(source_dir))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
    except (package_descriptor.DescriptorError, ValueError) as error:
        sys.exit(f'error: {error}')
