#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import os
import sys

import distutils.core
try:
    import setuptools
except ImportError:
    pass

from argparse import ArgumentParser


def _get_locations(pkgs, package_dir):
    """
    based on setuptools logic and the package_dir dict, builds a dict
    of location roots for each pkg in pkgs.
    See http://docs.python.org/distutils/setupscript.html

    :returns: a dict {pkgname: root} for each pkgname in pkgs (and each of their parents)
    """
    # package_dir contains a dict {package_name: relativepath}
    # Example {'': 'src', 'foo': 'lib', 'bar': 'lib2'}
    #
    # '' means where to look for any package unless a parent package
    # is listed so package bar.pot is expected at lib2/bar/pot,
    # whereas package sup.dee is expected at src/sup/dee
    #
    # if package_dir does not state anything about a package,
    # setuptool expects the package folder to be in the root of the
    # project
    locations = {}
    allprefix = package_dir.get('', '')
    for pkg in pkgs:
        parent_location = None
        splits = pkg.split('.')
        # we iterate over compound name from parent to child
        # so once we found parent, children just append to their parent
        for key_len in range(len(splits)):
            key = '.'.join(splits[:key_len + 1])
            if key not in locations:
                if key in package_dir:
                    locations[key] = package_dir[key]
                elif parent_location is not None:
                    locations[key] = parent_location
                else:
                    locations[key] = allprefix
            parent_location = locations[key]
    return locations


def generate_cmake_file(package_name, version, scripts, package_dir, pkgs, modules):
    """
    Generates lines to add to a cmake file which will set variables

    :param version: str, format 'int.int.int'
    :param scripts: [list of str]: relative paths to scripts
    :param package_dir: {modulename: path}
    :pkgs: [list of str] python_packages declared in catkin package
    :modules: [list of str] python modules
    """
    prefix = '%s_SETUP_PY' % package_name
    result = []
    result.append(r'set(%s_VERSION "%s")' % (prefix, version))
    result.append(r'set(%s_SCRIPTS "%s")' % (prefix, ';'.join(scripts)))

    # Remove packages with '.' separators.
    #
    # setuptools allows specifying submodules in other folders than
    # their parent
    #
    # The symlink approach of catkin does not work with such submodules.
    # In the common case, this does not matter as the submodule is
    # within the containing module.  We verify this assumption, and if
    # it passes, we remove submodule packages.
    locations = _get_locations(pkgs, package_dir)
    for pkgname, location in locations.items():
        if not '.' in pkgname:
            continue
        splits = pkgname.split('.')
        # hack: ignore write-combining setup.py files for msg and srv files
        if splits[1] in ['msg', 'srv']:
            continue
        # check every child has the same root folder as its parent
        parent_name = '.'.join(splits[:1])
        if location != locations[parent_name]:
            raise RuntimeError(
                "catkin_export_python does not support setup.py files that combine across multiple directories: %s in %s, %s in %s" % (pkgname, location, parent_name, locations[parent_name]))

    # If checks pass, remove all submodules
    pkgs = [p for p in pkgs if '.' not in p]

    resolved_pkgs = []
    for pkg in pkgs:
        resolved_pkgs += [os.path.join(locations[pkg], pkg)]

    result.append(r'set(%s_PACKAGES "%s")' % (prefix, ';'.join(pkgs)))
    result.append(r'set(%s_PACKAGE_DIRS "%s")' % (prefix, ';'.join(resolved_pkgs).replace("\\", "/")))

    # skip modules which collide with package names
    filtered_modules = []
    for modname in modules:
        splits = modname.split('.')
        # check all parents too
        equals_package = [('.'.join(splits[:-i]) in locations) for i in range(len(splits))]
        if any(equals_package):
            continue
        filtered_modules.append(modname)
    module_locations = _get_locations(filtered_modules, package_dir)

    result.append(r'set(%s_MODULES "%s")' % (prefix, ';'.join(['%s.py' % m.replace('.', '/') for m in filtered_modules])))
    result.append(r'set(%s_MODULE_DIRS "%s")' % (prefix, ';'.join([module_locations[m] for m in filtered_modules]).replace("\\", "/")))

    return result


def _create_mock_setup_function(package_name, outfile):
    """
    Creates a function to call instead of distutils.core.setup or
    setuptools.setup, which just captures some args and writes them
    into a file that can be used from cmake

    :param package_name: name of the package
    :param outfile: filename that cmake will use afterwards
    :returns: a function to replace disutils.core.setup and setuptools.setup
    """

    def setup(*args, **kwargs):
        '''
        Checks kwargs and writes a scriptfile
        '''
        if 'version' not in kwargs:
            sys.stderr.write("\n*** Unable to find 'version' in setup.py of %s\n" % package_name)
            raise RuntimeError("version not found in setup.py")
        version = kwargs['version']
        package_dir = kwargs.get('package_dir', {})

        pkgs = kwargs.get('packages', [])
        scripts = kwargs.get('scripts', [])
        modules = kwargs.get('py_modules', [])

        unsupported_args = [
            'entry_points',
            'exclude_package_data',
            'ext_modules ',
            'ext_package',
            'include_package_data',
            'namespace_packages',
            'setup_requires',
            'use_2to3',
            'zip_safe']
        used_unsupported_args = [arg for arg in unsupported_args if arg in kwargs]
        if used_unsupported_args:
            sys.stderr.write("*** Arguments %s to setup() not supported in catkin devel space in setup.py of %s\n" % (used_unsupported_args, package_name))

        result = generate_cmake_file(package_name=package_name,
                                     version=version,
                                     scripts=scripts,
                                     package_dir=package_dir,
                                     pkgs=pkgs,
                                     modules=modules)
        with open(outfile, 'w') as out:
            out.write('\n'.join(result))

    return setup


def main():
    """
    Script main, parses arguments and invokes Dummy.setup indirectly.
    """
    parser = ArgumentParser(description='Utility to read setup.py values from cmake macros. Creates a file with CMake set commands setting variables.')
    parser.add_argument('package_name', help='Name of catkin package')
    parser.add_argument('setupfile_path', help='Full path to setup.py')
    parser.add_argument('outfile', help='Where to write result to')

    args = parser.parse_args()

    # print("%s" % sys.argv)
    # PACKAGE_NAME = sys.argv[1]
    # OUTFILE = sys.argv[3]
    # print("Interrogating setup.py for package %s into %s " % (PACKAGE_NAME, OUTFILE),
    #      file=sys.stderr)

    # print("executing %s" % args.setupfile_path)

    # be sure you're in the directory containing
    # setup.py so the sys.path manipulation works,
    # so the import of __version__ works
    os.chdir(os.path.dirname(os.path.abspath(args.setupfile_path)))

    # patch setup() function of distutils and setuptools for the
    # context of evaluating setup.py
    try:
        fake_setup = _create_mock_setup_function(package_name=args.package_name,
                                                outfile=args.outfile)

        distutils_backup = distutils.core.setup
        distutils.core.setup = fake_setup
        try:
            setuptools_backup = setuptools.setup
            setuptools.setup = fake_setup
        except NameError:
            pass

        with open(args.setupfile_path, 'r') as fh:
            exec(fh.read())
    finally:
        distutils.core.setup = distutils_backup
        try:
            setuptools.setup = setuptools_backup
        except NameError:
            pass

if __name__ == '__main__':
    main()
