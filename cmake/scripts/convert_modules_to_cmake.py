#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import sys
import fnmatch
import re
import shutil
import jinja2

src_path = os.path.join(os.path.curdir, 'src')

parser = argparse.ArgumentParser('converts module.mk to CMakeList.txt, run in root of repo')
parser.add_argument('path', help='directory of modules to convert')
parser.add_argument('--overwrite', help='overwrite existing files', dest='overwrite', action='store_true')
parser.add_argument('--backup', help='create backup of existing files if overwriting', dest='backup', action='store_true')
parser.set_defaults(overwrite=False, backup=False)
args = parser.parse_args()

cmake_template = jinja2.Template(open('cmake/scripts/cmake_lists.jinja', 'r').read())

module_files = []
for root, dirnames, filenames in os.walk(args.path):
    for filename in fnmatch.filter(filenames, 'module.mk'):
        module_files.append(os.path.join(root, filename))


search_data = [
    # name             # re string
    ('command', r'.*MODULE_COMMAND\s*[\+]?=\s*([^\n]+)'),
    ('stacksize', r'.*MODULE_STACKSIZE\s*[\+]?=([^\n]+)'),
    ('extracxxflags', r'.*EXTRACXXFLAGS\s*[\+]?=([^\n]+)'),
    ('extracflags', r'.*EXTRACFLAGS\s*[\+]?=\s*([^\s]+)\s*'),
    ('priority', r'.*MODULE_PRIORITY\s*[\+]?=\s*([^\s]+)\s*'),
    ('maxoptimization', r'.*MAXOPTIMIZATION\s*[\+]?=\s*([^\s]+)\s*'),
    ('srcs', '.*SRCS\s*[\+]?=([^\n\\\]*([\\\]\s*\n[^\n\\\]*)*)'),
    ('include_dirs', '.*INCLUDE_DIRS\s*[\+]?=([^\n\\\]*([\\\]\s*\n[^\n\\\]*)*)'),
    ]

progs = {}
for name, re_str in search_data:
    progs[name] = re.compile(re_str)

for module_file in module_files:

    data = {}
    with open(module_file, 'r') as f:
        module_text = f.read()
        data['text'] = module_text
        module_dir = os.path.dirname(module_file)
        data['module'] = os.path.relpath(module_dir, src_path).replace(
                os.sep, '__').split('.')[0]
        #print(module_text)
        for name, re_str in search_data:
            result = progs[name].search(module_text)
            if result is not None:
                d = result.group(1).strip()
                if name in ['srcs', 'extracxxflags', 'extracflags']:
                    d_store = d.replace('\\', '').split()
                elif name == 'include_dirs':
                    d_store = d.replace('(', '{').replace(')', '}').split()
                else:
                    d_store = d
                data[name] = d_store
            else:
                data[name] = ''

    cmake_file = os.path.join(os.path.dirname(module_file), 'CMakeLists.txt')
    cmake_file_backup = cmake_file + '.backup'

    if os.path.exists(cmake_file):
        if args.backup:
            if os.path.exists(cmake_file_backup):
                print('error: file already exists:', cmake_file_backup)
                continue
            else:
                shutil.copyfile(cmake_file, cmake_file_backup)
        if args.overwrite:
            print('overwriting', cmake_file)
        else:
            print('error: file already exists:', cmake_file)
            continue

    with open(cmake_file, 'w') as f:
        data_rendered = cmake_template.render(data=data)
        f.write(data_rendered)


# vim: set et fenc= ff=unix sts=4 sw=4 ts=4 : 
