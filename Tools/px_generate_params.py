#!/usr/bin/env python
"""
Param source code generation script.
"""
from __future__ import print_function
import xml.etree.ElementTree as ET
import codecs
import argparse
from jinja2 import Environment, FileSystemLoader
from px4params import scope, cmakeparser
import os

def generate():
    """
    Generate px4 param source from xml.
    """
    # pylint: disable=broad-except
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("xml", help="parameter xml file")
    arg_parser.add_argument("--scope", help="cmake file scope", default=None)
    arg_parser.add_argument("--dest", help="destination path", default=os.path.curdir)
    args = arg_parser.parse_args()

    cmake_scope = scope.Scope()
    if args.scope is not None:
        with codecs.open(args.scope, 'r', 'utf-8') as fid:
            try:
                contents = fid.read()
                fid.close()
                parser = cmakeparser.CMakeParser()
                parser.Parse(cmake_scope, contents)
            except Exception as exc:
                contents = ''
                print('Failed reading file: %s, skipping scoping. %s' %
                      args.scope, exc)

    tree = ET.parse(args.xml)
    root = tree.getroot()

    params = []
    for group in root:
        if group.tag == "group" and "no_code_generation" not in group.attrib:
            for param in group:
                scope_ = param.find('scope').text
                if not cmake_scope.Has(scope_):
                    continue
                params.append(param)

    params = sorted(params, key=lambda name: name.attrib["name"])

    script_path = os.path.dirname(os.path.realpath(__file__))

    # for jinja docs see: http://jinja.pocoo.org/docs/2.9/api/
    env = Environment(
        loader=FileSystemLoader(os.path.join(script_path, 'templates')))

    if not os.path.isdir(args.dest):
        os.path.mkdir(args.dest)

    for template_file in ['px4_parameters.h.jinja', 'px4_parameters.c.jinja']:
        template = env.get_template(template_file)
        with open(os.path.join(
                args.dest, template_file.replace('.jinja','')), 'w') as fid:
            fid.write(template.render(params=params))

if __name__ == "__main__":
    generate()
