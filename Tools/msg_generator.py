#!/usr/bin/env python
import re
import glob
import argparse
import os
from jinja2 import Environment, FileSystemLoader


def msg_parser(msg_dir):
    """
    Parses msgs and renderes templates

    : param msg_dir: directory with .msg files
    : param dest: directory for generated files
    """
    msg_files = glob.glob(os.path.join(os.path.abspath(args.msg_dir), '*.msg'))
    field_pattern = re.compile(
        '\s*'
        '(?P<type>(float32|float64|uint32|uint64|uint8|bool))'
        '(\[(?P<num>\d+)\])?'
        '\s+'
        '(?P<name>\w+)'
        '\s+'
        '#\s*(?P<comment>.+)'
        )

    topic_pattern = re.compile(
        '\s*'
        '# TOPICS'
        '\s+'
        '(?P<topic>(\w+\s+)*)'
        )

    data = {}

    for msg_file in msg_files:
        msg_name = os.path.basename(msg_file).split('.')[0]
        data[msg_name] = {
            'fields': {},
            'topics': [],
        }
        with open(msg_file, 'r') as f:
            for l in f.readlines():

                # try to read line as field
                m = re.match(field_pattern, l)
                if m is not None:
                    if m.group('num') is None:
                        num = 1
                    else:
                        num = int(m.group('num'))
                    data[msg_name]['fields'][m.group('name')] = {
                        'comment':  m.group('comment'),
                        'type':  m.group('type'),
                        'len':  num,
                    }
                    continue

                # try to read line as topic
                m = re.match(topic_pattern, l)
                if m is not None:
                    data[msg_name]['topics'].extend(m.group('topic').strip().split(' '))
                    continue
    return data


def msg_generator(data, dest):
    # for jinja docs see: http://jinja.pocoo.org/docs/2.9/api/
    script_path = os.path.dirname(os.path.realpath(__file__))
    env = Environment(
        loader=FileSystemLoader(os.path.join(script_path, 'templates')))

    if not os.path.isdir(dest):
        os.path.mkdir(dest)

    template_files = [
        'topic_listener.cpp.jinja',
    ]
    for template_file in template_files:
        template = env.get_template(template_file)
        with open(os.path.join(
                dest, template_file.replace('.jinja','')), 'w') as fid:
            fid.write(template.render(data=data))

if __name__ == "__main__":
    # parse args
    parser = argparse.ArgumentParser('msg parser')
    parser.add_argument('msg_dir')
    parser.add_argument('dest_dir')
    args = parser.parse_args()

    # call parser
    data = msg_parser(args.msg_dir)

    # for debugging, can print json
    # import json
    # print(json.dumps(data, indent=2))

    # call generator
    msg_generator(data, args.dest_dir)
