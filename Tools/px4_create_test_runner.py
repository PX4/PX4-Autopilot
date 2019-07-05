#!/usr/bin/env python3


import argparse


class Config:
    def __init__(self, out_file, tests):
        self.out_file = out_file
        self.tests = tests


def main():
    (out_file, tests) = parse_cli_args()
    content = generate_content(tests)
    write_script(content, out_file)


def parse_cli_args():
    parser = argparse.ArgumentParser(
        description="Generates a script to run all tests in a shell.")
    parser.add_argument(
        "--out-file",
        required=True,
        help="Path to script to create")
    parser.add_argument(
        "--tests",
        required=True,
        nargs='+',
        help="Space separated list of unit test commands to run")
    args = parser.parse_args()

    return args.out_file, args.tests


def generate_content(tests):
    content = '#!/bin/sh\n\n'
    content += 'set tests_failed false\n'
    for test in tests:
        content += 'if ! {}\n'.format(test)
        content += 'then\n'.format(test)
        content += '\tset tests_failed true\n'.format(test)
        content += 'fi\n\n'

    content += 'if [ $tests_failed = false ]\n'
    content += 'then\n'
    content += '\treturn 0\n'
    content += 'else\n'
    content += '\treturn 1\n'
    content += 'fi\n'

    return content


def write_script(content, out_file):
    print("Write file to {}".format(out_file))
    with open(out_file, 'w') as f:
        f.write(content)


if __name__ == '__main__':
    main()
