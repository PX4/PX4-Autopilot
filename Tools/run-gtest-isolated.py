#!/usr/bin/env python

from sys import argv, stderr
from subprocess import call, check_output


def list_all_tests(bin_path):
    out = check_output(['./' + bin_path,"--gtest_list_tests"])
    out = out.split("\n")
    if len(out) > 1:
        prefix = ''
        tests_list = []
        for token in out:
            if len(token) == 0:
                continue
            if token[-1] == '.':
                prefix = token
                continue
            if token[0] == ' ' and token[1] == ' ' and len(token) > 0:
                tests_list.append(''.join([prefix, token[2:]]))
                continue

        return tests_list

    else:
        return []



def run_tests(bin_path, tests_list, args):
    for test in tests_list:
        call_args = ['./' + bin_path, ' '.join(args), '--gtest_filter=*{}'.format(test)]
        print(' '.join(call_args))
        result = call(call_args)
        if result != 0:
            return result
    return 0



def main():
    if len(argv) >= 2:
        try:
            exit(run_tests(argv[1], list_all_tests(argv[1]), argv[2:]))
        except:
            print('Error with arguments "' + ' '.join(argv[1:]) + '"')
            raise
    print('Runs each gtest test in a difference process')
    print('Usage: ' + argv[0] + ' path/to/gtest.bin [GTEST_OPTIONS]')
    exit(1)


if __name__ == '__main__':
    main()
