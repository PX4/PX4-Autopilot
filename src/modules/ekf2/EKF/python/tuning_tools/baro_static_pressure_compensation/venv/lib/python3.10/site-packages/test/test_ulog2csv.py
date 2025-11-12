import os
import inspect
import unittest
from pyulog import ulog2csv, info, params, messages, extract_gps_dump
import sys
import tempfile
from ddt import ddt, data
try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO

TEST_PATH = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))

@ddt
class Test(unittest.TestCase):

    def run_against_file(self, expected_output_file, test):
        """
        run a test and compare the output against an expected file
        """
        saved_stdout = sys.stdout
        with open(expected_output_file, 'r') as file_handle:
            expected_output = file_handle.read().strip()
            output = None
            try:
                out = StringIO()
                sys.stdout = out
                test()
                output = out.getvalue().strip()
                assert output == expected_output
            finally:
                if output is not None:
                    sys.stdout = saved_stdout
                    print("Got output:")
                    print(output)
                    print("\nExpected output:")
                    print(expected_output)

    @data('sample')
    def test_ulog2csv(self, test_case):
        tmpdir = tempfile.gettempdir()
        print('writing files to ', tmpdir)
        ulog_file_name = os.path.join(TEST_PATH, test_case+'.ulg')
        messages = []
        output=tmpdir
        delimiter=','
        ulog2csv.convert_ulog2csv(ulog_file_name, messages, output, delimiter)

    @data('sample', 'sample_appended', 'sample_appended_multiple')
    def test_pyulog_info_cli(self, test_case):
        sys.argv = [
            '',
            os.path.join(TEST_PATH, test_case+'.ulg'),
            '-v'
        ]
        self.run_against_file(
                os.path.join(TEST_PATH, test_case+'_info.txt'), info.main)

    @unittest.skip("no gps data in log file")
    def test_extract_gps_dump_cli(self):
        sys.argv = [
            '',
            os.path.join(TEST_PATH, 'sample.ulg')
        ]
        extract_gps_dump.main()

    @data('sample', 'sample_appended')
    def test_messages_cli(self, test_case):
        sys.argv = [
            '',
            os.path.join(TEST_PATH, test_case+'.ulg')
        ]
        self.run_against_file(
                os.path.join(TEST_PATH, test_case+'_messages.txt'), messages.main)

    @data('sample', 'sample_appended')
    def test_params_cli(self, test_case):
        sys.argv = [
            '',
            os.path.join(TEST_PATH, test_case+'.ulg')
        ]
        params.main()


# vim: set et fenc=utf-8 ft=python ff=unix sts=4 sw=4 ts=4 : 
