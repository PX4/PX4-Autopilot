#!/usr/bin/env python
#############################################################################
#
#   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################

"""
px_generate_uorb_topic_headers.py
Generates c/cpp header files for uorb topics from .msg (ROS syntax)
message files
"""
from __future__ import print_function
import os
import shutil
import filecmp
import argparse

try:
        import genmsg.template_tools
except ImportError as e:
        print("python import error: ", e)
        print('''
Required python packages not installed.

On a Debian/Ubuntu system please run:

  sudo apt-get install python-empy
  sudo pip install catkin_pkg
  
On MacOS please run:
  sudo pip install empy catkin_pkg

On Windows please run:
  easy_install empy catkin_pkg
''')
        exit(1)

__author__ = "Thomas Gubler"
__copyright__ = "Copyright (C) 2013-2014 PX4 Development Team."
__license__ = "BSD"
__email__ = "thomasgubler@gmail.com"


msg_template_map = {'msg.h.template': '@NAME@.h'}
srv_template_map = {}
incl_default = ['std_msgs:./msg/std_msgs']
package = 'px4'


def convert_file(filename, outputdir, templatedir, includepath):
        """
        Converts a single .msg file to a uorb header
        """
        print("Generating headers from {0}".format(filename))
        genmsg.template_tools.generate_from_file(filename,
                                                 package,
                                                 outputdir,
                                                 templatedir,
                                                 includepath,
                                                 msg_template_map,
                                                 srv_template_map)


def convert_dir(inputdir, outputdir, templatedir):
        """
        Converts all .msg files in inputdir to uORB header files
        """

        # Find the most recent modification time in input dir
        maxinputtime = 0
        for f in os.listdir(inputdir):
                fni = os.path.join(inputdir, f)
                if os.path.isfile(fni):
                    it = os.path.getmtime(fni)
                    if it > maxinputtime:
                        maxinputtime = it;

        # Find the most recent modification time in output dir
        maxouttime = 0
        if os.path.isdir(outputdir):
            for f in os.listdir(outputdir):
                    fni = os.path.join(outputdir, f)
                    if os.path.isfile(fni):
                        it = os.path.getmtime(fni)
                        if it > maxouttime:
                            maxouttime = it;

        # Do not generate if nothing changed on the input
        if (maxinputtime != 0 and maxouttime != 0 and maxinputtime < maxouttime):
            return False

        includepath = incl_default + [':'.join([package, inputdir])]
        for f in os.listdir(inputdir):
                fn = os.path.join(inputdir, f)
                if os.path.isfile(fn):
                        convert_file(
                            fn,
                            outputdir,
                            templatedir,
                            includepath)

        return True


def copy_changed(inputdir, outputdir, prefix=''):
        """
        Copies files from inputdir to outputdir if they don't exist in
        ouputdir or if their content changed
        """

        # Make sure output directory exists:
        if not os.path.isdir(outputdir):
                os.makedirs(outputdir)

        for f in os.listdir(inputdir):
                fni = os.path.join(inputdir, f)
                if os.path.isfile(fni):
                        # Check if f exists in outpoutdir, copy the file if not
                        fno = os.path.join(outputdir, prefix + f)
                        if not os.path.isfile(fno):
                                shutil.copy(fni, fno)
                                print("{0}: new header file".format(f))
                                continue

                        if os.path.getmtime(fni) > os.path.getmtime(fno):
                                # The file exists in inputdir and outputdir
                                # only copy if contents do not match
                                if not filecmp.cmp(fni, fno):
                                        shutil.copy(fni, fno)
                                        print("{0}: updated".format(f))
                                        continue

                        #print("{0}: unchanged".format(f))


def convert_dir_save(inputdir, outputdir, templatedir, temporarydir, prefix):
        """
        Converts all .msg files in inputdir to uORB header files
        Unchanged existing files are not overwritten.
        """
        # Create new headers in temporary output directory
        convert_dir(inputdir, temporarydir, templatedir)
        # Copy changed headers from temporary dir to output dir
        copy_changed(temporarydir, outputdir, prefix)

if __name__ == "__main__":
        parser = argparse.ArgumentParser(
            description='Convert msg files to uorb headers')
        parser.add_argument('-d', dest='dir', help='directory with msg files')
        parser.add_argument('-f', dest='file',
                            help="files to convert (use only without -d)",
                            nargs="+")
        parser.add_argument('-e', dest='templatedir',
                            help='directory with template files',)
        parser.add_argument('-o', dest='outputdir',
                            help='output directory for header files')
        parser.add_argument('-t', dest='temporarydir',
                            help='temporary directory')
        parser.add_argument('-p', dest='prefix', default='',
                            help='string added as prefix to the output file '
                            ' name when converting directories')
        args = parser.parse_args()

        if args.file is not None:
                for f in args.file:
                        convert_file(
                            f,
                            args.outputdir,
                            args.templatedir,
                            incl_default)
        elif args.dir is not None:
                convert_dir_save(
                    args.dir,
                    args.outputdir,
                    args.templatedir,
                    args.temporarydir,
                    args.prefix)
