#!/usr/bin/env python3
#############################################################################
#
#   Copyright (C) 2013-2022 PX4 Pro Development Team. All rights reserved.
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
px_generate_zenoh_topic_files.py
Generates c/cpp header/source files for use with zenoh
message files
"""

import os
import argparse
import re
import sys

try:
    import em
except ImportError as e:
    print("Failed to import em: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user empy")
    print("")
    sys.exit(1)

try:
    import genmsg.template_tools
except ImportError as e:
    print("Failed to import genmsg: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyros-genmsg")
    print("")
    sys.exit(1)


__author__ = "Sergey Belash, Thomas Gubler, Beat Kueng"
__copyright__ = "Copyright (C) 2013-2022 PX4 Development Team."
__license__ = "BSD"
__email__ = "thomasgubler@gmail.com"

ZENOH_TEMPLATE_FILE = ['Kconfig.topics.em', 'uorb_pubsub_factory.hpp.em']
TOPICS_TOKEN = '# TOPICS '


def get_topics(filename):
    """
    Get TOPICS names from a "# TOPICS" line
    """
    ofile = open(filename, 'r')
    text = ofile.read()
    result = []
    for each_line in text.split('\n'):
        if each_line.startswith(TOPICS_TOKEN):
            topic_names_str = each_line.strip()
            topic_names_str = topic_names_str.replace(TOPICS_TOKEN, "")
            topic_names_list = topic_names_str.split(" ")
            for topic in topic_names_list:
                # topic name PascalCase (file name) to snake_case (topic name)
                topic_name = re.sub(r'(?<!^)(?=[A-Z])', '_', topic).lower()
                result.append(topic_name)
    ofile.close()

    if len(result) == 0:
        # topic name PascalCase (file name) to snake_case (topic name)
        file_base_name = os.path.basename(filename).replace(".msg", "")
        topic_name = re.sub(r'(?<!^)(?=[A-Z])', '_', file_base_name).lower()
        result.append(topic_name)

    return result

def generate_by_template(output_file, template_file, em_globals):
    """
    Invokes empy intepreter to geneate output_file by the
    given template_file and predefined em_globals dict
    """
    # check if folder exists:
    folder_name = os.path.dirname(output_file)
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    ofile = open(output_file, 'w')
    # todo, reuse interpreter
    interpreter = em.Interpreter(output=ofile, globals=em_globals, options={
                                 em.RAW_OPT: True, em.BUFFERED_OPT: True})
    try:
        interpreter.file(open(template_file))
    except OSError as e:
        ofile.close()
        os.remove(output_file)
        raise
    interpreter.shutdown()
    ofile.close()
    return True


def generate_topics_list_file_from_files(files, outputdir, template_filename, templatedir):
    # generate cpp file with topics list
    filenames = []
    for filename in [os.path.basename(p) for p in files if os.path.basename(p).endswith(".msg")]:
        filenames.append(re.sub(r'(?<!^)(?=[A-Z])', '_', filename).lower())

    datatypes = []
    for filename in [os.path.basename(p) for p in files if os.path.basename(p).endswith(".msg")]:
        datatypes.append(re.sub(r'(?<!^)(?=[A-Z])', '_', filename).lower().replace(".msg",""))

    full_base_names = []
    for filename in [os.path.basename(p) for p in files if os.path.basename(p).endswith(".msg")]:
        full_base_names.append(filename.replace(".msg",""))

    topics = []
    for msg_filename in files:
        topics.extend(get_topics(msg_filename))

    tl_globals = {"msgs": filenames, "topics": topics, "datatypes": datatypes, "full_base_names": full_base_names}
    tl_template_file = os.path.join(templatedir, template_filename)
    tl_out_file = os.path.join(outputdir, template_filename.replace(".em", ""))

    generate_by_template(tl_out_file, tl_template_file, tl_globals)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert msg files to uorb headers/sources')
    parser.add_argument('--zenoh-config', help='Generate Zenoh Kconfig file', action='store_true')
    parser.add_argument('--zenoh-pub-sub', help='Generate Zenoh Pubsub factory', action='store_true')
    parser.add_argument('-f', dest='file',
                        help="files to convert (use only without -d)",
                        nargs="+")
    parser.add_argument('-e', dest='templatedir',
                        help='directory with template files',)
    parser.add_argument('-o', dest='outputdir',
                        help='output directory for header files')
    parser.add_argument('-p', dest='prefix', default='',
                        help='string added as prefix to the output file '
                        ' name when converting directories')
    args = parser.parse_args()

    if args.zenoh_config:
        generate_topics_list_file_from_files(args.file, args.outputdir, ZENOH_TEMPLATE_FILE[0], args.templatedir)
        exit(0)
    elif args.zenoh_pub_sub:
        generate_topics_list_file_from_files(args.file, args.outputdir, ZENOH_TEMPLATE_FILE[1], args.templatedir)
        exit(0)
    else:
        print('Error: either --headers or --sources must be specified')
        exit(-1)
