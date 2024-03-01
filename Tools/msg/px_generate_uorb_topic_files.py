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
px_generate_uorb_topic_files.py
Generates c/cpp header/source files for uorb topics from .msg
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


TEMPLATE_FILE = ['msg.h.em', 'msg.cpp.em', 'uorb_idl_header.h.em', 'msg.json.em']
TOPICS_LIST_TEMPLATE_FILE = ['uORBTopics.hpp.em', 'uORBTopics.cpp.em', None, None]
INCL_DEFAULT = ['std_msgs:./msg/std_msgs']
PACKAGE = 'px4'
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


def generate_output_from_file(format_idx, filename, outputdir, package, templatedir, includepath, all_topics):
    """
    Converts a single .msg file to an uorb header/source file
    """
    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(package, os.path.basename(filename))

    file_base_name = os.path.basename(filename).replace(".msg", "")

    full_type_name_snake = re.sub(r'(?<!^)(?=[A-Z])', '_', file_base_name).lower()

    spec = genmsg.msg_loader.load_msg_from_file(msg_context, filename, full_type_name)

    field_name_and_type = {}
    for field in spec.parsed_fields():
        field_name_and_type.update({field.name: field.type})

    # assert if the timestamp field exists
    try:
        assert 'timestamp' in field_name_and_type
    except AssertionError:
        print("[ERROR] uORB topic files generator:\n\tgenerate_output_from_file:\tNo 'timestamp' field found in " +
              spec.short_name + " msg definition!")
        exit(1)

    # assert if the timestamp field is of type uint64
    try:
        assert field_name_and_type.get('timestamp') == 'uint64'
    except AssertionError:
        print("[ERROR] uORB topic files generator:\n\tgenerate_output_from_file:\t'timestamp' field in " + spec.short_name +
              " msg definition is not of type uint64 but rather of type " + field_name_and_type.get('timestamp') + "!")
        exit(1)

    topics = get_topics(filename)

    if includepath:
        search_path = genmsg.command_line.includepath_to_dict(includepath)
    else:
        search_path = {}

    genmsg.msg_loader.load_depends(msg_context, spec, search_path)

    em_globals = {
        "name_snake_case": full_type_name_snake,
        "file_name_in": filename,
        "file_base_name": file_base_name,
        "search_path": search_path,
        "msg_context": msg_context,
        "spec": spec,
        "topics": topics,
        "all_topics": all_topics,
    }

    # Make sure output directory exists:
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    template_file = os.path.join(templatedir, TEMPLATE_FILE[format_idx])
    extension = os.path.splitext(os.path.splitext(TEMPLATE_FILE[format_idx])[0])[1]
    if format_idx == 2:
        output_file = os.path.join(outputdir, file_base_name + extension)
    else:
        output_file = os.path.join(outputdir, full_type_name_snake + extension)

    return generate_by_template(output_file, template_file, em_globals)


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


def generate_topics_list_file_from_files(files, outputdir, template_filename, templatedir, all_topics):
    # generate cpp file with topics list
    filenames = []
    for filename in [os.path.basename(p) for p in files if os.path.basename(p).endswith(".msg")]:
        filenames.append(re.sub(r'(?<!^)(?=[A-Z])', '_', filename).lower())

    tl_globals = {"msgs": filenames, "all_topics": all_topics}
    tl_template_file = os.path.join(templatedir, template_filename)
    tl_out_file = os.path.join(outputdir, template_filename.replace(".em", ""))

    generate_by_template(tl_out_file, tl_template_file, tl_globals)


def append_to_include_path(path_to_append, curr_include, package):
    for p in path_to_append:
        curr_include.append('%s:%s' % (package, p))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert msg files to uorb headers/sources')
    parser.add_argument('--headers', help='Generate header files', action='store_true')
    parser.add_argument('--sources', help='Generate source files', action='store_true')
    parser.add_argument('--uorb-idl-header', help='Generate uORB compatible idl header', action='store_true')
    parser.add_argument('--json', help='Generate json files', action='store_true')
    parser.add_argument('-f', dest='file',
                        help="files to convert",
                        nargs="+")
    parser.add_argument('-i', dest="include_paths",
                        help='Additional Include Paths', nargs="*",
                        default=None)
    parser.add_argument('-e', dest='templatedir',
                        help='directory with template files',)
    parser.add_argument('-k', dest='package', default=PACKAGE,
                        help='package name')
    parser.add_argument('-o', dest='outputdir',
                        help='output directory for header files')
    parser.add_argument('-p', dest='prefix', default='',
                        help='string added as prefix to the output file '
                        ' name when converting directories')
    args = parser.parse_args()

    if args.include_paths:
        append_to_include_path(args.include_paths, INCL_DEFAULT, args.package)

    if args.headers:
        generate_idx = 0
    elif args.sources:
        generate_idx = 1
    elif args.uorb_idl_header:
        generate_idx = 2
    elif args.json:
        generate_idx = 3
    else:
        print('Error: either --headers, --sources or --json must be specified')
        exit(-1)
    if args.file is not None:
        all_topics = []
        for msg_filename in args.file:
            all_topics.extend(get_topics(msg_filename))
        all_topics.sort()

        for f in args.file:
            generate_output_from_file(generate_idx, f, args.outputdir, args.package, args.templatedir, INCL_DEFAULT, all_topics)

        # Generate topics list header and source file
        if TOPICS_LIST_TEMPLATE_FILE[generate_idx] is not None and os.path.isfile(os.path.join(args.templatedir, TOPICS_LIST_TEMPLATE_FILE[generate_idx])):
            generate_topics_list_file_from_files(args.file, args.outputdir, TOPICS_LIST_TEMPLATE_FILE[generate_idx], args.templatedir, all_topics)
