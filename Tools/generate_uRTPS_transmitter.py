#!/usr/bin/env python

import sys, os
'''
print('sys.argv[0] =', sys.argv[0])
pathname = os.path.dirname(sys.argv[0])
print('path =', pathname)
print('full path =', os.path.abspath(pathname))

print(os.path.realpath(__file__))
'''

import px_generate_uorb_topic_files

px_generate_uorb_topic_files.append_to_include_path({"msg/"}, px_generate_uorb_topic_files.INCL_DEFAULT)

px_generate_uorb_topic_files.generate_uRTPS_application_file(sys.argv[1], "Tools/tmp", "msg/templates/uorb/", px_generate_uorb_topic_files.INCL_DEFAULT)


#python px_generate_uorb_topic_files.py --app -q -f msg/message_uRTPS.msg msg/tipo_anidado.msg -i msg/ -o . -e templates_uorb -t topics_temporary_sources
