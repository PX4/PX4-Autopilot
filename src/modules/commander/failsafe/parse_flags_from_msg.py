#!/usr/bin/env python3

"""
Extract fields from .msg for failsafe state machine simulation
"""

import os
import argparse
import sys
import re


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract fields from .msg files')
    parser.add_argument('msg_file', help='vehicle_status_flags.msg file')
    parser.add_argument('header_file', help='generated_uorb_struct_field_mapping.h')
    parser.add_argument('html_template', help='HTML template input file')
    parser.add_argument('html_output', help='HTML output file')
    args = parser.parse_args()

    msg_filename = args.msg_file
    header_file = args.header_file
    html_template_file = args.html_template
    html_output_file = args.html_output
    msg_name = os.path.splitext(os.path.basename(msg_filename))[0]

    fields = [] # tuples of (field_name, comment)

    # TODO: use all fields
    included_field = [
        'local_position_valid',
        'global_position_valid',
        'local_altitude_valid',
        'auto_mission_available',
        'geofence_violated',
        'home_position_valid',
        'escs_failure',
        'offboard_control_signal_lost',
        'rc_signal_lost',
        'data_link_lost',
        'vtol_transition_failure',
        'mission_failure',
    ]

    with open(msg_filename, 'r') as lineparser:
        while True:
            line = lineparser.readline()
            if not line:
                break
            line = line.strip()
            if line == '' or line.startswith('#'):
                continue
            if not line.startswith('bool'):
                # only booleans supported
                continue
            field_search = re.search('^(\w+)\s+(\w+)', line)
            field_name = field_search.group(2)
            if field_name not in included_field:
                continue

            field_search = re.search('^(\w+)\s+(\w+)\s+#(.*)$', line)
            if not field_search:
                raise Exception("{:}:\nFailed to extract fields from '{:}' (missing comment?)".format(msg_filename, line))
            fields.append((field_search.group(2), field_search.group(3).strip()))


    # Header file
    macro_lines = ''
    for field_name, comment in fields:
        macro_lines += '   .property("{0}", &{1}_s::{0}) \\\n'.format(field_name, msg_name)

    cpp_emscription_macro = '#define UORB_STRUCT_FIELD_MAPPING \\\n{}\n'.format(macro_lines)

    with open(header_file, 'w') as file:
        file.write(cpp_emscription_macro)

    # JS + Html
    js_tag = '{{{ JS_STATE_CODE }}}'
    html_tag = '{{{ HTML_CONDITIONS }}}'
    js_code = ''
    html_conditions = ''
    for field_name, comment in fields:
        js_code += "    state.{0} = document.querySelector('input[id=\"{0}\"]').checked;\n".format(field_name)
        if field_name.endswith('_valid'): # TODO remove
            checked = 'checked'
        else:
            checked = ''
        html_conditions += '''
                    <div>
                        <input type="checkbox" {2} id="{0}" name="{0}">
                        <label for="{0}">{1}</label>
                    </div>
'''.format(field_name, comment, checked)

    with open(html_template_file, 'r') as file:
        html_template_content = file.read()
    assert js_tag in html_template_content, "js_tag not found in {}".format(html_template_file)
    assert html_tag in html_template_content, "html_tag not found in {}".format(html_template_file)
    html_template_content = html_template_content.replace(js_tag, js_code)
    html_template_content = html_template_content.replace(html_tag, html_conditions)
    with open(html_output_file, 'w') as file:
        file.write(html_template_content)

