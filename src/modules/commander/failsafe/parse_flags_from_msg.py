#!/usr/bin/env python3

"""
Extract fields from .msg for failsafe state machine simulation
"""
import math
import os
import argparse
import sys
import re


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract fields from .msg files')
    parser.add_argument('msg_file', help='FailsafeFlags.msg file')
    parser.add_argument('header_file', help='generated_uorb_struct_field_mapping.h')
    parser.add_argument('html_template', help='HTML template input file')
    parser.add_argument('html_output', help='HTML output file')
    args = parser.parse_args()

    msg_filename = args.msg_file
    header_file = args.header_file
    html_template_file = args.html_template
    html_output_file = args.html_output
    msg_name = os.path.splitext(os.path.basename(msg_filename))[0]

    groups = []
    class Group:
        def __init__(self, group_name):
            self.name = group_name
            self.fields = [] # tuples of (field_name, comment)
            self.new_row = False

    group_name = ''
    new_group = True
    num_fields = 0

    with open(msg_filename, 'r') as lineparser:
        while True:
            line = lineparser.readline()
            if not line:
                break
            line = line.strip()
            if line == '':
                new_group = True
                continue
            if line.startswith('#'):
                # use comment & empty lines for grouping and group names
                group_name = line[1:].strip()
                continue
            if not line.startswith('bool') and not line.startswith('uint8'):
                # only booleans & uint8 supported
                continue
            field_search = re.search('^(\w+)\s+(\w+)', line)
            field_name = field_search.group(2)
            if field_name.startswith('mode_req_') or field_name == 'timestamp':
                continue

            field_search = re.search('^(\w+)\s+(\w+)\s+#(.*)$', line)
            if not field_search:
                raise Exception("{:}:\nFailed to extract fields from '{:}' (missing comment?)".format(msg_filename, line))

            if new_group:
                groups.append(Group(group_name))
                new_group = False
                group_name = ''
            groups[-1].fields.append((field_search.group(1), field_search.group(2), field_search.group(3).strip()))
            num_fields += 1

    # Split conditions into rows
    num_rows = 2
    num_fields_per_row = math.ceil(num_fields / num_rows)
    current_num_fields = 0
    for group in groups:
        current_num_fields += len(group.fields)
        if current_num_fields > num_fields_per_row:
            group.new_row = True
            current_num_fields = 0

    # Header file
    macro_lines = ''
    for group in groups:
        for field_type, field_name, comment in group.fields:
            macro_lines += '   .property("{0}", &px4::msg::{1}::{0}) \\\n'.format(field_name, msg_name)

    cpp_emscription_macro = '#define UORB_STRUCT_FIELD_MAPPING \\\n{}\n'.format(macro_lines)

    with open(header_file, 'w') as file:
        file.write(cpp_emscription_macro)

    # JS + Html
    js_tag = '{{{ JS_STATE_CODE }}}'
    html_tag = '{{{ HTML_CONDITIONS }}}'
    js_code = ''
    html_conditions = ''
    html_conditions += '<table><tr><td>'
    for group in groups:
        if group.new_row:
            html_conditions += '</td><td>'
        html_conditions += '<p>'
        if group.name != '':
            html_conditions += '<h5>' + group.name + '</h5>'
        for field_type, field_name, comment in group.fields:
            if field_type == 'bool':
                js_code += "    state.{0} = document.querySelector('input[id=\"{0}\"]').checked;\n".format(field_name)
                html_conditions += '''
                            <div class="checkbox-field">
                                <input type="checkbox" id="{0}" name="{0}">
                                <label for="{0}">{1}</label>
                            </div>
    '''.format(field_name, comment)

            elif field_type == 'uint8':
                assert field_name == 'battery_warning', "only battery_warning supported for uint8 type"
                js_code += "    state.{0} = document.querySelector('select[id=\"{0}\"]').value;\n".format(field_name)
                html_conditions += '''
                            <div>
                                <label for="{0}">{1}:&nbsp;</label>
                                <select id="{0}" name="{0}">
                                    <option value="0">None</option>
                                    <option value="1">Low</option>
                                    <option value="2">Critical</option>
                                    <option value="3">Emergency</option>
                                </select>
                            </div>
    '''.format(field_name, comment)
            else:
                raise Exception("Unsupported type: {:}".format(field_type))
        html_conditions += '</p>'
    html_conditions += '</td></tr></table>'

    with open(html_template_file, 'r') as file:
        html_template_content = file.read()
    assert js_tag in html_template_content, "js_tag not found in {}".format(html_template_file)
    assert html_tag in html_template_content, "html_tag not found in {}".format(html_template_file)
    html_template_content = html_template_content.replace(js_tag, js_code)
    html_template_content = html_template_content.replace(html_tag, html_conditions)
    with open(html_output_file, 'w') as file:
        file.write(html_template_content)
