#!/usr/bin/env python3
""" Script to params from module.yaml config file(s)
    Note: serial params are handled in Tools/serial/generate_config.py
"""

import argparse
import os
import sys


try:
    import yaml
except ImportError as e:
    print("Failed to import yaml: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyyaml")
    print("")
    sys.exit(1)

parser = argparse.ArgumentParser(description='Generate params from module.yaml file(s)')

parser.add_argument('--config-files', type=str, nargs='*', default=[],
                    help='YAML module config file(s)')
parser.add_argument('--params-file', type=str, action='store',
                    help='Parameter output file')
parser.add_argument('--ethernet', action='store_true',
                    help='Ethernet support')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

verbose = args.verbose
params_output_file = args.params_file
ethernet_supported = args.ethernet

def parse_yaml_parameters_config(yaml_config, ethernet_supported):
    """ parse the parameters section from the yaml config file """
    if 'parameters' not in yaml_config:
        return ''
    parameters_section_list = yaml_config['parameters']
    ret = ''
    for parameters_section in parameters_section_list:
        if 'definitions' not in parameters_section:
            continue
        definitions = parameters_section['definitions']
        param_group = parameters_section.get('group', None)
        for param_name in definitions:
            param = definitions[param_name]
            if param.get('requires_ethernet', False) and not ethernet_supported:
                continue
            num_instances = param.get('num_instances', 1)
            instance_start = param.get('instance_start', 0) # offset

            # get the type and extract all tags
            tags = '@group {:}'.format(param_group)
            if param['type'] == 'enum':
                param_type = 'INT32'
                for key in param['values']:
                    tags += '\n * @value {:} {:}'.format(key, param['values'][key])
            elif param['type'] == 'boolean':
                param_type = 'INT32'
                tags += '\n * @boolean'
            elif param['type'] == 'int32':
                param_type = 'INT32'
            elif param['type'] == 'float':
                param_type = 'FLOAT'
            else:
                raise Exception("unknown param type {:}".format(param['type']))

            for tag in ['decimal', 'increment', 'category', 'volatile', 'bit',
                        'min', 'max', 'unit', 'reboot_required']:
                if tag in param:
                    tags += '\n * @{:} {:}'.format(tag, param[tag])

            for i in range(num_instances):
                # default value
                default_value = 0
                if 'default' in param:
                    # default can be a list of num_instances or a single value
                    if type(param['default']) == list:
                        assert len(param['default']) == num_instances
                        default_value = param['default'][i]
                    else:
                        default_value = param['default']

                if type(default_value) == bool:
                    default_value = int(default_value)

                # output the existing C-style format
                ret += '''
/**
 * {short_descr}
 *
 * {long_descr}
 *
 * {tags}
 */
PARAM_DEFINE_{param_type}({name}, {default_value});
'''.format(short_descr=param['description']['short'].replace("\n", "\n * "),
           long_descr=param['description']['long'].replace("\n", "\n * "),
           tags=tags,
           param_type=param_type,
           name=param_name,
           default_value=default_value,
          ).replace('${i}', str(i+instance_start))
    return ret

all_params = ""

for yaml_file in args.config_files:
    with open(yaml_file, 'r') as stream:
        try:
            yaml_config = yaml.load(stream, Loader=yaml.Loader)

            all_params += parse_yaml_parameters_config(yaml_config, ethernet_supported)

        except yaml.YAMLError as exc:
            print(exc)
            raise

if verbose: print("Generating {:}".format(params_output_file))
with open(params_output_file, 'w') as fid:
    fid.write(all_params)

