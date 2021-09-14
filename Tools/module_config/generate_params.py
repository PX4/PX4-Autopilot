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

root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../..")
output_functions_file = os.path.join(root_dir,"src/lib/mixer_module/output_functions.yaml")

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


def get_actuator_output_params(yaml_config, output_functions):
    """ parse the actuator_output section from the yaml config file
        :return: dict of param definitions
    """
    if not 'actuator_output' in yaml_config:
        return {}
    output_groups = yaml_config['actuator_output']['output_groups']
    all_params = {}
    for group in output_groups:
        num_channels = group['num_channels']
        param_prefix = group['param_prefix']
        channel_label = group['channel_label']
        standard_params = group.get('standard_params', {})
        if len(param_prefix) > 9: # 16 - len('_FAIL') - 2 (2 digits for index)
            raise Exception("param prefix {:} too long (max length=10)".format(param_prefix))
        # collect the functions
        function_groups = ['common']
        function_groups.extend(group.get('extra_function_groups', []))
        output_function_values = {}
        for function_group in function_groups:
            group = output_functions[function_group]
            for function_name in group:
                if isinstance(group[function_name], int):
                    output_function_values[group[function_name]] = function_name
                else:
                    start = group[function_name]['start']
                    count = group[function_name]['count']
                    for i in range(count):
                        output_function_values[start+i] = function_name+str(i+1)

        # function param
        param = {
            'description': {
                'short': channel_label+' ${i} Output Function',
                'long':
'''Select what should be output on {:} ${{i}}.

The default failsafe value is set according to the selected function:
- 'Min' for ConstantMin
- 'Max' for ConstantMax
- 'Max' for Parachute
- ('Max'+'Min')/2 for Servos
- 'Disarmed' for the rest
'''.format(channel_label),
                },
            'type': 'enum',
            'instance_start': 1,
            'num_instances': num_channels,
            'default': 0,
            'values': output_function_values
            }
        all_params[param_prefix+'_FUNC${i}'] = param

        # handle standard_params
        disarmed_description = \
'''This is the output value that is set when not armed.

Note that non-motor outputs might already be active in prearm state if COM_PREARM_MODE is set.
'''
        minimum_description = \
'''Minimum output value (when not disarmed).

The output range can be reversed by setting Min > Max.
'''
        maximum_description = \
'''Maxmimum output value (when not disarmed).

The output range can be reversed by setting Min > Max.
'''
        failsafe_description = \
'''This is the output value that is set when in failsafe mode.

When set to -1 (default), the value depends on the function (see {:}).
'''.format(param_prefix+'_FUNC${i}')
        standard_params_array = [
            ( 'disarmed', 'Disarmed', 'DIS', disarmed_description ),
            ( 'min', 'Minimum', 'MIN', minimum_description ),
            ( 'max', 'Maximum', 'MAX', maximum_description ),
            ( 'failsafe', 'Failsafe', 'FAIL', failsafe_description ),
            ]
        for key, label, param_suffix, description in standard_params_array:
            if key in standard_params:

                # values must be in range of an uint16_t
                if standard_params[key]['min'] < 0:
                    raise Exception('minimum value for {:} expected >= 0 (got {:})'.format(key, standard_params[key]['min']))
                if standard_params[key]['max'] >= 1<<16:
                    raise Exception('maximum value for {:} expected <= {:} (got {:})'.format(key, 1<<16, standard_params[key]['max']))

                if key == 'failsafe':
                    standard_params[key]['default'] = -1
                    standard_params[key]['min'] = -1

                param = {
                    'description': {
                        'short': channel_label+' ${i} '+label+' Value',
                        'long': description
                        },
                    'type': 'int32',
                    'instance_start': 1,
                    'num_instances': num_channels,
                    'min': standard_params[key]['min'],
                    'max': standard_params[key]['max'],
                    'default': standard_params[key]['default'],
                    }
                all_params[param_prefix+'_'+param_suffix+'${i}'] = param

    if verbose: print('adding actuator params: {:}'.format(all_params))
    return all_params

def load_yaml_file(file_name):
    with open(file_name, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            raise

output_functions_yaml = load_yaml_file(output_functions_file)
output_functions = output_functions_yaml['functions']

all_params = ""

for yaml_file in args.config_files:
    yaml_config = load_yaml_file(yaml_file)

    # convert 'output_groups' section into additional params
    try:
        actuator_output_params = get_actuator_output_params(yaml_config, output_functions)
    except Exception as e:
        print('Exception while parsing {:}:'.format(yaml_file))
        raise e
    # now add them to the yaml config
    if not 'parameters' in yaml_config:
        yaml_config['parameters'] = []
    group_name = 'Actuator Outputs'
    yaml_config['parameters'].append({'group': group_name, 'definitions': actuator_output_params})

    all_params += parse_yaml_parameters_config(yaml_config, ethernet_supported)


if verbose: print("Generating {:}".format(params_output_file))
with open(params_output_file, 'w') as fid:
    fid.write(all_params)

