#!/usr/bin/env python3
""" Script to params from module.yaml config file(s)
    Note: serial params are handled in Tools/serial/generate_config.py
"""

import argparse
import os
import sys

from output_groups_from_timer_config import get_timer_groups, get_output_groups

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
parser.add_argument('--timer-config', type=str, action='store',
                    help='board-specific timer_config.cpp file')
parser.add_argument('--ethernet', action='store_true',
                    help='Ethernet support')
parser.add_argument('--board', type=str, action='store',
                    help='board name, e.g. ')
parser.add_argument('--board-with-io', dest='board_with_io', action='store_true',
                    help='Indicate that the board as an IO for extra PWM',
                    default=False)
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

verbose = args.verbose
params_output_file = args.params_file
timer_config_file = args.timer_config
ethernet_supported = args.ethernet
board_with_io = args.board_with_io
board = args.board

root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../..")
output_functions_file = os.path.join(root_dir,"src/lib/mixer_module/output_functions.yaml")

def process_param_prefix(param_prefix):
    if param_prefix == '${PWM_MAIN_OR_HIL}':
        if board == 'px4_sitl': return 'PWM_MAIN'
        return 'HIL_ACT'
    if param_prefix == '${PWM_MAIN_OR_AUX}':
        if board_with_io: return 'PWM_AUX'
        return 'PWM_MAIN'
    if '${' in param_prefix:
        raise Exception('unhandled variable in {:}'.format(param_prefix))
    return param_prefix

def process_channel_label(channel_label):
    if channel_label == '${PWM_MAIN_OR_HIL}':
        if board == 'px4_sitl': return 'PWM Sim'
        return 'HIL actuator'
    if channel_label == '${PWM_MAIN_OR_AUX}':
        if board_with_io: return 'PWM Aux'
        return 'PWM Main'
    if '${' in channel_label:
        raise Exception('unhandled variable in {:}'.format(channel_label))
    return channel_label


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
            # 'definitions' either contains the param definition directly (dict),
            # or a list of definitions with that name (multiple entries for a
            # multi-instance param with different instance_start)
            param_list = definitions[param_name]
            if not isinstance(param_list, list):
                param_list = [param_list]

            for param in param_list:
                if param.get('requires_ethernet', False) and not ethernet_supported:
                    continue
                num_instances = param.get('num_instances', 1)
                instance_start = param.get('instance_start', 0) # offset
                instance_start_label = param.get('instance_start_label', instance_start)

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
           name=param_name.replace('${i}', str(i+instance_start)),
           default_value=default_value,
          ).replace('${i}', str(i+instance_start_label))
    return ret


def get_actuator_output_params(yaml_config, output_functions,
    timer_config_file, verbose):
    """ parse the actuator_output section from the yaml config file
        :return: dict of param definitions
    """
    if not 'actuator_output' in yaml_config:
        return {}
    output_groups = yaml_config['actuator_output']['output_groups']
    all_params = {}
    group_idx = 0

    def add_local_param(param_name, param_def):
        nonlocal all_params
        # add as a list, as there can be multiple entries with the same param_name
        if not param_name in all_params:
            all_params[param_name] = []
        all_params[param_name].append(param_def)

    while group_idx < len(output_groups):
        group = output_groups[group_idx]
        group_idx += 1

        if verbose: print("processing group: {:}".format(group))

        # Check for generator and generate additional data.
        # We do this by extending the output_groups list and parse in a later iteration
        if 'generator' in group:
            if group['generator'] == 'pwm':
                param_prefix = process_param_prefix(group['param_prefix'])
                channel_labels = [process_channel_label(label) for label in group['channel_labels']]
                standard_params = group.get('standard_params', [])
                extra_function_groups = group.get('extra_function_groups', [])
                pwm_timer_param = group.get('pwm_timer_param', None)
                if timer_config_file is None:
                    raise Exception('trying to generate pwm outputs, but --timer-config not set')
                timer_groups = get_timer_groups(timer_config_file, verbose)
                timer_output_groups, timer_params = get_output_groups(timer_groups,
                    param_prefix, channel_labels,
                    standard_params, extra_function_groups, pwm_timer_param,
                    verbose=verbose)
                all_params.update(timer_params)
                output_groups.extend(timer_output_groups)
            else:
                raise Exception('unknown generator {:}'.format(group['generator']))
            continue

        num_channels = group['num_channels']
        param_prefix = process_param_prefix(group['param_prefix'])
        channel_label = process_channel_label(group['channel_label'])
        standard_params = group.get('standard_params', {})
        instance_start = group.get('instance_start', 1)
        instance_start_label = group.get('instance_start_label', instance_start)
        if len(param_prefix) > 9: # 16 - len('_FAIL') - 2 (2 digits for index)
            raise Exception("param prefix {:} too long (max length=10)".format(param_prefix))
        # collect the functions
        function_groups = ['common']
        function_groups.extend(group.get('extra_function_groups', []))
        output_function_values = {}
        for function_group in function_groups:
            group = output_functions[function_group]
            for function_name in group:
                function_name_label = function_name.replace('_', ' ')
                if isinstance(group[function_name], int):
                    output_function_values[group[function_name]] = function_name_label
                else:
                    start = group[function_name]['start']
                    count = group[function_name]['count']
                    for i in range(count):
                        output_function_values[start+i] = function_name_label+' '+str(i+1)

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
            'instance_start': instance_start,
            'instance_start_label': instance_start_label,
            'num_instances': num_channels,
            'default': 0,
            'values': output_function_values
            }
        add_local_param(param_prefix+'_FUNC${i}', param)

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
                    'instance_start': instance_start,
                    'instance_start_label': instance_start_label,
                    'num_instances': num_channels,
                    'min': standard_params[key]['min'],
                    'max': standard_params[key]['max'],
                    'default': standard_params[key]['default'],
                    }
                add_local_param(param_prefix+'_'+param_suffix+'${i}', param)

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
        actuator_output_params = get_actuator_output_params(yaml_config,
                output_functions, timer_config_file, verbose)
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

