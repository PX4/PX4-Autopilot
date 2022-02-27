#!/usr/bin/env python3
""" Script to generate actuators.json metadata from module.yaml config file(s)
"""

import argparse
import lzma #to create .xz file
import json
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

parser = argparse.ArgumentParser(description='Generate actuators.json from module.yaml file(s)')

parser.add_argument('--config-files', type=str, nargs='*', default=[],
                    help='YAML module config file(s)')
parser.add_argument('--output-file', type=str, action='store',
                    help='JSON output file', required=True)
parser.add_argument('--compress', action='store_true', help='Add a compressed output file')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('--timer-config', type=str, action='store',
                    help='board-specific timer_config.cpp file')
parser.add_argument('--board', type=str, action='store',
                    help='board name, e.g. ')
parser.add_argument('--board-with-io', dest='board_with_io', action='store_true',
                    help='Indicate that the board as an IO for extra PWM',
                    default=False)

args = parser.parse_args()

compress = args.compress
verbose = args.verbose
output_file = args.output_file
timer_config_file = args.timer_config
board_with_io = args.board_with_io
board = args.board

root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../..")
output_functions_file = os.path.join(root_dir,"src/lib/mixer_module/output_functions.yaml")

def save_compressed(filename):
    #create lzma compressed version
    xz_filename=filename+'.xz'
    with lzma.open(xz_filename, 'wt', preset=9) as f:
        with open(filename, 'r') as content_file:
            f.write(content_file.read())

def load_yaml_file(file_name):
    with open(file_name, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            raise

# functions
output_functions_yaml = load_yaml_file(output_functions_file)
output_functions = output_functions_yaml['functions']
functions = {}

def add_function(functions, index, name, function_obj=None):
    functions[index] = {
        "label": name
    }
    if function_obj is not None:
        if function_obj.get('exclude_from_actuator_testing', False):
            functions[index]['exclude-from-actuator-testing'] = True
        if 'note' in function_obj:
            functions[index]['note'] = function_obj['note']

for group_key in output_functions:
    group = output_functions[group_key]
    for function_name in group:
        function_name_label = function_name.replace('_', ' ')
        if isinstance(group[function_name], int):
            add_function(functions, group[function_name], function_name_label)
        elif not 'count' in group[function_name]:
            add_function(functions, group[function_name]['start'], function_name_label, group[function_name])
        else:
            start = group[function_name]['start']
            count = group[function_name]['count']
            for i in range(count):
                add_function(functions, start+i, function_name_label+' '+str(i+1), group[function_name])

# outputs
outputs = []

def process_module_name(module_name):
    if module_name == '${PWM_MAIN_OR_AUX}':
        if board_with_io: return 'PWM AUX'
        return 'PWM MAIN'
    if '${' in module_name:
        raise Exception('unhandled variable in {:}'.format(module_name))
    return module_name

def process_param_prefix(param_prefix):
    if param_prefix == '${PWM_MAIN_OR_AUX}':
        if board_with_io: return 'PWM_AUX'
        return 'PWM_MAIN'
    if '${' in param_prefix:
        raise Exception('unhandled variable in {:}'.format(param_prefix))
    return param_prefix

def process_channel_label(module_name, channel_label, no_prefix):
    if channel_label == '${PWM_MAIN_OR_AUX_CAP}':
        return 'CAP'
    if channel_label == '${PWM_MAIN_OR_AUX}':
        if board_with_io: return 'AUX'
        return 'MAIN'
    if '${' in channel_label:
        raise Exception('unhandled variable in {:}'.format(channel_label))
    if no_prefix: return channel_label
    return channel_label

def get_actuator_output(yaml_config, output_functions, timer_config_file, verbose):
    """ parse the actuator_output section from the yaml config file
    """
    if not 'actuator_output' in yaml_config:
        return None


    actuator_output_yaml = yaml_config['actuator_output']
    output_groups = actuator_output_yaml['output_groups']
    module_name = process_module_name(yaml_config['module_name'])
    group_idx = 0

    if verbose: print('processing module: {}'.format(module_name))

    actuator_output = {
            'label': module_name
        }
    if 'show_subgroups_if' in actuator_output_yaml:
        actuator_output['show-subgroups-if'] = actuator_output_yaml['show_subgroups_if']

    # config parameters
    def get_config_params(param_list):
        """ convert config parameter list (per group or per subgroup) """
        parameters = []
        for config_param in param_list:
            if verbose:
                print('config param: {}'.format(config_param))
            param = {
                    'name': config_param['param'],
                }
            if 'label' in config_param:
                param['label'] = config_param['label']
            if 'function' in config_param:
                param['function'] = config_param['function']
            parameters.append(param)
        return parameters

    parameters = get_config_params(actuator_output_yaml.get('config_parameters', []))
    if len(parameters) > 0:
        actuator_output['parameters'] = parameters

    subgroups = []

    while group_idx < len(output_groups):
        group = output_groups[group_idx]
        group_idx += 1

        if verbose: print("processing group: {:}".format(group))

        # Check for generator and generate additional data.
        if 'generator' in group:
            if group['generator'] == 'pwm':
                param_prefix = process_param_prefix(group['param_prefix'])
                no_prefix = not group.get('channel_label_module_name_prefix', True)
                channel_labels = [process_channel_label(module_name, label, no_prefix)
                    for label in group['channel_labels']]
                standard_params = group.get('standard_params', [])
                extra_function_groups = group.get('extra_function_groups', [])
                pwm_timer_param = group.get('pwm_timer_param', None)
                if 'timer_config_file' in group:
                    timer_config_file = os.path.join(root_dir, group['timer_config_file'])
                if timer_config_file is None:
                    raise Exception('trying to generate pwm outputs, but --timer-config not set')
                timer_groups = get_timer_groups(timer_config_file, verbose)
                timer_output_groups, timer_params = get_output_groups(timer_groups,
                    param_prefix, channel_labels,
                    standard_params, extra_function_groups, pwm_timer_param,
                    verbose=verbose)
                output_groups.extend(timer_output_groups)
            else:
                raise Exception('unknown generator {:}'.format(group['generator']))
            continue
        
        subgroup = {}

        # supported actions
        if 'supported_actions' in group:
            actions = {}
            for action_name in group['supported_actions']:
                action = group['supported_actions'][action_name]
                action_name = action_name.replace('_', '-')
                actions[action_name] = {}
                if 'supported_if' in action:
                    actions[action_name]['supported-if'] = action['supported_if']
                if 'actuator_types' in action:
                    actions[action_name]['actuator-types'] = action['actuator_types']
            subgroup['supported-actions'] = actions

        # channels
        num_channels = group['num_channels']
        no_prefix = not group.get('channel_label_module_name_prefix', True)
        channel_label = process_channel_label(module_name, group['channel_label'], no_prefix)
        instance_start = group.get('instance_start', 1)
        instance_start_label = group.get('instance_start_label', instance_start)
        channels = []
        for channel in range(num_channels):
            channels.append({
                'label': channel_label + ' ' +str(channel+instance_start_label),
                'param-index': channel+instance_start
                })
        subgroup['channels'] = channels

        if 'group_label' in group:
            subgroup['label'] = group['group_label']


        # per-channel-params
        per_channel_params = []
        param_prefix = process_param_prefix(group['param_prefix'])
        standard_params = group.get('standard_params', {})
        standard_params_array = [
            ( 'function', 'Function', 'FUNC', False ),
            ( 'disarmed', 'Disarmed', 'DIS', False ),
            ( 'min', 'Minimum', 'MIN', False ),
            ( 'max', 'Maximum', 'MAX', False ),
            ( 'failsafe', 'Failsafe', 'FAIL', True ),
            ]
        for key, label, param_suffix, advanced in standard_params_array:
            show_if = None
            if key in standard_params and 'show_if' in standard_params[key]:
                show_if = standard_params[key]['show_if']

            if key in standard_params or key == 'function':
                param = {
                        'label': label,
                        'name': param_prefix+'_'+param_suffix+'${i}',
                        'function': key,
                    }
                if advanced: param['advanced'] = True
                if show_if: param['show-if'] = show_if
                per_channel_params.append(param)


        param = {
                'label': 'Rev Range\n(for Servos)',
                'name': param_prefix+'_REV',
                'index-offset': -1,
                'show-as': 'bitset',
            }
        per_channel_params.append(param)

        # TODO: support non-standard per-channel parameters

        subgroup['per-channel-parameters'] = per_channel_params

        # group config params
        parameters = get_config_params(group.get('config_parameters', []))
        if len(parameters) > 0:
            subgroup['parameters'] = parameters

        subgroups.append(subgroup)

    actuator_output['subgroups'] = subgroups
    return actuator_output

# Mixers
mixers = None
def get_mixers(yaml_config, output_functions, verbose):
    if not 'mixer' in yaml_config:
        return None

    actuator_types = {}
    for actuator_type_key in yaml_config['mixer']['actuator_types']:
        actuator_type_conf = yaml_config['mixer']['actuator_types'][actuator_type_key]
        actuator_type = { }
        if actuator_type_key != 'DEFAULT':
            actuator_type['label-index-offset'] = 1 # always 1
        if 'functions' in actuator_type_conf:
            function_name = actuator_type_conf['functions']
            # we expect the function to be in 'common' (this is not a requirement, just simplicity)
            output_function = output_functions['common'][function_name]
            actuator_type['function-min'] = output_function['start']
            actuator_type['function-max'] = output_function['start'] + output_function['count'] - 1

        values = actuator_type_conf['actuator_testing_values']
        actuator_type['values'] = {
                'min': values['min'],
                'max': values['max'],
            }
        if values.get('default_is_nan', False):
            actuator_type['values']['default-is-nan'] = True
        else:
            actuator_type['values']['default'] = values['default']
        if values.get('reversible', False):
            actuator_type['values']['reversible'] = True

        # per item params
        per_item_params = []
        for per_item_param in actuator_type_conf.get('per_item_parameters', []):
            per_item_params.append({k.replace('_','-'): v for k, v in per_item_param.items()})
        if len(per_item_params) > 0:
            actuator_type['per-item-parameters'] = per_item_params

        actuator_types[actuator_type_key] = actuator_type

    if verbose:
        print('Actuator types: {}'.format(actuator_types))

    config = []
    yaml_mixer_config = yaml_config['mixer']['config']
    select_param = yaml_mixer_config['param']
    types = yaml_mixer_config['types']
    for type_index in types:
        current_type = types[type_index]
        option = select_param + '==' + str(type_index)
        mixer_config = {
                'option': option,
                'help-url': 'https://docs.px4.io/master/en/config/actuators.html',
            }
        for optional in ['type', 'title']:
            if optional in current_type:
                mixer_config[optional] = current_type[optional]
        actuators = []
        for actuator_conf in current_type['actuators']:
            actuator = {
                    'actuator-type': actuator_conf['actuator_type'],
                    'required': True, # for now always set as required
                }
            # sanity check that actuator type exists
            if actuator_conf['actuator_type'] not in actuator_types:
                raise Exception('actuator type "{}" does not exist (valid: {})'.format(actuator_conf['actuator_type'], actuator_types.keys()))

            if 'group_label' in actuator_conf:
                actuator['group-label'] = actuator_conf['group_label']
            else:
                # infer from actuator type
                if actuator_conf['actuator_type'] == 'motor':
                    actuator['group-label'] = 'Motors'
                elif actuator_conf['actuator_type'] == 'servo':
                    actuator['group-label'] = 'Servos'
                else:
                    raise Exception('Missing group label for actuator type "{}"'.format(actuator_conf['actuator_type']))

            if 'count' in actuator_conf: # possibly dynamic size
                actuator['count'] = actuator_conf['count']
                per_item_params = actuator_conf.get('per_item_parameters', {})
                params = []
                if 'standard' in per_item_params:
                    standard_params = per_item_params['standard']
                    index_offset = standard_params.get('index_offset', 0)
                    if 'position' in standard_params:
                        params.extend([
                        {
                            'label': 'Position X',
                            'function': 'posx',
                            'name': standard_params['position'][0],
                            'index-offset': index_offset,
                        },
                        {
                            'label': 'Position Y',
                            'function': 'posy',
                            'name': standard_params['position'][1],
                            'index-offset': index_offset,
                        },
                        {
                            'label': 'Position Z',
                            'function': 'posz',
                            'name': standard_params['position'][2],
                            'advanced': True,
                            'index-offset': index_offset,
                        },
                        ])
                if 'extra' in per_item_params:
                    for extra_param in per_item_params['extra']:
                        params.append({k.replace('_','-'): v for k, v in extra_param.items()})
                actuator['per-item-parameters'] = params
                if 'item_label_prefix' in actuator_conf:
                    actuator['item-label-prefix'] = actuator_conf['item_label_prefix']
            else: # fixed size
                labels = []
                pos_x = []
                pos_y = []
                pos_z = []
                for instance in actuator_conf['instances']:
                    labels.append(instance['name'])
                    pos_x.append(instance['position'][0])
                    pos_y.append(instance['position'][1])
                    pos_z.append(instance['position'][2])
                actuator['count'] = len(labels)
                actuator['item-label-prefix'] = labels
                actuator['per-item-parameters'] = [
                        {
                            'label': 'Position X',
                            'function': 'posx',
                            'value': pos_x,
                        },
                        {
                            'label': 'Position Y',
                            'function': 'posy',
                            'value': pos_y,
                        },
                        {
                            'label': 'Position Z',
                            'function': 'posz',
                            'value': pos_z,
                            'advanced': True,
                        },
                    ]

            # actuator parameters
            parameters = []
            for param in actuator_conf.get('parameters', []):
                parameters.append({k.replace('_','-'): v for k, v in param.items()})
            actuator['parameters'] = parameters

            actuators.append(actuator)

        mixer_config['actuators'] = actuators
        config.append(mixer_config)

    if verbose:
        print('Mixer configs: {}'.format(config))

    rules = []
    for rule in yaml_config['mixer'].get('rules', []):
        rules.append({k.replace('_','-'): v for k, v in rule.items()})

    if verbose:
        print('Mixer rules: {}'.format(rules))
    
    mixers = {
            'actuator-types': actuator_types,
            'config': config,
            'rules': rules,
        }
    return mixers


for yaml_file in args.config_files:
    yaml_config = load_yaml_file(yaml_file)

    try:
        actuator_output = get_actuator_output(yaml_config,
                output_functions, timer_config_file, verbose)
        if actuator_output:
            outputs.append(actuator_output)

        parsed_mixers = get_mixers(yaml_config, output_functions, verbose)
        if parsed_mixers is not None:
            if mixers is not None:
                # only expected to be configured in one module
                raise Exception('multiple "mixer" sections in module config files')
            mixers = parsed_mixers
    except Exception as e:
        print('Exception while parsing {:}:'.format(yaml_file))
        raise e

if mixers is None:
    if len(outputs) > 0:
        raise Exception('Missing "mixer" section in yaml configs (CONFIG_MODULES_CONTROL_ALLOCATOR not added to the build?)')
    else:
        # set a minimal default
        mixers = {
                'actuator-types': { 'DEFAULT': { 'values': { 'min': 0, 'max': 1 } } },
            'config': [],
            }

actuators = {
    'version': 1,
    'show-ui-if': 'SYS_CTRL_ALLOC==1',
    'outputs_v1': outputs,
    'functions_v1': functions,
    'mixer_v1': mixers,
    }

with open(output_file, 'w') as outfile:
    indent = 2 if verbose else None
    json.dump(actuators, outfile, indent=indent)

if compress:
    save_compressed(output_file)
