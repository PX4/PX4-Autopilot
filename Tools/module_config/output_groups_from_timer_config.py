#!/usr/bin/env python3
""" Script to parse board-specific timer_config.cpp and print the output groups
and timer config params to stdout
"""

import argparse
import os
import sys
import re
from itertools import groupby
from copy import deepcopy


def find_matching_brackets(brackets, s, verbose):
    idx = 0
    opening = 0
    first_open = -1
    while idx < len(s):
        if s[idx] == brackets[0]:
            opening += 1
            if first_open == -1:
                first_open = idx
        if s[idx] == brackets[1]:
            opening -= 1
            if opening == 0:
                if verbose: print(first_open, idx, s[first_open:idx+1])
                return first_open+1, idx
        idx += 1
    raise Exception('Failed to find opening/closing brackets in {:}'.format(s))

def extract_timer(line):
    # Try format: initIOTimer(Timer::Timer5, DMA{DMA::Index1, DMA::Stream0, DMA::Channel6}),
    search = re.search('Timer::([0-9a-zA-Z_]+)[,\)]', line, re.IGNORECASE)
    if search:
        return search.group(1), 'generic'

    # NXP FlexPWM format format: initIOPWM(PWM::FlexPWM2),
    search = re.search('PWM::Flex([0-9a-zA-Z_]+)..PWM::Submodule([0-9])[,\)]', line, re.IGNORECASE)
    if search:
        return (search.group(1) + '_' +  search.group(2)), 'imxrt'

    return None, 'unknown'

def extract_timer_from_channel(line, timer_names):
    # Try format: initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
    search = re.search('Timer::([0-9a-zA-Z_]+), ', line, re.IGNORECASE)
    if search:
        return search.group(1)

    # NXP FlexPWM format: initIOTimerChannel(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule0}, IOMUX::Pad::GPIO_B0_06),
    search = re.search('PWM::(PWM[0-9]+).*PWM::Submodule([0-9])', line, re.IGNORECASE)
    if search:
        return str(timer_names.index((search.group(1) + '_' +  search.group(2))))

    return None

def imxrt_is_dshot(line):

    # NXP FlexPWM format format: initIOPWM(PWM::FlexPWM2),
    search = re.search('(initIOPWMDshot)', line, re.IGNORECASE)
    if search:
        return True

    return False

def get_timer_groups(timer_config_file, verbose=False):
    with open(timer_config_file, 'r') as f:
        timer_config = f.read()

    # timers
    dshot_support = {str(i): False for i in range(16)}
    timers_start_marker = 'io_timers_t io_timers'
    timers_start = timer_config.find(timers_start_marker)
    if timers_start == -1:
        raise Exception('"{:}" not found in {:}'.format(timers_start_marker, timer_config_file))
    timer_config = timer_config[timers_start:]
    open_idx, close_idx = find_matching_brackets(('{', '}'), timer_config, verbose)
    timers_str = timer_config[open_idx:close_idx]
    timers = []
    timer_names = []
    for line in timers_str.splitlines():
        line = line.strip()
        if len(line) == 0 or line.startswith('//'):
            continue
        timer, timer_type = extract_timer(line)

        if timer_type == 'imxrt':
            if verbose: print('imxrt timer found')
            timer_names.append(timer)
            if imxrt_is_dshot(line):
                dshot_support[str(len(timers))] = True
            timers.append(str(len(timers)))
        elif timer:
            if verbose: print('found timer def: {:}'.format(timer))
            dshot_support[timer] = 'DMA' in line
            timers.append(timer)
        else:
            # Make sure we don't miss anything (e.g. for different syntax) or misparse (e.g. multi-line comments)
            raise Exception('Unparsed timer in line: {:}'.format(line))


    # channels
    channels_start_marker = 'timer_io_channels_t timer_io_channels'
    channels_start = timer_config.find(channels_start_marker)
    if channels_start == -1:
        raise Exception('"{:}" not found in {:}'.format(channels_start_marker, timer_config_file))

    timer_config = timer_config[channels_start:]
    open_idx, close_idx = find_matching_brackets(('{', '}'), timer_config, verbose)
    channels = timer_config[open_idx:close_idx]
    channel_timers = []
    channel_types = []

    for line in channels.splitlines():
        line = line.strip()
        if len(line) == 0 or line.startswith('//'):
            continue

        if verbose: print('--'+line+'--')
        timer = extract_timer_from_channel(line, timer_names)

        if timer:
            if verbose: print('Found timer: {:} in channel line {:}'.format(timer, line))
            channel_types.append('cap' if 'capture' in line.lower() else 'pwm')
            channel_timers.append(timer)
        else:
            # Make sure we don't miss anything (e.g. for different syntax) or misparse (e.g. multi-line comments)
            raise Exception('Unparsed channel in line: {:}'.format(line))

    if len(channel_timers) == 0:
        raise Exception('No channels found in "{:}"'.format(channels))

    groups = [(timers.index(k), len(list(g)), dshot_support[k]) for k, g in groupby(channel_timers)]
    outputs = {
        'types': channel_types,
        'groups': groups
        }

    return outputs

def get_output_groups(timer_groups, param_prefix="PWM_MAIN",
        channel_labels=["PWM Main", "PWM Capture"],
        standard_params=[],
        extra_function_groups=[], pwm_timer_param=None,
        verbose=False):
    """ convert the timer groups into an output_groups section of module.yaml
    and extra timer params
    """

    instance_start = 1
    output_groups = []
    timer_params = {}
    instance_start_label = [ 1, 1 ]
    for timer_index, group_count, dshot_support in timer_groups['groups']:

        # check for capture vs normal pins for the label
        types = timer_groups['types'][instance_start-1:instance_start+group_count-1]
        if not all(types[0] == t for t in types):
            # Should this ever be needed, we can extend this script to handle that
            raise Exception('Implementation requires all channel types for a timer to be equal (types: {:})'.format(types))
        if types[0] == 'pwm':
            channel_type_idx = 0
        elif types[0] == 'cap':
            channel_type_idx = 1
        else:
            raise Exception('unsupported channel type: {:}'.format(types[0]))

        channel_label = channel_labels[channel_type_idx]
        channel_type_instance = instance_start_label[channel_type_idx]
        group_label = channel_label + ' ' + str(channel_type_instance)
        if group_count > 1:
            group_label += '-' + str(channel_type_instance+group_count-1)
        group = {
            'param_prefix': param_prefix,
            'channel_label': channel_label,
            'instance_start': instance_start,
            'instance_start_label': channel_type_instance,
            'extra_function_groups': deepcopy(extra_function_groups),
            'num_channels': group_count,
            'standard_params': deepcopy(standard_params),
            'group_label': group_label,
            'channel_label_module_name_prefix': False,
        }

        if pwm_timer_param is not None:
            pwm_timer_param_cp = deepcopy(pwm_timer_param)
            timer_param_name = param_prefix+'_TIM'+str(timer_index)

            group['config_parameters'] = [
                    {
                        'param': timer_param_name,
                        'function': 'primary',
                    }
                ]

            if dshot_support:
                # don't show pwm limit params when dshot enabled

                for standard_param in group['standard_params']:
                    group['standard_params'][standard_param]['show_if'] = timer_param_name + '>=-1'

                # indicate support for changing motor spin direction
                group['supported_actions'] = {
                        'set_spin_direction1': {
                            'supported_if': timer_param_name + '<-1',
                            'actuator_types': ['motor']
                        },
                        'set_spin_direction2': {
                            'supported_if': timer_param_name + '<-1',
                            'actuator_types': ['motor']
                        },
                    }
            else:
                # remove dshot entries if no dshot support
                values = pwm_timer_param_cp['values']
                for key in list(values.keys()):
                    if 'dshot' in values[key].lower():
                        del values[key]

            for descr_type in ['short', 'long']:
                descr = pwm_timer_param_cp['description'][descr_type]
                pwm_timer_param_cp['description'][descr_type] = \
                    descr.replace('${label}', group_label)
            timer_params[timer_param_name] = pwm_timer_param_cp
        output_groups.append(group)
        instance_start += group_count
        instance_start_label[channel_type_idx] += group_count
    return (output_groups, timer_params)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract output groups from timer_config.cpp')

    parser.add_argument('--timer-config', type=str, action='store',
                        help='timer_config.cpp file', required=True)
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                        help='Verbose Output')

    args = parser.parse_args()
    verbose = args.verbose
    timer_groups = get_timer_groups(args.timer_config, verbose)
    print('timer groups: {:}'.format(timer_groups))
    output_groups, timer_params = get_output_groups(timer_groups, verbose=verbose)
    print('output groups: {:}'.format(output_groups))
    print('timer params: {:}'.format(timer_params))
