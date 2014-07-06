#!/usr/bin/env python3

"""fsm_visualisation.py: Create dot code and dokuwiki table from a state transition table

convert dot code to png using graphviz:

dot fsm.dot -Tpng -o fsm.png
"""

import argparse
import re

__author__      = "Julian Oes"

def get_dot_header():

    return """digraph finite_state_machine {
    graph [ dpi = 300 ];
    ratio = 1.5
    node [shape = circle];"""

def get_dot_footer():

    return """}\n"""

def main():

    # parse input arguments
    parser = argparse.ArgumentParser(description='Create dot code and dokuwiki table from a state transition table.')
    parser.add_argument("-i", "--input-file", default=None, help="choose file to parse")
    parser.add_argument("-d", "--dot-file", default=None, help="choose file for output dot file")
    parser.add_argument("-t", "--table-file", default=None, help="choose file for output of table")
    args = parser.parse_args()
    
    # open source file
    if args.input_file == None:
        exit('please specify file')
    f = open(args.input_file,'r')
    source = f.read()

    # search for state transition table and extract the table itself
    #     first look for StateTable::Tran
    #     then accept anything including newline until {
    #     but don't accept the definition (without ;)
    #     then extract anything inside the brackets until };
    match = re.search(r'StateTable::Tran(?:.|\n!;)*\{((?:.|\n)*?)\};', source)

    if not match:
        exit('no state transition table found')

    table_source = match.group(1)

    # bookkeeping for error checking
    num_errors_found = 0

    states = []
    events = []

    # first get all states and events
    for table_line in table_source.split('\n'):

        match = re.search(r'/\*\s+\w+_STATE_(\w+)\s+\*/', table_line)
        if match:
            states.append(str(match.group(1)))
            # go to next line
            continue

        if len(states) == 1:
            match = re.search(r'/\*\s+EVENT_(\w+)\s+\*/', table_line)
            if match:
                events.append(str(match.group(1)))

    print('Found %d states and %d events' % (len(states), len(events)))


    # keep track of origin state
    state = None

    # fill dot code in here
    dot_code = ''

    # create table len(states)xlen(events)
    transition_table = [[[] for x in range(len(states))] for y in range(len(events))]

    # now fill the transition table and write the dot code
    for table_line in table_source.split('\n'):
        
        # get states
        #     from: /* NAV_STATE_NONE */
        #     extract only "NONE"
        match = re.search(r'/\*\s+\w+_STATE_(\w+)\s+\*/', table_line)
        if match:
            state = match.group(1)
            state_index = states.index(state)
            # go to next line
            continue

        # can't advance without proper state
        if state == None:
            continue

        # get event and next state
        #     from /* EVENT_READY_REQUESTED */      {ACTION(&Navigator::start_ready), NAV_STATE_READY}
        #     extract "READY_REQUESTED" and "READY" if there is ACTION
        match_action = re.search(r'/\*\s+EVENT_(\w+)\s+\*/\s+\{ACTION\((?:.|\n)*\w+_STATE_(\w+)', table_line)

        # get event and next state
        #     from      /* EVENT_NONE_REQUESTED */      {NO_ACTION, NAV_STATE_NONE},
        #     extract "NONE_REQUESTED" and "NAV_STATE_NONE" if there is NO_ACTION
        match_no_action = re.search(r'/\*\s+EVENT_(\w+)\s+\*/\s+\{NO_ACTION(?:.|\n)*\w+_STATE_(\w+)', table_line)
        
        # ignore lines with brackets only
        if match_action or match_no_action:
            
            # only write arrows for actions
            if match_action:                
                event = match_action.group(1)
                new_state = match_action.group(2)
                dot_code += '    ' + state + ' -> ' + new_state + '[ label = "' + event + '"];\n'

            elif match_no_action:
                event = match_no_action.group(1)
                new_state = match_no_action.group(2)

                # check for state changes without action
                if state != new_state:
                    print('Error: no action but state change:')
                    print('State: ' + state + ' changed to: ' + new_state)
                    print(table_line)
                    num_errors_found += 1

            # check for wrong events
            if event not in events:
                print('Error: unknown event: ' + event)
                print(table_line)
                num_errors_found += 1

            # check for wrong new states
            if new_state not in states:
                print('Error: unknown new state: ' + new_state)
                print(table_line)
                num_errors_found += 1

            # save new state in transition table
            event_index = events.index(event)

            # bold for action
            if match_action:
                transition_table[event_index][state_index] = '**' + new_state + '**'
            else:
                transition_table[event_index][state_index] = new_state

    
    
    # assemble dot code
    dot_code = get_dot_header() + dot_code + get_dot_footer()
    
    # write or print dot file
    if args.dot_file:
        f = open(args.dot_file,'w')
        f.write(dot_code)
        print('Wrote dot file')
    else:
        print('##########Dot-start##########')
        print(dot_code)
        print('##########Dot-end############')
    

    # assemble doku wiki table
    table_code = '| ^ '
    # start with header of all states
    for state in states:
        table_code += state + ' ^ '

    table_code += '\n'

    # add events and new states
    for event, row in zip(events, transition_table):
        table_code += '^ ' + event + ' | ' 
        for new_state in row:
            table_code += new_state + ' | '
        table_code += '\n'

    # write or print wiki table
    if args.table_file:
        f = open(args.table_file,'w')
        f.write(table_code)
        print('Wrote table file')
    else:
        print('##########Table-start########')
        print(table_code)
        print('##########Table-end##########')

    # report obvous errors
    if num_errors_found:
        print('Obvious errors found: %d' % num_errors_found)
    else:
        print('No obvious errors found')

if __name__ == '__main__':
    main()
