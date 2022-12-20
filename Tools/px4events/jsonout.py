import codecs
import json
import sys
import os


class JsonOutput():
    def __init__(self, groups):
        all_json = {}
        all_json['version'] = 2
        component = {}
        all_json['components'] = {1: component} #1: autopilot component

        all_events = {}
        component['namespace'] = "px4"
        component['event_groups'] = all_events

        for group in groups:
            current_group = {}
            current_events = {}
            current_group['events'] = current_events
            all_events[group] = current_group

            for e in groups[group]:
                event_obj = {}
                event_obj['name'] = e.name
                event_obj['message'] = e.message
                if e.type is not None:
                    event_obj['type'] = e.type
                if e.description is not None:
                    event_obj['description'] = e.description
                args = []
                for i in range(len(e.arguments)):
                    arg = {}
                    arg['type'] = e.arguments[i][0]
                    arg['name'] = e.arguments[i][1]
                    args.append(arg)
                if len(args) > 0:
                    event_obj['arguments'] = args
                sub_id = e.sub_id
                assert sub_id not in current_events, \
                        "Duplicate event ID for {0} (message: '{1}'), other event message: '{2}'".format(
                        e.name, e.message, current_events[sub_id]['message'])
                current_events[sub_id] = event_obj

        self.json = all_json

    def save(self, filename):
        need_to_write = True
        # only write if current file is not the same, to avoid updating the file
        # timestamp
        if os.path.isfile(filename):
            with open(filename, 'rb') as json_file:
                existing_data = json.load(json_file)
                if existing_data == self.json:
                    need_to_write = False
        if need_to_write:
            with codecs.open(filename, 'w', 'utf-8') as f:
                f.write(json.dumps(self.json,indent=2))
