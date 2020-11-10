import sys
import re
import math

def hash_32_fnv1a(data: str):
    hash_val = 0x811c9dc5
    prime = 0x1000193
    for i in range(len(data)):
        value = ord(data[i])
        hash_val = hash_val ^ value
        hash_val *= prime
        hash_val &= 0xffffffff
    return hash_val


class Event(object):
    """
    Single event definition
    """

    def __init__(self):
        self.name = None
        self.message = None
        self.description = None
        self.group = "default"
        self._arguments = []

    @staticmethod
    def _get_id(name):
        return 0xffffff & hash_32_fnv1a(name)

    @property
    def arguments(self):
        """ list of (type: str, name: str) tuples """
        return self._arguments

    def set_default_arguments(self, num_args):
        """ set argument names to default (if not specified) """
        for i in range(num_args):
            self.add_argument(None, "arg"+str(i))

    def _shift_printed_arguments(self, msg, offset):
        """ shift all {<idx> arguments by an offset """
        i = 0
        while i < len(msg):

            if msg[i] == '\\': # escaped character
                i += 2
                continue

            if msg[i] == '{':
                m = re.match(r"^(\d+)", msg[i+1:])
                if m:
                    arg_idx = int(m.group(1)) + offset
                    msg = msg[:i+1] + str(arg_idx) + msg[i+1+len(m.group(1)):]
            i += 1
        return msg

    def prepend_arguments(self, arguments):
        """ prepend additional arguments, and shift all '{<index>}' in the
            description and message
            :param arguments: list of (type: str, name: str) tuples
        """
        self._arguments = arguments + self._arguments
        num_added = len(arguments)
        if self.message is not None:
            self.message = self._shift_printed_arguments(self.message, num_added)
        if self.description is not None:
            self.description = self._shift_printed_arguments(self.description, num_added)

    def add_argument(self, arg_type, name):
        self._arguments.append((arg_type, name))

    @property
    def sub_id(self):
        return self._get_id(self.name)

    def validate(self):
        if self.name is None: raise Exception("missing event name")
        if self.message is None: raise Exception("missing event message for {}".format(self.name))
        # just to ensure a common convention
        assert self.message[-1] != '.', "Avoid event message ending in '.' ({:})".format(self.message)
        # description is optional

class SourceParser(object):
    """
    Parses provided data and stores all found events internally.
    """

    re_split_lines = re.compile(r'[\r\n]+')
    re_comment_start = re.compile(r'^\/\*\s*EVENT$')
    re_comment_content = re.compile(r'^\*\s*(.*)')
    re_comment_tag = re.compile(r'^@([a-zA-Z][a-zA-Z0-9_]*):?\s*(.*)')
    re_comment_end = re.compile(r'(.*?)\s*\*\/$')
    re_code_end = re.compile(r'(.*?)\s*;$')
    re_template_args = re.compile(r'([a-zA-Z0-9_:\.]+)\s*<([a-zA-Z0-9_,\s:]+)\s*>\s*\((.*)\);$')
    re_no_template_args = re.compile(r'([a-zA-Z0-9_:\.]+)\s*\((.*)\);$')
    re_event_id = re.compile(r'(events::)?ID\("([a-zA-Z0-9_]+)\"')

    def __init__(self):
        self._events = {}

    @property
    def events(self):
        """ dict of 'group': [Event] list """
        return self._events

    def Parse(self, contents):
        """
        Incrementally parse program contents and append all found events
        to the list.
        """
        # This code is essentially a comment-parsing grammar. "state"
        # represents parser state. It contains human-readable state
        # names.
        state = None
        def finalize_current_tag(event, tag, value):
            if tag is None: return
            if tag == "description":
                descr = value.strip()
                # merge continued lines (but not e.g. enumerations)
                for i in range(1, len(descr)-1):
                    if descr[i-1] != '\n' and descr[i] == '\n' and descr[i+1].isalpha():
                        descr = descr[:i] + ' ' + descr[i+1:]
                event.description = descr
            elif tag == "group":
                known_groups = ["calibration", "health", "arming_check", "normal"]
                event.group = value.strip()
                if not event.group in known_groups:
                    raise Exception("Unknown event group: '{}'\nKnown groups: {}\n" \
                        "If this is not a typo, add the new group to the script".format(event.group, known_groups))
            elif tag.startswith("arg"):
                arg_index = int(tag[3:])-1
                arg_name = value.strip()
                assert len(event.arguments) == arg_index, "Invalid argument ordering/duplicate ({}, {})".format(tag, value)
                event.add_argument(None, arg_name)
            else:
                raise Exception("Invalid tag: {}\nvalue: {}".format(tag, value))

        for line in self.re_split_lines.split(contents):
            line = line.strip()
            # Ignore empty lines
            if line == "":
                continue
            if self.re_comment_start.match(line):
                state = "parse-comments"
                event = Event()
                current_tag = None
                current_value = None
                current_code = ""
                continue
            if state is None:
                continue
            if state == "parse-command":
                current_code += line
                m = self.re_code_end.search(line)
                if m:
                    # extract template arguments
                    m = self.re_template_args.search(current_code)
                    if m:
                        call, template_args, args = m.group(1, 2, 3)
                        template_args = template_args.split(',')
                    else:
                        m = self.re_no_template_args.search(current_code)
                        if m:
                            template_args = []
                            call, args = m.group(1, 2)
                        else:
                            raise Exception("Failed to parse code line {:}".format(current_code))

                    # if event arguments are not specified, use default naming
                    if len(event.arguments) == 0:
                        event.set_default_arguments(len(template_args))

                    # get argument types from template arguments
                    assert len(template_args) == len(event.arguments), \
                        "Number of arguments mismatch (args: {:})".format(template_args)
                    num_args = len(template_args)
                    for i in range(num_args):
                        arg_name = event.arguments[i][1]
                        arg_type = template_args[i].strip()
                        if arg_type.startswith('events::'):
                            arg_type = arg_type[8:]
                        arg_type = arg_type.replace('enums::', '')
                        event.arguments[i] = (arg_type, arg_name)
                    #print("method: {}, args: {}, template args: {}".format(call, args, event.arguments))

                    # extract function arguments
                    args_split = self._parse_arguments(args)
                    if call == "events::send" or call == "send":
                        assert len(args_split) == num_args + 3, \
                            "Unexpected Number of arguments for: {:}, {:}".format(args_split, num_args)
                        m = self.re_event_id.search(args_split[0])
                        if m:
                            _, event_name = m.group(1, 2)
                        else:
                            raise Exception("Could not extract event ID from {:}".format(args_split[0]))
                        event.name = event_name
                        event.message = args_split[2][1:-1]
                    elif call in ['reporter.healthFailure', 'reporter.armingCheckFailure']:
                        assert len(args_split) == num_args + 5, \
                            "Unexpected Number of arguments for: {:}, {:}".format(args_split, num_args)
                        m = self.re_event_id.search(args_split[2])
                        if m:
                            _, event_name = m.group(1, 2)
                        else:
                            raise Exception("Could not extract event ID from {:}".format(args_split[2]))
                        event.name = event_name
                        event.message = args_split[4][1:-1]
                        if 'health' in call:
                            event.group = "health"
                        else:
                            event.group = "arming_check"
                        event.prepend_arguments([('common::navigation_mode_category_t', 'modes'),
                                ('uint8_t', 'health_component_index')])
                    else:
                        raise Exception("unknown event method call: {}, args: {}".format(call, args))

                    event.validate()

                    # insert
                    if not event.group in self._events:
                        self._events[event.group] = []
                    self._events[event.group].append(event)

                    state = None

            else:
                m = self.re_comment_end.search(line)
                if m:
                    line = m.group(1)
                    last_comment_line = True
                else:
                    last_comment_line = False
                m = self.re_comment_content.match(line)
                if m:
                    comment_content = m.group(1)
                    m = self.re_comment_tag.match(comment_content)
                    if m:
                        finalize_current_tag(event, current_tag, current_value)
                        current_tag, current_value = m.group(1, 2)
                    elif current_tag is not None:
                        current_value += "\n"+comment_content
                    # else: empty line before any tag
                elif not last_comment_line:
                    # Invalid comment line (inside comment, but not starting with
                    # "*" or "*/".
                    raise Exception("Excpected a comment, got '{}'".format(line))
                if last_comment_line:
                    finalize_current_tag(event, current_tag, current_value)
                    state = "parse-command"
        return True

    def _parse_arguments(self, args):
        """
        given a string of arguments, returns a list of strings split into the
        arguments, with respecting brackets.
        args is expected to be a single line.
        Note: comments are not handled, also template arguments.

        e.g. "32, test(4,4), \"e(c\", ab" -> ["32", "test(4,4)", "\"e(c\"", "ab"]
        """
        args_split = []
        start = 0
        bracket = 0
        in_string = False
        for i in range(len(args)):
            if in_string and args[i] == "\"" and args[i-1] != "\\":
                in_string = False
            elif not in_string and args[i] == "\"":
                in_string = True
            if in_string:
                continue
            if args[i] in "{([":
                bracket += 1
            if args[i] in "})]":
                bracket -= 1
            if bracket == 0 and args[i] == ',':
                args_split.append(args[start:i].strip())
                start = i + 1
        args_split.append(args[start:].strip())
        return args_split
