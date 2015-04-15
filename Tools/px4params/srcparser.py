import sys
import re

class ParameterGroup(object):
    """
    Single parameter group
    """
    def __init__(self, name):
        self.name = name
        self.params = []

    def AddParameter(self, param):
        """
        Add parameter to the group
        """
        self.params.append(param)

    def GetName(self):
        """
        Get parameter group name
        """
        return self.name

    def GetParams(self):
        """
        Returns the parsed list of parameters. Every parameter is a Parameter
        object. Note that returned object is not a copy. Modifications affect
        state of the parser.
        """
        return sorted(self.params,
                key=lambda x: x.GetFieldValue("code"))

class Parameter(object):
    """
    Single parameter
    """

    # Define sorting order of the fields
    priority = {
        "code": 10,
        "type": 9,
        "short_desc": 8,
        "long_desc": 7,
        "default": 6,
        "min": 5,
        "max": 4,
        "unit": 3,
        # all others == 0 (sorted alphabetically)
    }

    def __init__(self):
        self.fields = {}

    def SetField(self, code, value):
        """
        Set named field value
        """
        self.fields[code] = value

    def GetFieldCodes(self):
        """
        Return list of existing field codes in convenient order
        """
        keys = self.fields.keys()
        keys = sorted(keys)
        keys = sorted(keys, key=lambda x: self.priority.get(x, 0), reverse=True)
        return keys

    def GetFieldValue(self, code):
        """
        Return value of the given field code or None if not found.
        """
        return self.fields.get(code)

class SourceParser(object):
    """
    Parses provided data and stores all found parameters internally.
    """

    re_split_lines = re.compile(r'[\r\n]+')
    re_comment_start = re.compile(r'^\/\*\*')
    re_comment_content = re.compile(r'^\*\s*(.*)') 
    re_comment_tag = re.compile(r'@([a-zA-Z][a-zA-Z0-9_]*)\s*(.*)')
    re_comment_end = re.compile(r'(.*?)\s*\*\/')
    re_parameter_definition = re.compile(r'PARAM_DEFINE_([A-Z_][A-Z0-9_]*)\s*\(([A-Z_][A-Z0-9_]*)\s*,\s*([^ ,\)]+)\s*\)\s*;')
    re_cut_type_specifier = re.compile(r'[a-z]+$')
    re_is_a_number = re.compile(r'^-?[0-9\.]')
    re_remove_dots = re.compile(r'\.+$')

    valid_tags = set(["group", "min", "max", "unit"])

    # Order of parameter groups
    priority = {
        # All other groups = 0 (sort alphabetically)
        "Miscellaneous": -10
    }

    def __init__(self):
        self.param_groups = {}

    def GetSupportedExtensions(self):
        """
        Returns list of supported file extensions that can be parsed by this
        parser.
        """
        return [".cpp", ".c"]

    def Parse(self, contents):
        """
        Incrementally parse program contents and append all found parameters
        to the list.
        """
        # This code is essentially a comment-parsing grammar. "state"
        # represents parser state. It contains human-readable state
        # names.
        state = None
        for line in self.re_split_lines.split(contents):
            line = line.strip()
            # Ignore empty lines
            if line == "":
                continue
            if self.re_comment_start.match(line):
                state = "wait-short"
                short_desc = None
                long_desc = None
                tags = {}
            elif state is not None and state != "comment-processed":
                m = self.re_comment_end.search(line)
                if m:
                    line = m.group(1)
                    last_comment_line = True
                else:
                    last_comment_line = False
                m = self.re_comment_content.match(line)
                if m:
                    comment_content = m.group(1)
                    if comment_content == "":
                        # When short comment ends with empty comment line,
                        # start waiting for the next part - long comment.
                        if state == "wait-short-end":
                            state = "wait-long"
                    else:
                        m = self.re_comment_tag.match(comment_content)
                        if m:
                            tag, desc = m.group(1, 2)
                            tags[tag] = desc
                            current_tag = tag
                            state = "wait-tag-end"
                        elif state == "wait-short":
                            # Store first line of the short description
                            short_desc = comment_content
                            state = "wait-short-end"
                        elif state == "wait-short-end":
                            # Append comment line to the short description
                            short_desc += "\n" + comment_content
                        elif state == "wait-long":
                            # Store first line of the long description
                            long_desc = comment_content
                            state = "wait-long-end"
                        elif state == "wait-long-end":
                            # Append comment line to the long description
                            long_desc += "\n" + comment_content
                        elif state == "wait-tag-end":
                            # Append comment line to the tag text
                            tags[current_tag] += "\n" + comment_content
                        else:
                            raise AssertionError(
                                    "Invalid parser state: %s" % state)
                elif not last_comment_line:
                    # Invalid comment line (inside comment, but not starting with
                    # "*" or "*/". Reset parsed content.
                    state = None
                if last_comment_line:
                    state = "comment-processed"
            else:
                # Non-empty line outside the comment
                m = self.re_parameter_definition.match(line)
                if m:
                    tp, code, defval = m.group(1, 2, 3)
                    # Remove trailing type specifier from numbers: 0.1f => 0.1
                    if self.re_is_a_number.match(defval):
                        defval = self.re_cut_type_specifier.sub('', defval)
                    param = Parameter()
                    param.SetField("code", code)
                    param.SetField("short_desc", code)
                    param.SetField("type", tp)
                    param.SetField("default", defval)
                    # If comment was found before the parameter declaration,
                    # inject its data into the newly created parameter.
                    group = "Miscellaneous"
                    if state == "comment-processed":
                        if short_desc is not None:
                            param.SetField("short_desc",
                                    self.re_remove_dots.sub('', short_desc))
                        if long_desc is not None:
                            param.SetField("long_desc", long_desc)
                        for tag in tags:
                            if tag == "group":
                                group = tags[tag]
                            elif tag not in self.valid_tags:
                                sys.stderr.write("Skipping invalid "
                                        "documentation tag: '%s'\n" % tag)
                            else:
                                param.SetField(tag, tags[tag])
                    # Store the parameter
                    if group not in self.param_groups:
                        self.param_groups[group] = ParameterGroup(group)
                    self.param_groups[group].AddParameter(param)
                # Reset parsed comment.
                state = None

    def GetParamGroups(self):
        """
        Returns the parsed list of parameters. Every parameter is a Parameter
        object. Note that returned object is not a copy. Modifications affect
        state of the parser.
        """
        groups = self.param_groups.values()
        groups = sorted(groups, key=lambda x: x.GetName())
        groups = sorted(groups, key=lambda x: self.priority.get(x.GetName(), 0), reverse=True)
        return groups
