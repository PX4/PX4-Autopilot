import re
import codecs
import sys

class CMakeParser(object):
    """
    Parses provided data and stores all found paths in scope.
    """
    re_split_lines = re.compile(r'[\r\n]+')
    re_comment = re.compile(r'^\#')
    re_start = re.compile(r'set\s*\(\s*config_module_list')
    re_end = re.compile(r'\)\s*')

    def Parse(self, scope, contents):
        """
        Incrementally parse cmake file contents and append all found path scope
        to scope.
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
            if self.re_comment.match(line):
                continue
            elif self.re_start.match(line):
                state = "gather"
                continue
            elif state is not None and state == "gather":
                if self.re_end.match(line):
                    return True
                scope.Add(line)
        return False
