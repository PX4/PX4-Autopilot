import os
import re

class Scope(object):
    """
    Single parameter group
    """
    re_deep_lines = re.compile(r'.*\/.*\/')
    def __init__(self,):
        self.scope = set()


    def __str__(self):
        return self.scope.__str__()

    def Add(self, scope):
        """
        Add Scope to set
        """
        self.scope.add(scope)

    def Has(self, scope):
        """
        Check for existance
        """
        if len(self.scope) == 0:
            return True
        # Anything in the form xxxxx/yyyyy/zzzzz....
        # is treated as xxxxx/yyyyy
        while (self.re_deep_lines.match(scope)):
            scope = os.path.dirname(scope)
        return scope in self.scope
