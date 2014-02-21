import codecs

class DokuWikiListingsOutput():
    def __init__(self, groups):
        result = ""
        for group in groups:
            result += "==== %s ====\n\n" % group.GetName()
            for param in group.GetParams():
                code = param.GetFieldValue("code")
                name = param.GetFieldValue("short_desc")
                if code != name:
                    name = "%s (%s)" % (name, code)
                result += "=== %s ===\n\n" % name
                long_desc = param.GetFieldValue("long_desc")
                if long_desc is not None:
                    result += "%s\n\n" % long_desc
                min_val = param.GetFieldValue("min")
                if min_val is not None:
                    result += "* Minimal value: %s\n" % min_val
                max_val = param.GetFieldValue("max")
                if max_val is not None:
                    result += "* Maximal value: %s\n" % max_val
                def_val = param.GetFieldValue("default")
                if def_val is not None:
                    result += "* Default value: %s\n" % def_val
                result += "\n"
        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
