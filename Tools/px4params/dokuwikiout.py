import output

class DokuWikiOutput(output.Output):
    def Generate(self, groups):
        result = ""
        for group in groups:
            result += "==== %s ====\n\n" % group.GetName()
            result += "^ Name            ^ Description    ^ Min    ^ Max    ^ Default   ^ Comment    ^\n"
            for param in group.GetParams():
                code = param.GetFieldValue("code")
                name = param.GetFieldValue("short_desc")
                name = name.replace("\n", "")
                result += "| %s   | %s  " % (code, name)
                min_val = param.GetFieldValue("min")
                if min_val is not None:
                    result += "| %s  " % min_val
                else:
                    result += "|"
                max_val = param.GetFieldValue("max")
                if max_val is not None:
                    result += "| %s  " % max_val
                else:
                    result += "|"
                def_val = param.GetFieldValue("default")
                if def_val is not None:
                    result += "| %s  " % def_val
                else:
                    result += "|"
                long_desc = param.GetFieldValue("long_desc")
                if long_desc is not None:
                    long_desc = long_desc.replace("\n", "")
                    result += "| %s  " % long_desc
                else:
                    result += "|"
                result += "|\n"
            result += "\n"
        return result
