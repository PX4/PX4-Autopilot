from xml.sax.saxutils import escape
import codecs

class DokuWikiTablesOutput():
    def __init__(self, groups):
        result = ("====== Parameter Reference ======\n"
                  "<note>**This list is auto-generated from the source code** and contains the most recent parameter documentation.</note>\n"
                  "\n")
        for group in groups:
            result += "==== %s ====\n\n" % group.GetName()
            result += "|< 100% 25% 45% 10% 10% 10% >|\n"
            result += "^ Name ^ Description ^  Min ^  Max ^  Default ^\n"
            result += "^ :::  ^ Comment ^^^^\n"
            for param in group.GetParams():
                code = param.GetName()
                def_val = param.GetDefault()
                name = param.GetFieldValue("short_desc")
                min_val = param.GetFieldValue("min")
                max_val = param.GetFieldValue("max")
                long_desc = param.GetFieldValue("long_desc")

                if name == code:
                    name = ""
                else:
                    name = name.replace("\n", " ")
                    name = name.replace("|", "%%|%%")
                    name = name.replace("^", "%%^%%")

                result += "| **%s** |" % code
                result += " %s |" % name
                result += "  %s |" % (min_val or "")
                result += "  %s |" % (max_val or "")
                result += "  %s |" % (def_val or "")
                result += "\n"

                if long_desc is not None:
                    result += "| ::: | <div>%s</div> ||||\n" % long_desc

            result += "\n"
        self.output = result;

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
