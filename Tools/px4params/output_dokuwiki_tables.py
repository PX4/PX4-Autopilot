from xml.sax.saxutils import escape
import codecs

class DokuWikiTablesOutput():
    def __init__(self, groups):
        result = "====== Parameter Reference ======\nThis list is auto-generated every few minutes and contains the most recent parameter names and default values.\n\n"
        for group in groups:
            result += "==== %s ====\n\n" % group.GetName()
            result += "|< 100% 20% 20% 10% 10% 10% 30%>|\n"
            result += "^ Name ^ Description ^  Min ^  Max ^  Default ^ Comment ^\n"
            for param in group.GetParams():
                code = param.GetFieldValue("code")
                name = param.GetFieldValue("short_desc")
                min_val = param.GetFieldValue("min")
                max_val = param.GetFieldValue("max")
                def_val = param.GetFieldValue("default")
                long_desc = param.GetFieldValue("long_desc")

                name = name.replace("\n", " ")
                result += "| %s | %s |" % (code, name)

                if min_val is not None:
                    result += "  %s |" % min_val
                else:
                    result += " |"

                if max_val is not None:
                    result += "  %s |" % max_val
                else:
                    result += " |"

                if def_val is not None:
                    result += "  %s |" % def_val
                else:
                    result += " |"

                if long_desc is not None:
                    long_desc = long_desc.replace("\n", " ")
                    result += " %s |" % long_desc
                else:
                    result += " |"

                result += "\n"
            result += "\n"
        self.output = result;

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)

    def SaveRpc(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write("""<?xml version='1.0'?>
<methodCall>
  <methodName>wiki.putPage</methodName>
      <params>
          <param> 
            <value>
              <string>:firmware:parameters</string>
            </value>
          </param>
          <param> 
            <value>
              <string>""")
            f.write(escape(self.output))
            f.write("""</string>
            </value>
        </param>
        <param> 
            <value>
                <name>sum</name>
                <string>Updated parameters automagically from code.</string>
            </value>
        </param>
    </params>
</methodCall>""")
