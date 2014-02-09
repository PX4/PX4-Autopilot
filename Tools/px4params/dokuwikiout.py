import output
from xml.sax.saxutils import escape

class DokuWikiOutput(output.Output):
    def Generate(self, groups):
        pre_text = """<?xml version='1.0'?>
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
                  <string>"""
        result = "====== Parameter Reference ======\nThis list is auto-generated every few minutes and contains the most recent parameter names and default values."
        for group in groups:
            result += "==== %s ====\n\n" % group.GetName()
            result += "|< 100% 20% 20% 10% 10% 10% 30%>|\n"
            result += "^ Name            ^ Description    ^ Min    ^ Max    ^ Default   ^ Comment    ^\n"
            for param in group.GetParams():
                code = param.GetFieldValue("code")
                name = param.GetFieldValue("short_desc")
                name = name.replace("\n", "")
                result += "| %s   | %s  " % (code, name)
                min_val = param.GetFieldValue("min")
                if min_val is not None:
                    result += " | %s  " % min_val
                else:
                    result += " | "
                max_val = param.GetFieldValue("max")
                if max_val is not None:
                    result += " | %s  " % max_val
                else:
                    result += " | "
                def_val = param.GetFieldValue("default")
                if def_val is not None:
                    result += "| %s  " % def_val
                else:
                    result += " | "
                long_desc = param.GetFieldValue("long_desc")
                if long_desc is not None:
                    long_desc = long_desc.replace("\n", "")
                    result += "| %s  " % long_desc
                else:
                    result += " | "
                result += " |\n"
            result += "\n"
        post_text = """</string>
        </value>
      </param> 
           <param> 
        <value>
            <name>sum</name>
            <string>Updated parameters automagically from code.</string>
        </value>
      </param> 
    </params> 
    </methodCall>"""
        return pre_text + escape(result) + post_text
