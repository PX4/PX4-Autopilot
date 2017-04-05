from xml.sax.saxutils import escape
import codecs

class MarkdownTablesOutput():
    def __init__(self, groups):
        result = ("# Parameter Reference\n"
                  "> **Note** **This list is auto-generated from the source code** and contains the most recent parameter documentation.\n"
                  "\n")
        for group in groups:
            result += '## %s\n\n' % group.GetName()
            result += '<table style="width: 100%;">\n'
            result += '<colgroup><col style="width: 25%"><col style="width: 45%"><col style="width: 10%"><col style="width: 10%"><col style="width: 10%"></colgroup>\n'
            result += '<thead><tr><th class="col0" rowspan="2">Name</th><th class="col1">Description</th><th class="col2 rightalign">Min</th><th class="col3 rightalign">Max</th><th class="col4 rightalign">Default</th></tr>\n'
            result += '</thead>\n'
            result += '<tbody>\n'

            for param in group.GetParams():
                code = param.GetName()
                def_val = param.GetDefault() or ''
                name = param.GetFieldValue("short_desc") or ''
                min_val = param.GetFieldValue("min") or ''
                max_val = param.GetFieldValue("max") or ''
                long_desc = param.GetFieldValue("long_desc") or ''
                if long_desc is not '':
                    long_desc = '<p><strong>Comment:</strong> %s</p>' % long_desc

                if name == code:
                    name = ""
                code='<strong id="%s">%s</strong>' % (code, code)
                
                result += '<tr><td class="col0">%s</td><td class="col1"><p>%s</p>%s</td><td class="col2 rightalign">%s</td><td class="col3 rightalign">%s</td><td class="col4 rightalign">%s</td></tr>\n' % (code,name, long_desc, min_val,max_val,def_val)

            #Close the table.
            result += '</tbody></table>\n\n'

        self.output = result;

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
