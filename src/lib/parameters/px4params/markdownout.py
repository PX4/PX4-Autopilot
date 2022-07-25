from xml.sax.saxutils import escape
import codecs

class MarkdownTablesOutput():
    def __init__(self, groups):
        result = (
"""# Parameter Reference

:::note
This documentation was auto-generated from the source code for this PX4 version (using `make parameters_metadata`).
:::

:::tip
If a listed parameter is missing from the Firmware see: [Finding/Updating Parameters](../advanced_config/parameters.md#parameter-not-in-firmware).
:::

<!-- markdown generator: src/lib/parameters/px4params/markdownout.py -->

<style>
tr > * {
    vertical-align : top;
}
td:nth-child(1),td:nth-child(2) {
  text-align : left;
  }
table {
  width: fit-content;
}
</style>

"""
                  )

        for group in groups:
            result += '## %s\n\n' % group.name
            result += (
"""<table>
 <colgroup><col style="width: 23%"><col style="width: 46%"><col style="width: 11%"><col style="width: 11%"><col style="width: 9%"></colgroup>
 <thead>
   <tr><th>Name</th><th>Description</th><th>[Min, Max] (Incr.)</th><th>Default</th><th>Units</th></tr>
 </thead>
<tbody>
"""
            )
            for param in group.parameters:
                code = param.name
                name = param.fields.get("short_desc", "")
                long_desc = param.fields.get("long_desc", "")
                min_val = param.fields.get("min", "")
                max_val = param.fields.get("max", "")
                increment = param.fields.get("increment", "")
                def_val = param.default
                unit = param.fields.get("unit", "")
                type = param.type
                is_boolean = param.boolean
                reboot_required = param.fields.get("reboot_required", "")
                #board = param.GetFieldValue("board") or '' ## Disabled as no board values are defined in any parameters!
                #decimal = param.GetFieldValue("decimal") or '' #Disabled as is intended for GCS not people
                #field_codes = param.GetFieldCodes() ## Disabled as not needed for display.
                #boolean = param.GetFieldValue("boolean") # or '' # Disabled - does not appear useful.


                # Format values for display.
                # Display min/max/increment value based on what values are defined.
                max_min_combined = ''
                if min_val or max_val:
                    if not min_val:
                        min_val='?'
                    if not max_val:
                        max_val='?'
                    max_min_combined+='[%s, %s] ' % (min_val, max_val)
                if increment:
                    max_min_combined+='(%s)' % increment

                if long_desc != '':
                    long_desc = '<p><strong>Comment:</strong> %s</p>' % long_desc

                if name == code:
                    name = ""
                code='<strong id="%s">%s</strong>' % (code, code)

                if reboot_required:
                    reboot_required='<p><b>Reboot required:</b> %s</p>\n' % reboot_required

                enum_codes=param.enum.keys() or '' # Gets numerical values for parameter.
                enum_output=''
                # Format codes and their descriptions for display.
                if enum_codes:
                    enum_output+='<strong>Values:</strong><ul>'
                    enum_codes=sorted(enum_codes,key=float)
                    for item in enum_codes:
                        enum_output+='\n<li><strong>%s:</strong> %s</li> \n' % (item, param.enum[item])
                    enum_output+='</ul>\n'


                bitmask_list=param.bitmask.keys() #Gets bitmask values for parameter
                bitmask_output=''
                #Format bitmask values
                if bitmask_list:
                    bitmask_output+='<strong>Bitmask:</strong><ul>'
                    for bit in bitmask_list:
                        bit_text = param.bitmask[bit]
                        bitmask_output+='  <li><strong>%s:</strong> %s</li> \n' % (bit, bit_text)
                    bitmask_output+='</ul>\n'

                if is_boolean and def_val=='1':
                    def_val='Enabled (1)'
                if is_boolean and def_val=='0':
                    def_val='Disabled (0)'

                result += '<tr>\n <td>%s (%s)</td>\n <td>%s %s %s %s %s</td>\n <td>%s</td>\n <td>%s</td>\n <td>%s</td>\n</tr>\n' % (code, type, name, long_desc, enum_output, bitmask_output, reboot_required, max_min_combined, def_val, unit)

            #Close the table.
            result += '</tbody></table>\n\n'

        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
