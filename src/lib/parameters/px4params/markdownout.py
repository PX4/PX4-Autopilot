from xml.sax.saxutils import escape
import codecs
import html

class MarkdownTablesOutput():
    def __init__(self, groups):
        result = (
"""# Parameter Reference

::: info
This documentation was auto-generated from the source code for this PX4 version (using `make parameters_metadata`).
:::

::: tip
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
            result += '## %s\n\n' % group.GetName()
            result += (
"""<table>
 <colgroup><col style="width: 23%"><col style="width: 46%"><col style="width: 11%"><col style="width: 11%"><col style="width: 9%"></colgroup>
 <thead>
   <tr><th>Name</th><th>Description</th><th>[Min, Max] (Incr.)</th><th>Default</th><th>Units</th></tr>
 </thead>
<tbody>
"""
            )
            for param in group.GetParams():
                code = param.GetName()
                name = param.GetFieldValue("short_desc") or ''
                name = html.escape(name)
                long_desc = param.GetFieldValue("long_desc") or ''
                long_desc = html.escape(long_desc)
                min_val = param.GetFieldValue("min") or ''
                max_val = param.GetFieldValue("max") or ''
                increment = param.GetFieldValue("increment") or ''
                def_val = param.GetDefault() or ''
                unit = param.GetFieldValue("unit") or ''
                type = param.GetType()
                is_boolean = param.GetBoolean()
                reboot_required = param.GetFieldValue("reboot_required") or ''
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
                    max_min_combined+=f"[{min_val}, {max_val}] "
                if increment:
                    max_min_combined+='(%s)' % increment

                if long_desc != '':
                    long_desc = f"<p><strong>Comment:</strong> {long_desc}</p>"

                if name == code:
                    name = ""
                code=f"<strong id=\"{code}\">{code}</strong>"

                if reboot_required:
                    reboot_required='<p><b>Reboot required:</b> %s</p>\n' % reboot_required

                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                enum_output=''
                # Format codes and their descriptions for display.
                if enum_codes:
                    enum_output+='<strong>Values:</strong><ul>'
                    enum_codes=sorted(enum_codes,key=float)
                    for item in enum_codes:
                        enum_output+=f"\n<li><strong>{item}:</strong> {html.escape(param.GetEnumValue(item))}</li>"
                    enum_output+='\n</ul>'


                bitmask_list=param.GetBitmaskList() #Gets bitmask values for parameter
                bitmask_output=''
                #Format bitmask values
                if bitmask_list:
                    bitmask_output+='<strong>Bitmask:</strong><ul>'
                    for bit in bitmask_list:
                        bit_text = param.GetBitmaskBit(bit)
                        bit_text = html.escape(bit_text)
                        bitmask_output+=f"  <li><strong>{bit}:</strong> {bit_text}</li>\n"
                    bitmask_output+='</ul>\n'

                if is_boolean and def_val=='1':
                    def_val='Enabled (1)'
                if is_boolean and def_val=='0':
                    def_val='Disabled (0)'

                result += f"<tr>\n <td>{code} ({type})</td>\n <td>{name} {long_desc} {enum_output} {bitmask_output} {reboot_required}</td>\n <td>{max_min_combined}</td>\n <td>{def_val}</td>\n <td>{unit}</td>\n</tr>\n"

            #Close the table.
            result += '</tbody></table>\n\n'

        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
