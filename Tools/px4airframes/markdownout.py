from xml.sax.saxutils import escape
import codecs
import os
import html

class MarkdownTablesOutput():
    def __init__(self, groups, board, image_path):
        result = """# Airframes Reference

:::note
**This list is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/master/Tools/px4airframes/markdownout.py) from the source code** using the build command: `make airframe_metadata`.
:::

This page lists all supported airframes and types including the motor assignment and numbering.
The motors in **green** rotate clockwise, the ones in **blue** counterclockwise.

**AUX** channels may not be present on some flight controllers.
If present, PWM AUX channels are commonly labelled **AUX OUT**.

<style>
div.frame_common table, div.frame_common table {
   display: table;
   table-layout: fixed;
   margin-bottom: 5px;
}

div.frame_common table {
   float: right;
   width: 70%;
}

div.frame_common img {
  max-height: 180px;
  width: 29%;
  padding-top: 10px;
}

div.frame_variant table {
   width: 100%;
}

div.frame_variant th:nth-child(1) {
  width: 30%;
  }

div.frame_variant tr > * {
    vertical-align : top;
}

div.frame_variant td, div.frame_variant th {
  text-align : left;
}
</style>\n\n"""
 
        type_set = set()
        
        if len(image_path) > 0 and image_path[-1] != '/':
            image_path = image_path + '/'

        for group in groups:
            if group.GetClass() not in type_set:
               result += '## %s\n\n' % group.GetClass()
               type_set.add(group.GetClass())

            result += '### %s\n\n' % group.GetName()

            # Display an image of the frame
            image_name = group.GetImageName()
            result += '<div class="frame_common">\n'
            image_name = image_path + image_name
            result += '<img src="%s.svg"/>\n' % (image_name)

            # check if all outputs are equal for the group: if so, show them
            # only once
            all_outputs = {}
            num_configs = len(group.GetParams())
            for param in group.GetParams():
                if not self.IsExcluded(param, board):
                    for output_name in param.GetOutputCodes():
                        value = param.GetOutputValue(output_name)
                        key_value_pair = (output_name, value)
                        if key_value_pair not in all_outputs:
                            all_outputs[key_value_pair] = 0
                        all_outputs[key_value_pair] += 1
            has_common_outputs = any(all_outputs[k] == num_configs for k in all_outputs)

            if has_common_outputs:
                outputs_common = ''.join(['<li><b>{:}</b>: {:}</li>'.format(k[0], k[1]) \
                    for k in all_outputs if all_outputs[k] == num_configs])
                result += '<table>\n'
                result += ' <thead>\n'
                result += '   <tr><th>Common Outputs</th></tr>\n'
                result += ' </thead>\n'
                result += ' <tbody>\n'
                result += '<tr>\n <td><ul>%s</ul></td>\n</tr>\n' % (outputs_common)
                result += '</tbody></table>\n'

            result += '</div>\n\n'

            result += '<div class="frame_variant">\n'
            result += '<table>\n'
            result += ' <thead>\n'
            result += '   <tr><th>Name</th><th></th></tr>\n'
            result += ' </thead>\n'
            result += '<tbody>\n'

            for param in group.GetParams():
                if not self.IsExcluded(param, board):
                    #print("generating: {0} {1}".format(param.GetName(), excluded))
                    name = param.GetName()
                    airframe_id = param.GetId()
                    airframe_id_entry = '<p><code>SYS_AUTOSTART</code> = %s</p>' % (airframe_id)
                    maintainer = param.GetMaintainer()
                    maintainer_entry = ''
                    if maintainer != '':
                        maintainer_entry = 'Maintainer: %s' % (html.escape(maintainer))
                    url = param.GetFieldValue('url')
                    name_anchor='%s_%s_%s' % (group.GetClass(),group.GetName(),name)
                    name_anchor=name_anchor.replace(' ','_').lower()
                    name_anchor=name_anchor.replace('"','_').lower()
                    name_anchor='id="%s"' % name_anchor
                    name_entry = name
                    if url != '':
                        name_entry = '<a href="%s">%s</a>' % (url, name)
                    outputs = '<ul>'
                    has_outputs = False
                    for output_name in param.GetOutputCodes():
                        value = param.GetOutputValue(output_name)
                        valstrs = value.split(";")
                        key_value_pair = (output_name, value)
                        if all_outputs[key_value_pair] < num_configs:
                            outputs += '<li><b>%s</b>: %s</li>' % (output_name, value)
                            has_outputs = True

                        for attrib in valstrs[1:]:
                            attribstrs = attrib.split(":")
                            # some airframes provide more info, like angle=60, direction=CCW
                            #print(output_name,value, attribstrs[0].strip(),attribstrs[1].strip())
                    outputs += '</ul>'
                    if has_outputs:
                        outputs_entry = '<p><b>Specific Outputs:</b>' + outputs + '</p>'
                    else:
                        outputs_entry = ''

                    result += ('<tr %s>\n <td>%s</td>\n <td>%s%s%s</td>\n</tr>\n' %
                        (name_anchor, name_entry, maintainer_entry, airframe_id_entry,
                        outputs_entry))


            #Close the table.
            result += '</tbody>\n</table>\n</div>\n\n'

        self.output = result

    def IsExcluded(self, param, board):
        for code in param.GetArchCodes():
            if "CONFIG_ARCH_BOARD_{0}".format(code) == board and param.GetArchValue(code) == "exclude":
                return True
        return False

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
