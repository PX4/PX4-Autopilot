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
table {
   display: table;
   table-layout: fixed;
   margin-bottom: 5px;
}
table.common {
   float: right; 
   width: 70%;
}
table.airframes {
   width: 100%;
}
table.airframes th:nth-child(1) {
  width: 30%;
  }

tr > * {
    vertical-align : top;
}
td, th {
  text-align : left;
  }
img {
  max-height: 180px;
  width: 29%;
  padding-top: 10px;
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
            result += '<div>\n'
            image_name = image_path + image_name
            result += '<img src="%s.svg"/>\n' % (image_name)

            # check if all outputs are equal for the group: if so, show them
            # only once
            outputs_prev = ['', ''] # split into MAINx and others (AUXx)
            outputs_match = [True, True]
            for param in group.GetParams():
                if not self.IsExcluded(param, board):
                    outputs_current = ['', '']
                    for output_name in param.GetOutputCodes():
                        value = param.GetOutputValue(output_name)
                        if output_name.lower().startswith('main'):
                            idx = 0
                        else:
                            idx = 1
                        outputs_current[idx] += '<li><b>%s</b>: %s</li>' % (output_name, value)
                    for i in range(2):
                        if len(outputs_current[i]) != 0:
                            if outputs_prev[i] == '':
                                outputs_prev[i] = outputs_current[i]
                            elif outputs_current[i] != outputs_prev[i]:
                                outputs_match[i] = False

            for i in range(2):
                if len(outputs_prev[i]) == 0:
                    outputs_match[i] = False
                if not outputs_match[i]:
                    outputs_prev[i] = ''

            if outputs_match[0] or outputs_match[1]:
                result += '<table class="common">\n'
                result += ' <thead>\n'
                result += '   <tr><th>Common Outputs</th></tr>\n'
                result += ' </thead>\n'
                result += ' <tbody>\n'
                result += '<tr>\n <td><ul>%s%s</ul></td>\n</tr>\n' % (outputs_prev[0], outputs_prev[1])
                result += '</tbody></table>\n'

            result += '</div>\n\n'

            result += '<table class="generic">\n'
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
                        if output_name.lower().startswith('main'):
                            idx = 0
                        else:
                            idx = 1
                        if not outputs_match[idx]:
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
            result += '</tbody></table>\n\n'

        self.output = result

    def IsExcluded(self, param, board):
        for code in param.GetArchCodes():
            if "CONFIG_ARCH_BOARD_{0}".format(code) == board and param.GetArchValue(code) == "exclude":
                return True
        return False

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
