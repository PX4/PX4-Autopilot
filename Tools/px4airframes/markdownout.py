from xml.sax.saxutils import escape
import codecs

class MarkdownTablesOutput():
    def __init__(self, groups, board):
        result = ("# Airframes Reference\n"
                  "> **Note** **This list is auto-generated from the source code** and contains the most recent airframes documentation.\n"
                  "\n")

        # TODO: describe meaning of green + blue color...

        for group in groups:

            result += '## %s\n\n' % group.GetName()

            # Display an image of the frame
            image_name = group.GetImageName()
            if image_name != 'AirframeUnknown':
                result += '<img src="images/airframes/types/%s.svg" width="25%%" style="max-height: 150px;"/>\n' % (image_name)

            result += '<table style="width: 100%; table-layout:fixed; font-size:1.5rem;">\n'
            result += ' <colgroup><col style="width: 30%"><col style="width: 30%"><col style="width: 40%"></colgroup>\n'
            result += ' <thead>\n'
            result += '   <tr><th>Name</th><th></th><th>Outputs</th></tr>\n'
            result += ' </thead>\n'
            result += '<tbody>\n'

            for param in group.GetParams():

                # check if there is an exclude tag for this airframe
                excluded = False
                for code in param.GetArchCodes():
                    if "CONFIG_ARCH_BOARD_{0}".format(code) == board and param.GetArchValue(code) == "exclude":
                        excluded = True

                if not excluded:
                    #print("generating: {0} {1}".format(param.GetName(), excluded))
                    name = param.GetName()
                    airframe_id = param.GetId()
                    airframe_id_entry = '<p><code>SYS_AUTOSTART</code> = %s</p>' % (airframe_id)
                    maintainer = param.GetMaintainer()
                    maintainer_entry = ''
                    if maintainer != '':
                        maintainer_entry = '<p>Maintainer: %s</p>' % (maintainer)
                    url = param.GetFieldValue('url')
                    name_entry = name
                    if url != '':
                        name_entry = '<a href="%s">%s</a>' % (url, name)
                    outputs = '<ul>'
                    for code in param.GetOutputCodes():
                        value = param.GetOutputValue(code)
                        valstrs = value.split(";")
                        output_name = code
                        outputs += '<li><b>%s</b>: %s</li>' % (output_name, value)
                        for attrib in valstrs[1:]:
                            attribstrs = attrib.split(":")
                            # some airframes provide more info, like angle=60, direction=CCW
                            #print(output_name,value, attribstrs[0].strip(),attribstrs[1].strip())
                    outputs += '</ul>'

                    result += '<tr>\n <td style="vertical-align: top;">%s</td>\n <td style="vertical-align: top;">%s%s</td>\n <td style="vertical-align: top;">%s</td>\n</tr>\n' % (name_entry, maintainer_entry, airframe_id_entry, outputs)


            #Close the table.
            result += '</tbody></table>\n\n'

        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
