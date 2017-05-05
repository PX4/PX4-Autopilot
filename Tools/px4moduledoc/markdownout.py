from xml.sax.saxutils import escape
import codecs
import os

class MarkdownTablesOutput():
    def __init__(self, module_groups):

        self._outputs = {}

        result = """
# Modules & Commands Reference
The following pages document the PX4 modules, drivers and commands. They
describe the provided functionality, high-level implementation overview and how
to use the command-line interface.

> **Note** **This is auto-generated from the source code** and contains the
> most recent modules documentation.

It is not a complete list and NuttX provides some additional commands
as well (such as `free`). Use `help` on the console to get a list of all
available commands, and in most cases `command help` will print the usage.

Since this is generated from source, errors must be reported/fixed
in the [Firmware](https://github.com/PX4/Firmware) repository.

## Categories
"""
        for category in sorted(module_groups):
            result += "- [%s](modules_%s.md)\n" % (category.capitalize(), category)

        self._outputs['main'] = result


        for category in sorted(module_groups):
            result = "# Modules Reference: %s\n" % category.capitalize()
            module_list = module_groups[category]
            for module in module_list:
                result += "## %s\n" % module.name()
                result += "Source: [%s](https://github.com/PX4/Firmware/tree/master/src/%s)\n\n" % (module.scope(), module.scope())
                doc = module.documentation()
                if len(doc) > 0:
                    result += "%s\n" % doc
                usage_string = module.usage_string()
                if len(usage_string) > 0:
                    result += "### Usage\n```\n%s\n```\n" % usage_string

            self._outputs[category] = result

    def Save(self, dirname):
        for output_name in self._outputs:
            output = self._outputs[output_name]
            with codecs.open(os.path.join(dirname, 'modules_'+output_name+'.md'), 'w', 'utf-8') as f:
                f.write(output)
