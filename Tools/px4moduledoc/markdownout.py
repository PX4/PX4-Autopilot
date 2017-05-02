from xml.sax.saxutils import escape
import codecs
import os

class MarkdownTablesOutput():
    def __init__(self, module_groups):

        result = """
# Modules & Commands Reference
Documentation of PX4 modules, drivers and commands. It describes the provided
functionality, high-level implementation overview and how to use the
command-line interface.

> **Note** **This list is auto-generated from the source code** and contains the
> most recent modules documentation.

This is not a complete list and NuttX provides some additional commands
as well (such as `free`). Use `help` on the console to get a list of all
available commands, and in most cases `command help` will print the usage.

Since this is generated from source, errors must be reported/fixed
in the [Firmware](https://github.com/PX4/Firmware) repository.

"""

        for category in sorted(module_groups):
            result += "# Category: %s\n" % category.capitalize()
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

        self.output = result

    def Save(self, dirname):
        with codecs.open(os.path.join(dirname, 'modules.md'), 'w', 'utf-8') as f:
            f.write(self.output)
