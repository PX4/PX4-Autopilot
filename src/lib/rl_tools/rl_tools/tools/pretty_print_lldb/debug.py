import lldb
import re

class RLToolsTypeDumper:
    def __init__(self, namespace, output_file):
        self.namespace = namespace
        self.output_file = output_file

    def is_type_not_function(self, type_name):
        # Check if the type name doesn't end with function-like parentheses
        return not re.search(r'\([^)]*\)$', type_name)

    def dump_types(self):
        # Get the debugger's target
        target = lldb.debugger.GetSelectedTarget()

        # Open the output file
        with open(self.output_file, 'w') as file:
            # Iterate over all modules in the target
            for module in target.module_iter():
                # Get all types in the module
                types = module.GetTypes()
                for type in types:
                    type_name = type.GetName()
                    if type_name and type_name.startswith(self.namespace) and self.is_type_not_function(type_name):
                        file.write(type_name + "\n")
                        print(f"Type matched and written to file: {type_name}")

def __lldb_init_module(debugger, internal_dict):
    # Define the namespace and output file
    namespace = "rl_tools::Tensor"
    output_file = "lldb_type_dump.txt"  # Change this path to your desired output location

    # Create an instance of RLToolsTypeDumper and dump types
    dumper = RLToolsTypeDumper(namespace, output_file)
    dumper.dump_types()

    print(f"Types in namespace '{namespace}' have been dumped to {output_file}")

# To use this script, load it in LLDB with the command:
# (lldb) command script import /path/to/rl_tools_type_dumper.py

