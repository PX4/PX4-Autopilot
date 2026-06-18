import re
import gdb

# rl_tools::Matrix<rl_tools::matrix::Specification<([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*, rl_tools::matrix::layouts::RowMajorAlignment<([^,]+)\s*,\s*([^(,)]+)>\s*,\s*([^,]+)>\s>
class MatrixPrinter:
    def __init__(self, val):
        self.val = val
    def to_string(self):

        regex = re.search("rl_tools::Matrix<rl_tools::matrix::Specification<.+$")
        result = regex.match(str(self.val.type))
        if result is None:
            return f"This is a Matrix with type: {str(self.val.type)}"
        else:
            groups = result.groups()
            print(groups)
            acc = ""
            for match in groups[1:]:
                acc += str(match) + "   "
                return f"This is a Matrix: str"

# class MatrixPrinterControl(gdb.printing.PrettyPrinter):
#     def __init__(self):
#         super().__init__('Matrix printer')
#     def __call__(self, val):
#         lookup_tag = val.type.tag
#         regex = re.compile("^rl_tools::Matrix<rl_tools::matrix::Specification<.+$")
#         if regex.match(lookup_tag):
#             return MatrixPrinter(val)
#         return None

def matrix_printer_lookup(val):
    lookup_tag = val.type.tag
    if lookup_tag is None:
        return None
    regex = re.compile("^rl_tools::Matrix<rl_tools::matrix::Specification<.+$")
    if regex.match(lookup_tag):
        return MatrixPrinter(val)
    return None

# gdb.pretty_printers.append(MatrixPrinterControl())
gdb.pretty_printers = []
gdb.pretty_printers.append(matrix_printer_lookup)
