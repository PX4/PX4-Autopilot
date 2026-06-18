import re
import lldb
import json


# workflow
# lldb cmake-build-debug/tests/test_rl_algorithms_td3_second_stage_mlp
# breakpoint set -f include/rl_tools/containers/operations_generic.h -l 125
# run
# type summary clear
# command script import tools/pretty_print_lldb/matrix.py
# type summary add -F matrix.pretty_print_row_major_alignment -x "^rl_tools::Matrix<rl_tools::matrix::Specification<"
# p m1


def decode_row_major(valobj):
    regex = r"^\s*(?:const|\s*)\s*rl_tools\s*::\s*Matrix\s*<\s*rl_tools\s*::\s*matrix\s*::\s*Specification\s*<\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*rl_tools\s*::\s*matrix\s*::\s*layouts\s*::\s*RowMajorAlignment\s*<\s*([^,]+)\s*,\s*([^,]+)\s*>\s*,\s*([^,]+)\s*>\s*>\s*(&|\s*)\s*$"
    typename = valobj.GetType().GetCanonicalType().GetName()
    result = re.match(regex, typename)
    if result is None:
        return None
    else:
        meta = dict(zip(["T", "TI", "ROWS", "COLS", "TI2", "DYNAMIC_ALLOCATION", "ROW_MAJOR_ALIGNMENT", "CONST"], result.groups()))
        meta["ROWS"] = int(meta["ROWS"])
        meta["COLS"] = int(meta["COLS"])
        meta["ROW_MAJOR_ALIGNMENT"] = int(meta["ROW_MAJOR_ALIGNMENT"])
        ALIGNMENT = meta["ROW_MAJOR_ALIGNMENT"]
        meta["ROW_PITCH"] = ((meta["COLS"] + ALIGNMENT - 1) // ALIGNMENT) * ALIGNMENT
        return meta

def decode_fixed(valobj):
    regex = r"^\s*(?:const|\s*)\s*rl_tools\s*::\s*Matrix\s*<\s*rl_tools\s*::\s*matrix\s*::\s*Specification\s*<\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*rl_tools\s*::\s*matrix\s*::\s*layouts\s*::\s*Fixed\s*<\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*>\s*,\s*([^,]+)\s*>\s*>\s*(&|\s*)\s*$"
    typename = valobj.GetType().GetCanonicalType().GetName()
    result = re.match(regex, typename)
    if result is None:
        return None
    else:
        meta = dict(zip(["T", "TI", "ROWS", "COLS", "TI2", "DYNAMIC_ALLOCATION", "ROW_PITCH", "COL_PITCH", "CONST"], result.groups()))
        meta["ROWS"] = int(meta["ROWS"])
        meta["COLS"] = int(meta["COLS"])
        meta["ROW_PITCH"] = int(meta["ROW_PITCH"])
        meta["COL_PITCH"] = int(meta["COL_PITCH"])
        return meta

def pretty_print_row_major_alignment(valobj, internal_dict, options):
    # regex = r"^\s*(?:const|\s*)\s*rl_tools\s*::\s*Matrix\s*<\s*rl_tools\s*::\s*matrix\s*::\s*Specification\s*<\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*rl_tools\s*::\s*matrix\s*::\s*layouts\s*::\s*RowMajorAlignment\s*<\s*([^,]+)\s*,\s*([^,]+)\s*>\s*,\s*([^,]+)\s*>\s*>\s*$"
    float_ptr = valobj.GetChildMemberWithName("_data")
    float_type = float_ptr.GetType().GetPointeeType()
    target = valobj.GetTarget()

    meta = decode_row_major(valobj)
    if meta is None:
        return f"Matrix type could not be parsed: {valobj.type.name}"
    else:
        acc = f"{json.dumps(meta)}\n"
        for row_i in range(meta["ROWS"]):
            for col_i in range(meta["COLS"]):
                pos = row_i * meta["ROW_PITCH"] + col_i
                offset = float_ptr.GetValueAsUnsigned() + pos * float_type.GetByteSize()
                val_wrapper = target.CreateValueFromAddress("temp", lldb.SBAddress(offset, target), float_type)
                val = val_wrapper.GetValue()
                acc += str(val) + ", "
            acc += "\n"


        return f"Matrix type: {acc}"
        # return float_ptr.Dereference()

def pretty_print_fixed_alignment(valobj, internal_dict, options):
    # regex = r"^\s*(?:const|\s*)\s*rl_tools\s*::\s*Matrix\s*<\s*rl_tools\s*::\s*matrix\s*::\s*Specification\s*<\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*rl_tools\s*::\s*matrix\s*::\s*layouts\s*::\s*RowMajorAlignment\s*<\s*([^,]+)\s*,\s*([^,]+)\s*>\s*,\s*([^,]+)\s*>\s*>\s*$"
    float_ptr = valobj.GetChildMemberWithName("_data")
    float_type = float_ptr.GetType().GetPointeeType()
    target = valobj.GetTarget()

    meta = decode_fixed(valobj)
    if meta is None:
        return f"Matrix type could not be parsed: {valobj.type.name}"
    else:
        acc = f"{json.dumps(meta)}\n"
        for row_i in range(meta["ROWS"]):
            for col_i in range(meta["COLS"]):
                pos = row_i * meta["ROW_PITCH"] + col_i * meta["COL_PITCH"]
                offset = float_ptr.GetValueAsUnsigned() + pos * float_type.GetByteSize()
                val_wrapper = target.CreateValueFromAddress("temp", lldb.SBAddress(offset, target), float_type)
                val = val_wrapper.GetValue()
                acc += str(val) + ", "
            acc += "\n"

        return f"Matrix type: {acc}"
        # return float_ptr.Dereference()
class PrettyPrintRowMajorAlignment:
    def __init__(self, valobj, internal_dict):
        self.meta = decode_row_major(valobj)
        self.val = valobj
        self.float_ptr = valobj.GetChildMemberWithName("_data")
        self.float_type = self.float_ptr.GetType().GetPointeeType()
        self.target = valobj.GetTarget()
    def num_children(self):
        return self.meta["ROWS"] if self.meta is not None else 0
    def get_child_index(self,name):
        return -1;
    def get_child_at_index(self, row_i):
        if self.meta is None:
            return f"Matrix type could not be parsed {self.val.type.name}"
        else:
            col_i = 0
            pos = row_i * self.meta["ROW_PITCH"] + col_i
            offset = pos * self.float_type.GetByteSize()
            # val_wrapper = self.target.CreateValueFromAddress("temp", lldb.SBAddress(offset, self.target), self.float_type)
            # val = val_wrapper.GetValue()
            return self.float_ptr.Dereference().CreateChildAtOffset('[' + str(row_i) + ']', offset, self.float_type)
            # return val;

    def update(self):
        pass
    def has_children(self):
        return True
    def get_value(self):
        return None