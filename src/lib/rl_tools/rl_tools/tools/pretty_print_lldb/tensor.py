import re
import lldb
import json
import os

from parse_tensor import parse_string


def pad_number(number, length):
    return str(number).rjust(length)

render_outer_limit = os.environ["RL_TOOLS_LLDB_RENDER_OUTER_LIMIT"] if "RL_TOOLS_LLDB_RENDER_OUTER_LIMIT" in os.environ else 20
print(f"RLtools Tensor renderer outer limit: {render_outer_limit}")
render_inner_limit = os.environ["RL_TOOLS_LLDB_RENDER_INNER_LIMIT"] if "RL_TOOLS_LLDB_RENDER_INNER_LIMIT" in os.environ else 500
print(f"RLtools Tensor renderer inner limit: {render_inner_limit}")
def render(target, float_type, ptr, shape, stride, title="", use_title=False, outer_limit=render_outer_limit, inner_limit=render_inner_limit, base_offset=0):
    if len(shape) == 1:
        output = "["
        for element_i in range(shape[0]):
            pos = element_i * stride[0]
            offset = ptr.GetValueAsUnsigned() + base_offset + pos * float_type.GetByteSize()
            val_wrapper = target.CreateValueFromAddress("temp", lldb.SBAddress(offset, target), float_type)
            val = val_wrapper.GetValue()
            output += str(val) + ", "
        if shape[0] > 0:
            output = output[:-2]
        return output + "]"
    elif len(shape) == 2:
        output = "[ " + (("\\\\ Subtensor: " + title + f"({shape[0]}x{shape[1]})") if use_title and len(title) > 0 else "") + "\n"
        for row_i in range(shape[0]) if shape[0] < inner_limit else list(range(inner_limit // 2)) + ["..."] + list(range(shape[0] - inner_limit // 2, shape[0])):
            if row_i == "...":
                output += "...\n"
                continue
            output += "[" if shape[0] < inner_limit else f"{row_i}: ["
            for col_i in range(shape[1]) if shape[1] < inner_limit else list(range(inner_limit // 2)) + ["..."] + list(range(shape[1] - inner_limit // 2, shape[1])):
                if col_i == "...":
                    output += "..., "
                    continue
                pos = row_i * stride[0] + col_i * stride[1]
                offset = ptr.GetValueAsUnsigned() + base_offset + pos * float_type.GetByteSize()
                val_wrapper = target.CreateValueFromAddress("temp", lldb.SBAddress(offset, target), float_type)
                val = val_wrapper.GetValue()
                output += str(val) + ", "
            if shape[1] > 0:
                output = output[:-2]
            output += "], \n"
        if shape[0] > 0:
            output = output[:-3]
            output += "\n"
        return output + "]\n"
    else:
        output = "[" + ("\n" if len(shape) == 3 else "")
        for i in range(shape[0]) if shape[0] < outer_limit else list(range(outer_limit // 2)) + ["..."] + list(range(shape[0] - outer_limit // 2, shape[0])):
            if i != "...":
                current_title = title + pad_number(i, 10) + " | "
                new_base_offset = base_offset + i * stride[0] * float_type.GetByteSize()
                output += render(target, float_type, ptr, shape[1:], stride[1:], title=current_title, use_title=use_title, base_offset=new_base_offset) + ", \n"
            else:
                output += "...\n"
                output += "...\n"
                output += "...\n"
        if shape[0] > 0:
            output = output[:-3]
        return output + "]"


def pretty_print(valobj, internal_dict, options):
    float_ptr = valobj.GetChildMemberWithName("_data")
    float_type = float_ptr.GetType().GetPointeeType()
    # float_type = valobj.GetType().GetPointeeType()
    # print(f"Float type: {float_type}")
    target = valobj.GetTarget()

    typename = valobj.GetType().GetCanonicalType().GetName()
    print(f"Typename is: {typename}")
    tensor = parse_string(str(float_ptr), typename)
    if tensor is None:
        print(f"Parse error on: {typename}")
        parse_string(valobj.type.name, verbose=True)

    if tensor is None:
        return valobj.type.name

    if len(tensor.shape) != len(tensor.stride):
        return str(tensor)

    print(valobj.type.name)
    # if not valobj.type.name[-1] in [">", "&"]:
    # if not valobj.type.name.endswith("DATA_TYPE"):
    #     return str(tensor)

    use_title = True

    print(f"Rendering {valobj.GetName()} at address: {hex(float_ptr.GetValueAsUnsigned())}")
    return str(tensor)  + "\n" + render(target, float_type, float_ptr, tensor.shape, tensor.stride, use_title=use_title)









    meta = decode_row_major(valobj)
    # if meta is None:
    #     return f"Matrix type could not be parsed: {valobj.type.name}"
    # else:
    #     acc = f"{json.dumps(meta)}\n"
    #     for row_i in range(meta["ROWS"]):
    #         for col_i in range(meta["COLS"]):
    #             pos = row_i * meta["ROW_PITCH"] + col_i
    #             offset = float_ptr.GetValueAsUnsigned() + pos * float_type.GetByteSize()
    #             val_wrapper = target.CreateValueFromAddress("temp", lldb.SBAddress(offset, target), float_type)
    #             val = val_wrapper.GetValue()
    #             acc += str(val) + ", "
    #         acc += "\n"
    #
    #
    #     return f"Matrix type: {acc}"
        # return float_ptr.Dereference()
