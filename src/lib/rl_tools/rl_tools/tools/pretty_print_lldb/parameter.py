import re
import lldb
import json
import os

from tensor import parse_string as parse_string_tensor, render

test = "rl_tools::nn::parameters::Adam::instance<rl_tools::nn::parameters::Adam::spec<rl_tools::Tensor<rl_tools::tensor::Specification<float, unsigned long, rl_tools::tensor::Shape<unsigned long, 1536, 5>, rl_tools::tensor::Append<rl_tools::tensor::PopFront<rl_tools::tensor::CumulativeProduct<rl_tools::tensor::Shape<unsigned long, 1536, 5> > >, 1>, false, false> >, rl_tools::nn::parameters::Gradient, rl_tools::nn::parameters::categories::Weights> >::CONTAINER"



def extract_template(string):
    template = None
    stack = []
    for i, char in enumerate(string):
        if char == "<":
            stack.append(i)
        elif char == ">":
            if len(stack) == 1:
                template = string[:i+1]
                break
            else:
                stack.pop()
    return template
def parse_string(string, verbose=False):
    parameter_types = ["Gradient", "Adam", "Plain"]
    starts = [f"rl_tools::nn::parameters::{type1}::instance<rl_tools::nn::parameters::{type2}::spec<" for type1 in parameter_types for type2 in parameter_types]
    for start in starts:
        if string.startswith(start):
            tensor_template = extract_template(string[len(start):])
            if tensor_template is None:
                print(f"Template not found in {string}")
                return None
            return parse_string_tensor(tensor_template, verbose=verbose)
    print(f"Starts not found in {string}")
    return None

def pretty_print(valobj, internal_dict, options):
    float_ptr = valobj.GetChildMemberWithName("_data")
    float_type = float_ptr.GetType().GetPointeeType()
    target = valobj.GetTarget()

    tensor = parse_string(valobj.type.name)
    if tensor is None:
        print(f"Parse error on: {valobj.type.name}")
        parse_string(valobj.type.name, verbose=True)

    if tensor is None:
        return valobj.type.name

    if len(tensor.shape) != len(tensor.stride):
        print(f"Error: shape {tensor.shape} != stride {tensor.stride}")
        return str(tensor)

    if not valobj.type.name.endswith("::CONTAINER"):
        return str(tensor)

    use_title = True

    return str(tensor) + "\n" + render(target, float_type, float_ptr, tensor.shape, tensor.stride, use_title=use_title)

# print(parse_string(test))

# with open("lldb_type_dump.txt", "r") as file:
#     lines = file.readlines()
#     for line in lines:
#         tensor = parse_string(line)
#         if tensor is None:
#             print(f"Error: {repr(line)}")
#             parse_string(line, verbose=True)
#             break
#         else:
#             print(tensor)
