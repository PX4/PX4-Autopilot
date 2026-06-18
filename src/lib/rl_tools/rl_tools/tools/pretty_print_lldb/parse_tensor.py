# example = "rl_tools::Tensor<rl_tools::tensor::Specification<double, unsigned long, rl_tools::tensor::Replace<rl_tools::tensor::Shape<unsigned long, 20, 30, 40>, 11, 1>, rl_tools::tensor::Append<rl_tools::tensor::PopFront<rl_tools::tensor::CumulativeProduct<rl_tools::tensor::Shape<unsigned long, 20, 30, 40> > >, 1>, false, false> >"
# example = "rl_tools::Tensor<rl_tools::tensor::Specification<float, unsigned long, rl_tools::tensor::Replace<rl_tools::tensor::Replace<rl_tools::tensor::Shape<unsigned long, 1, 1, 5>, 10, 2>, 10, 2>, true, rl_tools::tensor::Append<rl_tools::tensor::PopBack<rl_tools::tensor::Append<rl_tools::tensor::PopFront<rl_tools::tensor::CumulativeProduct<rl_tools::tensor::Replace<rl_tools::tensor::Replace<rl_tools::tensor::Shape<unsigned long, 1, 1, 5>, 10, 2>, 10, 2> > >, 1> >, 1>, false> > &"
# example = " rl_tools::Tensor<rl_tools::tensor::Specification<float, unsigned long, rl_tools::tensor::Shape<unsigned long, 1, 100, 5>, false, rl_tools::tensor::Append<rl_tools::tensor::PopFront<rl_tools::tensor::CumulativeProduct<rl_tools::tensor::Shape<unsigned long, 1, 100, 5> > >, 1>, false> >"
example = "struct rl_tools::Tensor<rl_tools::tensor::Specification<float, unsigned long, rl_tools::tensor::Shape<unsigned long, 1, 100, 5>, false, rl_tools::tensor::Append<rl_tools::tensor::PopFront<rl_tools::tensor::CumulativeProduct<rl_tools::tensor::Shape<unsigned long, 1, 100, 5> > >, 1>, false> >"


class Node:
    def __init__(self, type, value=""):
        self.type = type
        self.children = []
        self.value = value
    
    def add_child(self, child):
        self.children.append(child)
    
    def print(self, level=0):
        print("  "*level + self.type + (f": {self.value}" if self.value else ""))
        for child in self.children:
            child.print(level+1)


def parse_templates(text, root, verbose=False):
    current = ''
    open_brackets = 0

    for i, char in enumerate(text):
        if char == '<':
            current = current.strip()
            if open_brackets == 0 and current:
                root.add_child(Node("text", value=current))
                current = ''
            else:
                current += char
            open_brackets += 1
        elif char == '>':
            if open_brackets == 2 and current:
                current += char
                n = Node("tree")
                root.add_child(n)
                if verbose:
                    print(f"Parsing: {current}")
                parse_templates(current, n)
                current = ''
            elif open_brackets != 1:
                current += char
            if open_brackets == 1:
                current = current.strip()
                if current:
                    root.add_child(Node("text", value=current))
                current = ''
            open_brackets -= 1
        elif open_brackets == 1 and char == ",":
            current = current.strip()
            if current:
                root.add_child(Node("text", value=current))
                current = ''
        else:
            current += char


class Tensor:
    def __init__(self, pointer, element_type, index_type, shape, dynamic_allocation, stride):
        self.pointer = pointer
        self.element_type = element_type
        self.index_type = index_type
        self.shape = shape
        self.dynamic_allocation = dynamic_allocation
        self.stride = stride
    
    def __str__(self):
        return f"Tensor(pointer={self.pointer}, shape={self.shape}, stride={self.stride}, element_type={self.element_type}, index_type={self.index_type}, dynamic_allocation={self.dynamic_allocation})"
    
    def __repr__(self):
        return str(self)

def parse_tuple(tree, verbose=False):
    if tree.type != "tree":
        if verbose:
            print(f"Parse Tuple: Expected tree node, got {tree.type}")
        return None
    if len(tree.children) < 1:
        if verbose:
            print(f"Parse Tuple: Expected at least 1 child, got {len(tree.children)}")
        return None
    name_node = tree.children[0]
    if name_node.type != "text":
        if verbose:
            print(f"Parse Tuple: Expected text node for name, got {name_node.type}")
        return None
    name = name_node.value
    if name == "rl_tools::tensor::Shape" or name == "rl_tools::tensor::Stride":
        if len(tree.children) < 3:
            if verbose:
                print(f"Parse Tuple: Expected at least 3 children for Shape, got {len(tree.children)}")
            return None
        shape_index_type_node = tree.children[1]
        if shape_index_type_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for shape index type, got {shape_index_type_node.type}")
            return None
        shape_index_type = shape_index_type_node.value
        shape_dimensions = []
        for i in range(2, len(tree.children)):
            dim_node = tree.children[i]
            if dim_node.type != "text":
                if verbose:
                    print(f"Parse Tuple: Expected text node for shape dimension, got {dim_node.type}")
                return None
            try: 
                dim_integer_value = int(dim_node.value)
            except ValueError:
                if verbose:
                    print(f"Parse Tuple: Expected integer value for shape dimension {i-2}, got {dim_node.value}")
                return None
            shape_dimensions.append(dim_integer_value)
        return shape_dimensions
    elif name == "rl_tools::tensor::Remove":
        if len(tree.children) != 3:
            if verbose:
                print(f"Parse Tuple: Expected 3 children for Remove, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        index_node = tree.children[2]
        if index_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for index, got {index_node.type}")
            return None
        try:
            index = int(index_node.value)
        except ValueError:
            if verbose:
                print(f"Parse Tuple: Expected integer value for index, got {index_node.value}")
            return None
        if index < 0 or index >= len(operand):
            if verbose:
                print(f"Parse Tuple: Index out of bounds for Remove: {index}")
            return None
        return operand[:index] + operand[index+1:]
    elif name == "rl_tools::tensor::Append":
        if len(tree.children) != 3:
            if verbose:
                print(f"Parse Tuple: Expected 3 children for Append, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        value_node = tree.children[2]
        if value_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for value, got {value_node.type}")
            return None
        try:
            value = int(value_node.value)
        except ValueError:
            if verbose:
                print(f"Parse Tuple: Expected integer value for value, got {value_node.value}")
            return None
        return operand + [value]
    elif name == "rl_tools::tensor::PopFront":
        if len(tree.children) != 2:
            if verbose:
                print(f"Parse Tuple: Expected 2 children for PopFront, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        if len(operand) == 0:
            if verbose:
                print(f"Parse Tuple: PopFront on empty tuple")
            return None
        return operand[1:]
    elif name == "rl_tools::tensor::PopBack":
        if len(tree.children) != 2:
            if verbose:
                print(f"Parse Tuple: Expected 2 children for PopBack, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        if len(operand) == 0:
            if verbose:
                print(f"Parse Tuple: PopBack on empty tuple")
            return None
        return operand[1:]
    elif name == "rl_tools::tensor::CumulativeProduct":
        if len(tree.children) != 2:
            if verbose:
                print(f"Parse Tuple: Expected 2 children for Product, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        product = 1
        values = []
        for value in operand[::-1]:
            product *= value
            values.append(product)
        return values[::-1]
    elif name == "rl_tools::tensor::Replace":
        if len(tree.children) != 4:
            if verbose:
                print(f"Parse Tuple: Expected 4 children for Replace, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        value_node = tree.children[2]
        if value_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for value, got {value_node.type}")
            return None
        try:
            value = int(value_node.value)
        except ValueError:
            if verbose:
                print(f"Parse Tuple: Expected integer value for value, got {value_node.value}")
            return None
        index_node = tree.children[3]
        if index_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for index, got {index_node.type}")
            return None
        try:
            index = int(index_node.value)
        except ValueError:
            if verbose:
                print(f"Parse Tuple: Expected integer value for index, got {index_node.value}")
            return None
        if index < 0 or index >= len(operand):
            if verbose:
                print(f"Parse Tuple: Index out of bounds for Replace: {index}")
            return None
        return operand[:index] + [value] + operand[index+1:]
    elif name == "rl_tools::tensor::Insert":
        if len(tree.children) != 4:
            if verbose:
                print(f"Parse Tuple: Expected 4 children for Insert, got {len(tree.children)}")
            return None
        operand_node = tree.children[1]
        if operand_node.type != "tree":
            if verbose:
                print(f"Parse Tuple: Expected tree node for operand, got {operand_node.type}")
            return None
        operand = parse_tuple(operand_node, verbose)
        if operand is None:
            return None
        value_node = tree.children[2]
        if value_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for value, got {value_node.type}")
            return None
        try:
            value = int(value_node.value)
        except ValueError:
            if verbose:
                print(f"Parse Tuple: Expected integer value for value, got {value_node.value}")
            return None
        index_node = tree.children[3]
        if index_node.type != "text":
            if verbose:
                print(f"Parse Tuple: Expected text node for index, got {index_node.type}")
            return None
        try:
            index = int(index_node.value)
        except ValueError:
            if verbose:
                print(f"Parse Tuple: Expected integer value for index, got {index_node.value}")
            return None
        if index < 0 or index >= len(operand):
            if verbose:
                print(f"Parse Tuple: Index out of bounds for Insert: {index}")
            return None
        return operand[:index] + [value] + operand[index:]
    else:
        if verbose:
            print(f"Parse Tuple: Unknown tuple name: {name}")
        return None
    


def parse_tensor(pointer, tree, verbose=False):
    if tree.type != "tree":
        if verbose:
            print(f"Parse Tensor: Expected tree node, got {tree.type}")
        return None
    if len(tree.children) != 2:
        if verbose:
            print(f"Parse Tensor: Expected 2 child (self + Specification), got {len(tree.children)}")
        return None
    spec = tree.children[1]
    if spec.type != "tree":
        if verbose:
            print(f"Parse Tensor: Expected tree node for spec, got {spec.type}")
        return None
    if len(spec.children) != 7:
        if verbose:
            print(f"Parse Tensor: Expected 7 children in spec, got {len(spec.children)}")
        return None
    element_type_node = spec.children[1]
    if element_type_node.type != "text":
        if verbose:
            print(f"Parse Tensor: Expected text node for element_type, got {element_type_node.type}")
        return None
    element_type = element_type_node.value

    index_type_node = spec.children[2]
    if index_type_node.type != "text":
        if verbose:
            print(f"Parse Tensor: Expected text node for index type, got {index_type_node.type}")
        return None
    index_type = index_type_node.value

    shape_node = spec.children[3]
    if shape_node.type != "tree":
        if verbose:
            print(f"Parse Tensor: Expected tree node for shape, got {shape_node.type}")
        return None
    shape = parse_tuple(shape_node, verbose)
    if shape is None:
        return None

    dynamic_allocation_node = spec.children[4]
    if dynamic_allocation_node.type != "text":
        if verbose:
            print(f"Parse Tensor: Expected text node for dynamic_allocation flag, got {dynamic_allocation_node.type}")
        return None
    dynamic_allocation = dynamic_allocation_node.value

    stride_node = spec.children[5]
    if stride_node.type != "tree":
        if verbose:
            print(f"Parse Tensor: Expected tree node for stride, got {stride_node.type}")
        return None
    stride = parse_tuple(stride_node, verbose)
    if stride is None:
        return None

    return Tensor(pointer, element_type, index_type, shape, dynamic_allocation, stride)
    



def parse_string(pointer, text, verbose=False):
    root = Node("tree")
    parse_templates(text, root, verbose=verbose)
    if verbose:
        root.print()
    return parse_tensor(pointer, root, verbose=verbose)


    

if __name__ == "__main__":
    tensor = parse_string(0x00, example, verbose=True)
    print(tensor)

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
