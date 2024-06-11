#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2023 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: derivation_utils.py
Description:
    Common functions used for the derivation of most estimators
"""

import symforce.symbolic as sf

import re

def sign_no_zero(x) -> sf.Scalar:
    """
    Returns -1 if x is negative, 1 if x is positive, and 1 if x is zero
    """
    return 2 * sf.Min(sf.sign(x), 0) + 1

def add_epsilon_sign(expr, var, eps):
    # Avoids a singularity at 0 while keeping the derivative correct
    return expr.subs(var, var + eps * sign_no_zero(var))

def generate_px4_function(function_name, output_names):
    from symforce.codegen import Codegen, CppConfig
    import os
    import fileinput

    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=CppConfig(zero_initialization_sparsity_threshold=1))
    metadata = codegen.generate_function(
            output_dir="generated",
            skip_directory_nesting=True)

    for f in metadata.generated_files:
        print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))

    # Replace cstdlib and Eigen functions by PX4 equivalents
    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True) as file:
        for line in file:
            line = line.replace("std::max", "math::max")
            line = line.replace("std::min", "math::min")
            line = line.replace("Eigen", "matrix")
            line = line.replace("matrix/Dense", "matrix/math.hpp")

            # don't allow underscore + uppercase identifier naming (always reserved for any use)
            line = re.sub(r'_([A-Z])', lambda x: '_' + x.group(1).lower(), line)

            print(line, end='')

def generate_python_function(function_name, output_names):
    from symforce.codegen import Codegen, PythonConfig
    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=PythonConfig())

    metadata = codegen.generate_function(
            output_dir="generated",
            skip_directory_nesting=True)

def build_state_struct(state, T="float"):
    out = "struct StateSample {\n"

    def TypeFromLength(len):
        if len == 1:
            return f"{T}"
        elif len == 2:
            return f"matrix::Vector2<{T}>"
        elif len == 3:
            return f"matrix::Vector3<{T}>"
        elif len == 4:
            return f"matrix::Quaternion<{T}>"
        else:
            raise NotImplementedError

    for key, val in state.items():
        out += f"\t{TypeFromLength(val.storage_dim())} {key}{{}};\n"

    state_size = state.storage_dim()
    out += f"\n\tmatrix::Vector<{T}, {state_size}> Data() const {{\n" \
           + f"\t\tmatrix::Vector<{T}, {state_size}> state;\n"

    index = state.index()
    for key in index:
        out += f"\t\tstate.slice<{index[key].storage_dim}, 1>({index[key].offset}, 0) = {key};\n"

    out += "\t\treturn state;\n"
    out += "\t};\n" # Data

    # const ref vector access
    first_field = next(iter(state))

    out += f"\n\tconst matrix::Vector<{T}, {state_size}>& vector() const {{\n" \
        + f"\t\treturn *reinterpret_cast<matrix::Vector<{T}, {state_size}>*>(const_cast<float*>(reinterpret_cast<const {T}*>(&{first_field})));\n" \
        + f"\t}};\n\n"

    out += "};\n" # StateSample

    out += f"static_assert(sizeof(matrix::Vector<{T}, {state_size}>) == sizeof(StateSample), \"state vector doesn't match StateSample size\");\n"

    return out

def build_tangent_state_struct(state, tangent_state_index):
    out = "struct IdxDof { unsigned idx; unsigned dof; };\n"

    out += "namespace State {\n"

    start_index = 0
    for key in tangent_state_index.keys():
        out += f"\tstatic constexpr IdxDof {key}{{{tangent_state_index[key].idx}, {tangent_state_index[key].dof}}};\n"

    out += f"\tstatic constexpr uint8_t size{{{state.tangent_dim()}}};\n"
    out += "};\n" # namespace State
    return out

def generate_px4_state(state, tangent_state_index):
    print("Generate EKF tangent state definition")
    filename = "state.h"
    f = open(f"./generated/{filename}", "w")
    header = ["// --------------------------------------------------\n",
              "// This file was autogenerated, do NOT modify by hand\n",
              "// --------------------------------------------------\n",
              "\n#ifndef EKF_STATE_H",
              "\n#define EKF_STATE_H\n\n",
              "#include <matrix/math.hpp>\n\n",
              "namespace estimator\n{\n"]
    f.writelines(header)

    f.write(build_state_struct(state))
    f.write("\n")
    f.write(build_tangent_state_struct(state, tangent_state_index))

    f.write("}\n") # namespace estimator
    f.write("#endif // !EKF_STATE_H\n")
    f.close()
    print(f"  |- {filename}")
