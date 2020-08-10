#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 14 12:47:24 2020

@author: roman
"""
from sympy import ccode
from sympy.codegen.ast import float32, real

class CodeGenerator:
    def __init__(self, file_name):
        self.file_name = file_name
        self.file = open(self.file_name, 'w')

    def print_string(self, string):
        self.file.write("// " + string + "\n")

    def get_ccode(self, expression):
        return ccode(expression, type_aliases={real:float32})

    def write_subexpressions(self,subexpressions):
        output_string = ""
        for item in subexpressions:
            output_string = output_string + "const float " + str(item[0]) + " = " + self.get_ccode(item[1]) + ";\n"

        output_string = output_string + "\n\n"
        self.file.write(output_string)

    def write_matrix(self, matrix, identifier, is_symmetric=False):
        output_string = ""

        if matrix.shape[0] * matrix.shape[1] == 1:
            output_string = output_string + identifier + " = " + self.get_ccode(matrix[0]) + ";\n"
        elif matrix.shape[0] == 1 or matrix.shape[1] == 1:
            for i in range(0,len(matrix)):
                if (identifier == "Kfusion"):
                    # Vector f format used by Kfusion
                    output_string = output_string + identifier + "(" + str(i) + ") = " + self.get_ccode(matrix[i]) + ";\n"
                else:
                    # legacy array format used by Hfusion
                    output_string = output_string + identifier + "[" + str(i) + "] = " + self.get_ccode(matrix[i]) + ";\n"
        else:
            for j in range(0, matrix.shape[1]):
                for i in range(0, matrix.shape[0]):
                    if j >= i or not is_symmetric:
                        output_string = output_string + identifier + "(" + str(i) + "," + str(j) + ") = " + self.get_ccode(matrix[i,j]) + ";\n"
                        # legacy array format
                        # output_string = output_string + identifier + "[" + str(i) + "][" + str(j) + "] = " + self.get_ccode(matrix[i,j]) + ";\n"
            if is_symmetric:
                output_string = output_string + "\n"
                for j in range(0, matrix.shape[1]):
                    for i in range(0, matrix.shape[0]):
                        if j < i:
                            output_string = output_string + identifier + "(" + str(i) + "," + str(j) + ") = " + \
                                                            identifier + "(" + str(j) + "," + str(i) + ")" + ";\n"

        output_string = output_string + "\n\n"
        self.file.write(output_string)

    def close(self):
        self.file.close()
