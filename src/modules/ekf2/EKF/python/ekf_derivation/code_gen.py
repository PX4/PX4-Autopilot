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
        custom_functions = {
                "Pow": [
                    (lambda b, e: e == 2, lambda b, e: f"({b})*({b})"),
                    (lambda b, e: e == -1, lambda b, e: f"1.0F/({b})"),
                    (lambda b, e: e == -2, lambda b, e: f"1.0F/(({b})*({b}))"),
                    (lambda b, e: e == 0.5, lambda b, e: f"sqrtf({b})"),
                    (lambda b, e: e == -0.5, lambda b, e: f"1.0F/sqrtf({b})"),
                    (lambda b, e: True, "ecl::powf"),
                    ]
        }

        return ccode(expression, type_aliases={real:float32}, user_functions=custom_functions)

    def write_subexpressions(self,subexpressions):
        write_string = ""
        for item in subexpressions:
            write_string = write_string + "const float " + str(item[0]) + " = " + self.get_ccode(item[1]) + ";\n"

        write_string = write_string + "\n\n"
        self.file.write(write_string)

    def write_matrix(self, matrix, variable_name, is_symmetric=False, pre_bracket="(", post_bracket=")"):
        write_string = ""

        if matrix.shape[0] * matrix.shape[1] == 1:
            if matrix[0] != 0:
                write_string = write_string + variable_name + " = " + self.get_ccode(matrix[0]) + ";\n"
        elif matrix.shape[0] == 1 or matrix.shape[1] == 1:
            for i in range(0,len(matrix)):
                if matrix[i] == 0:
                    continue
                write_string = write_string + variable_name + pre_bracket + str(i) + post_bracket + " = " + self.get_ccode(matrix[i]) + ";\n"

        else:
            for j in range(0, matrix.shape[1]):
                for i in range(0, matrix.shape[0]):
                    if j >= i or not is_symmetric:
                        if matrix[i,j] == 0:
                            continue
                        write_string = write_string + variable_name + pre_bracket + str(i) + "," + str(j) + post_bracket + " = " + self.get_ccode(matrix[i,j]) + ";\n"

        write_string = write_string + "\n\n"
        self.file.write(write_string)

    def close(self):
        self.file.close()
