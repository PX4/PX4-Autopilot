# -*- coding: utf-8 -*-
"""
Created on Sun Nov  1 15:15:06 2015

@author: jgoppert
"""

import cloudpickle
import pickle
import sympy
from sympy import *

def save(varlist, filename, local_dict):    
    d = {key:local_dict[key] for key in varlist}
    with open(filename, 'wb') as f:
        cloudpickle.dump(d, f)

def load(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)

def sympy_save(varlist, filename, local_dict):
    d = {key:local_dict[key] for key in varlist}
    with open(filename, 'w') as f:
        f.write(sympy.python(d))

def sympy_load(filename):
    with open(filename, 'r') as f:
        s = f.read()
        exec(s)
        return locals()['e']
