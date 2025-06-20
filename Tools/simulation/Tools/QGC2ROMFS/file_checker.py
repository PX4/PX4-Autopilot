#!/usr/bin/env python3

import sys
from pathlib import Path
from collections import Counter

def main(romfs_path):
    param_name_set = set()
    param_name_counter = Counter() 

    with open(romfs_path, 'r') as pf:
        lines = pf.readlines()

    for line in lines:
        if line.startswith('#'):
            continue

        parts = line.strip().split(' ')
        if len(parts) > 2 and parts[0] == "param":
            param_name_set.add(parts[2])

    for line in lines:
        if line.startswith('#'):
            continue

        parts = line.strip().split(' ')

        if len(parts) > 2 and parts[0] == "param":
            param_name_counter[parts[2]] += 1  # Increment count for each occurrence

    # Check for duplicate parameters
    duplicates = [param for param, count in param_name_counter.items() if count > 1]
    if duplicates:
        print("\033[93m[WARNING]\033[0m: The following parameters are repeated:")
        for param in duplicates:
            print(f"  - {param}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python file_checker.py <romfs_filename>")
    else:
        main(sys.argv[1])
