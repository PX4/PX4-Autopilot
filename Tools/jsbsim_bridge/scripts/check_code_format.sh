#!/bin/bash

# Fix style recursively in all the repository
sh fix_code_style.sh ..

# Print the diff with the remote branch (empty if no diff)
git --no-pager diff -U0 --color

# Check if there are changes, and failed
if ! git diff-index --quiet HEAD --; then echo "Code style check failed, please run clang-format (e.g. with scripts/fix_code_style.sh)"; exit 1; fi
