#!/bin/bash

# Generate the C header library at build time for a single dialect
#
# ** NOTE **
# For any dialect that includes/extends another dialect, mavgen will
# generate that dialect as well, but with a different 'XML ID'.
#
# This XML ID is used to determine the include order of the various headers,
# so the usage of a multi-dialect message set depends on the proper setting of
# these IDs.
#
# As a result, this means that dialects like "common" that are included in
# other 'derived' dialects (like "standard" or "ardupilotmega") cannot be
# re-generated separately from the processing of the derived dialects.
#
# For example: With the 'standard' dialect:
#
#      --+ [0] standard (Parent / primary dialect)
#        |
#        +-+ [1] common (Child dialect)
#          |
#          +-- [2] minimal (Child dialect)
#
# - Dialect 'standard' includes (and extends) dialect 'common'
# - Dialect 'common' includes (and extends) dialect 'minimal'
#
# - When generating headers for dialect 'standard':
#   - The 'standard' extensions will be created first, and its mavlink.h header
#     will be given the XML ID of 0, to identify it as the "parent" header file
#   - 'common' will be generated next, and its headers will be given an ID of 1
#     to denote it as the first "child" header file
#   - 'minimal' will be generated last, and given an ID of 2, denoting it as
#     the "grandchild" / second "child" header file
#
# - If you then go back and generate headers for another dialect, any prior
#   headers generated because of the 'standard' dialect may get overwritten,
#   and possibly with different IDs.  This could break the header include
#   ordering for the first 'standard' dialect
#   - Specific example: if you generate 'common' after 'standard', the ID
#     assigned to 'common' will be reset to 0, breaking its usage by 'standard'
#     in the process
#
# - In summary: We are ONLY producing the specific dialect requested here
#   (and its immediate dependencies).
#
# Author: Jacob Crabill <jacob@flyvoly.com>

function generate_headers() {
$PYTHON_EXECUTABLE $MAVLINK_GIT_PATH/pymavlink/tools/mavgen.py \
    --output $CLIBRARY_PATH \
    --lang C \
    --wire-protocol 2.0 \
    $MAVLINK_GIT_PATH/message_definitions/v1.0/$1.xml
}

# Arguments
# Which dialect to generate?
MAV_DIALECT=$1
# Path of this MAVLink repository
MAVLINK_GIT_PATH=$2
# Output path for the MAVLink C header library
CLIBRARY_PATH=$3
# The Python executable to use
PYTHON_EXECUTABLE=$4

# delete old c headers
echo -e "\0033[35mMAVLink: Remove C library: ${CLIBRARY_PATH}\0033[0m"
rm -rf $CLIBRARY_PATH/*

# generate new c headers
echo -e "\0033[34mMAVLink: Starting to generate c headers\0033[0m"
echo -e "\0033[34mMAVLink: --Generating dialect '$MAV_DIALECT'\0033[0m"
generate_headers $MAV_DIALECT 2
echo -e "\0033[34mMAVLink: Finished generating c headers\0033[0m\n"
