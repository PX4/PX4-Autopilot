# Excelsior is the code name of a board currently in development.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.
include(configs/qcom/qurt_common)

# The config between different QURT builds is shared.

add_definitions(
   -D__PX4_QURT_EAGLE
   )

