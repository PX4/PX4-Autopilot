#
# Template for .gdbinit
# Copy the file to .gdbinit in your project root, and adjust the path below to match your system
#

target extended /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_DDE578CC-if00
# target extended /dev/ttyACM0

monitor swdp_scan
attach 1
monitor vector_catch disable hard
