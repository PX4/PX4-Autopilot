# copy the file to .gdbinit in your Firmware tree, and adjust the path
# below to match your system
# For example:
# target extended /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_DDE5A1C4-if00
# target extended /dev/ttyACM4


monitor swdp_scan
attach 1
monitor vector_catch disable hard
set mem inaccessible-by-default off
set print pretty
source Debug/PX4
