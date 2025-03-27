MYROOTDIR=/Users/vidma/flyuav/px4_rpi/PX4-Autopilot

sleep 1

# echo "clean pyc"
# cd /opt/xpack/
# find . -name "*.pyc" -delete
# find . -name "*.pyc" -print
# cd /Applications/CLion.app/Contents/bin/gdb/
# find . -name "*.pyc" -delete
# find . -name "*.pyc" -print

cd /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/

#source ./emgdb_venv/bin/activate

cd /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/build/raspberrypi_pico-2_default


# /opt/homebrew/bin/python3.11 -m venv ./emgdb_venv
# source ./emgdb_venv/bin/activate
# python3 -m pip install emdbg
# python3 -m pip uninstall numpy

# /opt/homebrew/bin/python3.13 install emdbg
# pip3 install emdbg
# 

export PYTHONUTF8=1
export LANG=en_US.UTF-8

echo "$MYROOTDIR/emgdb_venv/bin/python3.11"

# /opt/homebrew/bin/python3.11
# $MYROOTDIR/emgdb_venv/bin/python3.11 


# PYTHONPATH=/Applications/CLion.app/Contents/bin/gdb/renderers:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/arm-none-eabi/share/gdb/python:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python311.zip:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11/lib-dynload:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11/site-packages \
#   /opt/homebrew/bin/python3.11 -m emdbg.debug.gdb \
#   --elf /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/build/raspberrypi_pico-2_default/raspberrypi_pico-2_default.elf \
#    --svd /Users/vidma/flyuav/px4_rpi/pico-sdk/src/rp2350/hardware_regs/RP2350.svd \
#    --python \
#    -x .gdbinit openocd


PYTHONPATH=/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11/:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/arm-none-eabi/share/gdb/python:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11/lib-dynload:/opt/xpack/xpack-arm-none-eabi-gcc-12.2.1-1.2/lib/python3.11/site-packages \
/opt/homebrew/bin/python3.11 -m emdbg.debug.gdb \
  --elf /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/build/raspberrypi_pico-2_default/raspberrypi_pico-2_default.elf \
   --svd /Users/vidma/flyuav/px4_rpi/pico-sdk/src/rp2350/hardware_regs/RP2350.svd \
   --python \
   -x .gdbinit openocd
