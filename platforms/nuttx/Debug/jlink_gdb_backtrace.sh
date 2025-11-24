#! /bin/sh

if command -v gdb-multiarch &> /dev/null
then
	GDB_CMD=$(command -v gdb-multiarch)

elif command -v arm-none-eabi-gdb &> /dev/null
then
	GDB_CMD=$(command -v arm-none-eabi-gdb)

else
	echo "gdb arm-none-eabi or multi-arch not found"
	exit 1
fi

file ${1}

gdb_cmd_file=$(mktemp)

cat >"${gdb_cmd_file}" <<EOL

source ${WORKSPACE}/platforms/nuttx/Debug/ARMv7M
source ${WORKSPACE}/platforms/nuttx/Debug/NuttX
source ${WORKSPACE}/platforms/nuttx/Debug/PX4

set mem inaccessible-by-default off
set print pretty
set pagination off

target remote localhost:2331

monitor regs

dmesg

perf

showtasks
backtrace

vecstate

info_nxthreads

nxthread_all_bt

EOL

${GDB_CMD} -silent --nh --nx --nw -batch -ix=${WORKSPACE}/platforms/nuttx/NuttX/nuttx/tools/nuttx-gdbinit -x ${gdb_cmd_file} ${1}
