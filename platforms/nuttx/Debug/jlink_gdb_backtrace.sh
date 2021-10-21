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

source ${WORKSPACE}/platforms/nuttx/Debug/NuttX
source ${WORKSPACE}/platforms/nuttx/Debug/ARMv7M

target remote localhost:2331

monitor regs

showtasks

vecstate

info threads

backtrace

bt full

EOL

${GDB_CMD} -silent --nh --nx --nw -batch -x ${gdb_cmd_file} ${1}
