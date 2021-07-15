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
target remote localhost:2331
monitor reset 0
load
monitor reset 0
monitor go
EOL

for i in 1 2 3;
do
	${GDB_CMD} -silent --nh --nx --nw -batch -x ${gdb_cmd_file} ${1}
	gdb_ret=$?

	if [ $gdb_ret -ne 0 ]; then
		echo "GDB error, retrying"
		sleep 3
	else
		exit 0
	fi
done

exit 1
