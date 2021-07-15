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

for i in 1 2 3;
do
	${GDB_CMD} -silent --nh --nx --nw -batch \
		-ex "target remote localhost:2331" \
		-ex "monitor reset 0" \
		-ex "load" \
		-ex "monitor reset 0" \
		-ex "monitor go" \
		-ex "monitor sleep 1000" \
		-ex "disconnect" \
		-ex "quit" \
		${1}

	if [ $? -ne 0 ]; then
		echo "GDB error, retrying"
		sleep 3
	else
		exit 0
	fi
done

exit 1
