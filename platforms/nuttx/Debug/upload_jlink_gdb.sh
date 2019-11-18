#! /bin/sh

set -o xtrace

gdb-multiarch -silent -nx -batch \
	-ex "target remote localhost:2331" \
	-ex "monitor reset 0" \
	-ex "load" \
	-ex "monitor reset 0" \
	-ex "monitor go" \
	-ex "monitor sleep 3000" \
	-ex "disconnect" \
	-ex "quit" \
	${1}
