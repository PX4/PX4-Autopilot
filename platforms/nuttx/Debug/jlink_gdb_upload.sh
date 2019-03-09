#! /bin/sh

gdb-multiarch -nx -batch \
	-ex "target remote localhost:2331" \
	-ex "monitor reset 0" \
	-ex "load" \
	-ex "compare-sections" \
	-ex "monitor reset 0" \
	-ex "monitor sleep 1000" \
	-ex "monitor go" \
	-ex "kill" \
	${1}
