set(directories
	./posix/tests/hrt_test
	./posix/tests/wqueue
	./posix/tests/hello
	./posix/tests/muorb
	./posix/tests/vcdev_test
	./posix/work_queue
	./posix/px4_layer
	./posix/drivers/adcsim
	./posix/drivers/gpssim
	./posix/drivers/tonealrmsim
	./posix/drivers/accelsim
	./posix/drivers/airspeedsim
	./posix/drivers/barosim
	./posix/drivers/gyrosim
	./qurt/tests/hello
	./qurt/tests/muorb
	./qurt/px4_layer
	./common
	./nuttx/px4_layer
	./nuttx
	)

foreach(directory ${directories})
	add_subdirectory(${directory} EXCLUDE_FROM_ALL)
endforeach()
