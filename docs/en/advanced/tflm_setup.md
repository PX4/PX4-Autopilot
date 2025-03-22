# Setup of TFLM

Building PX4 with TensorFlow Lite Micro breaks some of the other standard builds since it requires a change of the toolchain. Therefore it does not build directly when you clone it, but this step-by-step guide will take you through how to build PX4 with TFLM on your own computer.

:::warning
This is an experimental setup. It might break other parts of PX4. All flying at your own risk.
:::

:::info
This guide assumes that you can build PX4 locally from before. So if you have not installed the standard toolchain, please do so first: [Initial Setup](../dev_setup/config_initial.md)
:::

1. First you need to add TFLM as a submodule:

	```sh
	cd src/lib
	mkdir tflm
	cd ../..
	```
	```sh
	git submodule add -b main https://github.com/tensorflow/tflite-micro.git src/lib/tflm/tflite_micro/
	```

1. Then we need to install the TFLM dependencies. This is automatically done when you build it as a static library, enter the tflite-micro folder and do the following command:

	```sh
	cd src/lib/tflm/tflite_micro
	```
	```sh
	make -f tensorflow/lite/micro/tools/make/Makefile TARGET=cortex_m_generic TARGET_ARCH=cortex-m7 microlite
	```

1. While this is building (it can take a couple of minutes) we can some other changes. We need to switch to C++ version 17. Go to the main Cmakelists.txt file in the PX4-Autopilot repo and change the line

	```python
	# Change
	set(CMAKE_CXX_STANDARD 14)
	#To:
	set(CMAKE_CXX_STANDARD 17)
	```

1. The toolchain file in platforms/nuttx/cmake/Toolchain-arm-none-eabi.cmake needs to be switched out with the one in src/modules/mc_nn_control/setup/Toolchain-arm-none-eabi.cmake. And in this file you also need to add your local path to the PX4-Autopilot repo. This line is marked with a TODO comment.

1. In the src/modules/mc_nn_control/setup folder; move the CMakeLists.txt file over to the src/lib/tflm folder.

1. PX4 excludes standard libraries by default, if they are enabled they will break the nuttx build. To get around this we extract some of the standard library header files.
	```sh
	cd src/lib/tflm
	mkdir include
	cp -r tflite_micro/tensorflow/lite/micro/tools/make/downloads/gcc_embedded/arm-none-eabi/include/c++/13.2.1/ include
	rm include/13.2.1/arm-none-eabi/bits/ctype_base.h
	cp ../../modules/mc_nn_control/setup/ctype_base.h include/13.2.1/arm-none-eabi/bits/
	cd ../../..
	```

1. Add tflm as a library in PX4. Go to the src/lib/CMakeLists.txt and add the line

	```python
	add_subdirectory(tflm EXCLUDE_FROM_ALL)
	```

	Anywhere in the file (preferably alpabetically).


1. In the src/modules/mc_nn_control/CMakelists.txt file, uncomment the commented code and delete the dummy.cpp file, both in the directory and the CMakeLists.txt file.

1. Then we need a board file. To include the neural network controller module add

	```
	CONFIG_MODULES_MC_NN_CONTROL=y
	```

	to your .px4board file. There are three pre-made board config files where other modules are removed to make sure the entire executable fits in the flash memory of the boards. These are in the src/modules/mc_nn_control/setup/boards folder. To use them copy them to their respective folders like boards/px4/sitl and remove everything from the file file except neural.px4board. So it ends up as boards/px4/sitl/neural.px4board

1. Now everything should be set up and you can build it using the standard make commands:

	```sh
	make px4_sitl_neural
	```
