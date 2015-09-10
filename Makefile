d=$(PWD)


nuttx_px4fmu-v2_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-arm-none-eabi.cmake \
		-DOS=nuttx -DBOARD=px4fmu-v2 -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP 
px4fmu-v2_simple: nuttx_px4fmu-v2_simple

nuttx_sim_simple:
	echo "nuttx-sim-simple is a work in progress"
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-native.cmake \
		-DOS=nuttx -DBOARD=sim -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP 

nuttx_px4fmu-v2_simple_upload: nuttx_px4fmu-v2_simple
	cd $d/build_$< && make upload
px4fmu-v2_simple_upload: px4fmu-v2_simple_upload

posix_sitl_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-posix-clang-native.cmake \
		-DOS=posix -DBOARD=sitl -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP

qurt_hil_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DQURT_ENABLE_STUBS=1 -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-hexagon.cmake \
		-DOS=qurt -DBOARD=hil -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP

clean:
	rm -rf build_*/

.PHONY: clean
