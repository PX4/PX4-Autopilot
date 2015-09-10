d=$(PWD)

#----------------------------------------------------------------------------
# OS: nuttx BOARD: px4fmu-v2 LABEL: simple
#
nuttx_px4fmu-v2_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-arm-none-eabi.cmake \
		-DOS=nuttx -DBOARD=px4fmu-v2 -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP 
nuttx_px4fmu-v2_simple_upload: nuttx_px4fmu-v2_simple
	make -C build_$< upload
nuttx_px4fmu-v2_simple_test: nuttx_px4fmu-v2_simple
	make -C build_$< test


#----------------------------------------------------------------------------
# OS: nuttx BOARD: px4fmu-v2 LABEL: default
#
nuttx_px4fmu-v2_default:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-arm-none-eabi.cmake \
		-DOS=nuttx -DBOARD=px4fmu-v2 -DLABEL=default && \
		make -s && ctest -V && cpack -G ZIP 
nuttx_px4fmu-v2_default_upload: nuttx_px4fmu-v2_default
	make -C build_$< upload
nuttx_px4fmu-v2_default_test: nuttx_px4fmu-v2_default
	make -C build_$< test

#----------------------------------------------------------------------------
# OS: nuttx BOARD: sim LABEL: simple
#
nuttx_sim_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-native.cmake \
		-DOS=nuttx -DBOARD=sim -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP 

#----------------------------------------------------------------------------
# OS: posix BOARD: sitl LABEL: simple
#
posix_sitl_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-posix-clang-native.cmake \
		-DOS=posix -DBOARD=sitl -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP

#----------------------------------------------------------------------------
# OS: qurt BOARD: hil LABEL: simple
#
qurt_hil_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DQURT_ENABLE_STUBS=1 -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/Toolchain-hexagon.cmake \
		-DOS=qurt -DBOARD=hil -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP

#----------------------------------------------------------------------------
# misc targets
#
clean:
	rm -rf build_*/

.PHONY: clean
