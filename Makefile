d=$(PWD)

px4fmu-v2_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake \
		-DOS=nuttx -DBOARD=px4fmu-v2 -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP 

px4fmu-v2_simple-upload: px4fmu-v2_simple
	cd $d/build_$< && make upload

posix-sitl_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-posix-clang-native.cmake \
		-DOS=posix -DBOARD=sitl -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP

qurt-hil_simple:
	mkdir -p $d/build_$@ && cd $d/build_$@ && \
		cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-hexagon.cmake \
		-DOS=qurt -DBOARD=hil -DLABEL=simple && \
		make -s && ctest -V && cpack -G ZIP

clean:
	rm -rf build_*/

.PHONY: clean
