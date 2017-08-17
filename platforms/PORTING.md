# Porting

The variables ${PX4_SOURCE_DIR}, ${OS}, ${LABE} and ${BOARD} are passed by the build system.

## Adding a New Board Config

If the OS is currently supported, but you want to add a new board file, add the file to:
```
${PX4_SOURCE_DIR}/platforms/${OS}/boards/${BOARD}/config\_${LABEL}.cmake
```

You can create multiple config files for the same board (e.g config_test.cmake vs config_default.cmake).

If you have a SoC platform that has a single board but multiple builds of PX4 for different processors, and there are shared board files, the ${BOARD_FILE_DIRS} should be set to point to the locations of the board files. If you use src/drivers/boards/common/board_config.h you should add ${PX4_SOURCE_DIR}/src/drivers/boards to ${BOARD_FILE_DIRS} and include "common/board_config.h" in your custom board_config.h.

If not specified in the config file, ${BOARD_FILE_DIRS} defaults to:

```
set(BOARD_FILE_DIRS ${PX4_SOURCE_DIR}platforms/${OS}/src/drivers/boards/${BOARD} \
    ${PX4_SOURCE_DIR}/src/drivers/boards)
```

The board_config.h file must go in:
```
${PX4_SOURCE_DIR}/platforms/${OS}/boards/${BOARD}
```
## Overriding a Driver or Module

Drivers specified in the config file ${PX4_SOURCE_DIR}/platforms/${OS}/cmake/configs/${OS}\_${BOARD}_CONFIG will use an instance found in ${PX4_SOURCE_DIR}/platforms/${OS}/src/ before looking in src/. For instance, to override the src/drivers/rgbled driver for a new platform, you could create a ${PX4_SOURCE_DIR}/platforms/${OS}/src/drivers/rgbled directory with the modfified CMakeLists.txt and src code to implement the driver.

## Adding New OS Platforms

Platforms must provide the following

* ${PX4_SOURCE_DIR}/platforms/${OS}/
  - cmake/${OS}/px4_impl_${OS}.cmake
  - src/firmware/${OS}/CMakeLists.txt

and one or more supported boards:

* ${PX4_SOURCE_DIR}/platforms/${OS}/
  - boards/${BOARD}/config\_${LABEL}.cmake
  - boards/${BOARD}/board_config.h

and an implementation for the OS:

* ${PX4_SOURCE_DIR}/platforms/${OS}/
  - src/platforms/${OS}

