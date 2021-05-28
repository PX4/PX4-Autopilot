
px4_add_board(
	PLATFORM nuttx
	VENDOR ssrc
	MODEL saluki
	LABEL bootloader
	TOOLCHAIN riscv64-unknown-elf
	ARCHITECTURE rv64gc
#	CRYPTO sw_crypto
#	KEYSTORE stub_keystore

	DRIVERS
#	bootloader

	MODULES

	SYSTEMCMDS

	EXAMPLES

	)
