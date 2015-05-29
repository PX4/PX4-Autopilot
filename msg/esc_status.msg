uint8 CONNECTED_ESC_MAX = 8				# The number of ESCs supported. Current (Q2/2013) we support 8 ESCs

uint8 ESC_VENDOR_GENERIC = 0				# generic ESC
uint8 ESC_VENDOR_MIKROKOPTER = 1			# Mikrokopter
uint8 ESC_VENDOR_GRAUPNER_HOTT = 2			# Graupner HoTT ESC

uint8 ESC_CONNECTION_TYPE_PPM = 0			# Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1			# Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOOT = 2			# One Shoot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3			# I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4			# CAN-Bus

uint16 counter  					# incremented by the writing thread everytime new data is stored
uint64 timestamp 					# in microseconds since system start, is set whenever the writing thread stores new data

uint8 esc_count						# number of connected ESCs
uint8 esc_connectiontype				# how ESCs connected to the system

esc_report[8] esc
