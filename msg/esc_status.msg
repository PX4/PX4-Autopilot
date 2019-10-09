uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_ESC_MAX = 8				# The number of ESCs supported. Current (Q2/2013) we support 8 ESCs

uint8 ESC_VENDOR_GENERIC = 0				# generic ESC
uint8 ESC_VENDOR_MIKROKOPTER = 1			# Mikrokopter
uint8 ESC_VENDOR_GRAUPNER_HOTT = 2			# Graupner HoTT ESC
uint8 ESC_VENDOR_TAP = 3				# TAP ESC

uint8 ESC_CONNECTION_TYPE_PPM = 0			# Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1			# Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOOT = 2			# One Shoot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3			# I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4			# CAN-Bus

uint16 counter  					# incremented by the writing thread everytime new data is stored

uint8 esc_count						# number of connected ESCs
uint8 esc_connectiontype				# how ESCs connected to the system

uint8 esc_online_flags					# Bitmask indicating which ESC is online/offline
# esc_online_flags bit 0 : Set to 1 if ESC0 is online
# esc_online_flags bit 1 : Set to 1 if ESC1 is online
# esc_online_flags bit 2 : Set to 1 if ESC2 is online
# esc_online_flags bit 3 : Set to 1 if ESC3 is online
# esc_online_flags bit 4 : Set to 1 if ESC4 is online
# esc_online_flags bit 5 : Set to 1 if ESC5 is online
# esc_online_flags bit 6 : Set to 1 if ESC6 is online
# esc_online_flags bit 7 : Set to 1 if ESC7 is online

esc_report[8] esc
