uint64 timestamp                      # time since system start (microseconds)
uint8 CONNECTED_ESC_MAX = 12          # The number of ESCs supported (Motor1-Motor12)

uint8 ESC_CONNECTION_TYPE_PPM = 0     # Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1  # Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2 # One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3     # I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4     # CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5   # DShot

uint16 counter                        # incremented by the writing thread everytime new data is stored

uint8 esc_count                       # number of connected ESCs
uint8 esc_connectiontype              # how ESCs connected to the system

uint16 esc_online_flags               # Bitmask indicating which ESC is online/offline (in motor order)
# esc_online_flags bit 0 : Set to 1 if Motor1 is online
# esc_online_flags bit 1 : Set to 1 if Motor2 is online
# esc_online_flags bit 2 : Set to 1 if Motor3 is online
# esc_online_flags bit 3 : Set to 1 if Motor4 is online
# esc_online_flags bit 4 : Set to 1 if Motor5 is online
# esc_online_flags bit 5 : Set to 1 if Motor6 is online
# esc_online_flags bit 6 : Set to 1 if Motor7 is online
# esc_online_flags bit 7 : Set to 1 if Motor8 is online
# esc_online_flags bit 8 : Set to 1 if Motor9 is online
# esc_online_flags bit 9 : Set to 1 if Motor10 is online
# esc_online_flags bit 10: Set to 1 if Motor11 is online
# esc_online_flags bit 11: Set to 1 if Motor12 is online

uint16 esc_armed_flags                # Bitmask indicating which ESC is armed (in motor order)

EscReport[12] esc
