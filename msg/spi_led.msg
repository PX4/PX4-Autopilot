# Control up to 1001 LEDs on APA102

uint64 timestamp                # time since system start (microseconds)

uint16 number_leds              #total number of leds [0,1000]
uint8 offset_group              #offset group (multiple of 10) of leds to control [0,100]
uint32[10] led_values           #follows spi 32 bit LED schema for apa102

# [  PAD 1's  ][ BRIGHT 0:31 ][ BLU 0:255 ][ GRN 0:255 ][ RED 0:255 ]
# [ 31 ... 29 ][ 28  ...  24 ][ 23 ... 16 ][ 15 ...  8 ][ 7  ...  0 ]
