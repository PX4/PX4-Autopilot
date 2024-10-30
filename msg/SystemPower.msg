uint64 timestamp                # time since system start (microseconds)

float32 voltage5v_v             # peripheral 5V rail voltage
float32[4] sensors3v3           # Sensors 3V3 rail voltage
uint8 sensors3v3_valid          # Sensors 3V3 rail voltage was read (bitfield).

bool usb_connected		# USB is connected when true
uint8 usb_connected_count       # number of times USB has been connected

uint8 brick_valid		# brick bits power is good (bitfield).
bool usb_valid                  # USB is valid when true

bool servo_valid                # servo power is good when true
bool periph_5v_oc               # peripheral overcurrent when true
bool hipower_5v_oc              # high power peripheral overcurrent when true
bool comp_5v_valid              # 5V to companion valid
bool can1_gps1_5v_valid         # 5V for CAN1/GPS1 valid

uint8 BRICK1_VALID_SHIFTS=0
uint8 BRICK1_VALID_MASK=1
uint8 BRICK2_VALID_SHIFTS=1
uint8 BRICK2_VALID_MASK=2
uint8 BRICK3_VALID_SHIFTS=2
uint8 BRICK3_VALID_MASK=4
uint8 BRICK4_VALID_SHIFTS=3
uint8 BRICK4_VALID_MASK=8
