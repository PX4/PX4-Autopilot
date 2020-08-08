uint64 timestamp                 # time since system start (microseconds)

bool voltage5v_available         # peripheral 5V rail voltage status available
float32 voltage5v_v              # peripheral 5V rail voltage

bool[4] sensors3v3_available     # sensor 3.3V rail voltage status available
float32[4] sensors3v3            # Sensors 3V3 rail voltage

bool usb_connected               # USB is connected when true
bool usb_power_valid             # On USB power when true

bool servo_power_available       # servo power status available
bool servo_power_valid           # servo power is good when true

bool peripheral_5v_available     # 5V peripheral status available
bool peripheral_5v_overcurrent   # 5V peripheral overcurrent when true

bool hipower_5v_available        # 5V hi power peripheral status available
bool hipower_5v_overcurrent      # 5V hi power peripheral overcurrent when true

bool comp_5v_available           # 5V to companion status available
bool comp_5v_valid               # 5V to companion valid

bool can1_gps1_5v_available      # 5V for CAN1/GPS1 status available
bool can1_gps1_5v_valid          # 5V for CAN1/GPS1 valid

uint8 bricks_supported
bool[4] brick_valid              # brick power is good when true
float32[4] brick_voltage_adc     # brick voltage ADC
float32[4] brick_current_adc     # brick current ADC
