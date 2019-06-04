uint64 timestamp		# time since system start (microseconds)

bool v5_valid			# Sensor 5V rail voltage was read.
float32 voltage5v_v		# peripheral 5V rail voltage

bool v3v3_valid			# Sensor 3V3 rail voltage was read.
float32 voltage3v3_v		# Sensor 3V3 rail voltage

bool usb_connected		# USB is connected when true

bool brick_valid		# brick bits power is good when true

bool usb_valid 			# USB is valid when true

bool servo_valid		# servo power is good when true

bool periph_5v_oc		# peripheral overcurrent when true
bool hipower_5v_oc		# hi power peripheral overcurrent when true
