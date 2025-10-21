uint64 timestamp                # Time since system start (microseconds)

float32[3] compression_gains	# [-] Multiplicative gain to modify the output of the controller per axis [@frame FRD], [@range 0, 1]
float32[3] spectral_damper_hpf  # Squared output of the spectral damper’s high-pass filter [@frame FRD]
float32[3] spectral_damper_out  # Spectral damper's output squared [@frame FRD]
float32[3] input                # Torque input [@frame FRD]
