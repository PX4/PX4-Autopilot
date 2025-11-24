uint64 timestamp                # Time since system start (microseconds)

float32[3] compression_gains	# [-] [@frame FRD] [@range 0, 1] Multiplicative gain to modify the output of the controller per axis
float32[3] spectral_damper_hpf  # [-] [@frame FRD] Squared output of spectral damper high-pass filter
float32[3] spectral_damper_out  # [-] [@frame FRD] Spectral damper output squared
