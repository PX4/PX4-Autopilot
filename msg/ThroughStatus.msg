# THROUGH flight mode internal status (logging and debugging only)

uint64 timestamp                  # time since system start (us)

uint8 PHASE_IDLE = 0
uint8 PHASE_ALIGN = 1
uint8 PHASE_RAMP = 2
uint8 PHASE_GUIDE = 3
uint8 PHASE_TERMINAL = 4
uint8 PHASE_EXIT = 5
uint8 phase

uint8 REASON_NONE = 0
uint8 REASON_COMPLETED = 1        # TERMINAL finished normally
uint8 REASON_USER_UNLOCK = 2
uint8 REASON_USER_MODE_SWITCH = 3
uint8 REASON_ALIGN_TIMEOUT = 4
uint8 REASON_TARGET_LOST = 5
uint8 REASON_MIN_ALT = 6
uint8 REASON_TILT_SAT = 7
uint8 REASON_MODE_TIMEOUT = 8
uint8 REASON_OPENING = 9          # Vc < 0 persistently (moving away)
uint8 exit_reason

float32 time_in_phase             # [s]
float32 time_in_mode              # [s]

float32[3] los_ned                # unit LOS vector (NED)
float32[3] los_rate               # omega_LOS [rad/s] (NED)
float32 los_elevation             # [rad] + = above horizon
float32 los_azimuth               # [rad]
float32 vc                        # closing velocity [m/s] (before Vc_min clamp)

float32[3] a_perp                 # PN lateral accel command [m/s^2] (NED)
float32 s_along                   # along-LOS accel [m/s^2]
float32[3] f_cmd                  # specific force command [m/s^2] (NED)
float32 thrust_cmd                # normalized [0..1]
float32 hover_thrust              # normalized
bool thrust_saturated
bool tilt_limited
bool a_perp_limited

float32 ramp_alpha                # 0..1

float32 pan_error                 # [rad]
float32 tilt_error                # [rad]
float32 target_size               # [-]
bool target_valid
bool uncage

float32 gimbal_pitch_cmd          # [rad] internal convention: + = camera down
float32 gimbal_pitch_meas         # [rad] internal convention: + = camera down
