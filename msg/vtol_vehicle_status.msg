# VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE
uint8 VEHICLE_VTOL_STATE_UNDEFINED = 0
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VEHICLE_VTOL_STATE_MC = 3
uint8 VEHICLE_VTOL_STATE_FW = 4

uint64 timestamp			# time since system start (microseconds)

uint64  ticks_since_trans               # for ILC debug
float32 ilc_input                      # feed-forward command of thrust
float32 aoa
float32 pitch_ang
float32 vert_acc_cmd
float32 thrust_cmd
float32 bx_acc_cmd
float32 bx_acc_e
float32 bx_acc_i              # integrator for body-x acc controller
float32 rollrot
float32 lat_dist

bool vtol_in_rw_mode			# true: vtol vehicle is in rotating wing mode
bool vtol_in_trans_mode
bool in_transition_to_fw		# True if VTOL is doing a transition from MC to FW
bool vtol_transition_failsafe		# vtol in transition failsafe mode
bool fw_permanent_stab			# In fw mode stabilize attitude even if in manual mode
