# Internal combustion engine (ICE) actuator setpoints
#
# Published by the internal_combustion_engine_control module, which runs the engine
# start/stop state machine and the idle RPM governor.
# The mixer maps these setpoints onto the IC Engine output functions (ignition,
# throttle, choke, starter), which are assigned to the flight controller outputs.
# The resulting state machine state is reported in InternalCombustionEngineStatus.

uint64 timestamp # [us] Time since system start

bool ignition_on # Activate/deactivate the ignition (spark plug)
float32 throttle_control # [norm] [@range 0, 1] [@invalid NaN Stops the engine, output goes to its disarmed value] Throttle actuator setpoint, rate limited by ICE_THR_SLEW. 0 is closed throttle
float32 choke_control # [norm] [@range 0, 1] Choke actuator setpoint, 1 is fully closed
float32 starter_engine_control # [norm] [@range 0, 1] (Electric) starter motor setpoint

bool user_request_motor_on # User intent for the engine to be running, from the source selected by ICE_ON_SOURCE
