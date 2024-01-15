# ParameterClientSetValueRequest : Used by the server to update the value for a parameter on the client

uint64 timestamp                        # time since system start (microseconds)

char[17] parameter_name     # Parameter name with null termination
int32 int_value             # Optional value for an integer parameter
float32 float_value         # Optional value for a float parameter

uint8 ORB_QUEUE_LENGTH = 4
