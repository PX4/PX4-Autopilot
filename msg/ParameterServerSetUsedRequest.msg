# ParameterServerSetUsedRequest : Used by a client to update the used flag for a parameter

uint64 timestamp                        # time since system start (microseconds)

char[17] parameter_name     # Parameter name with null termination

uint8 ORB_QUEUE_LENGTH = 8
