char[17] name                   # parameter name
int16 index		# -1 if the param_id field should be used as identifier
int16 index_used		# -1 if the param_id field should be used as identifier

uint8 TYPE_INVALID = 0
uint8 TYPE_BOOL    = 1
uint8 TYPE_INT32   = 2
uint8 TYPE_FLOAT32 = 3
uint8 type                # parameter type

int32 int32_value               # current value if param_type is int-like
float32 float32_value           # current value if param_type is float-like
