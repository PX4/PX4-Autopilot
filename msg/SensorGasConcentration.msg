uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # time of data sample (microseconds)

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 concentration_ppm # gas concentration in parts per million (ppm)
float32 temperature       # sensor temperature (Celsius)

uint8 gas_type            # gas type identifier reported by the sensor

uint8 GAS_TYPE_O2  = 1
uint8 GAS_TYPE_CO  = 2
uint8 GAS_TYPE_H2S = 3
uint8 GAS_TYPE_NH3 = 4
uint8 GAS_TYPE_H2  = 5
uint8 GAS_TYPE_O3  = 6
uint8 GAS_TYPE_SO2 = 7
uint8 GAS_TYPE_NO2 = 8
uint8 GAS_TYPE_HCL = 9
uint8 GAS_TYPE_CL2 = 10
uint8 GAS_TYPE_HF  = 11
uint8 GAS_TYPE_PH3 = 12
