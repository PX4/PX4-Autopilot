# Gas concentration
#
# Provides gas concentration measurements from a gas sensor, including concentration, temperature, and gas type.
# Used for environmental monitoring and reporting to the operator.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Time of data sample

uint32 device_id # [-] Unique device ID for the sensor that does not change between power cycles

float32 concentration_ppm # [ppm] Gas concentration
float32 temperature # [degC] Sensor temperature

uint8 gas_type # [@enum GAS_TYPE] Gas type identifier reported by the sensor

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
