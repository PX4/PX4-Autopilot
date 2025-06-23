#pragma once

#define MODULE_ID 0x10

union converter{
    uint8_t bytes[4];
    float value;
};

// To understand how this works see the following link: https://godbolt.org/z/9edexce1j
union can_id_u {
		uint32_t id;
		struct  {
            		uint32_t command_type : 4;
            		uint32_t session : 3;
            		uint32_t client_id_src : 5;
            		uint32_t module_id_src : 5;
			uint32_t client_id_des : 5;
            		uint32_t module_id_des : 5;
			uint32_t emergency : 2; // Emergency status
			uint32_t rest : 3;
		} can_id_seg;
	};

// Enum of available HU Robosub module IDs
typedef enum
{
  GLOBAL    = 0x00,
  PIXHAWK   = 0x10,
  MAINBRAIN = 0x11,
  BUOYANCY  = 0x04,
  HYDRAULIC = 0X05,
  POWER     = 0x08,
  TEST      = 0x0F
} modules;
