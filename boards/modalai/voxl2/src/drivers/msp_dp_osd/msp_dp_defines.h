#pragma once

// protocol bytes
#define MSP_HEADER '$'
#define MSP_START  'M'
#define MSP_REPLY  '>'
#define MSP_CMD    '<'

// requests & replies
#define MSP_VTX_CONFIG            88 //out message         	Get vtx settings 
#define MSP_SET_VTX_CONFIG        89 //in message          	Set vtx settings 
#define MSP_CMD_DISPLAYPORT      182 //out message			Write osd to display port 
#define MSP_SET_OSD_CANVAS       188 // in message          Set osd canvas size COLSxROWS
#define MSP_OSD_CANVAS           189 // out message         Get osd canvas size COLSxROWS

#define MSP_OSD_DP_WRITE_PAYLOAD 4
#define MSP_OSD_MAX_STRING_LENGTH 30 
#define DISPLAYPORT_MSP_ATTR_BLINK   (1UL << (6)) // Device local blink

// VTX CONFIG 
struct msp_dp_vtx_config_t {
	uint8_t  protocol;
	uint8_t  band;	
	uint8_t  channel;
	uint8_t  power;	
	uint8_t  pit;	 
	uint16_t freq;	
} __attribute__((packed));

// STATUS 
struct msp_dp_status_t {
	uint8_t  unused0;
	uint8_t  unused1;
	uint8_t  unused2;
	uint8_t  unused3;
	uint8_t  unused4;
	uint8_t  unused5;
	uint8_t  armed;				//msp_rx_buf[6] 
	uint8_t  arming_disable_flags_count;
	uint32_t arming_disable_flags;
} __attribute__((packed));

#define SD_COL_MAX 30
#define SD_ROW_MAX 16
#define HD_COL_MAX 50
#define HD_ROW_MAX 18
// OSD CANVAS 
struct msp_dp_canvas_t {
	uint8_t  row_max;
	uint8_t  col_max;
} __attribute__((packed));

// DISPLAYPORT CONFIG
struct msp_dp_config_t {
	uint8_t subcmd;
	uint8_t fontType;
	uint8_t resolution;
} __attribute__((packed));

// DISPLAYPORT WRITE COMMAND 
struct msp_dp_cmd_t {	// Message is added later since it can be variable size and HDZero OSD writes the whole buffer sent, doesn't stop at delimiter
	uint8_t subcmd;
	uint8_t row;	
	uint8_t col;	
	uint8_t attr{0};		// HDZero VTX doesn't support blinking
} __attribute__((packed));

struct msp_dp_rc_sticks_t {
	int32_t throttle;
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
} __attribute__((packed));

// Display Port command enums
typedef enum __attribute__((packed)){
    MSP_DP_HEARTBEAT = 0,       // Heartbeat
    MSP_DP_RELEASE = 1,         // Release the display after clearing and updating
    MSP_DP_CLEAR_SCREEN = 2,    // Clear the display
    MSP_DP_WRITE_STRING = 3,    // Write a string at given coordinates
    MSP_DP_DRAW_SCREEN = 4,     // Trigger a screen draw
    MSP_DP_CONFIG = 5,          // CONFIG COMMAND -Not used by Betaflight, used by Ardupilot and INAV // CONFIG COMM
    MSP_DP_SYS = 6,             // Display system element displayportSystemElement_e at given coordinates
    MSP_DP_COUNT,
} displayportMspCommand_e ;

// OSD width x height
typedef enum __attribute__((packed)) {
    SD_3016,	
    HD_5018,
    HD_3016,
    HD_5320,
} resolutionType_e;

