# External gimbal manager status
#
# Status of a gimbal manager running on another component (not us), decoded from
# an incoming GIMBAL_MANAGER_STATUS message. Used to track which component
# currently controls an external gimbal manager.

uint64 timestamp # [us] Time since system start

uint8 manager_sysid # [-] System id of the external gimbal manager
uint8 manager_compid # [-] Component id of the external gimbal manager

uint32 flags # [-]
uint8 gimbal_device_id
uint8 primary_control_sysid
uint8 primary_control_compid
uint8 secondary_control_sysid
uint8 secondary_control_compid
