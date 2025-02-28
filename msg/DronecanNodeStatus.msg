uint64 timestamp					# time since system start (microseconds)

uint16 node_id						# The node ID which this data comes from

# From the uavcan.protocol.NodeStatus message
uint32 uptime_sec					# Node uptime

#
# Abstract node health.
#
uint8 HEALTH_OK         = 0     # The node is functioning properly.
uint8 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.
uint8 HEALTH_ERROR      = 2     # The node encountered a major failure.
uint8 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.
uint8 health

#
# Current mode.
#
# Mode OFFLINE can be actually reported by the node to explicitly inform other network
# participants that the sending node is about to shutdown. In this case other nodes will not
# have to wait OFFLINE_TIMEOUT_MS before they detect that the node is no longer available.
#
# Reserved values can be used in future revisions of the specification.
#
uint8 MODE_OPERATIONAL      = 0         # Normal operating mode.
uint8 MODE_INITIALIZATION   = 1         # Initialization is in progress; this mode is entered immediately after startup.
uint8 MODE_MAINTENANCE      = 2         # E.g. calibration, the bootloader is running, etc.
uint8 MODE_SOFTWARE_UPDATE  = 3         # New software/firmware is being loaded.
uint8 MODE_OFFLINE          = 7         # The node is no longer available.
uint8 mode

#
# Not used currently, keep zero when publishing, ignore when receiving.
#
uint8 sub_mode

#
# Optional, vendor-specific node status code, e.g. a fault code or a status bitmask.
#
uint16 vendor_specific_status_code
