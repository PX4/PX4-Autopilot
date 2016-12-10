uint8 TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0
uint8 TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO = 1
uint8 TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET = 2
uint8 TELEMETRY_STATUS_RADIO_TYPE_WIRE = 3
uint8 TELEMETRY_STATUS_RADIO_TYPE_USB = 4

uint64 heartbeat_time			# Time of last received heartbeat from remote system 上一次从远程系统接收到心跳的时间
uint64 telem_time			# Time of last received telemetry status packet, 0 for none 		  
							# 上一次接收到数传状态包的时间，0表示没有接收到过
uint8 type				# type of the radio hardware  无线电硬件的类型
uint8 rssi				# local signal strength 本地信号强度
uint8 remote_rssi			# remote signal strength 远程信号强度
uint16 rxerrors				# receive errors 
uint16 fixed				# count of error corrected packets  校正包的错误数
uint8 noise					# background noise level  北京噪声水平
uint8 remote_noise			# remote background noise level 远程噪声水平
uint8 txbuf					# how full the tx buffer is as a percentage
uint8 system_id				# system id of the remote system
uint8 component_id			# component id of the remote system
