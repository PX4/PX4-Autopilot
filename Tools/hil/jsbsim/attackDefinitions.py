from array import array

class Attack(object):
	def __init__(self,nominal,name,units,label,scripts,file_name,variable,attack_values,attack_comment):
		self.nominal = nominal
		self.name = name
		self.units = units
		self.label = label
		self.scripts = scripts
		self.file_name = file_name
		self.variable = variable
		self.attack_values = attack_values
		self.attack_comment = attack_comment

# Generated in attack_cases.ods

digitalUpdateRate = Attack(1,'Digital Update Rate','Hz','Digital Update Rate (Hz)','','digitalUpdateRate','attack.digitalUpdateRate',[(float(x)-0)*1/4 for x in range(0,5)],'0')
throttle = Attack(1,'Throttle Actuator Effectiveness','%','Throttle Actuator Effectiveness (%)','','throttle','attack.actuator.throttle',[(float(x)-0)*1/4 for x in range(0,5)],'Doesn\'t fail it alone')
aileron = Attack(1,'Aileron Actuator Effectiveness','%','Aileron Actuator Effectiveness (%)','','aileron','attack.actuator.aileron',[(float(x)-0)*1/4 for x in range(0,5)],'Not connected')
elevator = Attack(1,'Elevator Actuator Effectiveness','%','Elevator Actuator Effectiveness (%)','','elevator','attack.actuator.elevator',[(float(x)-0)*1/4 for x in range(0,5)],'Doesn\'t fail it alone')
rudder = Attack(1,'Rudder Actuator Effectiveness','%','Rudder Actuator Effectiveness (%)','','rudder','attack.actuator.rudder',[(float(x)-0)*1/4 for x in range(0,5)],'Close to failure at 0')
adsbFreq = Attack(0,'ADS-B Injection Frequency','Hz','ADS-B Injection Frequency (Hz)','','adsbFreq','attack.adsb.freq',[(float(x)-0)*1/4 for x in range(0,5)],'Doesn\'t fail alone, Ineffective at high frequencies ')
gpsLatFreq = Attack(0,'GPS Latitude Injection Frequency','Hz','GPS Latitude Injection Frequency (Hz)','','gpsLatFreq','attack.gps.latFreq',[(float(x)-0)*30/4 for x in range(0,5)],'Fails at low frequencies')
gpsLatAmplitude = Attack(0.001,'GPS Latitude Injection Amplitude','rad','GPS Latitude Injection Amplitude (rad)','','gpsLatAmplitude','attack.gps.latAmplitude',[(float(x)-0)*100/4 for x in range(0,5)],'0')
gpsLonFreq = Attack(0,'GPS Longitude Injection Frequency','Hz','GPS Longitude Injection Frequency (Hz)','','gpsLonFreq','attack.gps.lonFreq',[(float(x)-0)*30/4 for x in range(0,5)],'Fails at low frequencies')
gpsLonAmplitude = Attack(0.001,'GPS Longitude Injection Amplitude','rad','GPS Longitude Injection Amplitude (rad)','','gpsLonAmplitude','attack.gps.lonAmplitude',[(float(x)-0)*100/4 for x in range(0,5)],'0')
gpsAltFreq = Attack(0,'GPS Altitude Injection Frequency','Hz','GPS Altitude Injection Frequency (Hz)','','gpsAltFreq','attack.gps.altFreq',[(float(x)-0)*20/4 for x in range(0,5)],'Fails at low frequencies')
gpsAltAmplitude = Attack(0,'GPS Altitude Injection Amplitude','ft','GPS Altitude Injection Amplitude (ft)','','gpsAltAmplitude','attack.gps.altAmplitude',[(float(x)-0)*100/4 for x in range(0,5)],'0')
gpsLatLonNoise = Attack(0,'GPS Lat-Lon Noise Variance','rad','GPS Lat-Lon Noise Variance (rad)','','gpsLatLonNoise','attack.gps.latLonNoise',[(float(x)-0)*1/4 for x in range(0,5)],'Doesn\'t fail by 10,000')
gpsAltNoise = Attack(0,'GPS Altitude Noise Variance','ft','GPS Altitude Noise Variance (ft)','','gpsAltNoise','attack.gps.altNoise',[(float(x)-0)*100/4 for x in range(0,5)],'0')
gpsVelNoise = Attack(0,'GPS Velocity Noise Deviation','ft/s','GPS Velocity Noise Deviation (ft/s)','','gpsVelNoise','attack.gps.velNoise',[(float(x)-0)*200/4 for x in range(0,5)],'0')
imuGyroNoise = Attack(0,'IMU Gyro Noise Deviation','deg/s','IMU Gyro Noise Deviation (deg/s)','','imuGyroNoise','attack.imu.gyroNoise',[(float(x)-0)*5/4 for x in range(0,5)],'0')
imuAccelNoise = Attack(0,'IMU Accelerometer Noise Variance','ft/s^2','IMU Accelerometer Noise Variance (ft/s^2)','','imuAccelNoise','attack.imu.accelNoise',[(float(x)-0)*100/4 for x in range(0,5)],'0')
magDecNoise = Attack(0,'Magnetometer Dec Noise Variance','rad','Magnetometer Dec Noise Variance (rad)','','magDecNoise','attack.mag.decNoise',[(float(x)-0)*2/4 for x in range(0,5)],'0')
magDipNoise = Attack(0,'Magnetometer Dip Noise Variance','rad','Magnetometer Dip Noise Variance (rad)','','magDipNoise','attack.mag.dipNoise',[(float(x)-0)*0.3/4 for x in range(0,5)],'0')
gainVd = Attack(1,'Down Velocity Gain','%','Down Velocity Gain (%)','','gainVd','attack.gain.Vd',[(float(x)-0.5)*20/4 for x in range(0,5)],'0')
gainVe = Attack(1,'East Velocity Gain','%','East Velocity Gain (%)','','gainVe','attack.gain.Ve',[(float(x)-0.5)*2000/4 for x in range(0,5)],'Won\'t fail by 500')
gainVn = Attack(1,'North Velocity Gain','%','North Velocity Gain (%)','','gainVn','attack.gain.Vn',[(float(x)-0.5)*2000/4 for x in range(0,5)],'Won\'t fail by 500')
gainAccelX = Attack(1,'X Accelerometer Gain','%','X Accelerometer Gain (%)','','gainAccelX','attack.gain.AccelX',[(float(x)-(-0.5))*1/4 for x in range(0,5)],'0')
gainP = Attack(1,'P Gain','%','P Gain (%)','','gainP','attack.gain.P',[(float(x)-0.5)*1/4 for x in range(0,5)],'0')
gainQ = Attack(1,'Q Gain','%','Q Gain (%)','','gainQ','attack.gain.Q',[(float(x)-0.5)*1/4 for x in range(0,5)],'0')
gainQ = Attack(1,'R Gain','%','R Gain (%)','','gainQ','attack.gain.R',[(float(x)-0.5)*1/4 for x in range(0,5)],'0')
gainVt = Attack(1,'True Velocity Gain','%','True Velocity Gain (%)','','gainVt','attack.gain.Vt',[(float(x)-0.5)*1/4 for x in range(0,5)],'0')
gainAlpha = Attack(1,'Alpha Gain','%','Alpha Gain (%)','','gainAlpha','attack.gain.Alpha',[(float(x)-0.5)*1/4 for x in range(0,5)],'0')
gainAlt = Attack(1,'Altimeter Gain','%','Altimeter Gain (%)','','gainAlt','attack.gain.Alt',[(float(x)-0.5)*1/4 for x in range(0,5)],'0')
navA = Attack(0,'A Quaternion Offset','0','A Quaternion Offset','','navA','attack.aNav(xNav.a)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navB = Attack(0,'B Quaternion Offset','0','B Quaternion Offset','','navB','attack.aNav(xNav.b)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navC = Attack(0,'C Quaternion Offset','0','C Quaternion Offset','','navC','attack.aNav(xNav.c)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navD = Attack(0,'D Quaternion Offset','0','D Quaternion Offset','','navD','attack.aNav(xNav.d)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navVn = Attack(0,'North Velocity Offset','ft/s','North Velocity Offset (ft/s)','','navVn','attack.aNav(xNav.vn)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navVe = Attack(0,'East Velocity Offset','ft/s','East Velocity Offset (ft/s)','','navVe','attack.aNav(xNav.ve)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navVd = Attack(0,'Down Velocity Offset','ft/s','Down Velocity Offset (ft/s)','','navVd','attack.aNav(xNav.vd)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navLat = Attack(0,'Latitude Offset','rad','Latitude Offset (rad)','','navLat','attack.aNav(xNav.lat)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navLon = Attack(0,'Longitude Offset','rad','Longitude Offset (rad)','','navLon','attack.aNav(xNav.lon)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')
navAlt = Attack(0,'Altitude Offset','ft','Altitude Offset (ft)','','navAlt','attack.aNav(xNav.alt)',[(float(x)-0.5)*10/4 for x in range(0,5)],'0')

# Add all attacks to list
L=[digitalUpdateRate,throttle,aileron,elevator,rudder,adsbFreq,gpsLatFreq,gpsLatAmplitude,gpsLonFreq,gpsLonAmplitude,gpsAltFreq,gpsAltAmplitude,gpsLatLonNoise,gpsAltNoise,gpsVelNoise,imuGyroNoise,imuAccelNoise,magDecNoise,magDipNoise,gainVd,gainVe,gainVn,gainAccelX,gainP,gainQ,gainQ,gainVt,gainAlpha,gainAlt,navA,navB,navC,navD,navVn,navVe,navVd,navLat,navLon,navAlt]
