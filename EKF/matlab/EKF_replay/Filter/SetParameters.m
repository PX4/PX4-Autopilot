%% Filter Control
param.control.waitForGps = 0; % set to 1 if the filter start should be delayed until GPS checks to pass
param.control.gpsSpdErrLim = 1.0; % GPS use will not start if reported GPS speed error is greater than this (m/s)
param.control.gpsPosErrLim = 5.0; % GPS use will not start if reported GPS position error is greater than this (m)
param.control.velDriftTimeLim = 5.0; % The maximum time without observations to constrain velocity drift before a zero velocity is fused to prevent the filter diverging (sec)
param.control.gpsOffTime = 0; % GPS aiding will be turned off at this time (sec)
param.control.gpsOnTime = 0; % GPS aiding will be turned back on at this time (sec)
param.control.flowOffTime = 0; % optical flow aiding will be turned off at this time (sec)
param.control.flowOnTime = 0; % optical flow aiding will be turned back on on at this time (sec)
param.control.visoOffTime = 0; % visual odometry aiding will be turned off at this time (sec)
param.control.visoOnTime = 0; % visual odometry aiding will be turned back on at this time (sec)
param.control.rollAlignErr = 0.0; % initial roll misalignment (rad)
param.control.pitchAlignErr = 0.0; % initial pitch misalignment (rad)
param.control.yawAlignErr = 0.0; % initial yaw misalignment (rad)

%% GPS fusion
param.fusion.gpsTimeDelay = 0.1; % GPS measurement delay relative to IMU (sec)
param.fusion.gpsVelGate = 5.0; % Size of the IMU velocity innovation consistency check gate in SD
param.fusion.gpsPosGate = 5.0; % Size of the IMU velocity innovation consistency check gate in SD
param.fusion.gpsCheckTimeout = 10.0; % Length of time that GPS measurements will be rejected by the filter before states are reset to the GPS velocity. (sec)

%% Baro fusion
param.fusion.baroTimeDelay = 0.05; % Baro measurement delay relative to IMU (sec)
param.fusion.baroHgtGate = 5.0; % Size of the IMU velocity innovation consistency check gate in SD
param.fusion.baroHgtNoise = 2.0; % 1SD observation noise of the baro measurements (m)

%% Magnetometer measurement fusion
param.fusion.magTimeDelay = 0.0; % magnetometer time delay relative to IMU (sec)
param.fusion.magFuseMethod = 1; % 0: 3-Axis field fusion with free declination, 1: 3-Axis field fusion with constrained declination, 2: magnetic heading fusion. (None)
param.fusion.magFieldError = 0.05; % Magnetic field measurement 1SD error including hard and soft iron interference. Used when magFuseMethod = 0 or 1. (gauss)
param.fusion.magHdgError = 0.1745; % Magnetic heading measurement 1SD error including hard and soft iron interference. Used when magFuseMethod = 2. (rad)
param.fusion.magFieldGate = 5.0; % Size of the magnetic field innovation consistency check gate in SD
param.fusion.magHdgGate = 5.0; % Size of the magnetic heading innovation consistency check gate in SD
param.fusion.magDeclDeg = 10.5; % Magnetic declination in deg

%% Optical flow measurement fusion
param.fusion.rangeTimeDelay = 0.05; % range fidner sensor delay relative to IMU (sec)
param.fusion.flowTimeDelay = 0.05; % Optical flow sensor time delay relative to IMU (sec)
param.fusion.flowRateError = 0.15; % Observation noise 1SD for the flow sensor (rad/sec)
param.fusion.flowGate = 5.0; % Size of the optical flow rate innovation consistency check gate in SD
param.fusion.rngValidMin = 0.05; % range measurements wil be constrained to be  no less than this (m)
param.fusion.rngValidMax = 5.0; % ignore range measurements larger than this (m)
param.fusion.rngTimeout = 2.0; % optical flow measurements will not be used if more than this time since valid range finder data was received (sec)

%% Visual odometry body frame velocity measurement fusion
param.fusion.bodyVelTimeDelay = 0.01; % Optical flow sensor time delay relative to IMU (sec)
param.fusion.bodyVelErrorMin = 0.1; % Observation noise 1SD for the odometry sensor at the highest quality value (m/sec)
param.fusion.bodyVelErrorMax = 0.9; % Observation noise 1SD for the odometry sensor at the lowest quality value (m/sec)
param.fusion.bodyVelGate = 5.0; % Size of the optical flow rate innovation consistency check gate in SD

%% State prediction error growth
param.prediction.magPnoiseNED = 1e-3; % Earth magnetic field 1SD rate of change. (gauss/sec)
param.prediction.magPnoiseXYZ = 1e-3; % Body magnetic field 1SD rate of change. (gauss/sec)
param.prediction.dAngBiasPnoise = 0.001; % IMU gyro bias 1SD rate of change (rad/sec^2)
param.prediction.dVelBiasPnoise = 0.03; % IMU accel bias 1SD rate of change (m/sec^3)
param.prediction.angRateNoise = 0.015; % IMU gyro 1SD rate process noise (rad/sec)
param.prediction.accelNoise = 0.35; % IMU accelerometer 1SD error noise including switch on bias uncertainty. (m/sec^2)

%% Initial Uncertainty
param.alignment.posErrNE = 10.0; % Initial 1SD position error when aligning without GPS. (m/sec)
param.alignment.velErrNE = 5.0; % Initial 1SD velocity error when aligning without GPS. (m/sec)
param.alignment.velErrD = 1.0; % Initial 1SD vertical velocity error when aligning without GPS. (m/sec)
param.alignment.delAngBiasErr = 0.05*pi/180; % Initial 1SD rate gyro bias uncertainty. (rad/sec)
param.alignment.delVelBiasErr = 0.07; % Initial 1SD accelerometer bias uncertainty. (m/sec^2)
param.alignment.quatErr = 0.1; % Initial 1SD uncertainty in quaternion.
param.alignment.magErrXYZ = 0.01; % Initial 1SD uncertainty in body frame XYZ magnetic field states. (gauss)
param.alignment.magErrNED = 0.5; % Initial 1SD uncertainty in earth frame NED magnetic field states. (gauss)
param.alignment.hgtErr = 0.5; % Initial 1SD uncertainty in height. (m)
param.alignment.windErrNE = 5.0; % Initial 1SD error in wind states. (m/sec)
