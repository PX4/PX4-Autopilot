function output = RunFilter(param,imu_data,mag_data,baro_data,gps_data,varargin)

% compulsory inputs

% param : parameters defined  by SetParameterDefaults.m
% imu_data : IMU delta angle and velocity data in body frame
% mag_data : corrected magnetometer field measurements in body frame
% baro_data : barometric height measurements
% gps_data : GPS NED pos vel measurements in local earth frame

% optional inputs

% rng _data : measurements for a Z body axis aligned range finder
% flow_data : XY axis optical flow angular rate measurements in body frame
% viso_data : ZED camera visula odometry measurements

nVarargs = length(varargin);
if nVarargs >= 2
    useOpticalFlow = ~isempty(varargin{1}) && ~isempty(varargin{2});
    rng_data = varargin{1};
    flow_data = varargin{2};
    if useOpticalFlow
        fprintf('Using optical Flow Data\n',nVarargs);
    end
else
    useOpticalFlow = 0;
end

if nVarargs >= 3
    useVisualOdometry = ~isempty(varargin{3});
    viso_data = varargin{3};
    if useVisualOdometry
        fprintf('Using ZED camera odometry data\n',nVarargs);
    end
else
    useVisualOdometry = 0;
end


%% Set initial conditions

% constants
deg2rad = pi/180;
gravity = 9.80665; % initial value of gravity - will be updated when WGS-84 position is known

% initialise the state vector at the first position where there is OK GPS
[states, imu_start_index] = InitStates(param,imu_data,gps_data,mag_data,baro_data);

dt_imu_avg = 0.5 * (median(imu_data.gyro_dt) + median(imu_data.accel_dt));
indexStop = length(imu_data.time_us) - imu_start_index;
indexStart = 1;

% create static structs for output data
output = struct('time_lapsed',[]',...
    'euler_angles',[],...
    'velocity_NED',[],...
    'position_NED',[],...
    'gyro_bias',[],...
    'accel_bias',[],...
    'mag_NED',[],...
    'mag_XYZ',[],...
    'wind_NE',[],...
    'dt',0,...
    'state_variance',[],...
    'innovations',[],...
    'magFuseMethod',[]);

% initialise the state covariance matrix
covariance = InitCovariance(param,dt_imu_avg,1,gps_data);

%% Main Loop
imuIndex = imu_start_index;
last_gps_index = 0;
gps_fuse_index = 0;
last_baro_index = 0;
baro_fuse_index = 0;
last_mag_index = 0;
mag_fuse_index = 0;
last_flow_index = 0;
flow_fuse_index = 0;
last_viso_index = 0;
viso_fuse_index = 0;
delAngCov = [0;0;0];
delVelCov = [0;0;0];
dtCov = 0;
dtCovInt = 0;
covIndex = 0;
output.magFuseMethod = param.fusion.magFuseMethod;
range = 0.1;
latest_range_index = 1;
for index = indexStart:indexStop
    
    % read IMU measurements
    localTime=imu_data.time_us(imuIndex)*1e-6;
    delta_angle(:,1) = imu_data.del_ang(imuIndex,:);
    delta_velocity(:,1) = imu_data.del_vel(imuIndex,:);
    dt_imu = 0.5 * (imu_data.accel_dt(imuIndex) + imu_data.gyro_dt(imuIndex));
    imuIndex = imuIndex+1;
    
    % predict states
    [states, delAngCorrected, delVelCorrected]  = PredictStates(states,delta_angle,delta_velocity,imu_data.accel_dt(imuIndex),gravity,gps_data.refLLH(1,1)*deg2rad);
    
    % constrain states
    [states]  = ConstrainStates(states,dt_imu_avg);
    
    dtCov = dtCov + dt_imu;
    delAngCov = delAngCov + delAngCorrected;
    delVelCov = delVelCov + delVelCorrected;
    if (dtCov > 0.01)
        % predict covariance
        covariance  = PredictCovariance(delAngCov,delVelCov,states,covariance,dtCov,param);
        delAngCov = [0;0;0];
        delVelCov = [0;0;0];
        dtCovInt = dtCovInt + dtCov;
        dtCov = 0;
        covIndex = covIndex + 1;
        
        % output state data
        output.time_lapsed(covIndex) = localTime;
        output.euler_angles(covIndex,:) = QuatToEul(states(1:4)')';
        output.velocity_NED(covIndex,:) = states(5:7)';
        output.position_NED(covIndex,:) = states(8:10)';
        output.gyro_bias(covIndex,:) = states(11:13)';
        output.accel_bias(covIndex,:) = states(14:16)';
        output.mag_NED(covIndex,:) = states(17:19);
        output.mag_XYZ(covIndex,:) = states(20:22);
        output.wind_NE(covIndex,:) = states(23:24);
        
        % output covariance data
        for i=1:24
            output.state_variances(covIndex,i) = covariance(i,i);
        end
        
        % Fuse new GPS data that has fallen beind the fusion time horizon
        latest_gps_index = find((gps_data.time_us - 1e6 * param.fusion.gpsTimeDelay) < imu_data.time_us(imuIndex), 1, 'last' );
        if (latest_gps_index > last_gps_index)
            last_gps_index = latest_gps_index;
            gps_fuse_index = gps_fuse_index + 1;
            
            % fuse NED GPS velocity
            [states,covariance,velInnov,velInnovVar] = FuseVelocity(states,covariance,gps_data.vel_ned(latest_gps_index,:),param.fusion.gpsVelGate,gps_data.spd_error(latest_gps_index));
            
            % data logging
            output.innovations.vel_time_lapsed(gps_fuse_index) = localTime;
            output.innovations.vel_innov(gps_fuse_index,:) = velInnov';
            output.innovations.vel_innov_var(gps_fuse_index,:) = velInnovVar';
            
            % fuse NE GPS position
            [states,covariance,posInnov,posInnovVar] = FusePosition(states,covariance,gps_data.pos_ned(latest_gps_index,:),param.fusion.gpsPosGate,gps_data.pos_error(latest_gps_index));
            
            % data logging
            output.innovations.pos_time_lapsed(gps_fuse_index) = localTime;
            output.innovations.posInnov(gps_fuse_index,:) = posInnov';
            output.innovations.posInnovVar(gps_fuse_index,:) = posInnovVar';
        end
        
        % Fuse new Baro data that has fallen beind the fusion time horizon
        latest_baro_index = find((baro_data.time_us - 1e6 * param.fusion.baroTimeDelay) < imu_data.time_us(imuIndex), 1, 'last' );
        if (latest_baro_index > last_baro_index)
            last_baro_index = latest_baro_index;
            baro_fuse_index = baro_fuse_index + 1;
            
            % fuse baro height
            [states,covariance,hgtInnov,hgtInnovVar] = FuseBaroHeight(states,covariance,baro_data.height(latest_baro_index),param.fusion.baroHgtGate,param.fusion.baroHgtNoise);
            
            % data logging
            output.innovations.hgt_time_lapsed(baro_fuse_index) = localTime;
            output.innovations.hgtInnov(baro_fuse_index) = hgtInnov;
            output.innovations.hgtInnovVar(baro_fuse_index) = hgtInnovVar;
        end
        
        % Fuse new mag data that has fallen behind the fusion time horizon
        latest_mag_index = find((mag_data.time_us - 1e6 * param.fusion.magTimeDelay) < imu_data.time_us(imuIndex), 1, 'last' );
        if (latest_mag_index > last_mag_index)
            last_mag_index = latest_mag_index;
            mag_fuse_index = mag_fuse_index + 1;
            
            % output magnetic field length to help with diagnostics
            output.innovations.magLength(mag_fuse_index) = sqrt(dot(mag_data.field_ga(latest_mag_index,:),mag_data.field_ga(latest_mag_index,:)));
            
            % fuse magnetometer data
            if (param.fusion.magFuseMethod == 0 || param.fusion.magFuseMethod == 1)
                [states,covariance,magInnov,magInnovVar] = FuseMagnetometer(states,covariance,mag_data.field_ga(latest_mag_index,:),param.fusion.magFieldGate, param.fusion.magFieldError^2);
                
                % data logging
                output.innovations.mag_time_lapsed(mag_fuse_index) = localTime;
                output.innovations.magInnov(mag_fuse_index,:) = magInnov;
                output.innovations.magInnovVar(mag_fuse_index,:) = magInnovVar;
                
                if (param.fusion.magFuseMethod == 1)
                    % fuse in the local declination value
                    [states, covariance] = FuseMagDeclination(states, covariance, param.fusion.magDeclDeg*deg2rad);
                    
                end
                
            elseif (param.fusion.magFuseMethod == 2)
                % fuse magnetomer data as a single magnetic heading measurement
                [states, covariance, hdgInnov, hdgInnovVar] = FuseMagHeading(states, covariance, mag_data.field_ga(latest_mag_index,:), param.fusion.magDeclDeg*deg2rad, param.fusion.magHdgGate, param.fusion.magHdgError^2);
                
                % log data
                output.innovations.mag_time_lapsed(mag_fuse_index) = localTime;
                output.innovations.hdgInnov(mag_fuse_index) = hdgInnov;
                output.innovations.hdgInnovVar(mag_fuse_index) = hdgInnovVar;
                
            end
            
        end
        
        % Attempt to use optical flow and range finder data if available
        if (useOpticalFlow)
            
            % Get latest range finder data and gate to remove dropouts
            latest_range_index = find((rng_data.time_us - 1e6 * param.fusion.rangeTimeDelay) < imu_data.time_us(imuIndex), 1, 'last' );
            if (rng_data.dist(latest_range_index) < 5.0 && rng_data.dist(latest_range_index) > 0.05)
                range = rng_data.dist(latest_range_index);
            end
            
            % Fuse optical flow data that has fallen behind the fusion time horizon
            latest_flow_index = find((flow_data.time_us - 1e6 * param.fusion.flowTimeDelay) < imu_data.time_us(imuIndex), 1, 'last' );
            if (latest_flow_index > last_flow_index)
                last_flow_index = latest_flow_index;
                flow_fuse_index = flow_fuse_index + 1;
                
                % fuse flow data
                flowRate = [flow_data.flowX(latest_flow_index);flow_data.flowY(latest_flow_index)];
                bodyRate = [flow_data.bodyX(latest_flow_index);flow_data.bodyY(latest_flow_index)];
                [states,covariance,flowInnov,flowInnovVar] = FuseOpticalFlow(states, covariance, flowRate, bodyRate, range, param.fusion.flowRateError^2, param.fusion.flowGate);
                
                % data logging
                output.innovations.flow_time_lapsed(flow_fuse_index) = localTime;
                output.innovations.flowInnov(flow_fuse_index,:) = flowInnov;
                output.innovations.flowInnovVar(flow_fuse_index,:) = flowInnovVar;
                
            end
            
        end
        
        % attempt to use ZED camera visual odmetry data if available
        if (useVisualOdometry)
            
            % Fuse ZED camera body frame odmometry data that has fallen behind the fusion time horizon
            latest_viso_index = find((viso_data.time_us - 1e6 * param.fusion.bodyVelTimeDelay) < imu_data.time_us(imuIndex), 1, 'last' );
            if (latest_viso_index > last_viso_index)
                last_viso_index = latest_viso_index;
                viso_fuse_index = viso_fuse_index + 1;
                
                % convert delta positon measurements to velocity
                relVelBodyMea = [viso_data.dVelX(latest_viso_index);viso_data.dVelY(latest_viso_index);viso_data.dVelZ(latest_viso_index)]./viso_data.dt(latest_viso_index);
                
                % convert quality metric to equivalent observation error
                % (this is a guess)
                quality = viso_data.qual(latest_viso_index);
                bodyVelError = param.fusion.bodyVelErrorMin * quality + param.fusion.bodyVelErrorMax * (1 - quality);
                
                % fuse measurements
                [states,covariance,bodyVelInnov,bodyVelInnovVar] = FuseBodyVel(states, covariance, relVelBodyMea, bodyVelError^2, param.fusion.bodyVelGate);
                
                % data logging
                output.innovations.bodyVel_time_lapsed(viso_fuse_index) = localTime;
                output.innovations.bodyVelInnov(viso_fuse_index,:) = bodyVelInnov;
                output.innovations.bodyVelInnovVar(viso_fuse_index,:) = bodyVelInnovVar;
                
            end
            
        end
        
    end
    
    % update average delta time estimate
    output.dt = dtCovInt / covIndex;
    
end

end