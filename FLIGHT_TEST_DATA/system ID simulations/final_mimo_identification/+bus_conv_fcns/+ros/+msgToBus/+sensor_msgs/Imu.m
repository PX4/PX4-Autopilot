function slBusOut = Imu(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    currentlength = length(slBusOut.Orientation);
    for iter=1:currentlength
        slBusOut.Orientation(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Quaternion(msgIn.Orientation(iter),slBusOut(1).Orientation(iter),varargin{:});
    end
    slBusOut.Orientation = bus_conv_fcns.ros.msgToBus.geometry_msgs.Quaternion(msgIn.Orientation,slBusOut(1).Orientation,varargin{:});
                    currentlength = length(slBusOut.OrientationCovariance);
                    slBusOut.OrientationCovariance = double(msgIn.OrientationCovariance(1:currentlength));
    currentlength = length(slBusOut.AngularVelocity);
    for iter=1:currentlength
        slBusOut.AngularVelocity(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.AngularVelocity(iter),slBusOut(1).AngularVelocity(iter),varargin{:});
    end
    slBusOut.AngularVelocity = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.AngularVelocity,slBusOut(1).AngularVelocity,varargin{:});
                    currentlength = length(slBusOut.AngularVelocityCovariance);
                    slBusOut.AngularVelocityCovariance = double(msgIn.AngularVelocityCovariance(1:currentlength));
    currentlength = length(slBusOut.LinearAcceleration);
    for iter=1:currentlength
        slBusOut.LinearAcceleration(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.LinearAcceleration(iter),slBusOut(1).LinearAcceleration(iter),varargin{:});
    end
    slBusOut.LinearAcceleration = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.LinearAcceleration,slBusOut(1).LinearAcceleration,varargin{:});
                    currentlength = length(slBusOut.LinearAccelerationCovariance);
                    slBusOut.LinearAccelerationCovariance = double(msgIn.LinearAccelerationCovariance(1:currentlength));
end
