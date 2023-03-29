function slBusOut = Twist(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Linear);
    for iter=1:currentlength
        slBusOut.Linear(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Linear(iter),slBusOut(1).Linear(iter),varargin{:});
    end
    slBusOut.Linear = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Linear,slBusOut(1).Linear,varargin{:});
    currentlength = length(slBusOut.Angular);
    for iter=1:currentlength
        slBusOut.Angular(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Angular(iter),slBusOut(1).Angular(iter),varargin{:});
    end
    slBusOut.Angular = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Angular,slBusOut(1).Angular,varargin{:});
end
