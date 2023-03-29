function slBusOut = PoseStamped(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    currentlength = length(slBusOut.Pose);
    for iter=1:currentlength
        slBusOut.Pose(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose(iter),slBusOut(1).Pose(iter),varargin{:});
    end
    slBusOut.Pose = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose,slBusOut(1).Pose,varargin{:});
end
