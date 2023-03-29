function slBusOut = TwistStamped(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    currentlength = length(slBusOut.Twist);
    for iter=1:currentlength
        slBusOut.Twist(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist(iter),slBusOut(1).Twist(iter),varargin{:});
    end
    slBusOut.Twist = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist,slBusOut(1).Twist,varargin{:});
end
