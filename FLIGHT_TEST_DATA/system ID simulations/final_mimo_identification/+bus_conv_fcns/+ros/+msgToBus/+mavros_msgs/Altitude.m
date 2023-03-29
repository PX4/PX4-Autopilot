function slBusOut = Altitude(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    slBusOut.Monotonic = single(msgIn.Monotonic);
    slBusOut.Amsl = single(msgIn.Amsl);
    slBusOut.Local = single(msgIn.Local);
    slBusOut.Relative = single(msgIn.Relative);
    slBusOut.Terrain = single(msgIn.Terrain);
    slBusOut.BottomClearance = single(msgIn.BottomClearance);
end
