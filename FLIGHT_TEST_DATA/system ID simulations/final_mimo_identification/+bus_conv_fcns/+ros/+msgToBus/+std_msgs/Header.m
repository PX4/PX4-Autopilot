function slBusOut = Header(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Seq = uint32(msgIn.Seq);
    currentlength = length(slBusOut.Stamp);
    for iter=1:currentlength
        slBusOut.Stamp(iter) = bus_conv_fcns.ros.msgToBus.ros.Time(msgIn.Stamp(iter),slBusOut(1).Stamp(iter),varargin{:});
    end
    slBusOut.Stamp = bus_conv_fcns.ros.msgToBus.ros.Time(msgIn.Stamp,slBusOut(1).Stamp,varargin{:});
    slBusOut.FrameId_SL_Info.ReceivedLength = uint32(strlength(msgIn.FrameId));
    currlen  = min(slBusOut.FrameId_SL_Info.ReceivedLength, length(slBusOut.FrameId));
    slBusOut.FrameId_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.FrameId(1:currlen) = uint8(char(msgIn.FrameId(1:currlen))).';
end
