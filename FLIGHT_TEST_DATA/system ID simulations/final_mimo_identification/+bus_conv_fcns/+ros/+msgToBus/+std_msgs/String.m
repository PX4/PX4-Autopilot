function slBusOut = String(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Data_SL_Info.ReceivedLength = uint32(strlength(msgIn.Data));
    currlen  = min(slBusOut.Data_SL_Info.ReceivedLength, length(slBusOut.Data));
    slBusOut.Data_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.Data(1:currlen) = uint8(char(msgIn.Data(1:currlen))).';
end
