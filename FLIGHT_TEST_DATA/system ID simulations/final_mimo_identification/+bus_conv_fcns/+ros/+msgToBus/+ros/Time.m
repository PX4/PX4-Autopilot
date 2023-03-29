function slBusOut = Time(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Sec = double(msgIn.Sec);
    slBusOut.Nsec = double(msgIn.Nsec);
end
