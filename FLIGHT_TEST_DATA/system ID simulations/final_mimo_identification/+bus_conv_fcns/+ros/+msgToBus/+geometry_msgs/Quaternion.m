function slBusOut = Quaternion(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.X = double(msgIn.X);
    slBusOut.Y = double(msgIn.Y);
    slBusOut.Z = double(msgIn.Z);
    slBusOut.W = double(msgIn.W);
end
