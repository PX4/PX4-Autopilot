function slBusOut = RCIn(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    slBusOut.Rssi = uint8(msgIn.Rssi);
    maxlength = length(slBusOut.Channels);
    recvdlength = length(msgIn.Channels);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'Channels', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.Channels_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.Channels_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.Channels = uint16(msgIn.Channels(1:slBusOut.Channels_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.Channels(recvdlength+1:maxlength) = 0;
    end
end
