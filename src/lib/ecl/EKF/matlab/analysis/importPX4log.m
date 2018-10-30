function allData = importPX4log(fname,keep_msgs)

% import a .px4log file
% INPUTS
%   fname: path to a valid .px4log file
%   keep_msgs: cell array of message names to keep
% OUTPUT
%   allData: a Matlab struct with a field names for each message name in the log
%   file. The content of each field is itself a struct with field names for
%   each label in the corresponding message. The content of each of *these*
%   fields will be an array of message data that is nonempty if the message
%   name appears in keep_msgs

% import method essentially translated from sdlog2_dump.py
% George Hines, 3D Robotics, Berkeley, CA
% 7 April 2016

BLOCK_SIZE = 8192;
MSG_HEADER_LEN = 3;
MSG_HEAD1 = uint8(hex2dec('A3'));
MSG_HEAD2 = uint8(hex2dec('95'));
MSG_FORMAT_PACKET_LEN = 89;
MSG_TYPE_FORMAT = uint8(hex2dec('80'));
% MSG_FORMAT_STRUCT = {'uint8','uint8','char4','char16','char64'};
FORMAT_TO_STRUCT = {
    'b', {'int8', 1};
    'B', {'uint8', 1};
    'h', {'int16', 1};
    'H', {'uint16', 1};
    'i', {'int32', 1};
    'I', {'uint32', 1};
    'f', {'single', 1};
    'd', {'double', 1};
    'n', {'char4', 1};
    'N', {'char16', 1};
    'Z', {'char64', 1};
    'c', {'int16', 0.01};
    'C', {'uint16', 0.01};
    'e', {'int32', 0.01};
    'E', {'uint32', 0.01};
    'L', {'int32', 0.0000001};
    'M', {'uint8', 1};
    'q', {'int64', 1};
    'Q', {'uint64', 1}};

fid = fopen(fname,'r','b');
fInfo = dir(fname);
totalBytes = fInfo.bytes;
bytes_read = 0;
msg_descrs = {};
buffer = [];
ptr = 1;
allData = [];
nextPrint = 0;
disp('Reading file');
while 1
    percentDone = bytes_read / totalBytes * 100;
    if percentDone >= nextPrint
        fprintf('%.0f%%\n',percentDone);
        nextPrint = nextPrint + 5;
    end
    
    chunk = fread(fid,BLOCK_SIZE,'uint8');
    if numel(chunk) == 0;
        break
    end
    buffer = [buffer(ptr:end), chunk'];
    ptr = 1;
    while numel(buffer) - ptr > MSG_HEADER_LEN
        head1 = buffer(ptr);
        head2 = buffer(ptr+1);
        if head1 ~= MSG_HEAD1 || head2 ~= MSG_HEAD2
            ptr = ptr + 1;
            continue;
        end
        msg_type = buffer(ptr+2);
        
        if msg_type == MSG_TYPE_FORMAT
            if numel(buffer) - ptr <= MSG_FORMAT_PACKET_LEN
                break;
            end
            % return new pointer, and all message descriptor info
            [ptr, msg_descr] = LOCAL_parse_message_descriptors(buffer, ptr, MSG_TYPE_FORMAT, MSG_FORMAT_PACKET_LEN, FORMAT_TO_STRUCT);
            msg_descrs(msg_descr{1},:) = msg_descr;
            cells = repmat({inf(1,500000)},1,numel(msg_descr{5}));
            cells(msg_descr{4}=='n' | msg_descr{4} == 'N' | msg_descr{4} == 'Z') = {[]};
            seed = [{'index'},{'Tsec'},msg_descr{5};[{1},{inf(1,500000)},cells]];
            allData.(msg_descr{3}) = struct(seed{:});
        else
            msg_descr = msg_descrs(msg_type,:);
            msg_length = msg_descr{2};
            if numel(buffer) - ptr <= msg_length
                break;
            end
            % return new pointer, and all message info
            if strcmp(msg_descr{3},'TIME') || any(strcmp(msg_descr{3}, keep_msgs)) || isempty(keep_msgs)
                [ptr,msg_data] = LOCAL_parse_message(buffer, ptr, MSG_HEADER_LEN, msg_descr);
                ind = allData.(msg_descr{3}).index;
                for k = 1:numel(msg_data)
                    if isnumeric(msg_data{k})
                        allData.(msg_descr{3}).(msg_descr{5}{k})(ind) = msg_data{k};
                        try
                            allData.(msg_descr{3}).Tsec(ind) = double(allData.TIME.StartTime(max(1,allData.TIME.index-1)))*1e-6;
                        end
                        noInc = false;
                    else
                        allData.(msg_descr{3}).(msg_descr{5}{k}) = [allData.(msg_descr{3}).(msg_descr{5}{k}), msg_data(k)];
                        noInc = true;
                    end
                end
                if ~noInc
                    allData.(msg_descr{3}).index = ind + 1;
                end
            else
                ptr = ptr + msg_descr{2};
            end
        end
    end
    bytes_read = bytes_read + ptr;
end

disp('Releasing excess preallocated memory');
% clean out inf values
fields1 = fieldnames(allData);
for k = 1:numel(fields1)
    fields2 = fieldnames(allData.(fields1{k}));
    for m = 1:numel(fields2)
        if isnumeric(allData.(fields1{k}).(fields2{m}))
            allData.(fields1{k}).(fields2{m})(isinf(allData.(fields1{k}).(fields2{m}))) = [];
        end
    end
end
disp('Done');
end

function [ptr, msg_descr] = LOCAL_parse_message_descriptors(buffer, ptr, MSG_TYPE_FORMAT, MSG_FORMAT_PACKET_LEN, FORMAT_TO_STRUCT)
thisBlock = buffer((ptr+3):(ptr+MSG_FORMAT_PACKET_LEN+1));
msg_descr = cell(1,7);
msg_type = thisBlock(1);
%if msg_type ~= MSG_TYPE_FORMAT
    msg_length = thisBlock(2);
    msg_name = LOCAL_parse_string(thisBlock(3:6));
    msg_format = LOCAL_parse_string(thisBlock(7:22));
    msg_labels = strsplit(LOCAL_parse_string(thisBlock(23:end)),',');
    msg_struct = cell(1,numel(msg_format));
    msg_mults = zeros(1,numel(msg_format));
    
    for k = 1:numel(msg_format)
        info = FORMAT_TO_STRUCT{strcmp(msg_format(k),FORMAT_TO_STRUCT(:,1)),2};
        msg_struct{k} = info{1};
        msg_mults(k) = info{2};
    end
    if isempty([msg_labels{:}])
        msg_labels = {'none'};
    end
    msg_descr = {msg_type, msg_length, msg_name, msg_format, msg_labels, msg_struct, msg_mults};
%end
ptr = ptr + MSG_FORMAT_PACKET_LEN;
end

function [ptr, data] = LOCAL_parse_message(buffer, ptr, MSG_HEADER_LEN, msg_descr)
[~, msg_length, ~, ~, ~, msg_struct, msg_mults] = msg_descr{:};

% unpack message per the format specifiers in msg_struct
data = cell(1,numel(msg_struct));
thisBytes = buffer((ptr+MSG_HEADER_LEN):(ptr+msg_length+1));
thisPtr = 1;
for k = 1:numel(msg_struct)
    % parse the data chunk per msg_struct{k}
    [dataLen,data{k}] = LOCAL_unpack(thisPtr, thisBytes, msg_struct{k}, msg_mults(k));
    thisPtr = thisPtr + dataLen;
end
ptr = ptr + msg_length;
end

function [dataLen, data] = LOCAL_unpack(thisPtr, byte_array, format_type, mult)
if strncmp('char',format_type,4)
    dataLen = str2double(format_type(5:end));
    data = LOCAL_parse_string(byte_array(thisPtr:(thisPtr+dataLen)));
else
    if strncmp('int',format_type,3)
        dataLen = str2double(format_type(4:end))/8;
    elseif strncmp('uint',format_type,4)
        dataLen = str2double(format_type(5:end))/8;
    elseif strcmp('single',format_type)
        dataLen = 4;
    end
    data = double(typecast(uint8(byte_array(thisPtr:(thisPtr+dataLen-1))),format_type))*mult;
end
end

function str = LOCAL_parse_string(byteArray)
firstZero = find(byteArray==0);
if ~isempty(firstZero)
    str = char(byteArray(1:firstZero-1));
else
    str = char(byteArray);
end
end