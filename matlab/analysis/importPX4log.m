function importPX4log(fname)
fid = fopen(fname,'r','b');
bytes_read = 0;

BLOCK_SIZE = 8192;
MSG_HEADER_LEN = 3;
MSG_HEAD1 = uint8(hex2dec('A3'));
MSG_HEAD2 = uint8(hex2dec('95'));
MSG_FORMAT_PACKET_LEN = 89;
MSG_FORMAT_STRUCT = 'BB4s16s64s';
MSG_TYPE_FORMAT = uint8(hex2dec('80'));

% FORMAT_TO_STRUCT = {
%     "b": ("b", None),
%     "B": ("B", None),
%     "h": ("h", None),
%     "H": ("H", None),
%     "i": ("i", None),
%     "I": ("I", None),
%     "f": ("f", None),
%     "n": ("4s", None),
%     "N": ("16s", None),
%     "Z": ("64s", None),
%     "c": ("h", 0.01),
%     "C": ("H", 0.01),
%     "e": ("i", 0.01),
%     "E": ("I", 0.01),
%     "L": ("i", 0.0000001),
%     "M": ("b", None),
%     "q": ("q", None),
%     "Q": ("Q", None),
%     }

while 1
    chunk = fread(fid,BLOCK_SIZE,'uint8');
    if numel(chunk) == 0;
        break
    end
    buffer = chunk;
    ptr = 1;
    while numel(buffer) - ptr >= MSG_HEADER_LEN
        head1 = buffer(ptr);
        head2 = buffer(ptr+1);
        msg_type = buffer(ptr+2);
        
        if msg_type == MSG_TYPE_FORMAT
            if numel(buffer) - ptr < MSG_FORMAT_PACKET_LEN
                break;
            end
            % return new pointer, and all message descriptor info
            LOCAL_parse_message_descriptors(buffer, ptr, MSG_TYPE_FORMAT, MSG_FORMAT_STRUCT, MSG_FORMAT_PACKET_LEN);
        else
            msg_descr = msg_descrs{msg_type,:};
            msg_length = msg_descr{1};
            if numel(buffer) - ptr < msg_length
                break;
            end
            % return new pointer, and all message info
            LOCAL_parse_message(buffer, MSG_HEADER_LEN, msg_descr);
        end
    end
    bytes_read = bytes_read + ptr;
end

end

function [ptr, msg_descrs] = LOCAL_parse_message_descriptors(buffer, ptr, MSG_TYPE_FORMAT, MSG_FORMAT_STRUCT, MSG_FORMAT_PACKET_LEN)
thisBlock = buffer((ptr+3):(ptr+MSG_FORMAT_PACKET_LEN));
msg_type = thisBlock(1);
if msg_type ~= MSG_TYPE_FORMAT
    msg_length = thisBlock(2);
    msg_name = LOCAL_parse_string(thisBlock(3:6));
    msg_format = LOCAL_parse_string(thisBlock(7:22));
    msg_labels = strsplit(LOCAL_parse_string(thisBlock(23:end)),',');
    msg_struct = [];
    msg_mults = [];
end
ptr = ptr + MSG_FORMAT_PACKET_LEN;
end

function LOCAL_parse_message(buffer, ptr, MSG_HEADER_LEN, msg_descr)
end

function str = LOCAL_parse_string(byteArray)
    firstZero = find(byteArray==0);
    if ~isempty(firstZero)
        str = char(byteArray(1:firstZero-1))';
    else
        str = char(byteArray)';
    end
end