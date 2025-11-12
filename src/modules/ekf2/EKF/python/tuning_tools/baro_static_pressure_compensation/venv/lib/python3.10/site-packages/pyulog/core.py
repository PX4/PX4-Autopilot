""" Main Module to load and parse an ULog file """

from __future__ import print_function

import struct
import sys
import numpy as np
#pylint: disable=too-many-instance-attributes, unused-argument, missing-docstring
#pylint: disable=protected-access, too-many-branches

__author__ = "Beat Kueng"


# check python version
if sys.hexversion >= 0x030000F0:
    _RUNNING_PYTHON3 = True
    def _parse_string(cstr, errors='strict'):
        return str(cstr, 'utf-8', errors)
else:
    _RUNNING_PYTHON3 = False
    def _parse_string(cstr):
        return str(cstr)


class ULog(object):
    """
    This class parses an ulog file
    """

    ## constants ##
    HEADER_BYTES = b'\x55\x4c\x6f\x67\x01\x12\x35'
    SYNC_BYTES = b'\x2F\x73\x13\x20\x25\x0C\xBB\x12'

    # message types
    MSG_TYPE_FORMAT = ord('F')
    MSG_TYPE_DATA = ord('D')
    MSG_TYPE_INFO = ord('I')
    MSG_TYPE_INFO_MULTIPLE = ord('M')
    MSG_TYPE_PARAMETER = ord('P')
    MSG_TYPE_PARAMETER_DEFAULT = ord('Q')
    MSG_TYPE_ADD_LOGGED_MSG = ord('A')
    MSG_TYPE_REMOVE_LOGGED_MSG = ord('R')
    MSG_TYPE_SYNC = ord('S')
    MSG_TYPE_DROPOUT = ord('O')
    MSG_TYPE_LOGGING = ord('L')
    MSG_TYPE_LOGGING_TAGGED = ord('C')
    MSG_TYPE_FLAG_BITS = ord('B')

    _UNPACK_TYPES = {
        'int8_t':   ['b', 1, np.int8],
        'uint8_t':  ['B', 1, np.uint8],
        'int16_t':  ['h', 2, np.int16],
        'uint16_t': ['H', 2, np.uint16],
        'int32_t':  ['i', 4, np.int32],
        'uint32_t': ['I', 4, np.uint32],
        'int64_t':  ['q', 8, np.int64],
        'uint64_t': ['Q', 8, np.uint64],
        'float':    ['f', 4, np.float32],
        'double':   ['d', 8, np.float64],
        'bool':     ['?', 1, np.int8],
        'char':     ['c', 1, np.int8]
        }


    @staticmethod
    def get_field_size(type_str):
        """
        get the field size in bytes.
        :param type_str: type string, eg. 'int8_t'
        """
        return ULog._UNPACK_TYPES[type_str][1]


    # pre-init unpack structs for quicker use
    _unpack_ushort_byte = struct.Struct('<HB').unpack
    _unpack_ushort = struct.Struct('<H').unpack
    _unpack_uint64 = struct.Struct('<Q').unpack

    # when set to True disables string parsing exceptions
    _disable_str_exceptions = False

    @staticmethod
    def parse_string(cstr):
        """
        wrapper for _parse_string with
        parametrized exception handling
        """
        ret = ''
        if _RUNNING_PYTHON3 and ULog._disable_str_exceptions:
            ret = _parse_string(cstr, 'ignore')
        else:
            ret = _parse_string(cstr)
        return ret

    def __init__(self, log_file, message_name_filter_list=None, disable_str_exceptions=True):
        """
        Initialize the object & load the file.

        :param log_file: a file name (str) or a readable file object
        :param message_name_filter_list: list of strings, to only load messages
               with the given names. If None, load everything.
        :param disable_str_parser_exceptions: If True, ignore string parsing errors
        """

        self._debug = False

        self._file_corrupt = False

        self._start_timestamp = 0
        self._last_timestamp = 0
        self._msg_info_dict = {}
        self._msg_info_multiple_dict = {}
        self._initial_parameters = {}
        self._default_parameters = {}
        self._changed_parameters = []
        self._message_formats = {}
        self._logged_messages = []
        self._logged_messages_tagged = {}
        self._dropouts = []
        self._data_list = []

        self._subscriptions = {} # dict of key=msg_id, value=_MessageAddLogged
        self._filtered_message_ids = set() # _MessageAddLogged id's that are filtered
        self._missing_message_ids = set() # _MessageAddLogged id's that could not be found
        self._file_version = 0
        self._compat_flags = [0] * 8
        self._incompat_flags = [0] * 8
        self._appended_offsets = [] # file offsets for appended data
        self._has_sync = True # set to false when first file search for sync fails
        self._sync_seq_cnt = 0 # number of sync packets found in file

        ULog._disable_str_exceptions = disable_str_exceptions

        self._load_file(log_file, message_name_filter_list)


    ## parsed data

    @property
    def start_timestamp(self):
        """ timestamp of file start """
        return self._start_timestamp

    @property
    def last_timestamp(self):
        """ timestamp of last message """
        return self._last_timestamp

    @property
    def msg_info_dict(self):
        """ dictionary of all information messages (key is a string, value
        depends on the type, usually string or int) """
        return self._msg_info_dict

    @property
    def msg_info_multiple_dict(self):
        """ dictionary of all information multiple messages (key is a string, value
        is a list of lists that contains the messages) """
        return self._msg_info_multiple_dict

    @property
    def initial_parameters(self):
        """ dictionary of all initially set parameters (key=param name) """
        return self._initial_parameters

    def get_default_parameters(self, default_type):
        """ dictionary of all default parameters (key=param name).
        Note that defaults are generally only stored for parameters where
        the default is different from the configured value

        :param default_type: 0: system, 1: current_setup
        """
        return self._default_parameters.get(default_type, {})

    @property
    def changed_parameters(self):
        """ list of all changed parameters (tuple of (timestamp, name, value))"""
        return self._changed_parameters

    @property
    def message_formats(self):
        """ dictionary with key = format name (MessageFormat.name),
        value = MessageFormat object """
        return self._message_formats

    @property
    def logged_messages(self):
        """ list of MessageLogging objects """
        return self._logged_messages

    @property
    def logged_messages_tagged(self):
        """ dict of MessageLoggingTagged objects """
        return self._logged_messages_tagged

    @property
    def dropouts(self):
        """ list of MessageDropout objects """
        return self._dropouts

    @property
    def data_list(self):
        """ extracted data: list of Data objects """
        return self._data_list

    @property
    def has_data_appended(self):
        """ returns True if the log has data appended, False otherwise """
        return self._incompat_flags[0] & 0x1

    @property
    def file_corruption(self):
        """ True if a file corruption got detected """
        return self._file_corrupt

    @property
    def has_default_parameters(self):
        """ True if compat flag DEFAULT_PARAMETERS is set """
        return self._compat_flags[0] & (0x1 << 0)

    def get_dataset(self, name, multi_instance=0):
        """ get a specific dataset.

        example:
        try:
            gyro_data = ulog.get_dataset('sensor_gyro')
        except (KeyError, IndexError, ValueError) as error:
            print(type(error), "(sensor_gyro):", error)

        :param name: name of the dataset
        :param multi_instance: the multi_id, defaults to the first
        :raises KeyError, IndexError, ValueError: if name or instance not found
        """
        return [elem for elem in self._data_list
                if elem.name == name and elem.multi_id == multi_instance][0]


    class Data(object):
        """ contains the final topic data for a single topic and instance """

        def __init__(self, message_add_logged_obj):
            self.multi_id = message_add_logged_obj.multi_id
            self.name = message_add_logged_obj.message_name
            self.field_data = message_add_logged_obj.field_data
            self.timestamp_idx = message_add_logged_obj.timestamp_idx

            # get data as numpy.ndarray
            np_array = np.frombuffer(message_add_logged_obj.buffer,
                                     dtype=message_add_logged_obj.dtype)
            # convert into dict of np.array (which is easier to handle)
            self.data = {}
            for name in np_array.dtype.names:
                self.data[name] = np_array[name]

        def list_value_changes(self, field_name):
            """ get a list of (timestamp, value) tuples, whenever the value
            changes. The first data point with non-zero timestamp is always
            included, messages with timestamp = 0 are ignored """

            t = self.data['timestamp']
            x = self.data[field_name]
            indices = t != 0 # filter out 0 values
            t = t[indices]
            x = x[indices]
            if len(t) == 0: return []
            ret = [(t[0], x[0])]
            indices = np.where(x[:-1] != x[1:])[0] + 1
            ret.extend(zip(t[indices], x[indices]))
            return ret



    ## Representations of the messages from the log file ##

    class _MessageHeader(object):
        """ 3 bytes ULog message header """

        def __init__(self):
            self.msg_size = 0
            self.msg_type = 0

        def initialize(self, data):
            self.msg_size, self.msg_type = ULog._unpack_ushort_byte(data)

    class _MessageInfo(object):
        """ ULog info message representation """

        def __init__(self, data, header, is_info_multiple=False):
            if is_info_multiple: # INFO_MULTIPLE message
                self.is_continued, = struct.unpack('<B', data[0:1])
                data = data[1:]
            key_len, = struct.unpack('<B', data[0:1])
            type_key = ULog.parse_string(data[1:1+key_len])
            type_key_split = type_key.split(' ')
            self.type = type_key_split[0]
            self.key = type_key_split[1]
            if self.type.startswith('char['): # it's a string
                self.value = ULog.parse_string(data[1+key_len:])
            elif self.type in ULog._UNPACK_TYPES:
                unpack_type = ULog._UNPACK_TYPES[self.type]
                self.value, = struct.unpack('<'+unpack_type[0], data[1+key_len:])
            else: # probably an array (or non-basic type)
                self.value = data[1+key_len:]

    class _MessageParameterDefault(object):
        """ ULog parameter default message representation """

        def __init__(self, data, header):
            self.default_types, = struct.unpack('<B', data[0:1])
            msg_info = ULog._MessageInfo(data[1:], header)
            self.type = msg_info.type
            self.key = msg_info.key
            self.value = msg_info.value

    class _MessageFlagBits(object):
        """ ULog message flag bits """

        def __init__(self, data, header):
            if header.msg_size > 8 + 8 + 3*8:
                # we can still parse it but might miss some information
                print('Warning: Flags Bit message is longer than expected')

            self.compat_flags = list(struct.unpack('<'+'B'*8, data[0:8]))
            self.incompat_flags = list(struct.unpack('<'+'B'*8, data[8:16]))
            self.appended_offsets = list(struct.unpack('<'+'Q'*3, data[16:16+3*8]))
            # remove the 0's at the end
            while len(self.appended_offsets) > 0 and self.appended_offsets[-1] == 0:
                self.appended_offsets.pop()


    class MessageFormat(object):
        """ ULog message format representation """

        def __init__(self, data, header):
            format_arr = ULog.parse_string(data).split(':')
            self.name = format_arr[0]
            types_str = format_arr[1].split(';')
            self.fields = [] # list of tuples (type, array_size, name)
            for t in types_str:
                if len(t) > 0:
                    self.fields.append(self._extract_type(t))

        @staticmethod
        def _extract_type(field_str):
            field_str_split = field_str.split(' ')
            type_str = field_str_split[0]
            name_str = field_str_split[1]
            a_pos = type_str.find('[')
            if a_pos == -1:
                array_size = 1
                type_name = type_str
            else:
                b_pos = type_str.find(']')
                array_size = int(type_str[a_pos+1:b_pos])
                type_name = type_str[:a_pos]
            return type_name, array_size, name_str

    class MessageLogging(object):
        """ ULog logged string message representation """

        def __init__(self, data, header):
            self.log_level, = struct.unpack('<B', data[0:1])
            self.timestamp, = struct.unpack('<Q', data[1:9])
            self.message = ULog.parse_string(data[9:])

        def log_level_str(self):
            return {ord('0'): 'EMERGENCY',
                    ord('1'): 'ALERT',
                    ord('2'): 'CRITICAL',
                    ord('3'): 'ERROR',
                    ord('4'): 'WARNING',
                    ord('5'): 'NOTICE',
                    ord('6'): 'INFO',
                    ord('7'): 'DEBUG'}.get(self.log_level, 'UNKNOWN')

    class MessageLoggingTagged(object):
        """ ULog tagged log string message representation """

        def __init__(self, data, header):
            self.log_level, = struct.unpack('<B', data[0:1])
            self.tag = struct.unpack('<H', data[1:3])
            self.timestamp, = struct.unpack('<Q', data[3:11])
            self.message = ULog.parse_string(data[11:])

        def log_level_str(self):
            return {ord('0'): 'EMERGENCY',
                    ord('1'): 'ALERT',
                    ord('2'): 'CRITICAL',
                    ord('3'): 'ERROR',
                    ord('4'): 'WARNING',
                    ord('5'): 'NOTICE',
                    ord('6'): 'INFO',
                    ord('7'): 'DEBUG'}.get(self.log_level, 'UNKNOWN')

    class MessageDropout(object):
        """ ULog dropout message representation """
        def __init__(self, data, header, timestamp):
            self.duration, = struct.unpack('<H', data)
            self.timestamp = timestamp

    class _FieldData(object):
        """ Type and name of a single ULog data field """
        def __init__(self, field_name, type_str):
            self.field_name = field_name
            self.type_str = type_str

    class _MessageAddLogged(object):
        """ ULog add logging data message representation """
        def __init__(self, data, header, message_formats):
            self.multi_id, = struct.unpack('<B', data[0:1])
            self.msg_id, = struct.unpack('<H', data[1:3])
            self.message_name = ULog.parse_string(data[3:])
            self.field_data = [] # list of _FieldData
            self.timestamp_idx = -1
            self._parse_format(message_formats)

            self.timestamp_offset = 0
            for field in self.field_data:
                if field.field_name == 'timestamp':
                    break
                self.timestamp_offset += ULog._UNPACK_TYPES[field.type_str][1]

            self.buffer = bytearray() # accumulate all message data here

            # construct types for numpy
            dtype_list = []
            for field in self.field_data:
                numpy_type = ULog._UNPACK_TYPES[field.type_str][2]
                dtype_list.append((field.field_name, numpy_type))
            self.dtype = np.dtype(dtype_list).newbyteorder('<')


        def _parse_format(self, message_formats):
            self._parse_nested_type('', self.message_name, message_formats)

            # remove padding fields at the end
            while (len(self.field_data) > 0 and
                   self.field_data[-1].field_name.startswith('_padding')):
                self.field_data.pop()

        def _parse_nested_type(self, prefix_str, type_name, message_formats):
            # we flatten nested types
            message_format = message_formats[type_name]
            for (type_name_fmt, array_size, field_name) in message_format.fields:
                if type_name_fmt in ULog._UNPACK_TYPES:
                    if array_size > 1:
                        for i in range(array_size):
                            self.field_data.append(ULog._FieldData(
                                prefix_str+field_name+'['+str(i)+']', type_name_fmt))
                    else:
                        self.field_data.append(ULog._FieldData(
                            prefix_str+field_name, type_name_fmt))
                    if prefix_str+field_name == 'timestamp':
                        self.timestamp_idx = len(self.field_data) - 1
                else: # nested type
                    if array_size > 1:
                        for i in range(array_size):
                            self._parse_nested_type(prefix_str+field_name+'['+str(i)+'].',
                                                    type_name_fmt, message_formats)
                    else:
                        self._parse_nested_type(prefix_str+field_name+'.',
                                                type_name_fmt, message_formats)

    class _MessageData(object):
        def __init__(self):
            self.timestamp = 0

        def initialize(self, data, header, subscriptions, ulog_object):
            msg_id, = ULog._unpack_ushort(data[:2])
            if msg_id in subscriptions:
                subscription = subscriptions[msg_id]
                # accumulate data to a buffer, will be parsed later
                subscription.buffer += data[2:]
                t_off = subscription.timestamp_offset
                # TODO: the timestamp can have another size than uint64
                self.timestamp, = ULog._unpack_uint64(data[t_off+2:t_off+10])
            else:
                if not msg_id in ulog_object._filtered_message_ids:
                    # this is an error, but make it non-fatal
                    if not msg_id in ulog_object._missing_message_ids:
                        ulog_object._missing_message_ids.add(msg_id)
                        if ulog_object._debug:
                            print(ulog_object._file_handle.tell())
                        print('Warning: no subscription found for message id {:}. Continuing,'
                              ' but file is most likely corrupt'.format(msg_id))
                self.timestamp = 0

    def _add_parameter_default(self, msg_param):
        """ add a _MessageParameterDefault object """
        default_types = msg_param.default_types
        while default_types: # iterate over each bit
            def_type = default_types & (~default_types+1)
            default_types ^= def_type
            def_type -= 1
            if def_type not in self._default_parameters:
                self._default_parameters[def_type] = {}
            self._default_parameters[def_type][msg_param.key] = msg_param.value

    def _add_message_info_multiple(self, msg_info):
        """ add a message info multiple to self._msg_info_multiple_dict """
        if msg_info.key in self._msg_info_multiple_dict:
            if msg_info.is_continued:
                self._msg_info_multiple_dict[msg_info.key][-1].append(msg_info.value)
            else:
                self._msg_info_multiple_dict[msg_info.key].append([msg_info.value])
        else:
            self._msg_info_multiple_dict[msg_info.key] = [[msg_info.value]]

    def _load_file(self, log_file, message_name_filter_list):
        """ load and parse an ULog file into memory """
        if isinstance(log_file, str):
            self._file_handle = open(log_file, "rb")
        else:
            self._file_handle = log_file

        # parse the whole file
        self._read_file_header()
        self._last_timestamp = self._start_timestamp
        self._read_file_definitions()

        if self.has_data_appended and len(self._appended_offsets) > 0:
            if self._debug:
                print('This file has data appended')
            for offset in self._appended_offsets:
                self._read_file_data(message_name_filter_list, read_until=offset)
                self._file_handle.seek(offset)

        # read the whole file, or the rest if data appended
        self._read_file_data(message_name_filter_list)

        self._file_handle.close()
        del self._file_handle

    def _read_file_header(self):
        header_data = self._file_handle.read(16)
        if len(header_data) != 16:
            raise Exception("Invalid file format (Header too short)")
        if header_data[:7] != self.HEADER_BYTES:
            raise Exception("Invalid file format (Failed to parse header)")
        self._file_version, = struct.unpack('B', header_data[7:8])
        if self._file_version > 1:
            print("Warning: unknown file version. Will attempt to read it anyway")

        # read timestamp
        self._start_timestamp, = ULog._unpack_uint64(header_data[8:])

    def _read_file_definitions(self):
        header = self._MessageHeader()
        while True:
            data = self._file_handle.read(3)
            if not data:
                break
            header.initialize(data)
            data = self._file_handle.read(header.msg_size)
            try:
                if header.msg_type == self.MSG_TYPE_INFO:
                    msg_info = self._MessageInfo(data, header)
                    self._msg_info_dict[msg_info.key] = msg_info.value
                elif header.msg_type == self.MSG_TYPE_INFO_MULTIPLE:
                    msg_info = self._MessageInfo(data, header, is_info_multiple=True)
                    self._add_message_info_multiple(msg_info)
                elif header.msg_type == self.MSG_TYPE_FORMAT:
                    msg_format = self.MessageFormat(data, header)
                    self._message_formats[msg_format.name] = msg_format
                elif header.msg_type == self.MSG_TYPE_PARAMETER:
                    msg_info = self._MessageInfo(data, header)
                    self._initial_parameters[msg_info.key] = msg_info.value
                elif header.msg_type == self.MSG_TYPE_PARAMETER_DEFAULT:
                    msg_param = self._MessageParameterDefault(data, header)
                    self._add_parameter_default(msg_param)
                elif (header.msg_type == self.MSG_TYPE_ADD_LOGGED_MSG or
                      header.msg_type == self.MSG_TYPE_LOGGING or
                      header.msg_type == self.MSG_TYPE_LOGGING_TAGGED):
                    self._file_handle.seek(-(3+header.msg_size), 1)
                    break # end of section
                elif header.msg_type == self.MSG_TYPE_FLAG_BITS:
                    # make sure this is the first message in the log
                    if self._file_handle.tell() != 16 + 3 + header.msg_size:
                        print('Error: FLAGS_BITS message must be first message. Offset:',
                              self._file_handle.tell())
                    msg_flag_bits = self._MessageFlagBits(data, header)
                    self._compat_flags = msg_flag_bits.compat_flags
                    self._incompat_flags = msg_flag_bits.incompat_flags
                    self._appended_offsets = msg_flag_bits.appended_offsets
                    if self._debug:
                        print('compat flags:  ', self._compat_flags)
                        print('incompat flags:', self._incompat_flags)
                        print('appended offsets:', self._appended_offsets)

                    # check if there are bits set that we don't know
                    unknown_incompat_flag_msg = \
                    "Unknown incompatible flag set: cannot parse the log"
                    if self._incompat_flags[0] & ~1:
                        raise Exception(unknown_incompat_flag_msg)
                    for i in range(1, 8):
                        if self._incompat_flags[i]:
                            raise Exception(unknown_incompat_flag_msg)

                else:
                    if self._debug:
                        print('read_file_definitions: unknown message type: %i (%s)' %
                              (header.msg_type, chr(header.msg_type)))
                        file_position = self._file_handle.tell()
                        print('file position: %i (0x%x) msg size: %i' % (
                            file_position, file_position, header.msg_size))
                    if self._check_packet_corruption(header):
                        # seek back to advance only by a single byte instead of
                        # skipping the message
                        self._file_handle.seek(-2-header.msg_size, 1)

            except IndexError:
                if not self._file_corrupt:
                    print("File corruption detected while reading file definitions!")
                    self._file_corrupt = True

    def _find_sync(self, last_n_bytes=-1):
        """
        read the file from a given location until the end of sync_byte sequence is found
            or an end condition is met(reached EOF or searched all last_n_bytes).
        :param last_n_bytes: optional arg to search only last_n_bytes for sync_bytes.
            when provided, _find_sync searches for sync_byte sequence in the last_n_bytes
            from current location, else, from current location till end of file.
        return true if successful, else return false and seek back to initial position and
            set _has_sync to false if searched till end of file
        """
        sync_seq_found = False
        initial_file_position = self._file_handle.tell()
        current_file_position = initial_file_position

        search_chunk_size = 512 # number of bytes that are searched at once

        if last_n_bytes != -1:
            current_file_position = self._file_handle.seek(-last_n_bytes, 1)
            search_chunk_size = last_n_bytes

        chunk = self._file_handle.read(search_chunk_size)
        while len(chunk) >= len(ULog.SYNC_BYTES):
            current_file_position += len(chunk)
            chunk_index = chunk.find(ULog.SYNC_BYTES)
            if chunk_index >= 0:
                if self._debug:
                    print("Found sync at %i" % (current_file_position - len(chunk) + chunk_index))
                # seek to end of sync sequence and break
                current_file_position = self._file_handle.seek(current_file_position - len(chunk)\
                         + chunk_index + len(ULog.SYNC_BYTES), 0)
                sync_seq_found = True
                break

            if last_n_bytes != -1:
                # we read the whole last_n_bytes and did not find sync
                break

            # seek back 7 bytes to handle boundary condition and read next chunk
            current_file_position = self._file_handle.seek(-(len(ULog.SYNC_BYTES)-1), 1)
            chunk = self._file_handle.read(search_chunk_size)

        if not sync_seq_found:
            current_file_position = self._file_handle.seek(initial_file_position, 0)

            if last_n_bytes == -1:
                self._has_sync = False
                if self._debug:
                    print("Failed to find sync in file from %i" % initial_file_position)
            else:
                if self._debug:
                    print("Failed to find sync in (%i, %i)" %\
                        (initial_file_position - last_n_bytes, initial_file_position))
        else:
            # declare file corrupt if we skipped bytes to sync sequence
            self._file_corrupt = True

        return sync_seq_found

    def _read_file_data(self, message_name_filter_list, read_until=None):
        """
        read the file data section
        :param read_until: an optional file offset: if set, parse only up to
                           this offset (smaller than)
        """

        if read_until is None:
            read_until = 1 << 50 # make it larger than any possible log file

        try:
            # pre-init reusable objects
            header = self._MessageHeader()
            msg_data = self._MessageData()

            curr_file_pos = self._file_handle.tell()

            while True:
                data = self._file_handle.read(3)
                curr_file_pos += len(data)
                header.initialize(data)
                data = self._file_handle.read(header.msg_size)
                curr_file_pos += len(data)
                if len(data) < header.msg_size:
                    break # less data than expected. File is most likely cut

                if curr_file_pos > read_until:
                    if self._debug:
                        print('read until offset=%i done, current pos=%i' %
                              (read_until, curr_file_pos))
                    break

                try:
                    if header.msg_type == self.MSG_TYPE_INFO:
                        msg_info = self._MessageInfo(data, header)
                        self._msg_info_dict[msg_info.key] = msg_info.value
                    elif header.msg_type == self.MSG_TYPE_INFO_MULTIPLE:
                        msg_info = self._MessageInfo(data, header, is_info_multiple=True)
                        self._add_message_info_multiple(msg_info)
                    elif header.msg_type == self.MSG_TYPE_PARAMETER:
                        msg_info = self._MessageInfo(data, header)
                        self._changed_parameters.append((self._last_timestamp,
                                                         msg_info.key, msg_info.value))
                    elif header.msg_type == self.MSG_TYPE_PARAMETER_DEFAULT:
                        msg_param = self._MessageParameterDefault(data, header)
                        self._add_parameter_default(msg_param)
                    elif header.msg_type == self.MSG_TYPE_ADD_LOGGED_MSG:
                        msg_add_logged = self._MessageAddLogged(data, header,
                                                                self._message_formats)
                        if (message_name_filter_list is None or
                                msg_add_logged.message_name in message_name_filter_list):
                            self._subscriptions[msg_add_logged.msg_id] = msg_add_logged
                        else:
                            self._filtered_message_ids.add(msg_add_logged.msg_id)
                    elif header.msg_type == self.MSG_TYPE_LOGGING:
                        msg_logging = self.MessageLogging(data, header)
                        self._logged_messages.append(msg_logging)
                    elif header.msg_type == self.MSG_TYPE_LOGGING_TAGGED:
                        msg_log_tagged = self.MessageLoggingTagged(data, header)
                        if msg_log_tagged.tag in self._logged_messages_tagged:
                            self._logged_messages_tagged[msg_log_tagged.tag].append(msg_log_tagged)
                        else:
                            self._logged_messages_tagged[msg_log_tagged.tag] = [msg_log_tagged]
                    elif header.msg_type == self.MSG_TYPE_DATA:
                        msg_data.initialize(data, header, self._subscriptions, self)
                        if msg_data.timestamp != 0 and msg_data.timestamp > self._last_timestamp:
                            self._last_timestamp = msg_data.timestamp
                    elif header.msg_type == self.MSG_TYPE_DROPOUT:
                        msg_dropout = self.MessageDropout(data, header,
                                                          self._last_timestamp)
                        self._dropouts.append(msg_dropout)
                    elif header.msg_type == self.MSG_TYPE_SYNC:
                        self._sync_seq_cnt = self._sync_seq_cnt + 1
                    else:
                        if self._debug:
                            print('_read_file_data: unknown message type: %i (%s)' %
                                  (header.msg_type, chr(header.msg_type)))
                            print('file position: %i msg size: %i' % (
                                curr_file_pos, header.msg_size))

                        if self._check_packet_corruption(header):
                            # seek back to advance only by a single byte instead of
                            # skipping the message
                            curr_file_pos = self._file_handle.seek(-2-header.msg_size, 1)

                            # try recovery with sync sequence in case of unknown msg_type
                            if self._has_sync:
                                self._find_sync()
                        else:
                            # seek back msg_size to look for sync sequence in payload
                            if self._has_sync:
                                self._find_sync(header.msg_size)

                except IndexError:
                    if not self._file_corrupt:
                        print("File corruption detected while reading file data!")
                        self._file_corrupt = True

        except struct.error:
            pass #we read past the end of the file

        # convert into final representation
        while self._subscriptions:
            _, value = self._subscriptions.popitem()
            if len(value.buffer) > 0: # only add if we have data
                data_item = ULog.Data(value)
                self._data_list.append(data_item)

    def _check_packet_corruption(self, header):
        """
        check for data corruption based on an unknown message type in the header
        set _file_corrupt flag to true if a corrupt packet is found
        We need to handle 2 cases:
        - corrupt file (we do our best to read the rest of the file)
        - new ULog message type got added (we just want to skip the message)
        return true if packet associated with header is corrupt, else return false
        """
        data_corrupt = False
        if header.msg_type == 0 or header.msg_size == 0 or header.msg_size > 10000:
            if not self._file_corrupt and self._debug:
                print('File corruption detected')
            data_corrupt = True
            self._file_corrupt = True

        return data_corrupt

    def get_version_info(self, key_name='ver_sw_release'):
        """
        get the (major, minor, patch, type) version information as tuple.
        Returns None if not found
        definition of type is:
         >= 0: development
         >= 64: alpha version
         >= 128: beta version
         >= 192: RC version
         == 255: release version
        """
        if key_name in self._msg_info_dict:
            val = self._msg_info_dict[key_name]
            return ((val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff)
        return None

    def get_version_info_str(self, key_name='ver_sw_release'):
        """
        get version information in the form 'v1.2.3 (RC)', or None if version
        tag either not found or it's a development version
        """
        version = self.get_version_info(key_name)
        if not version is None and version[3] >= 64:
            type_str = ''
            if version[3] < 128: type_str = ' (alpha)'
            elif version[3] < 192: type_str = ' (beta)'
            elif version[3] < 255: type_str = ' (RC)'
            return 'v{}.{}.{}{}'.format(version[0], version[1], version[2], type_str)
        return None
