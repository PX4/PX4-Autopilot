#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time
import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_version_h(directory, xml):
    '''generate version.h'''
    f = open(os.path.join(directory, "version.h"), mode='w')
    t.write(f,'''
/** @file
 *	@brief MAVLink comm protocol built from ${basename}.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
#ifndef MAVLINK_VERSION_H
#define MAVLINK_VERSION_H

#define MAVLINK_BUILD_DATE "${parse_time}"
#define MAVLINK_WIRE_PROTOCOL_VERSION "${wire_protocol_version}"
#define MAVLINK_MAX_DIALECT_PAYLOAD_SIZE ${largest_payload}
 
#endif // MAVLINK_VERSION_H
''', xml)
    f.close()

def generate_mavlink_h(directory, xml):
    '''generate mavlink.h'''
    f = open(os.path.join(directory, "mavlink.h"), mode='w')
    t.write(f,'''
/** @file
 *	@brief MAVLink comm protocol built from ${basename}.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#ifndef MAVLINK_STX
#define MAVLINK_STX ${protocol_marker}
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN ${mavlink_endian}
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS ${aligned_fields_define}
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA ${crc_extra_define}
#endif

#include "version.h"
#include "${basename}.h"

#endif // MAVLINK_H
''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, xml.basename + ".h"), mode='w')
    t.write(f, '''
/** @file
 *	@brief MAVLink comm protocol generated from ${basename}.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ${basename_upper}_H
#define ${basename_upper}_H

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {${message_lengths_array}}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {${message_crcs_array}}
#endif

#ifndef MAVLINK_MESSAGE_INFO
#define MAVLINK_MESSAGE_INFO {${message_info_array}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_${basename_upper}

${{include_list:#include "../${base}/${base}.h"
}}

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION ${version}
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION ${version}
#endif

// ENUM DEFINITIONS

${{enum:
/** @brief ${description} */
#ifndef HAVE_ENUM_${name}
#define HAVE_ENUM_${name}
enum ${name}
{
${{entry:	${name}=${value}, /* ${description} |${{param:${description}| }} */
}}
};
#endif
}}

// MESSAGE DEFINITIONS
${{message:#include "./mavlink_msg_${name_lower}.h"
}}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ${basename_upper}_H
''', xml)

    f.close()
             

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mavlink_msg_%s.h' % m.name_lower), mode='w')
    t.write(f, '''
// MESSAGE ${name} PACKING

#define MAVLINK_MSG_ID_${name} ${id}

typedef struct __mavlink_${name_lower}_t
{
${{ordered_fields: ${type} ${name}${array_suffix}; ///< ${description}
}}
} mavlink_${name_lower}_t;

#define MAVLINK_MSG_ID_${name}_LEN ${wire_length}
#define MAVLINK_MSG_ID_${id}_LEN ${wire_length}

${{array_fields:#define MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN ${array_length}
}}

#define MAVLINK_MESSAGE_INFO_${name} { \\
	"${name}", \\
	${num_fields}, \\
	{ ${{ordered_fields: { "${name}", ${c_print_format}, MAVLINK_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mavlink_${name_lower}_t, ${name}) }, \\
        }} } \\
}


/**
 * @brief Pack a ${name_lower} message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_${name_lower}_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						      ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[${wire_length}];
${{scalar_fields:	_mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:	_mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, ${wire_length});
#else
	mavlink_${name_lower}_t packet;
${{scalar_fields:	packet.${name} = ${putname};
}}
${{array_fields:	mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, ${wire_length});
#endif

	msg->msgid = MAVLINK_MSG_ID_${name};
	return mavlink_finalize_message(msg, system_id, component_id, ${wire_length}${crc_extra_arg});
}

/**
 * @brief Pack a ${name_lower} message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_${name_lower}_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           ${{arg_fields:${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[${wire_length}];
${{scalar_fields:	_mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:	_mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, ${wire_length});
#else
	mavlink_${name_lower}_t packet;
${{scalar_fields:	packet.${name} = ${putname};
}}
${{array_fields:	mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, ${wire_length});
#endif

	msg->msgid = MAVLINK_MSG_ID_${name};
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, ${wire_length}${crc_extra_arg});
}

/**
 * @brief Encode a ${name_lower} struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_${name_lower}_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_${name_lower}_t* ${name_lower})
{
	return mavlink_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 *
${{arg_fields: * @param ${name} ${description}
}}
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_${name_lower}_send(mavlink_channel_t chan,${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[${wire_length}];
${{scalar_fields:	_mav_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:	_mav_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, buf, ${wire_length}${crc_extra_arg});
#else
	mavlink_${name_lower}_t packet;
${{scalar_fields:	packet.${name} = ${putname};
}}
${{array_fields:	mav_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_${name}, (const char *)&packet, ${wire_length}${crc_extra_arg});
#endif
}

#endif

// MESSAGE ${name} UNPACKING

${{fields:
/**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${description}
 */
static inline ${return_type} mavlink_msg_${name_lower}_get_${name}(const mavlink_message_t* msg${get_arg})
{
	return _MAV_RETURN_${type}${array_tag}(msg, ${array_return_arg} ${wire_offset});
}
}}

/**
 * @brief Decode a ${name_lower} message into a struct
 *
 * @param msg The message to decode
 * @param ${name_lower} C-struct to decode the message contents into
 */
static inline void mavlink_msg_${name_lower}_decode(const mavlink_message_t* msg, mavlink_${name_lower}_t* ${name_lower})
{
#if MAVLINK_NEED_BYTE_SWAP
${{ordered_fields:	${decode_left}mavlink_msg_${name_lower}_get_${name}(msg${decode_right});
}}
#else
	memcpy(${name_lower}, _MAV_PAYLOAD(msg), ${wire_length});
#endif
}
''', m)
    f.close()


def generate_testsuite_h(directory, xml):
    '''generate testsuite.h per XML file'''
    f = open(os.path.join(directory, "testsuite.h"), mode='w')
    t.write(f, '''
/** @file
 *	@brief MAVLink comm protocol testsuite generated from ${basename}.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ${basename_upper}_TESTSUITE_H
#define ${basename_upper}_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
${{include_list:static void mavlink_test_${base}(uint8_t, uint8_t, mavlink_message_t *last_msg);
}}
static void mavlink_test_${basename}(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
${{include_list:	mavlink_test_${base}(system_id, component_id, last_msg);
}}
	mavlink_test_${basename}(system_id, component_id, last_msg);
}
#endif

${{include_list:#include "../${base}/testsuite.h"
}}

${{message:
static void mavlink_test_${name_lower}(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_${name_lower}_t packet_in = {
		${{ordered_fields:${c_test_value},
	}}};
	mavlink_${name_lower}_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        ${{scalar_fields:	packet1.${name} = packet_in.${name};
        }}
        ${{array_fields:	mav_array_memcpy(packet1.${name}, packet_in.${name}, sizeof(${type})*${array_length});
        }}

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_${name_lower}_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_${name_lower}_pack(system_id, component_id, &msg ${{arg_fields:, packet1.${name} }});
	mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_${name_lower}_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg ${{arg_fields:, packet1.${name} }});
	mavlink_msg_${name_lower}_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_${name_lower}_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_${name_lower}_send(MAVLINK_COMM_1 ${{arg_fields:, packet1.${name} }});
	mavlink_msg_${name_lower}_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}
}}

static void mavlink_test_${basename}(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
${{message:	mavlink_test_${name_lower}(system_id, component_id, last_msg);
}}
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ${basename_upper}_TESTSUITE_H
''', xml)

    f.close()

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil
    hlist = [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h', 'mavlink_protobuf_manager.hpp' ]
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'C/include_v%s' % xml.wire_protocol_version)
    print("Copying fixed headers")
    for h in hlist:
        if (not (h == 'mavlink_protobuf_manager.hpp' and xml.wire_protocol_version == '0.9')):
           src = os.path.realpath(os.path.join(srcpath, h))
           dest = os.path.realpath(os.path.join(directory, h))
           if src == dest:
               continue
           shutil.copy(src, dest)
    # XXX This is a hack - to be removed
    if (xml.basename == 'pixhawk' and xml.wire_protocol_version == '1.0'):
        h = 'pixhawk/pixhawk.pb.h'
        src = os.path.realpath(os.path.join(srcpath, h))
        dest = os.path.realpath(os.path.join(directory, h))
        shutil.copy(src, dest)
        
def copy_fixed_sources(directory, xml):
    # XXX This is a hack - to be removed
    import shutil
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'C/src_v%s' % xml.wire_protocol_version)
    if (xml.basename == 'pixhawk' and xml.wire_protocol_version == '1.0'):
        print("Copying fixed sources")
        src = os.path.realpath(os.path.join(srcpath, 'pixhawk/pixhawk.pb.cc'))
        dest = os.path.realpath(os.path.join(directory, '../../../share/mavlink/src/v%s/pixhawk/pixhawk.pb.cc' % xml.wire_protocol_version))
        destdir = os.path.realpath(os.path.join(directory, '../../../share/mavlink/src/v%s/pixhawk' % xml.wire_protocol_version))
        try:
           os.makedirs(destdir)
        except:
           print("Not re-creating directory")
        shutil.copy(src, dest)
        print("Copied to"),
        print(dest)

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating C implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = i[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    for mlen in xml.message_lengths:
        xml.message_lengths_array += '%u, ' % mlen
    xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    for crc in xml.message_crcs:
        xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    for name in xml.message_names:
        if name is not None:
            xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
        else:
            # Several C compilers don't accept {NULL} for
            # multi-dimensional arrays and structs
            # feed the compiler a "filled" empty message
            xml.message_info_array += '{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = '_array'
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s->%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
            else:
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s->%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'int64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_mavlink_h(directory, xml)
    generate_version_h(directory, xml)
    generate_main_h(directory, xml)
    for m in xml.message:
        generate_message_h(directory, m)
    generate_testsuite_h(directory, xml)


def generate(basename, xml_list):
    '''generate complete MAVLink C implemenation'''

    for xml in xml_list:
        generate_one(basename, xml)
    copy_fixed_headers(basename, xml_list[0])
    copy_fixed_sources(basename, xml_list[0])
