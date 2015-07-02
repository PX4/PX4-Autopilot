/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*  This file not a typical h file, is defines the UAVCAN dsdl
 * usage and may be included several times in header or source
 * file
 */

/* Components */

/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                  field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(CRC,                                   lsb,              0,         8,            0,             1)
   UAVCAN_DSDL_BIT_DEF(CRC,                                   msb,              0,         8,            1,             1)
   UAVCAN_DSDL_BIT_DEF(CRC,                                  data,              0,         8,            2,             4)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(Path,                  0,                false,             0,      200,          NA,            NA,              NA,              NA)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                  field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(Path,                                   path,              0,         8,            0,           200)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(SoftwareVersion,       0,                false,             0,        15,         NA,            NA,              NA,              NA)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
  UAVCAN_DSDL_BIT_DEF(SoftwareVersion,                        major,              0,         8,            0,              1)
  UAVCAN_DSDL_BIT_DEF(SoftwareVersion,                        minor,              0,         8,            1,              1)
  UAVCAN_DSDL_BIT_DEF(SoftwareVersion,          optional_field_mask,              0,         8,            2,              1)
  UAVCAN_DSDL_BIT_DEF(SoftwareVersion,                   vcs_commit,              0,         32,           3,              4)
  UAVCAN_DSDL_BIT_DEF(SoftwareVersion,                    image_crc,              0,         NA,           7,              8) // NA becuase bit mask is 64
  
/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(HardwareVersion,       0,                false,             0,       NA,         NA,            NA,              NA,              NA)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(HardwareVersion,                   unique_id,              0,         8,            0,            16)
   UAVCAN_DSDL_BIT_DEF(HardwareVersion, certificate_of_authenticity,              0,         8,            0,           255)



/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(NodeStatus,            341,                false,      0xf063,        7,         MailBox1,    FifoNone,   SingleFrameTailInit, SingleFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(NodeStatus,                       uptime_sec,              3,        29,             3,            1)
   UAVCAN_DSDL_BIT_DEF(NodeStatus,                      status_code,              0,         3,             3,            1)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(GetNodeInfo,             1,                 true,      0xfd3d,        0,         MailBox1,     MailBox1,  SingleFrameTailInit, MultiFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
                                                                                                                                       // Request is empty
   UAVCAN_DSDL_BIT_DEF(GetNodeInfo,                          status,              0,         8,             0,    PackedSizeNodeStatus) // Responce
   UAVCAN_DSDL_BIT_DEF(GetNodeInfo,                software_version,              0,         8,            NA,           NA)
   UAVCAN_DSDL_BIT_DEF(GetNodeInfo,                hardware_version,              0,         8,            NA,           NA)
   UAVCAN_DSDL_BIT_DEF(GetNodeInfo,                            name,              0,         8,            NA,           80)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(Allocation,              1,                false,      0xf258,        0,         MailBox0,     Fifo0,     MultiFrameTailInit, SingleFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(Allocation,                          node_id,              1,         7,            0,             1)
   UAVCAN_DSDL_BIT_DEF(Allocation,          first_part_of_unique_id,              0,         1,            0,             1)
   UAVCAN_DSDL_BIT_DEF(Allocation,                        unique_id,              0,         8,            1,            16)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(BeginFirmwareUpdate,    40,                  true,     0x729e,        0,         MailBox0,     Fifo0,      MultiFrameTailInit, SingleFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
  UAVCAN_DSDL_BIT_DEF(BeginFirmwareUpdate,           source_node_id,              0,          8,           0,             1) // Request
  UAVCAN_DSDL_BIT_DEF(BeginFirmwareUpdate,   image_file_remote_path,              0,          8,           1,             PayloadLengthPathpath)

  UAVCAN_DSDL_BIT_DEF(BeginFirmwareUpdate,                   error,               0,          8,           0,             1) // Response
  UAVCAN_DSDL_BIT_DEF(BeginFirmwareUpdate,  optional_error_message,               0,          8,            1,          128)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(GetInfo,                45,                  true,     0x14b9,          0,         MailBox0,     Fifo0,     SingleFrameTailInit, MultiFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(GetInfo,                                path,              0,          8,              0,           PayloadLengthPathpath) // Request

   UAVCAN_DSDL_BIT_DEF(GetInfo,                                size,              0,         32,              0,          4) // Response
   UAVCAN_DSDL_BIT_DEF(GetInfo,                             sizemsb,              0,          8,              4,          1)
   UAVCAN_DSDL_BIT_DEF(GetInfo,                               error,              0,         16,              5,          2)
   UAVCAN_DSDL_BIT_DEF(GetInfo,                          entry_type,              0,          8,              7,          1)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(Read,                   48,                  true,    0x2f12,          0,         MailBox0,     Fifo0,     SingleFrameTailInit, MultiFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(Read,                                 offset,              0,         32,              0,          4) // Request
   UAVCAN_DSDL_BIT_DEF(Read,                              msboffset,              0,          8,              4,          1)
   UAVCAN_DSDL_BIT_DEF(Read,                                   path,              0,          8,              5,          PayloadLengthPathpath)

   UAVCAN_DSDL_BIT_DEF(Read,                                  error,              0,         16,              0,          2) // Response
   UAVCAN_DSDL_BIT_DEF(Read,                                   data,              0,          8,              2,        256)

/*UAVCAN_DSDL_TYPE_DEF(name,                dtid,              service,    signature,  packed size    mailbox,         fifo,         inbound,        outbound) */
  UAVCAN_DSDL_TYPE_DEF(LogMessage,          16383,                 false,     0x4570,         7,         MailBox0,     Fifo0,            NA        , SingleFrameTailInit)
/* UAVCAN_DSDL_BIT_DEF(data_typ_name,                    field_name,        lsb_pos,    length,    payload_offset, payload_length) */
   UAVCAN_DSDL_BIT_DEF(LogMessage,                            level,              5,          3,              0,          1)
   UAVCAN_DSDL_BIT_DEF(LogMessage,                    source_length,              0,          5,              0,          1)
   UAVCAN_DSDL_BIT_DEF(LogMessage,                           source,              0,          8,              1,          4) // Bootloader specific uses is 4
   UAVCAN_DSDL_BIT_DEF(LogMessage,                             text,              0,          8,              5,          2) // Bootloader specific uses is 2

