#ifndef PACKET_FINDER_H
#define	PACKET_FINDER_H

/// PacketFinder enables extraction of well formed, crc-verified packets from a 
/// byte stream. It is a specialized queue/buffer which takes in raw bytes and 
/// returns packet data. The returned packet data is a byte array consisting of 
/// a type byte followed by data bytes.
///
/// General Packet Format:
/// | 0x55 | length | type | ---data--- | crcL | crcH |
///   'length' is the (uint8) number of bytes in 'data'
///   'type' is the (uint8) message type
///   'data' is a series of (uint8) bytes, serialized Little-Endian
///   'crc' is the (uint16) CRC value for 'length'+'type'+'data', Little-Endian
 
#include "byte_queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define COMPUTER
#ifdef COMPUTER
  #define PACKET_BUFFER_SIZE 1023                   /// size of input buffer
#else
  #define PACKET_BUFFER_SIZE 255                    /// size of input buffer
#endif
#define MAX_PACKET_SIZE 64                        /// size packet (including header and CRC)
#define MAX_PACKET_DATA_SIZE (MAX_PACKET_SIZE-5)  /// size data (not type, ect)

extern const uint8_t kStartByte;                  /// special start byte

/// Internally used state machine states, voluntarily opaque.
enum PacketFinderState {
  kStart,
  kLen,
  kType,
  kData,
  kCRCL,
  kCRCH,
};
 
/// Interally used instance state struct, voluntarily opaque.
struct PacketFinder {

  uint8_t* start_data;              /// pointer to first data byte in buffer
  uint8_t* end_data;                /// pointer to byte following last data byte in buffer
  
  enum PacketFinderState state;     /// parser state machine state
  
  uint16_t parse_index;             /// index of next parser operation
  uint16_t packet_start_index;      /// start index for current packet
  uint16_t received_length;         /// bytes in data reported for current packet
  
  uint16_t expected_crc;            /// crc calculated for recieved data
  uint16_t received_crc;            /// crc received in packet footer
  uint16_t data_bytes;              /// bytes of type+data read in current packet

  uint8_t buffer[PACKET_BUFFER_SIZE];   /// buffer of recieved packet data
  uint8_t out_buffer[MAX_PACKET_SIZE];  /// fixed output buffer for contiguous access
  
  struct ByteQueue* packet_indices;
      // maintain queue of indices in buffer at which to find complete packets
      // each index gives the location of a packet's length byte in buffer
      // length bytes of type+data always follow contiguously after
};
 
/// Initialize a PacketFinder state struct
/// @param pf Pointer to an uninitialized PacketFinder struct.
void InitPacketFinder(struct PacketFinder *pf, struct ByteQueue *bq);

/// Add a data byte array to the packet extractor queue.
/// @param pf Pointer to PacketFinder struct.
/// @param data Pointer to new data bytes.
/// @param length Number of bytes to read from data
/// @return  Returns 0 on success,
///                 -1 on buffer overflow (data loss),
///                 -2 for fatal error
int8_t PutBytes(struct PacketFinder *pf, const uint8_t *data, uint16_t length);

/// Peek at the next available packet in queue. If a packet is ready, pointer 
/// 'packet' will point to the first byte (the type) and 'length' will give the 
/// length of packet type+data, which may be read as a contiguous array. 
/// Arguments 'packet' and 'length' are ignored if no packet is ready.  Repeated 
/// calls to PeekPacket will return pointers to the same packet until DropPacket 
/// is used.
/// @param pf Pointer to PacketFinder struct.
/// @param packet Reference to a pointer to data, function redirects pointer.
/// @param length Reference to an integer byte length, function changes value.
/// @return Returns 1 if packet peek available,
///                 0 if no packet ready to peak
int8_t PeekPacket(struct PacketFinder *pf, uint8_t **packet, uint8_t *length);

/// Drop the next available packet from queue. Usually called after PeekPacket.
/// @param pf Pointer to PacketFinder struct
/// @return Returns 1 if packet removed,
///                 0 if no packet ready to remove,
int8_t DropPacket(struct PacketFinder *pf);

/// Copy the next available completed packet and remove from queue. Output 
/// argument 'packet' must reserve enough space to copy the packet. Output 
/// argument 'length' is of type+data. Use of the Peek/Drop functions will 
/// typically yield better performance with less copying.
/// @param pf Pointer to PacketFinder struct.
/// @param packet Pointer to byte data, function changes data.
/// @param length Reference to an integer byte length, function changes value.
/// @return Returns 1 if packet removed,
///                 0 if no packet ready to pop
int8_t GetPacketCopy(struct PacketFinder *pf, uint8_t *packet, uint8_t *length);

/// Embed a byte data string into a packet with start header, type, and CRC 
/// footer. The generated packet copied to out_data is in_len+5 bytes long, and 
/// sufficient space in out_data must be reserved!
/// @param type Message type byte.
/// @param in_data Pointer to data for packet.
/// @param in_length Number of bytes of data.
/// @param out_data Pointer to location for function to create packet.
/// @param out_len Pointer to integer packet length, function changes value.
int8_t FormPacket(uint8_t type,
                  const uint8_t *in_data, uint8_t in_len,
                  uint8_t *out_data, uint8_t *out_len);
                  
#ifdef __cplusplus
}
#endif // __cplusplus

#endif	// PACKET_FINDER_H
