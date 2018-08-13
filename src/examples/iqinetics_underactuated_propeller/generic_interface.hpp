#ifndef GENERIC_INTERFACE_H
#define GENERIC_INTERFACE_H

#include "communication_interface.h"
#include "packet_finder.h"
#include "byte_queue.h"
#include "bipbuffer.h"

#define GENERIC_PF_INDEX_DATA_SIZE 20   // size of index buffer in packet_finder

#ifndef GENERIC_TX_BUFFER_SIZE
  #define GENERIC_TX_BUFFER_SIZE 256
#endif

class GenericInterface: public CommunicationInterface{
  private:
      
  public:
    // Member Variables
    struct PacketFinder pf;        // packet_finder instance
    struct ByteQueue index_queue;              // needed by pf for storing indices
    uint8_t pf_index_data[GENERIC_PF_INDEX_DATA_SIZE]; // data for index_queue used by pf
    BipBuffer tx_bipbuf;   // bipbuffer for transmissions 
    uint8_t tx_buffer[GENERIC_TX_BUFFER_SIZE];   // raw buffer for transmissions 


    // Default Constructor
    GenericInterface();
    
    /*******************************************************************************
     * Receive
     ******************************************************************************/
    
    /// Poll the hardware for new byte data.
    ///   Returns: 1 packet ready
    ///            0 normal operation
    ///           -1 failure
    ///
    int8_t GetBytes();
    int8_t SetRxBytes(uint8_t* data_in, uint16_t length_in);
    
    /// Peek at the next available incoming packet. If a packet is ready, pointer 
    /// 'packet' will point to the first byte of type+data and 'length' will give 
    /// the length of packet type+data. Arguments 'packet' and 'length' are ignored 
    /// if no packet is ready.  Repeated calls to Peek will return pointers to the 
    /// same packet data until Drop is used.
    ///   Returns: 1 packet peek available
    ///            0 no packet peek available
    ///           -1 failure
    ///
    int8_t PeekPacket(uint8_t **packet, uint8_t *length);

    /// Drop the next available packet from queue. Usually called after Peek.
    ///   Returns: 1 packet removed
    ///            0 no packet ready to remove
    ///           -1 failure
    ///
    int8_t DropPacket();
    
    /*******************************************************************************
     * Send
     ******************************************************************************/

    /// Add a packet to the outgoing queue with automatically generated header 
    /// and CRC. If the buffer fills, the most recent data is lost.
    int8_t SendPacket(uint8_t msg_type, uint8_t *data, uint16_t length);

    /// Add bytes to the outgoing queue. 
    /// If the buffer fills, the most recent data is lost.
    int8_t SendBytes(uint8_t *bytes, uint16_t length);
    
    /// Does nothing in this interface
    void SendNow();
    
    /// Gets all outbound bytes 
    /// The data is copied into the user supplied data_out buffer.
    /// The length of data transferred is copied into length_out.
    /// Returns: 1 for data transferred
    ///          0 for no data transferred (buffer empty)
    int8_t GetTxBytes(uint8_t* data_out, uint8_t& length_out);
    
    /*******************************************************************************
     * Parsing
     ******************************************************************************/
     
    /// Read a given message and act appropriately.
    void ReadMsg(CommunicationInterface& com, uint8_t* data, uint8_t length);
}; // class GenericInterface

#endif // GENERIC_INTERFACE_H