#ifndef COMMUNICATION_INTERFACE_H
#define	COMMUNICATION_INTERFACE_H

#include <stdint.h>

class CommunicationInterface{
  private:
  
  public:
    /*******************************************************************************
     * Receive
     ******************************************************************************/

    /// Poll the hardware for new byte data.
    ///   Returns: 1 packet ready
    ///            0 normal operation
    ///           -1 failure
    ///
    virtual int8_t GetBytes() = 0;

    /// Peek at the next available incoming packet. If a packet is ready, pointer 
    /// 'packet' will point to the first byte of type+data and 'length' will give 
    /// the length of packet type+data. Arguments 'packet' and 'length' are ignored 
    /// if no packet is ready.  Repeated calls to Peek will return pointers to the 
    /// same packet data until Drop is used.
    ///   Returns: 1 packet peek available
    ///            0 no packet peek available
    ///           -1 failure
    ///
    virtual int8_t PeekPacket(uint8_t **packet, uint8_t *length) = 0;

    /// Drop the next available packet from queue. Usually called after Peek.
    ///   Returns: 1 packet removed
    ///            0 no packet ready to remove
    ///           -1 failure
    ///
    virtual int8_t DropPacket() = 0;


    /*******************************************************************************
     * Send
     ******************************************************************************/

    /// Add a packet to the outgoing USB queue with automatically generated header 
    /// and CRC. A hardware transmission is not immediately initiated unless the 
    /// endpoint is filled. To force a transmission, follow with SendNow(). This 
    /// operation is nonblocking. If the buffer fills, the most recent data is lost.
    virtual int8_t SendPacket(uint8_t msg_type, uint8_t *data, uint16_t length) = 0;

    /// Add bytes to the outgoing USB queue. A hardware transmission is not 
    /// immediately initiated unless the endpoint is filled. To force a 
    /// transmission, follow with SendUsbNow(). This operation is 
    /// nonblocking. If the buffer fills, the most recent data is lost.
    virtual int8_t SendBytes(uint8_t *bytes, uint16_t length) = 0;

    /// Initiate a hardware transmission, which will chain transmissions through 
    /// the endpoint IN interrupt until the buffer empties completely.
    virtual void SendNow() = 0;
    
    /*******************************************************************************
     * Parsing
     ******************************************************************************/
     
    /// Read a given message and act appropriately.
    virtual void ReadMsg(CommunicationInterface& com, uint8_t* data, uint8_t length) = 0;
}; // end class CommunicationInterface

#endif // COMMUNICATION_INTERFACE_H