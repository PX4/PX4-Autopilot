using System;
using MavLink;

namespace MavLink
{
    /// <summary>
    /// Mavlink communication class. 
    /// </summary>
    /// <remarks>
    /// Keeps track of state across send and receive of packets. 
    /// User of this class can just send Mavlink Messsages, and 
    /// receive them by feeding this class bytes off the wire as 
    /// they arrive
    /// </remarks>
   public class Mavlink
    {
       private byte[] leftovers;

       /// <summary>
       /// Event raised when a message is decoded successfully
       /// </summary>
       public event PacketReceivedEventHandler PacketReceived;

       /// <summary>
       /// Total number of packets successfully received so far
       /// </summary>
       public UInt32 PacketsReceived { get; private set; }

       /// <summary>
       /// Total number of packets which have been rejected due to a failed crc
       /// </summary>
       public UInt32 BadCrcPacketsReceived { get; private set; }
      
       /// <summary>
       /// Raised when a packet does not pass CRC
       /// </summary>
        public event PacketCRCFailEventHandler PacketFailedCRC;

       /// <summary>
       /// Raised when a number of bytes are passed over and cannot 
       /// be used to decode a packet
       /// </summary>
        public event PacketCRCFailEventHandler BytesUnused;

        // The current packet sequence number for transmission
        // public so it can be manipulated for testing
       // Normal usage would only read this
        public byte txPacketSequence; 

       /// <summary>
       /// Create a new MavlinkLink Object
       /// </summary>
       public Mavlink()
       {
           MavLinkSerializer.SetDataIsLittleEndian(MavlinkSettings.IsLittleEndian);
           leftovers = new byte[] {};
       }


        /// <summary>
       /// Process latest bytes from the stream. Received packets will be raised in the event
       /// </summary>
       public void ParseBytes(byte[] newlyReceived)
       {
           uint i = 0;

           // copy the old and new into a contiguous array
           // This is pretty inefficient...
           var bytesToProcess = new byte[newlyReceived.Length + leftovers.Length];
           int j = 0;

           for (i = 0; i < leftovers.Length; i++)
               bytesToProcess[j++] = leftovers[i];

           for (i = 0; i < newlyReceived.Length; i++)
               bytesToProcess[j++] = newlyReceived[i];

           i = 0;

           // we are going to loop and decode packets until we use up the data
           // at which point we will return. Hence one call to this method could
           // result in multiple packet decode events
           while (true)
           {
               // Hunt for the start char
               int huntStartPos = (int)i;

               while (i < bytesToProcess.Length && bytesToProcess[i] != MavlinkSettings.ProtocolMarker)
                   i++;

               if (i == bytesToProcess.Length)
               {
                   // No start byte found in all our bytes. Dump them, Exit.
                   leftovers = new byte[] { };
                   return;
               }

               if (i > huntStartPos)
               {
                   // if we get here then are some bytes which this code thinks are 
                   // not interesting and would be dumped. For diagnostics purposes,
                   // lets pop these bytes up in an event.
                   if (BytesUnused != null)
                   {
                       var badBytes = new byte[i - huntStartPos];
                       Array.Copy(bytesToProcess, huntStartPos, badBytes, 0, (int)(i - huntStartPos));
                       BytesUnused(this, new PacketCRCFailEventArgs(badBytes, bytesToProcess.Length - huntStartPos));
                   }
               }

               // We need at least the minimum length of a packet to process it. 
               // The minimum packet length is 8 bytes for acknowledgement packets without payload
               // if we don't have the minimum now, go round again
               if (bytesToProcess.Length - i < 8)
               {
                   leftovers = new byte[bytesToProcess.Length - i];
                   j = 0;
                   while (i < bytesToProcess.Length)
                       leftovers[j++] = bytesToProcess[i++];
                   return;
               }

               /*
                * Byte order:
                * 
                * 0  Packet start sign	
                * 1	 Payload length	 0 - 255
                * 2	 Packet sequence	 0 - 255
                * 3	 System ID	 1 - 255
                * 4	 Component ID	 0 - 255
                * 5	 Message ID	 0 - 255
                * 6 to (n+6)	 Data	 (0 - 255) bytes
                * (n+7) to (n+8)	 Checksum (high byte, low byte) for v0.9, lowbyte, highbyte for 1.0
                *
                */
               UInt16 payLoadLength = bytesToProcess[i + 1];

               // Now we know the packet length, 
               // If we don't have enough bytes in this packet to satisfy that packet lenghth,
               // then dump the whole lot in the leftovers and do nothing else - go round again
               if (payLoadLength > (bytesToProcess.Length - i - 8)) // payload + 'overhead' bytes (crc, system etc)
               {
                   // back up to the start char for next cycle
                   j = 0;

                   leftovers = new byte[bytesToProcess.Length - i];

                   for (; i < bytesToProcess.Length; i++)
                   {
                       leftovers[j++] = bytesToProcess[i];
                   }
                   return;
               }

               i++;

               // Check the CRC. Does not include the starting 'U' byte but does include the length
               var crc1 = Mavlink_Crc.Calculate(bytesToProcess, (UInt16)(i), (UInt16)(payLoadLength + 5));

               if (MavlinkSettings.CrcExtra)
               {
                   var possibleMsgId = bytesToProcess[i + 4];

                   if (!MavLinkSerializer.Lookup.ContainsKey(possibleMsgId))
                   {
                       // we have received an unknown message. In this case we don't know the special
                       // CRC extra, so we have no choice but to fail.

                       // The way we do this is to just let the procedure continue
                       // There will be a natural failure of the main packet CRC
                   }
                   else
                   {
                       var extra = MavLinkSerializer.Lookup[possibleMsgId];
                       crc1 = Mavlink_Crc.CrcAccumulate(extra.CrcExtra, crc1);
                   }
               }

               byte crcHigh = (byte)(crc1 & 0xFF);
               byte crcLow = (byte)(crc1 >> 8);

               byte messageCrcHigh = bytesToProcess[i +  5  + payLoadLength];
               byte messageCrcLow = bytesToProcess[i + 6  + payLoadLength];

               if (messageCrcHigh == crcHigh && messageCrcLow == crcLow)
               {
                   // This is used for data drop outs metrics, not packet windows
                   // so we should consider this here. 
                   // We pass up to subscribers only as an advisory thing
                   var rxPacketSequence = bytesToProcess[++i];
                   i++;
                   var packet = new byte[payLoadLength + 3];  // +3 because we are going to send up the sys and comp id and msg type with the data

                   for (j = 0; j < packet.Length; j++)
                       packet[j] = bytesToProcess[i + j];

                   var debugArray = new byte[payLoadLength + 7];
                   Array.Copy(bytesToProcess, (int)(i - 3), debugArray, 0, debugArray.Length);

                   //OnPacketDecoded(packet, rxPacketSequence, debugArray);

                   ProcessPacketBytes(packet, rxPacketSequence);

                   PacketsReceived++;

                   // clear leftovers, just incase this is the last packet
                   leftovers = new byte[] { };

                   //  advance i here by j to avoid unecessary hunting
                   // todo: could advance by j + 2 I think?
                   i = i + (uint)(j + 2);
               }
               else
               {
                   var badBytes = new byte[i + 7 + payLoadLength];
                   Array.Copy(bytesToProcess, (int)(i - 1), badBytes, 0, payLoadLength + 7);

                   if (PacketFailedCRC != null)
                   {
                       PacketFailedCRC(this, new PacketCRCFailEventArgs(badBytes, (int)(bytesToProcess.Length - i - 1)));
                   }

                   BadCrcPacketsReceived++;
               }
           }
       }  

       public byte[] Send(MavlinkPacket mavlinkPacket)
       {
           var bytes = this.Serialize(mavlinkPacket.Message, mavlinkPacket.SystemId, mavlinkPacket.ComponentId);
           return SendPacketLinkLayer(bytes);
       }

        // Send a raw message over the link - 
        // this  will add start byte, lenghth, crc and other link layer stuff
        private byte[] SendPacketLinkLayer(byte[] packetData)
        {
            /*
               * Byte order:
               * 
               * 0   Packet start sign	 
               * 1	 Payload length	 0 - 255
               * 2	 Packet sequence	 0 - 255
               * 3	 System ID	 1 - 255
               * 4	 Component ID	 0 - 255
               * 5	 Message ID	 0 - 255
               * 6 to (n+6)	 Data	 (0 - 255) bytes
               * (n+7) to (n+8)	 Checksum (high byte, low byte)
               *
               */
            var outBytes = new byte[packetData.Length + 5];

            outBytes[0] = MavlinkSettings.ProtocolMarker;
            outBytes[1] = (byte)(packetData.Length-3);  // 3 bytes for sequence, id, msg type which this 
                                                        // layer does not concern itself with
            outBytes[2] = unchecked(txPacketSequence++);

            int i;

            for ( i = 0; i < packetData.Length; i++)
            {
                outBytes[i + 3] = packetData[i];
            }

            // Check the CRC. Does not include the starting byte but does include the length
            var crc1 = Mavlink_Crc.Calculate(outBytes, 1, (UInt16)(packetData.Length + 2));

            if (MavlinkSettings.CrcExtra)
            {
                var possibleMsgId = outBytes[5];
                var extra = MavLinkSerializer.Lookup[possibleMsgId];
                crc1 = Mavlink_Crc.CrcAccumulate(extra.CrcExtra, crc1);
            }

            byte crc_high = (byte)(crc1 & 0xFF);
            byte crc_low = (byte)(crc1 >> 8);

            outBytes[i + 3] = crc_high;
            outBytes[i + 4] = crc_low;

            return outBytes;
        }


        // Process a raw packet in it's entirety in the given byte array
        // if deserialization is successful, then the packetdecoded event will be raised
        private void ProcessPacketBytes(byte[] packetBytes, byte rxPacketSequence)
        {
            //	 System ID	 1 - 255
            //	 Component ID	 0 - 255
            //	 Message ID	 0 - 255
            //   6 to (n+6)	 Data	 (0 - 255) bytes
            var packet = new MavlinkPacket
            {
                SystemId = packetBytes[0],
                ComponentId = packetBytes[1],
                SequenceNumber = rxPacketSequence,
                Message = this.Deserialize(packetBytes, 2)
            };

            if (PacketReceived != null)
            {
                PacketReceived(this, packet);
            }

            // else do what?
        }


        public MavlinkMessage Deserialize(byte[] bytes, int offset)
        {
            // first byte is the mavlink 
            var packetNum = (int)bytes[offset + 0];
            var packetGen = MavLinkSerializer.Lookup[packetNum].Deserializer;
            return packetGen.Invoke(bytes, offset + 1);
        }

        public byte[] Serialize(MavlinkMessage message, int systemId, int componentId)
        {
            var buff = new byte[256];

            buff[0] = (byte)systemId;
            buff[1] = (byte)componentId;

            var endPos = 3;

            var msgId = message.Serialize(buff, ref endPos);

            buff[2] = (byte)msgId;

            var resultBytes = new byte[endPos];
            Array.Copy(buff, resultBytes, endPos);

            return resultBytes;
        }
    }


   ///<summary>
   /// Describes an occurance when a packet fails CRC
   ///</summary>
   public class PacketCRCFailEventArgs : EventArgs
   {
       ///<summary>
       ///</summary>
       public PacketCRCFailEventArgs(byte[] badPacket, int offset)
       {
           BadPacket = badPacket;
           Offset = offset;
       }

       /// <summary>
       /// The bytes that filed the CRC, including the starting character
       /// </summary>
       public byte[] BadPacket;

       /// <summary>
       /// The offset in bytes where the start of the block begins, e.g 
       /// 50 would mean the block of badbytes would start 50 bytes ago 
       /// in the stread. No negative sign is necessary
       /// </summary>
       public int Offset;
   }

   ///<summary>
   /// Handler for an PacketFailedCRC Event
   ///</summary>
   public delegate void PacketCRCFailEventHandler(object sender, PacketCRCFailEventArgs e);


   public delegate void PacketReceivedEventHandler(object sender, MavlinkPacket e);


    ///<summary>
    /// Represents a Mavlink message - both the message object itself
    /// and the identified sending party
    ///</summary>
    public class MavlinkPacket
    {
        /// <summary>
        /// The sender's system ID
        /// </summary>
        public int SystemId;

        /// <summary>
        /// The sender's component ID
        /// </summary>
        public int ComponentId;

        /// <summary>
        /// The sequence number received for this packet
        /// </summary>
        public byte SequenceNumber;


        /// <summary>
        /// Time of receipt
        /// </summary>
        public DateTime TimeStamp;

        /// <summary>
        /// Object which is the mavlink message
        /// </summary>
        public MavlinkMessage Message;
    }

    /// <summary>
    /// Crc code copied/adapted from ardumega planner code
    /// </summary>
    internal static class Mavlink_Crc
    {
        const UInt16 X25_INIT_CRC = 0xffff;

        public static UInt16 CrcAccumulate(byte b, UInt16 crc)
        {
            unchecked
            {
                byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                ch = (byte)(ch ^ (ch << 4));
                return (UInt16)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
            }
        }


        // For a "message" of length bytes contained in the byte array
        // pointed to by buffer, calculate the CRC
        public static UInt16 Calculate(byte[] buffer, UInt16 start, UInt16 length)
        {
            UInt16 crcTmp = X25_INIT_CRC;

            for (int i = start; i < start + length; i++)
                crcTmp = CrcAccumulate(buffer[i], crcTmp);

            return crcTmp;
        }
    }
}
