
/*! @file packet.c
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author Joel Goodwin & Seamus O'Sullivan
 *  @date 2019-04-8
 */


/*!
 *  @addtogroup Packet_module Packet documentation
 *  @{
*/


#include "packet.h"
#include "UART.h"
#include "PE_Types.h"
#include "OS.h"

const uint8_t PACKET_ACK_MASK =  0x80;
TPacket Packet;
OS_ECB *Access;

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  Access = OS_SemaphoreCreate(1);
  return UART_Init(baudRate, moduleClk); /* Sends the paramaters to the function UART_Init */
}

bool Packet_Get(void)
{
  static uint8_t packetStore[5] = {0}; /* creates a temporary array that holds the data of the individual packets before they form a full packet */
  static uint8_t packetIndex = 0; /* Sets the position of the array */

  while (UART_InChar(&packetStore[packetIndex])) /* Whilst there is data inside the actual FIFO keep removing them */
  {
    if (packetIndex == 4) /* Once the packet is full we can test for validity of the packet */
    {
      uint8_t testChecksum = (packetStore[0] ^ packetStore[1]) ^ (packetStore[2] ^ packetStore[3]); /* XOR all of the previous parameters to checksum */

      if (packetStore[4] == testChecksum) /* if the check sum is equal to the 5th element than the packet is valid */
      {
        Packet_Command    = packetStore[0]; /* sets the array values to the packet structure */
        Packet_Parameter1 = packetStore[1];
        Packet_Parameter2 = packetStore[2];
        Packet_Parameter3 = packetStore[3];
        Packet_Checksum   = packetStore[4];

//        Packet_Parameter12 = (((uint16_t)(Packet_Parameter1 << 8)) || ((uint16_t)(Packet_Parameter2))); // cocatenate the two parameters
//        Packet_Parameter23 = (((uint16_t)(Packet_Parameter2 << 8)) || ((uint16_t)(Packet_Parameter3)));


        packetIndex = 0; /* the packet has been built set the index to one to check build next packet */



        return true;
      }
      else
      {
        packetStore[0] = packetStore[1];  /* if the checksum is not valid shift the elements along so that it can re-sync */
        packetStore[1] = packetStore[2];
        packetStore[2] = packetStore[3];
        packetStore[3] = packetStore[4];
      }
    }
    
    else packetIndex++; /* increment the index so we can get the next byte from the FIFO for the array */
  }
  return false; /* If FIFO is empty return false */
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{

  bool success;
  OS_SemaphoreWait(Access, 0);

  success = ((UART_OutChar(command))&& /* returns the packet each parameter at a time */
          (UART_OutChar(parameter1))&&
          (UART_OutChar(parameter2))&&
          (UART_OutChar(parameter3))&&
          (UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3))); /* the last parameter is a check sum (XOR) of the previous elements */

  OS_SemaphoreSignal(Access);

  return success;

}

/*!
 * @}
*/



