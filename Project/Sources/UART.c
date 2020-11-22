/*! @file UART.c
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Seamus O'Sullivan & Joel Goodwin
 *  @date 2019-04-18
 */

/*!
 *  @addtogroup UART_module UART Documentation
 *  @{
*/



#include "IO_Map.h"
#include "types.h"
#include "FIFO.h"
#include "OS.h"
#include "Cpu.h"
#include "PE_Types.h"


// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100

// Prototypes
void ReceiveThread(void* pData);
void TransmitThread(void* pData);

// Variable Declarations

static OS_ECB *TransmitSemaphore;       /*!< Semaphore for data transmit */
static OS_ECB *ReceiveSemaphore;        /*!< Semaphore for data receive */



// Create Static TX RX FIFO
static TFIFO RxFIFO;  /*!< RxFIFO - FIFO Struct to store recieving data */
static TFIFO TxFIFO;  /*!< TxFIFO - FIFO Struct to store sending data */


bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{

  OS_ERROR error;

  //Initialise Tx & Rx FIFO
  FIFO_Init(&TxFIFO);
  FIFO_Init(&RxFIFO);

  uint8_t	tempBRFA; 	    /*!< tempBRFA - temporary variable to store BRFA values */
  int16union_t	tempSBR; 	/*!< tempSBR - union variable to store temporary SBR values */

  //Clock gate
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK; //Write 1 to field 12 for UART2
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //Write 1 to field 13 for PORTE

  // open port
  PORTE_PCR16|=PORT_PCR_MUX(0x3); //Set to Alt 3
  PORTE_PCR17|=PORT_PCR_MUX(0x3); //Set to Alt 3

  //Set baudRate
  tempSBR.l = moduleClk / (16 * baudRate);
  tempBRFA =  2*((moduleClk/baudRate)%16);

  // Write the SBR
  UART2_BDH = tempSBR.s.Hi;
  UART2_BDL = tempSBR.s.Lo;

  //Write to the BRFA
  UART2_C4 |= UART_C4_BRFA(tempBRFA);

  // enable UART Transmit and Receive
  UART2_C2 |= UART_C2_TE_MASK;
  UART2_C2 |= UART_C2_RE_MASK;

  //Enable interrupts
  UART2_C2 |= UART_C2_RIE_MASK; //Receiver Interrupt enabled

  //Initialize NVIC for UART
  NVICICPR1 = NVIC_ICPR_CLRPEND(1 << 17); 	//Clear pending interrupts
  NVICISER1 = NVIC_ISER_SETENA(1 << 17);	//Enable interrupts for UART

  ReceiveSemaphore = OS_SemaphoreCreate(0);   // Receive semaphore initialise
  TransmitSemaphore = OS_SemaphoreCreate(0);  // Transmit semaphore initialise



  return true;
}


bool UART_InChar(uint8_t* const dataPtr)
{
  return FIFO_Get(&RxFIFO, dataPtr);   // Gets a character out of the FIFO function
}


bool UART_OutChar(const uint8_t data)
{
  bool success = 0; 	/*!< success - check for function success */

  success = FIFO_Put(&TxFIFO, data);	// Puts a character in to FIFO function
  UART2_C2 |= UART_C2_TIE_MASK;     //Enable transmit interrupt

  return success;
}

/*! @brief Thread that receives incoming data via UART.
 *
 *  @param pData Thread parameter.
 */
void ReceiveThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(ReceiveSemaphore, 0);
    FIFO_Put(&RxFIFO, UART2_D);
    UART2_C2 |= UART_C2_RIE_MASK;
  }
}

/*! @brief Thread that transmitts outgoing data via UART
 *
 *  @param pData Thread parameter.
 */
void TransmitThread(void *data)
{
  for (;;)
  {
    OS_SemaphoreWait(TransmitSemaphore, 0);
    FIFO_Get(&TxFIFO,(uint8_t* )&UART2_D);
    UART2_C2 |= UART_C2_TIE_MASK;
  }
}


void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();

  //Receive
  if(UART2_C2 & UART_C2_RIE_MASK) //If receive interrupt arm is enabled
  {
    //Check if Receive is not empty
    if(UART2_S1 & UART_S1_RDRF_MASK) //Check if RDRF is set then clear
    {
      UART2_C2 &= ~UART_C2_RIE_MASK;
      (void)OS_SemaphoreSignal(ReceiveSemaphore);
    }
  }
  //Transmit
  if(UART2_C2 & UART_C2_TIE_MASK) //if transmit interrupt arm is enabled
  {
    if(UART2_S1 & UART_S1_TDRE_MASK) //Clear flag
    {
      UART2_C2 &= ~UART_C2_TIE_MASK;
      (void)OS_SemaphoreSignal(TransmitSemaphore);
    }
  }

  OS_ISRExit();
}


/*!
 * @}
*/
