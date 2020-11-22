/*! @file
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the structure and "methods" for accessing a byte-wide FIFO.
 *
 *  @author Joel Goodwin and Seamus O'Sullivan
 *  @date 2019-03-21
 */

/*!
 *  @addtogroup FIFO_module FIFO Documentation
 *  @{
*/

#include "FIFO.h"
#include "PE_Types.h"
#include "Cpu.h"


bool FIFO_Init(TFIFO * const fifo)
{
   fifo->Start   = 0;  /*Initialising the start of the FIFO*/
   fifo->End   = 0;    /*Initialising the start of the FIFO*/
   fifo->NbBytes  = 0; /*Initialising the start of the FIFO*/
   fifo->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE); // Creates the space available in the Semaphore to block when full
   fifo->ItemsAvailable = OS_SemaphoreCreate(0); // Initialises the semaphore available to block when the semaphore is empty

   return true;
}


bool FIFO_Put(TFIFO * const fifo, const uint8_t data)
{
  if (fifo->NbBytes >= FIFO_SIZE)
  {
    OS_SemaphoreWait(fifo->SpaceAvailable,0); //If there is no space available it blocks the function
  }
  else
  {
    OS_DisableInterrupts(); // Start critical section protect from interupts
    fifo->Buffer[fifo->End]= data;  /*stores the data to the End location of the array Buffer*/
    fifo->NbBytes++;                /* Increases the amount of bytes*/
    fifo->End++;                    /* Increases the location of FIFO*/
    if (fifo->End >= FIFO_SIZE)     /* After the increases it checks to see if it is the same as FIFO Size if it is goes back to start*/
    {
      fifo->End = 0;                /* if the fifo is the same size as the maximum it moves the tail of the FIFO back to the start */
    }
    OS_EnableInterrupts(); // End of critical section interupts can now be used
    OS_SemaphoreSignal(fifo-> ItemsAvailable);    // Increments the amount of the items that are available in the buffer
    return true;         /* returns true if the fifo put data in to the fifo */
   }
}


bool FIFO_Get(TFIFO * const fifo, uint8_t * const dataPtr) /* gets the data from the buffer to send back */
{

  if (fifo->NbBytes <= 0) /* if the FIFO has nothing in it returns false */
  {
    OS_SemaphoreWait(fifo->ItemsAvailable,0); //blocks if there are no items available in the buffer
  }
  else
  {
    OS_DisableInterrupts();
    *dataPtr =fifo->Buffer[fifo->Start]; /* if the FIFO has elements it gets the oldest element of the Fifo buffer */
    fifo->NbBytes--;                      /* an element has left the array there bytes decrease */
    fifo->Start++;	                    /* increases the position of the start position to next oldest data */
    if (fifo->Start >= FIFO_SIZE)   /* if the fifo is now full place the start tail back at 0 */
    {
      fifo->Start =0;
    }
    OS_EnableInterrupts();
    OS_SemaphoreSignal(fifo-> SpaceAvailable);    // Increases the amount of space available in the buffer
    return true; /* returns true if data was captured from the FIFO */
  }
}

/*!
 * @}
*/
