/*! @file PIT.c
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Seamus O'Sullivan & Joel Goodwin
 *  @date 2019-04-29
 */

/*!
 *  @addtogroup PIT_module PIT documentation
 *  @{
*/


// new types
#include "types.h"
#include "IO_Map.h"
#include "OS.h"


static uint32_t ModuleClkPIT; 		/*!< ModuleClkPIT - Global private variable to store System clock frequency */
extern OS_ECB *PIT_Semaphore;           //


bool PIT_Init(const uint32_t moduleClk)
{
  ModuleClkPIT = moduleClk; 		//Assigning moduleClk to global private variable


  SIM_SCGC6 |=  SIM_SCGC6_PIT_MASK; 	//Enable clock gate
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;	//Enable Module

  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; 	//Enable interrupt for PIT

  //Configure NVIC for PIT
  NVICICPR2 =  NVIC_ICPR_CLRPEND(1 << 4); //Clear pending interrupts
  NVICISER2 = NVIC_ISER_SETENA(1 << 4); //Enable interrupts

  PIT_Semaphore = OS_SemaphoreCreate(0);

  return true;

}

void PIT_Enable(const bool enable)
{
  if(enable)
  {
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK; //Start timer
  }
  else
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; //Stop timer
  }
}

void PIT_Set(const uint32_t period, const bool restart)
{
  uint32_t tempLDVAL;   			/*!< tempLDVAL temporary value to store LDVAL */

  tempLDVAL = (period * (ModuleClkPIT / 1000) - 1);

  if(restart) //set new value, then enable PIT
  {
    PIT_Enable(false); 	    //Disables PIT just in case
    PIT_LDVAL0 = tempLDVAL;	//Writes time to LDVAL register
    PIT_Enable(true); 	    //Enables PIT timer
  }
  else
  {
    //No restart necessary, will wait until next trigger event
    PIT_LDVAL0 = tempLDVAL; 
  }
}

void __attribute__ ((interrupt)) PIT_ISR(void)
{

  OS_ISREnter();// Start of ISR
    
  PIT_TFLG0 = PIT_TFLG_TIF_MASK;  //w1c

  OS_SemaphoreSignal(PIT_Semaphore); // Sends the

  OS_ISRExit(); // end of ISR
}
