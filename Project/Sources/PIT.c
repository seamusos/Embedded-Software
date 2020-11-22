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
#include "PIT.h"


static uint32_t ModuleClkPIT; 		/*!< ModuleClkPIT - Global private variable to store System clock frequency */
//extern  OS_ECB *PIT_Semaphore;        
OS_ECB *PIT_Semaphore;
OS_ECB *Trip_Semaphore;



bool PIT_Init(const uint32_t moduleClk)
{
  PIT_Semaphore = OS_SemaphoreCreate(0);
  Trip_Semaphore = OS_SemaphoreCreate(0);

  ModuleClkPIT = moduleClk; 		//Assigning moduleClk to global private variable


  SIM_SCGC6 |=  SIM_SCGC6_PIT_MASK; 	//Enable clock gate
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;	//Enable Module

  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; 	//Enable interrupt for PIT
  PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK; 	//Enable interrupt for PIT

  //Configure NVIC for PIT
  NVICICPR2 =  NVIC_ICPR_CLRPEND(1 << 4); //Clear pending interrupts
  NVICISER2 = NVIC_ISER_SETENA(1 << 4); //Enable interrupts

  //Configure NVIC for PIT Channel 1
  NVICICPR2 =  NVIC_ICPR_CLRPEND(1 << 5); //Clear pending interrupts
  NVICISER2 = NVIC_ISER_SETENA(1 << 5); //Enable interrupts

  return true;

}

void PIT_Enable(const bool enable, uint8_t pitChannel)
{
  switch (pitChannel)
  {
    case 0:
      if(enable)
      {
        PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK; //Start timer
      }
      else
      {
        PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; //Stop timer
      }
      break;
    
    case 1:
      if(enable)
      {
        PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK; //Start timer
      }
      else
      {
        PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK; //Stop timer
      }
      break;
    case 2:
      if(enable)
      {
        PIT_TCTRL2 |= PIT_TCTRL_TEN_MASK; //Start timer
      }
      else
      {
        PIT_TCTRL2 &= ~PIT_TCTRL_TEN_MASK; //Stop timer
      }
      break;


  }

}

void PIT_Set(const uint32_t period, const bool restart, uint8_t pitChannel)
{
  uint32_t tempLDVAL;   			/*!< tempLDVAL temporary value to store LDVAL */
  uint32_t triggerPeriodHz = 1000000000 / period; //Converts Nano Seconds to Hz 

  tempLDVAL = (ModuleClkPIT / (triggerPeriodHz)) - 1;

  // tempLDVAL = (period * (ModuleClkPIT / 1000) - 1);
  // tempLDVAL = (period / ModuleClkPIT ) - 1;

  switch (pitChannel)
  {
    case 0:
      if(restart) //set new value, then enable PIT
      {
        PIT_Enable(false, pitChannel); 	    //Disables PIT just in case
        PIT_LDVAL0 = tempLDVAL;	//Writes time to LDVAL register
        PIT_Enable(true, pitChannel); 	    //Enables PIT timer
      }
      else
      {
        //No restart necessary, will wait until next trigger event
        PIT_LDVAL0 = tempLDVAL; 
      }
      break;
    
    case 1:
      if(restart) //set new value, then enable PIT
      {
        PIT_Enable(false, pitChannel); 	    //Disables PIT just in case
        PIT_LDVAL1 = tempLDVAL;	//Writes time to LDVAL register
        PIT_Enable(true, pitChannel); 	    //Enables PIT timer
      }
      else
      {
        //No restart necessary, will wait until next trigger event
        PIT_LDVAL1 = tempLDVAL; 
      }
      break;
  }
}

void PIT_Timer_Start(void)
{
  PIT_Enable(false, 2);
  PIT_LDVAL2 = 0xFFFFFFFF;
  PIT_Enable(true, 2);
}

//Returns PIT Value in seconds
uint32_t PIT_Get(uint8_t pitChannel)
{
  uint32_t PITRead;
  uint32_t time_Ns;
  if (pitChannel == 0)
  {
    PITRead = PIT_CVAL0;
  }
  else if(pitChannel == 2)
  {
    PITRead = PIT_CVAL2;
  }

  time_Ns = (0xFFFFFFFF - PITRead + 1) / (ModuleClkPIT / 1000000000);
  
  return time_Ns;
}

void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();// Start of ISR

  if (PIT_TFLG0 & PIT_TFLG_TIF_MASK) //PIT Timer 2
  {
    PIT_TFLG0 = PIT_TFLG_TIF_MASK;  //w1c
    OS_SemaphoreSignal(PIT_Semaphore); //
  }

  if (PIT_TFLG1 & PIT_TFLG_TIF_MASK) //PIT Timer 1
  {
    PIT_TFLG1 = PIT_TFLG_TIF_MASK;  //w1c
    OS_SemaphoreSignal(Trip_Semaphore); //
  }
  
    


  OS_ISRExit(); // end of ISR
}
