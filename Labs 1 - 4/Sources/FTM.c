/*! @file FTM.c
 *
 *  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the FlexTimer module (FTM).
 *
 *  @author Joel Goodwin & Seamus O'Sullivan
 *  @date 2019-04-29
 */

/*!
 *  @addtogroup FTM_Module FTM Documentation
 *  @{
*/

#include "FTM.h"
#include "types.h"
#include "IO_Map.h"
#include "OS.h"

#define NoChannels 8

//void* UserArgument;          /*!< Global variable containing callback function arguments */
//void (*UserCallback)(void*); /*!< Global Variable pointing to callback userFunction */
extern OS_ECB *FTM_Semaphore;


bool FTM_Init()
{
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;   // Sets the FTM Module

  FTM0_CNTIN = ~FTM_CNTIN_INIT_MASK; // Ensure the counter is a free running counter

  FTM0_MOD = FTM_MOD_MOD_MASK;

  FTM0_CNT = ~FTM_CNT_COUNT_MASK;

  FTM0_SC = FTM_SC_CLKS(0x002); // selects the correct frequency clock

  NVICICPR1 = (1 << 62 % 32); //Clears pending interrupts
  NVICISER1 = (1 << 62 % 32); //Enables the interrupt for FTM

  FTM_Semaphore = OS_SemaphoreCreate(0);

  return true;
}


bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  if(aFTMChannel -> timerFunction == TIMER_FUNCTION_INPUT_CAPTURE)
  {
    FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_MSB_MASK);
    FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_MSA_MASK);// Sets the Input Capture Register to 00
  }
  else if (aFTMChannel -> timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE)
  {
    FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_MSB_MASK); // Sets the output capture (01)
    FTM0_CnSC(aFTMChannel -> channelNb) |= (FTM_CnSC_MSA_MASK);
  }

  switch(aFTMChannel -> ioType.inputDetection)
  {
    case 1:
      FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_ELSB_MASK); // Sets for rising edge  (01)
      FTM0_CnSC(aFTMChannel -> channelNb) |= (FTM_CnSC_ELSA_MASK);
      break;

    case 2:
      FTM0_CnSC(aFTMChannel -> channelNb) |= (FTM_CnSC_ELSB_MASK); // Sets for rising edge  (10)
      FTM0_CnSC(aFTMChannel -> channelNb) |= ~(FTM_CnSC_ELSA_MASK);
      break;

    case 3:
      FTM0_CnSC(aFTMChannel -> channelNb) |= (FTM_CnSC_ELSB_MASK); // Sets for rising edge  (01)
      FTM0_CnSC(aFTMChannel -> channelNb) |= (FTM_CnSC_ELSA_MASK);
      break;

    default:
      FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_ELSB_MASK); // Sets the default for rising edge  (00)
      FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_ELSA_MASK);
      break;

  }

  //UserArgument = aFTMChannel->callbackArguments; // userArguments made globally(private) accessible
  //UserCallback = aFTMChannel->callbackFunction; // userFunction made globally(private) accessible

  return true;

}


bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  if (aFTMChannel -> channelNb < NoChannels)
  {
    if (aFTMChannel -> timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE)
    {
      FTM0_CnSC(aFTMChannel -> channelNb) &= ~(FTM_CnSC_CHF_MASK); // Sets the default for rising edge  (00)
      FTM0_CnSC(aFTMChannel -> channelNb) |= (FTM_CnSC_CHIE_MASK); // Enables the Interupt

      FTM0_CnV(aFTMChannel -> channelNb) =  FTM0_CNT + (aFTMChannel -> delayCount); // Sets the initial count value

      return true;
    }
  }
   return false;
}


void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  OS_ISREnter(); // Start of servicing interrupt

  for (uint8_t i = 0; i < NoChannels; i++)
  {
    FTM0_CnSC(i) &= ~(FTM_CnSC_CHF_MASK); // Clears the Interupt Flag
    FTM0_CnSC(i) &= ~(FTM_CnSC_CHIE_MASK); // Disable the Interupt

    OS_SemaphoreSignal(FTM_Semaphore);

   // if(UserCallback)
    //{
     // (*UserCallback)(UserArgument);
    //}
  }
  OS_ISRExit(); // End of servicing interrupt
}

/*!
 * @}
*/

