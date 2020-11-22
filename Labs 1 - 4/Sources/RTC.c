/*! @file RTC.c
 *
 *  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the real time clock (RTC).
 *
 *  @author Seamus O'Sullivan & Joel Goodwin
 *  @date 2019-04-29
 */

/*!
 *  @addtogroup RTC_module RTC module documentation
 *  @{
*/

// new types
#include "types.h"
#include "IO_Map.h"
#include "PE_Types.h"
#include "OS.h"


extern OS_ECB *UpdateClock_Semaphore; // declares the semaphore


bool RTC_Init(void)
{

  UpdateClock_Semaphore = OS_SemaphoreCreate(0); //Setup RTC Semaphore

  //Enable clock gate
  SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;

  //setup interrupt for TSR every second
  RTC_IER |= RTC_IER_TSIE_MASK;

  NVICICPR2 =  NVIC_ICPR_CLRPEND(1 << 3); //Clear pending interrupts for RTC
  NVICISER2 = NVIC_ISER_SETENA(1 << 3);   //Enable interrupts for RTC

  RTC_CR = RTC_CR_SWR_MASK; //Try software reset
  if(!(RTC_SR & RTC_SR_TIF_MASK))
  {
      //RTC Registers have been locked, assume setup
      return true;
  }
  RTC_CR &= ~RTC_CR_SWR_MASK;

  RTC_TSR = 0; //Start clock

  //setup capacitors for 18pF load
  RTC_CR |= RTC_CR_SC2P_MASK | RTC_CR_SC16P_MASK; //2pF + 16pF Load

  //Enable Oscillator
  RTC_CR |= RTC_CR_OSCE_MASK;

  //Wait for oscillator to stabilise
  for(int i = 0; i <= 60000; i++) //wait 60,000 loop cycles
  {
    //just chill
  }

  RTC_SR |= RTC_SR_TCE_MASK; //Start clock?

  //Lock control register
  RTC_LR &= ~RTC_LR_CRL_MASK;

  return true;
}


void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  uint32_t tempTSR = 0;	/*!< value to write and copy to TSR register */

  RTC_SR &= ~RTC_SR_TCE_MASK; //stop clock to write

  tempTSR = (hours * 3600) + (minutes * 60) + seconds; //Convert all to seconds
  RTC_TSR = tempTSR;

  RTC_SR |= RTC_SR_TCE_MASK; //restart clock
}


void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  uint32_t tempTSR = RTC_TSR; 		/*!< Reads snapshot of TSR */

  *hours = (tempTSR / 3600) % 24; //Converts to hours of the day
  tempTSR %= 3600; 			          //Removes hours

  *minutes = (tempTSR) / 60; 		  //creates minutes
  tempTSR %= 60; 			            //removes minutes

  *seconds = tempTSR;  			      //Update seconds
}


void __attribute__ ((interrupt)) RTC_ISR(void)
{
  OS_ISREnter();
  OS_SemaphoreSignal(UpdateClock_Semaphore);
  OS_ISRExit();
}

/*!
 * @}
*/
