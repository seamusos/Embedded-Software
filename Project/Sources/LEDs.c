
/*! @file LEDs.c
 *
 *  @brief Routines to access the LEDs on the TWR-K70F120M.
 *
 *  This contains the functions for operating the LEDs.
 *
 *  @author Joel Goodwin & Seamus O'Sullivan
 *  @date 2019-04-8
 */

/*!
 *  @addtogroup LED_module Led documentation
 *  @{
*/




// new types
#include "LEDs.h"
#include "MK70F12.h"



bool LEDs_Init(void)
{

  PORTA_PCR11 |= PORT_PCR_MUX(0x1); // Sets all of the multiplexer values to 1
  PORTA_PCR28 |= PORT_PCR_MUX(1);
  PORTA_PCR29 |= PORT_PCR_MUX(1);
  PORTA_PCR10 |= PORT_PCR_MUX(1);

  GPIOA_PDDR |= LED_ORANGE |  LED_YELLOW |  LED_GREEN |  LED_BLUE;// Configures the pins for the Leds

  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // turns on the port a clock

  GPIOA_PSOR |= LED_ORANGE |  LED_YELLOW |  LED_GREEN |  LED_BLUE; // turns all of the LEDs off
  return true;

}


void LEDs_On(const TLED color)
{
  // sets the colour of the led to the set register
  GPIOA_PCOR = (color);
}



void LEDs_Off(const TLED color)
{
  // led turns off writes to clear register
  GPIOA_PSOR = (color);
}


void LEDs_Toggle(const TLED color)
{
  // change the colour of the led
  GPIOA_PTOR =(color);
}


/*!
 * @}
*/
