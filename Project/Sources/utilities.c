/*! @file Utilities.c
 *
 *  @brief 
 *  Contains general functions for use 
 *  
 *
 *  @author Seamus O'Sullivan
 *  @date 25/06/2019
 */

/*!
 *  @addtogroup Utilities_module utilities module documentation
 *  @{
*/

#include "types.h"
#include "utilities.h"
#include "analog.h"
#include <math.h>



#define TOFIX(d, q) ((int)( (d)*(double)(1<<(q)) ))
#define TOFLT(a, q) ( (double)(a) / (double)(1<<(q)) )



/*  Source from Fixed Point Arithmetic on the ARM
 *
 * 
 */
/* The basic operations perfomed on two numbers a and b of fixed
 point q format returning the answer in q format */
#define FADD(a,b) ((a)+(b))
#define FSUB(a,b) ((a)-(b))
#define FMUL(a,b,q) (((a)*(b))>>(q))
#define FDIV(a,b,q) (((a)<<(q))/(b))

/* The basic operations where a is of fixed point q format and b is
 an integer */
#define FADDI(a,b,q) ((a)+((b)<<(q)))
#define FSUBI(a,b,q) ((a)-((b)<<(q)))
#define FMULI(a,b) ((a)*(b))
#define FDIVI(a,b) ((a)/(b))

/* convert a from q1 format to q2 format */
#define FCONV(a, q1, q2) (((q2)>(q1)) ? (a)<<((q2)-(q1)) : (a)>>((q1)-(q2)))

int16union_t VoltageRMS_To_CurrentQ8 (int16_t ADCVoltageRMS)
{
  const int32_t voltsPerADC = 20; //32Q16 format
  const int32_t resistanceQ16 = 22937;
  int32_t voltageQ16;
  int32union_t currentRMSQ16;
  voltageQ16 = ADCVoltageRMS * voltsPerADC; //Voltage in 32Q16 format
  // currentRMSQ64 = voltageQ16 / resistanceQ16;
  currentRMSQ16.l = FDIV(voltageQ16, resistanceQ16, 16); //If I do this do I need to typecast?
  int16union_t currentRMSQ8;
  currentRMSQ8.l = FCONV(currentRMSQ16.l, 16, 8);
  return currentRMSQ8;
}

int16_t CurrentQ8toMilli(int16union_t currentQ8)
{
  int16_t currentMilli;
  currentMilli = (currentQ8.l * 1000) / (2^8);

  return currentMilli;
}



#define SAMPLES 16
#define TOLERANCE 10 //0.003volts

/*! @brief
 *
 *  
 * @return  uint16_t RMS of samples provided
 */
double Newton_RMS (double ADCsamples[])
{
  // uint16_t guess = 1; //Estimate for RMS
  // uint16_t initialGuess;  //Starting Value for RMS calculation
  double voltageRMS;
  double sumSquares = 0;

  for (uint8_t i = 0; i < SAMPLES; i++)
  {
    sumSquares += ADCsamples[i] * ADCsamples[i];
  }
  
  voltageRMS = sumSquares / 2;

  for(uint8_t error = 0; error < 10; error++ )
  {
    if (voltageRMS <= 0)
    {
      voltageRMS = 1; //Avoid div by zero
    }
    voltageRMS = (((sumSquares / voltageRMS) + voltageRMS) / 2);
  }
  return voltageRMS;
}

#define INITIAL 3657
#define SAMPLE_COUNT 16

int16_t New_RMS(int16_t sample)
{
  static int16_t oldRms;
  static int16_t rms = INITIAL;
//  rms = sample / 2;

  static int32_t sumSquares;

  sumSquares = SAMPLE_COUNT * INITIAL * INITIAL;

  sumSquares -= sumSquares / SAMPLE_COUNT;
  sumSquares += (int32_t) sample * sample;

//  for(uint8_t error = 0; error < 10; error++)
//  {
  if(rms == 0)
    rms = 1;
  rms = (rms + sumSquares / SAMPLES /  rms) / 2;
//  }

  oldRms = rms;
  return rms;
//}


  return true;
}


///*! @brief Converts ADC reading to current in milliAmps
// *
// *
// */
//uint16_t VoltageToCurrent(uint16_t voltage)
//{
//  const float resistance = 0.35; //milli Ohms
//  uint16_t currentmA;
//  double voltageNormal;
//  voltageNormal = ADCVoltageToNormal(voltage);
//  currentmA = ((double) voltage / resistance) * 1000;
//  return currentmA;
//}
//
//double ADCVoltageToNormal(uint16_t ADCVoltage)
//{
//  double voltage;
//  voltage = (double) ADCVoltage * (0.00030517578125);
//  return voltage;
//}

double PeriodNano_To_FrequencyHz(uint32_t period_ns)
{
  double period_s = 1000000000 / (double)period_ns;
  return (1 / period_s);
}



double FloatingRMS(double samples[])
{
  double sumSquares = 0;
  double testRMS;
  for (uint8_t i = 0; i < SAMPLES; i++)
  {
    sumSquares += samples[i] * samples[i];
  }

  for(uint8_t error = 0; error < 10; error++ )
  {
    if (testRMS <= 0)
    {
      testRMS = 1; //Avoid div by zero
    }
    testRMS = (((sumSquares / testRMS) + testRMS) / 2);
  }

  return (sqrt( (sumSquares) / SAMPLE_COUNT));
}

/*!
 ** @}
 */