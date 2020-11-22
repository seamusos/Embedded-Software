/*! @file
 *
 *  @brief 
 *
 *  
 *
 *  @author 
 *  @date 
 */

#include "types.h"

#ifndef UTILITIES_H
#define UTILITIES_H


/*! @brief
 *
 *  
 * @return  uint16_t RMS of samples provided
 */
double Newton_RMS (double ADCsamples[]);

/*! @brief <Description>
 *
 *  @param (param) 
 *  @return <return type>
 */
int16union_t VoltageRMS_To_CurrentQ8 (int16_t ADCVoltageRMS);

/*! @brief <Description>
 *
 *  @param (param) 
 *  @return <return type>
 */
int16_t CurrentQ8toMilli(int16union_t currentQ8);

/*! @brief <Description>
 *
 *  @param (param) 
 *  @return <return type>
 */
double PeriodNano_To_FrequencyHz(uint32_t period_ns);

int16_t New_RMS(int16_t sample);

double FloatingRMS(double samples[]);

//bool FrequencyInterpolation(TAnalogThreadData* AnalogThreadData, uint8_t sampleCount);

/*! @brief <Description>
 *
 *  @param (param) 
 *  @return <return type>
 */


#endif
