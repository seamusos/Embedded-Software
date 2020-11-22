/*! @file   lookup.c
 *
 *  @brief COntains lookup table and function for finding the 
 *  inverse time of a Over Current Relay*
 *  
 *
 *  @author Seamus O'Sullivan
 *  @date 25/06/2019
 */

#include "types.h"


#ifndef LOOKUP_H
#define LOOKUP_H

/*! @brief TimingLookup
 *  Lookup table for inverse timing
 *  @param double - RMS current of Relay
 *  @param state - Relay characteristic  
 *  @return uint64_t returns Time in microseconds
 */
uint64_t TimingLookup(double current, uint8_t state);


#endif

/*!
 ** @}
 */