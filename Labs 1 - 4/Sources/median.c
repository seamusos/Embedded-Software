/*! @file
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author Seamus O'Sullivan & Joel Goodwin
 *  @date 2015-10-12
 */

/*!
 *  @addtogroup median_module Median module documentation
 *  @{
*/

// New types
#include "types.h"


uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3)
{
    uint8_t median; /*< paramater to store medium result*/

    if(n1 >= n2)
    {
        if(n1 <= n3)
        {
            median = n1; // n3 > n1 > n2
        }
        else if(n3 >= n2)
        {
            median = n3; //n1 > n3 > n2
        }
        else
        {
            median = n2;    //n1 > n2 > n3
        }

    }
    else
    {
        if(n2 <= n3)
        {
            median = n2;
        }
        else if(n3 >= n1)
        {
            median = n3;
        }
        else
        {
            median = n1;
        }

    }

    return median;

}


/*!
 * @}
*/
