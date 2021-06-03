/* ************************************************************************** */
// Nanolay - DAC Library Header File
//
// Description:     Custom dsPIC33CK library for DAC functions. Should be
//                  included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    13.Apr.21
/* ************************************************************************** */


#ifndef NANOLAY_DAC_H
#define	NANOLAY_DAC_H

#include <p33CK256MP202.h>
#include <stdint.h>
#include <stdbool.h>
#include "nanolay_sys.h"


// *****************************************************************************
// @desc:       Initialize DAC of CMP1 module. Enables DACOUT pin at PA3/RA3/AN3.
//                  Works only at Fosc = 16MHz and above
// @args:       None
// @returns:    None
// *****************************************************************************
void DAC_Init( void );


// *****************************************************************************
// @desc:       Outputs a 12bit value from 0.05AVDD to 0.95AVDD to PA3/RA3/AN3
// @args:       val [uint_fast16_t]: 12bit DAC value
// @returns:    None
// *****************************************************************************
void DAC_WriteDCValue( uint_fast16_t val );


#endif	// NANOLAY_DAC_H

