/* ************************************************************************** */
// Nanolay - ADC Library Header File
//
// Description:     Custom dsPIC33CK library for ADC functions. Should be
//                  included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    19.Apr.21
/* ************************************************************************** */

#ifndef _NANOLAY_ADC_H
#define	_NANOLAY_ADC_H


#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>


#define AN0     0x0
#define AN1     0x1
#define AN2     0x2
#define AN3     0x3
#define AN4     0x4
#define AN5     0x5
#define AN6     0x6
#define AN7     0x7
#define AN8     0x8
#define AN9     0x9
#define AN10    0x10
#define AN11    0x11


// *****************************************************************************
// @desc:       Initialize ADC module. This is an internal function
// @args:       None
// @returns:    None
// *****************************************************************************
void ADC_Init( void );


// *****************************************************************************
// @desc:       Enable ADC Core0
// @args:       None
// @returns:    None
// *****************************************************************************
void ADC_Core0En( void );


// *****************************************************************************
// @desc:       Enable ADC Core1
// @args:       None
// @returns:    None
// *****************************************************************************
void ADC_Core1En( void );


// *****************************************************************************
// @desc:       Enable ADC Shared Core
// @args:       None
// @returns:    None
// *****************************************************************************
void ADC_ShrCoreEn( void );


// *****************************************************************************
// @desc:       Enable ADC input functionality on a pin. Calls ADC_Init, and
//                  enables the corresponding ADC Core
// @args:       None
// @returns:    None
// *****************************************************************************
void Analog_SetPin( uint_fast8_t pin );


// *****************************************************************************
// @desc:       Get the ADC value from pin AN0 - dedicated pin of ADC Core0
// @args:       None
// @returns:    [uint_fast16_t]
// *****************************************************************************
uint_fast16_t ADC_ReadAN0( void );


// *****************************************************************************
// @desc:       Get the ADC value from pin AN1 - dedicated pin of ADC Core1
// @args:       None
// @returns:    [uint_fast16_t]
// *****************************************************************************
uint_fast16_t ADC_ReadAN1( void );


// *****************************************************************************
// @desc:       Get the ADC value from pin AN2-AN11 - pin of ADC Shared Core
// @args:       pin [uint_fast8_t]: AN2-AN11
// @returns:    [uint_fast16_t]
// *****************************************************************************
uint_fast16_t ADC_ReadANx( uint_fast8_t pin );


#endif  // _NANOLAY_ADC_H