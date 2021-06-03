/* ************************************************************************** */
// Nanolay - Main Library Header File
//
// Description:     Custom dsPIC33CK library to be used with Nanolay dev Board
//                  as well as any future projects that uses dcPIC33CKxxxMP202
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    02.Jun.21
/* ************************************************************************** */


#ifndef _NANOLAY_H
#define	_NANOLAY_H


#include "nanolay_sys.h"
#include "nanolay_gpio.h"
#include "nanolay_tmr1.h"
#include "nanolay_sccp.h"
#include "nanolay_adc.h"
#include "nanolay_dac.h"
#include "nanolay_pwmx.h"


#define ENABLE                  true
#define DISABLE                 false


// *****************************************************************************
// @desc:       Initialize system clock, peripherals, and default GPIO states.
//                  Must be called at the start of main program
// @args:       clk_freq [uint_fast8_t]: Choose from macro constants defined in
//                  Core library
// @returns:    None
// *****************************************************************************
void Sys_Init( uint_fast8_t clk_freq );


// *****************************************************************************
// @desc:       Disables all peripherals. Called by Sys_Init(). Each peripheral
//                  will be enabled once the corresponding init function is
//                  called in the main program.
// @args:       None
// @returns:    None
// *****************************************************************************
void DisableAllPeripherals( void );


#endif // _NANOLAY_H