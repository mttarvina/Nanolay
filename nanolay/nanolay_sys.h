/* ************************************************************************** */
// Nanolay - System Clock Library Source File
//
// Description:     Custom dsPIC33CK library for System Clock settings. Should
//                  be included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    12.Apr.21
/* ************************************************************************** */

#ifndef _NANOLAY_SYS_H
#define	_NANOLAY_SYS_H


#include <p33CK256MP202.h>
#include <stdint.h>


// *****************************************************************************
// 02.Mar.21 - mttarvina
// I don't think there is a need to define the exact value of frequency for now
// Clock_Init() should handle this and I just need a distinction between these
// frequencies. I think the frequency options listed here are more
// than enough for most applications. This also saves memory space
// *****************************************************************************
#define _8MHZ                   0x01
#define _20MHZ                  0x02
#define _50MHZ                  0x03
#define _100MHZ                 0x04


#define CLKOUTEN                false                                           // used for debug only, FOSC/2 at OSC2 pin


// *****************************************************************************
// @desc:       Initialize system CLK registers based on MASTER_CLK_xMHZ macro
//                  This is called by SysInit()
// @args:       [uint_fast8_t]: clk
// @returns:    None
// *****************************************************************************
void Clock_Init( uint_fast8_t clk );


// *****************************************************************************
// @desc:       Returns the macro value corresponding to Master Clk frequency
// @args:       None
// @returns:    [uint_fast8_t]: Master_Clock
// *****************************************************************************
uint_fast8_t GetMasterClkFrequency( void );

#endif // _NANOLAY_SYS_H