/* ************************************************************************** */
// Nanolay - Timer1 Library Header File
//
// Description:     Custom dsPIC33CK library for Timer1 functions. Should be
//                  included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    28.Apr.21
/* ************************************************************************** */

#ifndef _NANOLAY_TMR1_H
#define	_NANOLAY_TMR1_H


#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nanolay_sys.h"


// 16bit counter values for 1ms period 
#define TMR1_1MS_COUNT_8MHZ    3999;
#define TMR1_1MS_COUNT_20MHZ   9999;
#define TMR1_1MS_COUNT_50MHZ   24999;
#define TMR1_1MS_COUNT_100MHZ  49999;


// builtin timer1 struct
typedef struct _TIMER1_OBJ_STRUCT{
    bool                    en_interrupt;
    bool                    is_waiting;
    uint_fast16_t           int_countmax;
    volatile uint_fast16_t  int_counter; 
    volatile uint_fast16_t  tmr_counter;
    volatile unsigned long long millis_counter;
} TIMER1_OBJ;



// *****************************************************************************
// @desc:       Enable Timer1 peripheral, initialize registers. Enables millis()
//                  function by default.
// @args:       active_on_idle [bool]: module is active when device is in idle
//                  mode
// @returns:    None
// *****************************************************************************
void Timer1_Init( bool active_on_idle );


// *****************************************************************************
// @desc:       Assigns a user defined function as interrupt callback routine,
//                  sets the callback interval. Only works if Timer1 is enabled
//                  as Interrupt
// @args:       interval_ms [uint_fast16_t]: Callback interval in ms
//              InterruptHandler [pointer]: User defined ISR
//              priority [uint_fast8_t]: priority level from 1-7
// @returns:    None
// *****************************************************************************
void Timer1_SetInterrupt( uint_fast16_t interval_ms, void (* InterruptHandler)(void), uint_fast8_t priority );


// *****************************************************************************
// @desc:       Change Timer1 interrupt interval
// @args:       interval_ms [uint_fast16_t]: New callback interval in ms
// @returns:    None
// *****************************************************************************
void Timer1_SetInterruptInterval( uint_fast16_t interval_ms );


// *****************************************************************************
// @desc:       Start Timer1, enable interrupt, clear interrupt flag. Must be
//                  called after all necessary initializations for Timer1
// @args:       None
// @returns:    None
// *****************************************************************************
void Timer1_Start( void );


// *****************************************************************************
// @desc:       Stop Timer1, disable interrupt
// @args:       None
// @returns:    None
// *****************************************************************************
void Timer1_Stop( void );


// *****************************************************************************
// @desc:       Wait for a specified duration in ms. Uses Timer1
// @args:       duration [uint_fast16_t]: duration in ms
// @returns:    None
// *****************************************************************************
void wait( uint_fast16_t duration );


// *****************************************************************************
// @desc:       Returns the value of millisecond counter since Timer1 was
//                  started.
// @args:       None
// @returns:    [unsigned long long]: value of millisecond counter
// *****************************************************************************
unsigned long long millis( void );


#endif // _NANOLAY_TMR1_H