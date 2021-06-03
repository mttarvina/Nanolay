/* ************************************************************************** */
// Nanolay - Timer1 Library Source File
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

#include "nanolay_tmr1.h"


static TIMER1_OBJ timer1;
void (*Timer1_InterruptHandler)(void) = NULL;


void Timer1_Init( bool active_on_idle ) {    
    uint_fast8_t clk = GetMasterClkFrequency();
    
    PMD1bits.T1MD = 0;                                                          // enable TIMER1 peripheral
    
    TMR1 = 0x00;                                                                // Clear TMR1 register; 
    
    switch ( clk ) {
        case _8MHZ:
            PR1 = TMR1_1MS_COUNT_8MHZ;
            break;
        case _20MHZ:
            PR1 = TMR1_1MS_COUNT_20MHZ;
            break;
        case _50MHZ:
            PR1 = TMR1_1MS_COUNT_50MHZ;
            break;
        case _100MHZ:
            PR1 = TMR1_1MS_COUNT_100MHZ;
            break;
        default:
            break;
    }

    T1CON = 0x0000;
    T1CONbits.TSIDL = !active_on_idle;

    timer1.en_interrupt = false;
    timer1.is_waiting = false;

    T1CONbits.TON = false;                                                      // Disable Timer1
    IEC0bits.T1IE = false;                                                      // Disabled Timer1 interrupt
}


void Timer1_SetInterrupt( uint_fast16_t interval_ms, void (* InterruptHandler)(void), uint_fast8_t priority ){ 
    IPC0bits.T1IP = priority;
    Timer1_InterruptHandler = InterruptHandler;
    timer1.int_countmax = interval_ms;
    timer1.en_interrupt = true;
}


void Timer1_SetInterruptInterval( uint_fast16_t interval_ms ){
    timer1.int_countmax = interval_ms;
}


void Timer1_Start( void ) {
    timer1.int_counter = 0;
    timer1.tmr_counter = 0;
    timer1.millis_counter = 0;
    IFS0bits.T1IF = false;              // Reset Timer1 interrupt flag
    IEC0bits.T1IE = true;               // Enable Timer1 interrupt
    T1CONbits.TON = true;               // Enabled Timer1
}


void Timer1_Stop( void ) {
    IEC0bits.T1IE = false;              // Disabled Timer1 interrupt
    T1CONbits.TON = false;              // Disable Timer1
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt() {
    IFS0bits.T1IF = false;
    
    timer1.millis_counter++;

    if ( timer1.en_interrupt ){
        timer1.int_counter++;
        if ( timer1.int_counter >= timer1.int_countmax ){
            timer1.int_counter = 0;
            Timer1_InterruptHandler();
        }
    }

    if ( timer1.is_waiting ){
        timer1.tmr_counter++;
    } 
}


void wait( uint_fast16_t duration ){
    timer1.tmr_counter = 0;
    timer1.is_waiting = true;
    while (timer1.tmr_counter < duration ){
        // do nothing
    }
    timer1.is_waiting = false;
}


unsigned long long millis( void ){
    return timer1.millis_counter;
}