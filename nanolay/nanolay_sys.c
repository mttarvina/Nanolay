/* ************************************************************************** */
// Nanolay - System Clock Library Header File
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

#include "nanolay_sys.h"


uint_fast8_t Master_Clock;


void Clock_Init( uint_fast8_t clk ){
    Master_Clock = clk;

    switch ( clk ){
        case _8MHZ:
            CLKDIV = 0x3001;                                                    // FRCDIV FRC/1; PLLPRE 1; DOZE 1:8; DOZEN disabled; ROI disabled; 
            PLLFBD = 0x96;                                                      // PLLFBDIV 150;
            PLLDIV = 0x141;                                                     // POST1DIV 1:4; VCODIV FVCO/3; POST2DIV 1:1; 
            ACLKCON1 = 0x101;                                                   // APLLEN disabled; FRCSEL FRC; APLLPRE 1:1;
            APLLFBD1 = 0x96;                                                    // APLLFBDIV 150;  
            APLLDIV1 = 0x141;                                                   // APOST1DIV 1:4; APOST2DIV 1:1; AVCODIV FVCO/3;    
            break;
        case _20MHZ:
            CLKDIV = 0x3001;                                                    // FRCDIV FRC/1; PLLPRE 1; DOZE 1:8; DOZEN disabled; ROI disabled;
            PLLFBD = 0x7D;                                                      // PLLFBDIV 125;  
            PLLDIV = 0x255;                                                     // POST1DIV 1:5; VCODIV FVCO/2; POST2DIV 1:5;
            ACLKCON1 = 0x8101;                                                  // APLLEN enabled; FRCSEL FRC; APLLPRE 1:1; 
            APLLFBD1 = 0x7D;                                                    // APLLFBDIV 125;  
            APLLDIV1 = 0x255;                                                   // APOST1DIV 1:5; APOST2DIV 1:5; AVCODIV FVCO/2;        
            break;
        case _50MHZ:
            CLKDIV = 0x3001;                                                    // FRCDIV FRC/1; PLLPRE 1; DOZE 1:8; DOZEN disabled; ROI disabled;
            PLLFBD = 0x7D;                                                      // PLLFBDIV 125;  
            PLLDIV = 0x252;                                                     // POST1DIV 1:5; VCODIV FVCO/2; POST2DIV 1:2;
            ACLKCON1 = 0x8101;                                                  // APLLEN enabled; FRCSEL FRC; APLLPRE 1:1; 
            APLLFBD1 = 0x7D;                                                    // APLLFBDIV 125;  
            APLLDIV1 = 0x252;                                                   // APOST1DIV 1:5; APOST2DIV 1:2; AVCODIV FVCO/2;       
            break;
        case _100MHZ:
            CLKDIV = 0x3001;                                                    // FRCDIV FRC/1; PLLPRE 1; DOZE 1:8; DOZEN disabled; ROI disabled;
            PLLFBD = 0x7D;                                                      // PLLFBDIV 125;  
            PLLDIV = 0x251;                                                     // POST1DIV 1:5; VCODIV FVCO/2; POST2DIV 1:1;
            ACLKCON1 = 0x8101;                                                  // APLLEN enabled; FRCSEL FRC; APLLPRE 1:1; 
            APLLFBD1 = 0x7D;                                                    // APLLFBDIV 125;  
            APLLDIV1 = 0x251;                                                   // APOST1DIV 1:5; APOST2DIV 1:1; AVCODIV FVCO/2;          
            break;
        default:
            break;
    }

    OSCTUN = 0x00;                                                              // TUN Center frequency;  
    REFOCONL = 0x00;                                                            // ROEN disabled; ROSWEN disabled; ROSLP disabled; ROSEL FOSC; ROOUT disabled; ROSIDL disabled;
    REFOCONH = 0x00;                                                            // RODIV 0; 
    REFOTRIMH = 0x00;                                                           // ROTRIM 0; 
    RPCON = 0x00;                                                               // IOLOCK disabled;
    PMDCON = 0x00;                                                              // PMDLOCK disabled;

    if ( Master_Clock == _8MHZ ){                                               // CF no clock failure; NOSC FRC; CLKLOCK unlocked; OSWEN Switch is Complete; 
        __builtin_write_OSCCONH((uint_fast8_t) (0x00));
        __builtin_write_OSCCONL((uint_fast8_t) (0x00));        
    }
    else {                                                                      // CF no clock failure; NOSC FRCPLL; CLKLOCK unlocked; OSWEN Switch is Complete; 
        __builtin_write_OSCCONH((uint_fast8_t) (0x01));
        __builtin_write_OSCCONL((uint_fast8_t) (0x01));
        while (OSCCONbits.OSWEN != 0);                                          // Wait for Clock switch to occur
        while (OSCCONbits.LOCK != 1);                                           // Wait for Clock switch to occur
    }    
    WDTCONLbits.ON = 0;                                                         // Disable Watchdog Timer 
}


uint_fast8_t GetMasterClkFrequency( void ){
    return Master_Clock;
}
