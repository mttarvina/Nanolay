/* ************************************************************************** */
// Nanolay - SCCP Library Source File
//
// Description:     Custom dsPIC33CK library for SCCP functions. Should be
//                  included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    28.Apr.21
/* ************************************************************************** */


#include "nanolay_sccp.h"

// *****************************************************************************
// Timer2 [SCCP1] is a dual 16bit general purpose timer interrupt. Custom ISRs
//      can be individually assigned to Timer2_InterruptHandlerA and
//      Timer2_InterruptHandlerB                  
// *****************************************************************************

static TIMER2_OBJ timer2;
void (*Timer2_InterruptHandlerA)(void) = NULL;
void (*Timer2_InterruptHandlerB)(void) = NULL;


void Timer2_Init( bool active_on_idle, bool active_on_sleep ){
    PMD2bits.CCP1MD = 0;                                                        // enable SCCP1 peripheral

    CCP1CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP1CON1Lbits.CCPSIDL = !active_on_idle;
    CCP1CON1Lbits.CCPSLP = active_on_sleep;
    CCP1CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP1CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP1CON1Lbits.TMRSYNC = 0;                                                  // sync disabled
    CCP1CON1Lbits.T32 = 0;                                                      // SCCP1 is a dual 16bit timer
    CCP1CON1Lbits.CCSEL = 0;                                                    // Timer mode
    CCP1CON1Lbits.MOD = 0x0;                                                    // 16-Bit/32-Bit Timer mode, output functions are disabled

    CCP1CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP1CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP1CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP1CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP1STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 
    CCP1PRL = 0x0000;                                                           // initially set primary timer period to 0
    CCP1PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP1TMRL = 0x00;                                                            // TMR 0; 
    CCP1TMRH = 0x00;                                                            // TMR 0; 
    CCP1RA = 0x00;                                                              // CMP 0; 
    CCP1RB = 0x00;                                                              // CMP 0;
    CCP1BUFL = 0x00;                                                            // BUF 0; 
    CCP1BUFH = 0x00;                                                            // BUF 0;

    IEC0bits.CCT1IE = false;
    IEC0bits.CCP1IE = false;
}


void Timer2_SetInterruptA( uint_fast16_t interval_ms, void (* InterruptHandler)(void), uint_fast8_t priority ){

    IPC1bits.CCT1IP = priority;
    Timer2_InterruptHandlerA = InterruptHandler;
    
    uint_fast8_t clk = GetMasterClkFrequency();
    switch( clk ){
        case _8MHZ:
            CCP1PRL = SCCP_1MS_COUNT_8MHZ;
            break;
        case _20MHZ:
            CCP1PRL = SCCP_1MS_COUNT_20MHZ;
            break;
        case _50MHZ:
            CCP1PRL = SCCP_1MS_COUNT_50MHZ;
            break;
        case _100MHZ:
            CCP1PRL = SCCP_1MS_COUNT_100MHZ;
            break;
    }
    timer2.intA_counter = 0;
    timer2.intA_countmax = interval_ms;
    IFS0bits.CCT1IF = false;                                                    // clear primary timer flag
    IEC0bits.CCT1IE = true;                                                     // enable primary timer interrupt
}


void Timer2_SetInterruptIntervalA( uint_fast16_t interval_ms ){
    timer2.intA_countmax = interval_ms;
}


void Timer2_SetInterruptB( uint_fast16_t interval_ms, void (* InterruptHandler)(void), uint_fast8_t priority ){

    IPC1bits.CCP1IP = priority;
    Timer2_InterruptHandlerB = InterruptHandler;

    uint_fast8_t clk = GetMasterClkFrequency();
    switch( clk ){
        case _8MHZ:
            CCP1PRH = SCCP_1MS_COUNT_8MHZ;
            break;
        case _20MHZ:
            CCP1PRH = SCCP_1MS_COUNT_20MHZ;
            break;
        case _50MHZ:
            CCP1PRH = SCCP_1MS_COUNT_50MHZ;
            break;
        case _100MHZ:
            CCP1PRH = SCCP_1MS_COUNT_100MHZ;
            break;
    }
    timer2.intB_counter = 0;
    timer2.intB_countmax = interval_ms;
    IFS0bits.CCP1IF = false;                                                    // clear secondary timer flag
    IEC0bits.CCP1IE = true;                                                     // enable secondary timer interrupt
}


void Timer2_SetInterruptIntervalB( uint_fast16_t interval_ms ){
    timer2.intB_countmax = interval_ms;
}


void Timer2_Start( void ){
    CCP1CON1Lbits.CCPON = true;
}


void Timer2_EnableInterruptA( void ){
    IFS0bits.CCT1IF = false;                                                    // clear primary timer flag
    IEC0bits.CCT1IE = true;                                                     // enable primary timer interrupt
}


void Timer2_DisableInterruptA( void ){
    IFS0bits.CCT1IF = false;                                                    // clear primary timer flag
    IEC0bits.CCT1IE = false;                                                    // disable primary timer interrupt
}


void Timer2_EnableInterruptB( void ){
    IFS0bits.CCP1IF = false;                                                    // clear secondary timer flag
    IEC0bits.CCP1IE = true;                                                     // enable secondary timer interrupt
}


void Timer2_DisableInterruptB( void ){
    IFS0bits.CCP1IF = false;                                                    // clear secondary timer flag
    IEC0bits.CCP1IE = false;                                                    // disable secondary timer interrupt
}


void Timer2_Stop( void ){
    CCP1CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT1Interrupt ( void ){
    IFS0bits.CCT1IF = false;
    
    timer2.intA_counter++;
    if ( timer2.intA_counter >= timer2.intA_countmax ){
        timer2.intA_counter = 0;
        Timer2_InterruptHandlerA();
    }
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCP1Interrupt ( void ){
    IFS0bits.CCP1IF = false;

    timer2.intB_counter++;
    if ( timer2.intB_counter >= timer2.intB_countmax ){
        timer2.intB_counter = 0;
        Timer2_InterruptHandlerB();
    }
}




// *****************************************************************************
// Timer3 [SCCP2] is a single 32bit general purpose timer interrupt. Custom ISR
//      can be assigned to Timer3_InterruptHandler
// *****************************************************************************


void (*Timer3_InterruptHandler)(void) = NULL;


void Timer3_Init( bool active_on_idle, bool active_on_sleep ){
    PMD2bits.CCP2MD = 0;                                                        // enable SCCP2 peripheral

    CCP2CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP2CON1Lbits.CCPSIDL = !active_on_idle;
    CCP2CON1Lbits.CCPSLP = active_on_sleep;
    CCP2CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP2CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP2CON1Lbits.TMRSYNC = 0;                                                  // sync enabled
    CCP2CON1Lbits.T32 = 1;                                                      // SCCP2 is a single 32bit timer
    CCP2CON1Lbits.CCSEL = 0;                                                    // Timer mode
    CCP2CON1Lbits.MOD = 0x0;                                                    // 16-Bit/32-Bit Timer mode, output functions are disabled

    CCP2CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP2CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP2CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP2CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP2STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 
    CCP2PRL = 0x0000;                                                           // initially set primary timer period to 0
    CCP2PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP2TMRL = 0x00;                                                            // TMR 0; 
    CCP2TMRH = 0x00;                                                            // TMR 0; 
    CCP2RA = 0x00;                                                              // CMP 0; 
    CCP2RB = 0x00;                                                              // CMP 0;
    CCP2BUFL = 0x00;                                                            // BUF 0; 
    CCP2BUFH = 0x00;                                                            // BUF 0;

    IEC1bits.CCT2IE = false;
    IEC1bits.CCP2IE = false;
}


void Timer3_SetInterrupt( uint_fast32_t interval_us, void (* InterruptHandler)(void), uint_fast8_t priority ){

    IPC6bits.CCT2IP = priority;
    Timer3_InterruptHandler = InterruptHandler;
    
    uint_fast8_t clk = GetMasterClkFrequency();
    uint_fast32_t val = 0;

    switch( clk ){
        case _8MHZ:
            val = (interval_us * SCCP_1US_COUNT_8MHZ) - 1;
            break;
        case _20MHZ:
            val = (interval_us * SCCP_1US_COUNT_20MHZ) - 1;
            break;
        case _50MHZ:
            val = (interval_us * SCCP_1US_COUNT_50MHZ) - 1;
            break;
        case _100MHZ:
            val = (interval_us * SCCP_1US_COUNT_100MHZ) - 1;
            break;
    }
    
    CCP2PRL = (val & 0x0000FFFF);
    CCP2PRH = ((val & 0xFFFF0000) >> 16);
}


void Timer3_Start( void ){
    IFS1bits.CCT2IF = false;    // clear primary timer flag
    IEC1bits.CCT2IE = true;     // enable primary timer interrupt
    CCP2CON1Lbits.CCPON = true;
}


void Timer3_Stop( void ){
    IEC1bits.CCP2IE = false;
    IEC1bits.CCT2IE = false;
    CCP2CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT2Interrupt ( void ){
    IFS1bits.CCT2IF = false;
    Timer3_InterruptHandler();
}




// *****************************************************************************
// Timer4 [SCCP3] is a single 32bit general purpose timer interrupt. Custom ISR
//      can be assigned to Timer4_InterruptHandler
// *****************************************************************************


void (*Timer4_InterruptHandler)(void) = NULL;


void Timer4_Init( bool active_on_idle, bool active_on_sleep ){
    PMD2bits.CCP3MD = 0;                                                        // enable SCCP3 peripheral

    CCP3CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP3CON1Lbits.CCPSIDL = !active_on_idle;
    CCP3CON1Lbits.CCPSLP = active_on_sleep;
    CCP3CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP3CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP3CON1Lbits.TMRSYNC = 0;                                                  // sync enabled
    CCP3CON1Lbits.T32 = 1;                                                      // SCCP3 is a single 32bit timer
    CCP3CON1Lbits.CCSEL = 0;                                                    // Timer mode
    CCP3CON1Lbits.MOD = 0x0;                                                    // 16-Bit/32-Bit Timer mode, output functions are disabled

    CCP3CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP3CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP3CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP3CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP3STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 
    CCP3PRL = 0x0000;                                                           // initially set primary timer period to 0
    CCP3PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP3TMRL = 0x00;                                                            // TMR 0; 
    CCP3TMRH = 0x00;                                                            // TMR 0; 
    CCP3RA = 0x00;                                                              // CMP 0; 
    CCP3RB = 0x00;                                                              // CMP 0;
    CCP3BUFL = 0x00;                                                            // BUF 0; 
    CCP3BUFH = 0x00;                                                            // BUF 0;

    IEC2bits.CCT3IE = false;
    IEC2bits.CCP3IE = false;
}


void Timer4_SetInterrupt( uint_fast32_t interval_us, void (* InterruptHandler)(void), uint_fast8_t priority ){

    IPC9bits.CCT3IP = priority;
    Timer4_InterruptHandler = InterruptHandler;
    
    uint_fast8_t clk = GetMasterClkFrequency();
    uint_fast32_t val = 0;

    switch( clk ){
        case _8MHZ:
            val = (interval_us * SCCP_1US_COUNT_8MHZ) - 1;
            break;
        case _20MHZ:
            val = (interval_us * SCCP_1US_COUNT_20MHZ) - 1;
            break;
        case _50MHZ:
            val = (interval_us * SCCP_1US_COUNT_50MHZ) - 1;
            break;
        case _100MHZ:
            val = (interval_us * SCCP_1US_COUNT_100MHZ) - 1;
            break;
    }
    
    CCP3PRL = (val & 0x0000FFFF);
    CCP3PRH = ((val & 0xFFFF0000) >> 16);
}


void Timer4_Start( void ){
    IFS2bits.CCT3IF = false;    // clear primary timer flag
    IEC2bits.CCT3IE = true;     // enable primary timer interrupt
    CCP3CON1Lbits.CCPON = true;
}


void Timer4_Stop( void ){
    IEC2bits.CCP3IE = false;
    IEC2bits.CCT3IE = false;
    CCP3CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT3Interrupt ( void ){
    IFS2bits.CCT3IF = false;
    Timer4_InterruptHandler();
}




// *****************************************************************************
// PWMA [SCCP4] is a 16bit general purpose PWM generator. PWM can be assigned to
//      any GPIO pin of PORT_A
// *****************************************************************************

uint_fast16_t pwma_pin;
uint_fast16_t pwma_period;


void PWMA_Init( uint_fast16_t pin, uint_fast16_t period_us, bool active_on_idle, bool active_on_sleep, uint_fast8_t priority ){
    pwma_pin = pin;                                                             // assign PWM to gpio pin

    PMD2bits.CCP4MD = 0;                                                        // enable SCCP4 peripheral

    CCP4CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP4CON1Lbits.CCPSIDL = !active_on_idle;
    CCP4CON1Lbits.CCPSLP = active_on_sleep;
    CCP4CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP4CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP4CON1Lbits.TMRSYNC = 0;                                                  // sync enabled
    CCP4CON1Lbits.T32 = 0;                                                      // SCCP4 in 16bit timer since PWM works only at 16bit
    CCP4CON1Lbits.CCSEL = 0;                                                    // PWM mode
    CCP4CON1Lbits.MOD = 0x5;                                                    // Dual Edge Compare mode, buffered (PWM)

    CCP4CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP4CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP4CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP4CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP4STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 

    CCP4PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP4TMRL = 0x0000;                                                          // TMR 0; 
    CCP4TMRH = 0x0000;                                                          // TMR 0; 
    CCP4RA = 0x0000;                                                            // CMP 0; 
    CCP4RB = 0x0000;                                                            // CMP 0;
    CCP4BUFL = 0x0000;                                                          // BUF 0; 
    CCP4BUFH = 0x0000;                                                          // BUF 0;

    uint_fast8_t clk = GetMasterClkFrequency();

    switch( clk ){
        case _8MHZ:
            pwma_period = (period_us * SCCP_1US_COUNT_8MHZ) - 1;
            break;
        case _20MHZ:
            pwma_period = (period_us * SCCP_1US_COUNT_20MHZ) - 1;
            break;
        case _50MHZ:
            pwma_period = (period_us * SCCP_1US_COUNT_50MHZ) - 1;
            break;
        case _100MHZ:
            pwma_period = (period_us * SCCP_1US_COUNT_100MHZ) - 1;
            break;
    }
    
    CCP4PRL = pwma_period;

    IEC2bits.CCT4IE = false;
    IEC2bits.CCP4IE = false;
    IPC10bits.CCT4IP = priority;
}


void PWMA_Write( float duty_cycle ){
    CCP4RB = (int) (duty_cycle * pwma_period);
}


void PWMA_Start( void ){
    IFS2bits.CCT4IF = false;    // clear primary timer flag
    IFS2bits.CCP4IF = false;    // clear secondary timer flag
    IEC2bits.CCT4IE = true;     // enable primary interrupt
    IEC2bits.CCP4IE = true;     // enable secondary interrupt
    CCP4CON1Lbits.CCPON = true;
}


void PWMA_Stop( void ){
    IEC2bits.CCP4IE = false;
    IEC2bits.CCT4IE = false;
    CCP4CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT4Interrupt ( void ){
    // drive GPIO High at period reset
    IFS2bits.CCT4IF = false;
    LATA = LATA | pwma_pin;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCP4Interrupt ( void ){
    // drive GPIO low at pulse width compare 
    IFS2bits.CCP4IF = false;
    LATA = LATA & (~pwma_pin);
}




// *****************************************************************************
// PWMB1 [SCCP5] is a 16bit general purpose PWM generator. PWM can be assigned
//      to any GPIO pin of PORT_B 
// *****************************************************************************

uint_fast16_t pwmb1_pin;
uint_fast16_t pwmb1_period;


void PWMB1_Init( uint_fast16_t pin, uint_fast16_t period_us, bool active_on_idle, bool active_on_sleep, uint_fast8_t priority ){
    pwmb1_pin = pin;                                                            // assign PWM to gpio pin

    PMD2bits.CCP5MD = 0;                                                        // enable SCCP5 peripheral

    CCP5CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP5CON1Lbits.CCPSIDL = !active_on_idle;
    CCP5CON1Lbits.CCPSLP = active_on_sleep;
    CCP5CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP5CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP5CON1Lbits.TMRSYNC = 0;                                                  // sync enabled
    CCP5CON1Lbits.T32 = 0;                                                      // SCCP5 in 16bit timer since PWM works only at 16bit
    CCP5CON1Lbits.CCSEL = 0;                                                    // PWM mode
    CCP5CON1Lbits.MOD = 0x5;                                                    // Dual Edge Compare mode, buffered (PWM)

    CCP5CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP5CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP5CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP5CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP5STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 

    CCP5PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP5TMRL = 0x0000;                                                          // TMR 0; 
    CCP5TMRH = 0x0000;                                                          // TMR 0; 
    CCP5RA = 0x0000;                                                            // CMP 0; 
    CCP5RB = 0x0000;                                                            // CMP 0;
    CCP5BUFL = 0x0000;                                                          // BUF 0; 
    CCP5BUFH = 0x0000;                                                          // BUF 0;

    uint_fast8_t clk = GetMasterClkFrequency();

    switch( clk ){
        case _8MHZ:
            pwmb1_period = (period_us * SCCP_1US_COUNT_8MHZ) - 1;
            break;
        case _20MHZ:
            pwmb1_period = (period_us * SCCP_1US_COUNT_20MHZ) - 1;
            break;
        case _50MHZ:
            pwmb1_period = (period_us * SCCP_1US_COUNT_50MHZ) - 1;
            break;
        case _100MHZ:
            pwmb1_period = (period_us * SCCP_1US_COUNT_100MHZ) - 1;
            break;
    }
    
    CCP5PRL = pwmb1_period;

    IEC2bits.CCT5IE = false;
    IEC2bits.CCP5IE = false;
    IPC11bits.CCT5IP = priority;
}


void PWMB1_Write( float duty_cycle ){
    CCP5RB = (int) (duty_cycle * pwmb1_period);
}


void PWMB1_Start( void ){
    IFS2bits.CCT5IF = false;    // clear primary timer flag
    IFS2bits.CCP5IF = false;    // clear secondary timer flag
    IEC2bits.CCT5IE = true;     // enable primary interrupt
    IEC2bits.CCP5IE = true;     // enable secondary interrupt
    CCP5CON1Lbits.CCPON = true;
}


void PWMB1_Stop( void ){
    IEC2bits.CCP5IE = false;
    IEC2bits.CCT5IE = false;
    CCP5CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT5Interrupt ( void ){
    // drive GPIO High at period reset
    IFS2bits.CCT5IF = false;
    LATB = LATB | pwmb1_pin;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCP5Interrupt ( void ){
    // drive GPIO low at pulse width compare 
    IFS2bits.CCP5IF = false;
    LATB = LATB & (~pwmb1_pin);
}




// *****************************************************************************
// PWMB2 [SCCP6] is a 16bit general purpose PWM generator. PWM can be assigned
//      to any GPIO pin of PORT_B 
// *****************************************************************************

uint_fast16_t pwmb2_pin;
uint_fast16_t pwmb2_period;


void PWMB2_Init( uint_fast16_t pin, uint_fast16_t period_us, bool active_on_idle, bool active_on_sleep, uint_fast8_t priority ){
    pwmb2_pin = pin;                                                            // assign PWM to gpio pin

    PMD2bits.CCP6MD = 0;                                                        // enable SCCP6 peripheral

    CCP6CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP6CON1Lbits.CCPSIDL = !active_on_idle;
    CCP6CON1Lbits.CCPSLP = active_on_sleep;
    CCP6CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP6CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP6CON1Lbits.TMRSYNC = 0;                                                  // sync enabled
    CCP6CON1Lbits.T32 = 0;                                                      // SCCP6 in 16bit timer since PWM works only at 16bit
    CCP6CON1Lbits.CCSEL = 0;                                                    // PWM mode
    CCP6CON1Lbits.MOD = 0x5;                                                    // Dual Edge Compare mode, buffered (PWM)

    CCP6CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP6CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP6CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP6CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP6STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 

    CCP6PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP6TMRL = 0x0000;                                                          // TMR 0; 
    CCP6TMRH = 0x0000;                                                          // TMR 0; 
    CCP6RA = 0x0000;                                                            // CMP 0; 
    CCP6RB = 0x0000;                                                            // CMP 0;
    CCP6BUFL = 0x0000;                                                          // BUF 0; 
    CCP6BUFH = 0x0000;                                                          // BUF 0;

    uint_fast8_t clk = GetMasterClkFrequency();

    switch( clk ){
        case _8MHZ:
            pwmb2_period = (period_us * SCCP_1US_COUNT_8MHZ) - 1;
            break;
        case _20MHZ:
            pwmb2_period = (period_us * SCCP_1US_COUNT_20MHZ) - 1;
            break;
        case _50MHZ:
            pwmb2_period = (period_us * SCCP_1US_COUNT_50MHZ) - 1;
            break;
        case _100MHZ:
            pwmb2_period = (period_us * SCCP_1US_COUNT_100MHZ) - 1;
            break;
    }
    
    CCP6PRL = pwmb2_period;

    IEC2bits.CCT6IE = false;
    IEC2bits.CCP6IE = false;
    IPC11bits.CCT6IP = priority;
}


void PWMB2_Write( float duty_cycle ){
    CCP6RB = (int) (duty_cycle * pwmb2_period);
}


void PWMB2_Start( void ){
    IFS2bits.CCT6IF = false;    // clear primary timer flag
    IFS2bits.CCP6IF = false;    // clear secondary timer flag
    IEC2bits.CCT6IE = true;     // enable primary interrupt
    IEC2bits.CCP6IE = true;     // enable secondary interrupt
    CCP6CON1Lbits.CCPON = true;
}


void PWMB2_Stop( void ){
    IEC2bits.CCP6IE = false;
    IEC2bits.CCT6IE = false;
    CCP6CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT6Interrupt ( void ){
    // drive GPIO High at period reset
    IFS2bits.CCT6IF = false;
    LATB = LATB | pwmb2_pin;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCP6Interrupt ( void ){
    // drive GPIO low at pulse width compare 
    IFS2bits.CCP6IF = false;
    LATB = LATB & (~pwmb2_pin);
}




// *****************************************************************************
// PWMB3 [SCCP7] is a 16bit general purpose PWM generator. PWM can be assigned
//      to any GPIO pin of PORT_B 
// *****************************************************************************

uint_fast16_t pwmb3_pin;
uint_fast16_t pwmb3_period;


void PWMB3_Init( uint_fast16_t pin, uint_fast16_t period_us, bool active_on_idle, bool active_on_sleep, uint_fast8_t priority ){
    pwmb3_pin = pin;                                                            // assign PWM to gpio pin

    PMD2bits.CCP7MD = 0;                                                        // enable SCCP7 peripheral

    CCP7CON1Lbits.CCPON = false;                                                // make sure module is disabled at initialization
    CCP7CON1Lbits.CCPSIDL = !active_on_idle;
    CCP7CON1Lbits.CCPSLP = active_on_sleep;
    CCP7CON1Lbits.CLKSEL = 0x0;                                                 // FOSC/2 is the clock source
    CCP7CON1Lbits.TMRPS = 0;                                                    // 1:1 prescaler
    CCP7CON1Lbits.TMRSYNC = 0;                                                  // sync enabled
    CCP7CON1Lbits.T32 = 0;                                                      // SCCP7 in 16bit timer since PWM works only at 16bit
    CCP7CON1Lbits.CCSEL = 0;                                                    // PWM mode
    CCP7CON1Lbits.MOD = 0x5;                                                    // Dual Edge Compare mode, buffered (PWM)

    CCP7CON1H = 0x0000;                                                         // RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; OPS Each Time Base Period Match; SYNC None; OPSSRC Timer Interrupt Event;
    CCP7CON2L = 0x0000;                                                         // ASDGM disabled; SSDG disabled; ASDG 0; PWMRSEN disabled; 
    CCP7CON2H = 0x0000;                                                         // ICGSM Level-Sensitive mode; ICSEL IC1; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP7CON3H = 0x0000;                                                         // OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP7STATL = 0x0000;                                                         // ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; ICGARM disabled; TRCLR disabled; 

    CCP7PRH = 0x0000;                                                           // initially set secondary timer period to 0
    CCP7TMRL = 0x0000;                                                          // TMR 0; 
    CCP7TMRH = 0x0000;                                                          // TMR 0; 
    CCP7RA = 0x0000;                                                            // CMP 0; 
    CCP7RB = 0x0000;                                                            // CMP 0;
    CCP7BUFL = 0x0000;                                                          // BUF 0; 
    CCP7BUFH = 0x0000;                                                          // BUF 0;

    uint_fast8_t clk = GetMasterClkFrequency();

    switch( clk ){
        case _8MHZ:
            pwmb3_period = (period_us * SCCP_1US_COUNT_8MHZ) - 1;
            break;
        case _20MHZ:
            pwmb3_period = (period_us * SCCP_1US_COUNT_20MHZ) - 1;
            break;
        case _50MHZ:
            pwmb3_period = (period_us * SCCP_1US_COUNT_50MHZ) - 1;
            break;
        case _100MHZ:
            pwmb3_period = (period_us * SCCP_1US_COUNT_100MHZ) - 1;
            break;
    }
    
    CCP7PRL = pwmb3_period;

    IEC9bits.CCT7IE = false;
    IEC9bits.CCP7IE = false;
    IPC37bits.CCT7IP = priority;
}


void PWMB3_Write( float duty_cycle ){
    CCP7RB = (int) (duty_cycle * pwmb3_period);
}


void PWMB3_Start( void ){
    IFS9bits.CCT7IF = false;    // clear primary timer flag
    IFS9bits.CCP7IF = false;    // clear secondary timer flag
    IEC9bits.CCT7IE = true;     // enable primary interrupt
    IEC9bits.CCP7IE = true;     // enable secondary interrupt
    CCP7CON1Lbits.CCPON = true;
}


void PWMB3_Stop( void ){
    IEC9bits.CCP7IE = false;
    IEC9bits.CCT7IE = false;
    CCP7CON1Lbits.CCPON = false;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT7Interrupt ( void ){
    // drive GPIO High at period reset
    IFS9bits.CCT7IF = false;
    LATB = LATB | pwmb3_pin;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCP7Interrupt ( void ){
    // drive GPIO low at pulse width compare 
    IFS9bits.CCP7IF = false;
    LATB = LATB & (~pwmb3_pin);
}