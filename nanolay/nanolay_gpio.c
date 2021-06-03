/* ************************************************************************** */
// Nanolay - GPIO Library Source File
//
// Description:     Custom dsPIC33CK library for GPIO functions. Should be
//                  included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    12.Apr.21
/* ************************************************************************** */

#include "nanolay_gpio.h"


void (*PortAPin0_InterruptHandler)(void) = NULL;
void (*PortAPin1_InterruptHandler)(void) = NULL;
void (*PortAPin2_InterruptHandler)(void) = NULL;
void (*PortAPin3_InterruptHandler)(void) = NULL;
void (*PortAPin4_InterruptHandler)(void) = NULL;

void (*PortBPin0_InterruptHandler)(void) = NULL;
void (*PortBPin1_InterruptHandler)(void) = NULL;
void (*PortBPin2_InterruptHandler)(void) = NULL;
void (*PortBPin3_InterruptHandler)(void) = NULL;
void (*PortBPin4_InterruptHandler)(void) = NULL;
void (*PortBPin5_InterruptHandler)(void) = NULL;
void (*PortBPin6_InterruptHandler)(void) = NULL;
void (*PortBPin7_InterruptHandler)(void) = NULL;
void (*PortBPin8_InterruptHandler)(void) = NULL;
void (*PortBPin9_InterruptHandler)(void) = NULL;
void (*PortBPin10_InterruptHandler)(void) = NULL;
void (*PortBPin11_InterruptHandler)(void) = NULL;
void (*PortBPin12_InterruptHandler)(void) = NULL;
void (*PortBPin13_InterruptHandler)(void) = NULL;
void (*PortBPin14_InterruptHandler)(void) = NULL;
void (*PortBPin15_InterruptHandler)(void) = NULL;


void GPIO_Init( void ){

    // Setting the Output Latch SFR(s)
    LATA = 0x0000;
    LATB = 0x0000;

    // Setting the GPIO Direction SFR(s)
    TRISA = 0x001F;                                                             // default to INPUT
    TRISB = 0xFFFF;                                                             // default to INPUT
    // TRISA = 0x0000;                                                              // default to OUTPUT
    // TRISB = 0x0000;                                                              // default to OUTPUT

    // Setting the Analog/Digital Configuration SFR(s)
    ANSELA = 0x001F;                                                            // default as Analog INPUT
    ANSELB = 0x0385;                                                            // default as Analog INPUT
    // ANSELA = 0x0000;                                                             // default as Digital INPUT
    // ANSELB = 0x0000;                                                             // default as Digital INPUT

    // Setting the Weak Pull Up and Weak Pull Down SFR(s)
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPUA = 0x0000;
    CNPUB = 0x0000;

    // Setting the Open Drain SFR(s)
    ODCA = 0x0000;
    ODCB = 0x0000;
}


void GPIO_SetPortAPin( uint_fast16_t pin, bool dir, bool opendrain ) {
    if ( dir == INPUT ){
        TRISA = TRISA | pin;
        ANSELA = ANSELA | pin;
    }
    else {
        TRISA = TRISA & (~pin);
        ANSELA = ANSELA & (~pin);
    }

    if ( opendrain ){
        ODCA = ODCA | pin;
    }
}



void GPIO_SetPortBPin( uint_fast16_t pin, bool dir, bool opendrain ) {
    if ( dir == INPUT ){
        TRISB = TRISB | pin;
        ANSELB = ANSELB | pin;
        return;
    }
    else {
        TRISB = TRISB & (~pin);
        ANSELB = ANSELB & (~pin);
        return;
    }

    if ( opendrain ){
        ODCB = ODCB | pin;
    }
}


void GPIO_DrivePortAPin( uint_fast16_t pin, bool state ) {
    if ( state == HIGH ){
        LATA = LATA | pin;
    }
    else {
        LATA = LATA & (~pin);
    }
}


void GPIO_DrivePortBPin( uint_fast16_t pin, bool state ) {
    if ( state == HIGH ){
        LATB = LATB | pin;
    }
    else {
        LATB = LATB & (~pin);
    }
}


void GPIO_SetPortAInterrupt( uint_fast16_t pin, bool edgetype, void (* InterruptHandler)(void) ){
    IFS0bits.CNAIF = 0;                                                         // Clear CNAI interrupt flag
    IEC0bits.CNAIE = 0;                                                         // Disable CNAI interrupt

    CNCONAbits.ON = true;                                                       // Enable Change interrupt on Port A
    CNCONAbits.CNSTYLE = true;                                                  // Edge detect interrupt

    if ( edgetype == RISING ){                                                  // CNEN1x = 0, CNEN0x = 1
        CNEN1A = CNEN1A & (~pin);
        CNEN0A = CNEN0A | pin;
    }
    else {                                                                      // CNEN1x = 1, CNEN0x = 0
        CNEN1A = CNEN1A | pin;
        CNEN0A = CNEN0A & (~pin);
    }

    switch ( pin ){
        case PIN0:
            PortAPin0_InterruptHandler = InterruptHandler;
            break;
        case PIN1:
            PortAPin1_InterruptHandler = InterruptHandler;
            break;
        case PIN2:
            PortAPin2_InterruptHandler = InterruptHandler;
            break;
        case PIN3:
            PortAPin3_InterruptHandler = InterruptHandler;
            break;
        case PIN4:
            PortAPin4_InterruptHandler = InterruptHandler;
            break;
    }

    CNFA = 0x0000;                                                              // clear all pin specific flags
    IFS0bits.CNAIF = 0;                                                         // Clear CNAI interrupt flag
    IEC0bits.CNAIE = 1;                                                         // Enable CNAI interrupt
}


void GPIO_SetPortBInterrupt( uint_fast16_t pin, bool edgetype, void (* InterruptHandler)(void) ){
    IFS0bits.CNBIF = 0;                                                         // Clear CNBI interrupt flag
    IEC0bits.CNBIE = 0;                                                         // Disable CNBI interrupt

    CNCONBbits.ON = true;                                                       // Enable Change interrupt on Port B
    CNCONBbits.CNSTYLE = true;                                                  // Edge detect interrupt

    if ( edgetype == RISING ){                                                  // CNEN1x = 0, CNEN0x = 1
        CNEN1B = CNEN1B & (~pin);
        CNEN0B = CNEN0B | pin;
    }
    else {                                                                      // CNEN1x = 1, CNEN0x = 0
        CNEN1B = CNEN1B | pin;
        CNEN0B = CNEN0B & (~pin);
    }

    switch ( pin ){
        case PIN0:
            PortBPin0_InterruptHandler = InterruptHandler;
            break;
        case PIN1:
            PortBPin1_InterruptHandler = InterruptHandler;
            break;
        case PIN2:
            PortBPin2_InterruptHandler = InterruptHandler;
            break;
        case PIN3:
            PortBPin3_InterruptHandler = InterruptHandler;
            break;
        case PIN4:
            PortBPin4_InterruptHandler = InterruptHandler;
            break;
        case PIN5:
            PortBPin5_InterruptHandler = InterruptHandler;
            break;
        case PIN6:
            PortBPin6_InterruptHandler = InterruptHandler;
            break;
        case PIN7:
            PortBPin7_InterruptHandler = InterruptHandler;
            break;
        case PIN8:
            PortBPin8_InterruptHandler = InterruptHandler;
            break;
        case PIN9:
            PortBPin9_InterruptHandler = InterruptHandler;
            break;
        case PIN10:
            PortBPin10_InterruptHandler = InterruptHandler;
            break;
        case PIN11:
            PortBPin11_InterruptHandler = InterruptHandler;
            break;
        case PIN12:
            PortBPin12_InterruptHandler = InterruptHandler;
            break;
        case PIN13:
            PortBPin13_InterruptHandler = InterruptHandler;
            break;
        case PIN14:
            PortBPin14_InterruptHandler = InterruptHandler;
            break;
        case PIN15:
            PortBPin15_InterruptHandler = InterruptHandler;
            break;
    }

    CNFB = 0x0000;                                                              // clear all pin specific flags
    IFS0bits.CNBIF = 0;                                                         // Clear CNAI interrupt flag
    IEC0bits.CNBIE = 1;                                                         // Enable CNBI interrupt
}


/* Interrupt service routine for the CNAI interrupt. */
void __attribute__ (( interrupt, no_auto_psv )) _CNAInterrupt ( void ){
    if ( CNFAbits.CNFA0 ) {
        PortAPin0_InterruptHandler();
        CNFAbits.CNFA0 = 0;
    }
    if ( CNFAbits.CNFA1 ) {
        PortAPin1_InterruptHandler();
        CNFAbits.CNFA1 = 0;
    }
    if ( CNFAbits.CNFA2 ) {
        PortAPin2_InterruptHandler();
        CNFAbits.CNFA2 = 0;
    }
    if ( CNFAbits.CNFA3 ) {
        PortAPin3_InterruptHandler();
        CNFAbits.CNFA3 = 0;
    }
    if ( CNFAbits.CNFA4 ) {
        PortAPin4_InterruptHandler();
        CNFAbits.CNFA4 = 0;
    }
    IFS0bits.CNAIF = 0;
}


/* Interrupt service routine for the CNBI interrupt. */
void __attribute__ (( interrupt, no_auto_psv )) _CNBInterrupt ( void ){
    if ( CNFBbits.CNFB0 ) {
        PortBPin0_InterruptHandler();
        CNFBbits.CNFB0 = 0;
    }
    if ( CNFBbits.CNFB1 ) {
        PortBPin1_InterruptHandler();
        CNFBbits.CNFB1 = 0;
    }
    if ( CNFBbits.CNFB2 ) {
        PortBPin2_InterruptHandler();
        CNFBbits.CNFB2 = 0;
    }
    if ( CNFBbits.CNFB3 ) {
        PortBPin3_InterruptHandler();
        CNFBbits.CNFB3 = 0;
    }
    if ( CNFBbits.CNFB4 ) {
        PortBPin4_InterruptHandler();
        CNFBbits.CNFB4 = 0;
    }
    if ( CNFBbits.CNFB5 ) {
        PortBPin5_InterruptHandler();
        CNFBbits.CNFB5 = 0;
    }
    if ( CNFBbits.CNFB6 ) {
        PortBPin6_InterruptHandler();
        CNFBbits.CNFB6 = 0;
    }
    if ( CNFBbits.CNFB7 ) {
        PortBPin7_InterruptHandler();
        CNFBbits.CNFB7 = 0;
    }
    if ( CNFBbits.CNFB8 ) {
        PortBPin8_InterruptHandler();
        CNFBbits.CNFB8 = 0;
    }
    if ( CNFBbits.CNFB9 ) {
        PortBPin9_InterruptHandler();
        CNFBbits.CNFB9 = 0;
    }
    if ( CNFBbits.CNFB10 ) {
        PortBPin10_InterruptHandler();
        CNFBbits.CNFB10 = 0;
    }
    if ( CNFBbits.CNFB11 ) {
        PortBPin11_InterruptHandler();
        CNFBbits.CNFB11 = 0;
    }
    if ( CNFBbits.CNFB12 ) {
        PortBPin12_InterruptHandler();
        CNFBbits.CNFB12 = 0;
    }
    if ( CNFBbits.CNFB13 ) {
        PortBPin13_InterruptHandler();
        CNFBbits.CNFB13 = 0;
    }
    if ( CNFBbits.CNFB14 ) {
        PortBPin14_InterruptHandler();
        CNFBbits.CNFB14 = 0;
    }
    if ( CNFBbits.CNFB15 ) {
        PortBPin15_InterruptHandler();
        CNFBbits.CNFB15 = 0;
    }
    IFS0bits.CNBIF = 0;
}