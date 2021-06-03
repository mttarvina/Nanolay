/* ************************************************************************** */
// Nanolay - ADC Library Source File
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

#include "nanolay_adc.h"

bool adc_already_initialized = false;
bool adc_core0_is_on = false;
bool adc_core1_is_on = false;
bool adc_shrcore_is_on = false;


void ADC_Init( void ){
    PMD1bits.ADC1MD = 0;                                                        // enable ADC peripheral
    ADCON1Lbits.ADON = false;                                                   // Turn OFF the ADC Module

    ADCON3Hbits.CLKSEL = 0x1;                                                   // FOSC is the clock source
    ADCON3Hbits.CLKDIV = 0x0;                                                   // 1:1 clock divider

    ADCON2Lbits.REFCIE = false;                                                 // Common interrupt is disabled for the band gap ready event
    ADCON2Lbits.REFERCIE = false;                                               // Common interrupt is disabled for the band gap and reference voltage error event

    ADCON3Lbits.SWLCTRG = false;                                                // No software, level-sensitive common triggers are generated
    ADCON3Lbits.SWCTRG = false;                                                 // No common software trigger is generated, for now
    ADCON3Lbits.CNVRTCH = false;                                                // No software trigger is generated for any chaneel of shared core

    // shared core settings
    ADCON3Lbits.SHRSAMP = true;                                                 // Shared ADC core samples an analog input specified by the CNVCHSEL[5:0] bits
    ADCON3Lbits.CNVCHSEL= 0x0;                                                  // No channel selected for now
    ADCON2Lbits.EIEN = false;                                                   // disable early interrupts, no need to set ADCON2Lbits.SHREISEL
    ADCON2Lbits.SHRADCS = 0x0;                                                  // FOSC/2 is the clock source for Shared Core
    ADCON2Hbits.SHRSAMC = 0x0;                                                  // 2 x (1/(FOSC/2)) sample time for Shared Core
    ADCON1Hbits.FORM = false;                                                   // Use integer format for output data
    ADCON1Hbits.SHRRES = 0x3;                                                   // 12 bit resolution

    // core0 settings
    ADCON4Lbits.SAMC0EN = false;                                                // convert immediately after trigger, Core0
    ADCON4Hbits.C0CHS = 0x0;                                                    // AN0 is the dedicated input pin for Core0
    ADCORE0Hbits.ADCS = 0x0;                                                    // FOSC/2 is the clock source for Core0
    ADCORE0Lbits.SAMC = 0x0;                                                    // 2 x (1/(FOSC/2)) sample time for Core0
    ADCORE0Hbits.RES = 0x3;                                                     // 12 bit resolution

    // core1 settings
    ADCON4Lbits.SAMC1EN = false;                                                // convert immediately after trigger, Core1
    ADCON4Hbits.C1CHS = 0x0;                                                    // AN1 is the dedicated input pin for Core1
    ADCORE1Hbits.ADCS = 0x0;                                                    // FOSC/2 is the clock source for Core1
    ADCORE1Lbits.SAMC = 0x0;                                                    // 2 x (1/(FOSC/2)) sample time for Core1
    ADCORE1Hbits.RES = 0x3;                                                     // 12 bit resolution

    ADLVLTRGL = 0x0000;                                                         // Input trigger is edge-sensitive
    ADLVLTRGH = 0x0000;                                                         // Input trigger is edge-sensitive
    ADEIEL = 0x0000;                                                            // Disable early interrupt for any channel
    ADEIEH = 0x0000;                                                            // Disable early interrupt for any channel
    ADMOD0L = 0x0000;                                                           // All channels are single-ended and output data unsigned
    ADMOD0H= 0x0000;                                                            // All channels are single-ended and output data unsigned
    ADIEL = 0x0000;                                                             // Disable all common and individual interrupts
    ADIEH = 0x0000;                                                             // Disable all common and individual interrupts

    ADTRIG0L = 0x101;                                                           // TRGSRC0 Common Software Trigger; TRGSRC1 Common Software Trigger; 
    ADTRIG0H = 0x101;                                                           // TRGSRC3 Common Software Trigger; TRGSRC2 Common Software Trigger; 
    ADTRIG1L = 0x101;                                                           // TRGSRC4 Common Software Trigger; TRGSRC5 Common Software Trigger;     
    ADTRIG1H = 0x101;                                                           // TRGSRC6 Common Software Trigger; TRGSRC7 Common Software Trigger; 
    ADTRIG2L = 0x101;                                                           // TRGSRC8 Common Software Trigger; TRGSRC9 Common Software Trigger; 
    ADTRIG2H = 0x101;                                                           // TRGSRC11 Common Software Trigger; TRGSRC10 Common Software Trigger;
    ADTRIG6L = 0x00;                                                            // TRGSRC24 None; TRGSRC25 None; 

    ADCON5Hbits.WARMTIME = 0xF;                                                 // Warmup time after ADC ON is 32768 clck cycles

    ADCON1Lbits.ADON = true;                                                    // Turn ON the ADC Module
    adc_already_initialized = true;
}


void ADC_Core0En( void ){
    ADCON5Lbits.C0PWR = true;                                                   // turn on core0 power
    while( !ADCON5Lbits.C0RDY );                                                // wait for it to stabilize
    ADCON3Hbits.C0EN = true;                                                    // enable core0 operation
    adc_core0_is_on = true;
}


void ADC_Core1En( void ){
    ADCON5Lbits.C1PWR = true;                                                   // turn on core1 power
    while( !ADCON5Lbits.C1RDY );                                                // wait for it to stabilize
    ADCON3Hbits.C1EN = true;                                                    // enable core1 operation
    adc_core1_is_on = true;
}


void ADC_ShrCoreEn( void ){
    ADCON5Lbits.SHRPWR = true;                                                  // turn on shared core power                                                      
    while( !ADCON5Lbits.SHRRDY );                                               // wait for it to stabilize
    ADCON3Hbits.SHREN = true;                                                   // enable shared core operation
    adc_shrcore_is_on = true;
}


void Analog_SetPin( uint_fast8_t pin ){
    switch (pin) {
        case AN0:
            _TRISA0 = true;                                                     // set this pin as input
            _ANSELA0 = true;                                                    // set this pin as input   
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_core0_is_on )             ADC_Core0En();
            return;

        case AN1:
            _TRISB2 = true;                                                     // set this pin as input
            _ANSELB2 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_core1_is_on )             ADC_Core1En();
            return;

        case AN2:
            _TRISB7 = true;                                                     // set this pin as input
            _ANSELB7 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN3:
            if (!DAC1CONLbits.DACOEN) {
                _TRISA3 = true;                                                 // set this pin as input
                _ANSELA3 = true;                                                // set this pin as input
                if ( !adc_already_initialized )     ADC_Init();
                if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            }
            return;

        case AN4:
            _TRISA4 = true;                                                     // set this pin as input
            _ANSELA4 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN5:
            _TRISB0 = true;                                                     // set this pin as input
            _ANSELB0 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN6:
            _TRISB1 = true;                                                     // set this pin as input
            _ANSELB1 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN7:
            _TRISB2 = true;                                                     // set this pin as input
            _ANSELB2 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN8:
            _TRISB3 = true;                                                     // set this pin as input
            _ANSELB3 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN9:
            _TRISA2 = true;                                                     // set this pin as input
            _ANSELA2 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN10:
            _TRISB8 = true;                                                     // set this pin as input
            _ANSELB8 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        case AN11:
            _TRISB9 = true;                                                     // set this pin as input
            _ANSELB9 = true;                                                    // set this pin as input
            if ( !adc_already_initialized )     ADC_Init();
            if ( !adc_shrcore_is_on )           ADC_ShrCoreEn();
            return;

        default:
            return;
    }
}


uint_fast16_t ADC_ReadAN0( void ){
    ADCON3Lbits.SWCTRG = true;
    while(!ADSTATLbits.AN0RDY);
    return ADCBUF0;
}


uint_fast16_t ADC_ReadAN1( void ){
    ADCON3Lbits.SWCTRG = true;
    while(!ADSTATLbits.AN1RDY);
    return ADCBUF1;
}


uint_fast16_t ADC_ReadANx( uint_fast8_t pin ){
    switch( pin ){
        case AN2:
            ADCON3Lbits.CNVCHSEL = 0x2;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN2RDY );
            return ADCBUF2;
        case AN3:
            if ( !DAC1CONLbits.DACOEN ){
                ADCON3Lbits.CNVCHSEL = 0x3;
                ADCON3Lbits.CNVRTCH = true;
                while( !ADSTATLbits.AN3RDY );
                return ADCBUF3;
            }
        case AN4:
            ADCON3Lbits.CNVCHSEL = 0x4;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN4RDY );
            return ADCBUF4;
        case AN5:
            ADCON3Lbits.CNVCHSEL = 0x5;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN5RDY );
            return ADCBUF5;
        case AN6:
            ADCON3Lbits.CNVCHSEL = 0x6;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN6RDY );
            return ADCBUF6;
        case AN7:
            ADCON3Lbits.CNVCHSEL = 0x7;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN7RDY );
            return ADCBUF7;
        case AN8:
            ADCON3Lbits.CNVCHSEL = 0x8;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN8RDY );
            return ADCBUF8;
        case AN9:
            ADCON3Lbits.CNVCHSEL = 0x9;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN9RDY );
            return ADCBUF9;
        case AN10:
            ADCON3Lbits.CNVCHSEL = 0xA;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN10RDY );
            return ADCBUF10;
        case AN11:
            ADCON3Lbits.CNVCHSEL = 0xB;
            ADCON3Lbits.CNVRTCH = true;
            while( !ADSTATLbits.AN11RDY );
            return ADCBUF11;
    }
    return 0;
}

