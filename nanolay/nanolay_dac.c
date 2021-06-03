/* ************************************************************************** */
// Nanolay - DAC Library Source File
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

#include "nanolay_dac.h"


void DAC_Init( void ){           
    uint_fast16_t clk = GetMasterClkFrequency();

    if (clk > _8MHZ) {
		PMD7bits.CMP1MD = 0;													// enable CMP1 module
        
        DACCTRL1Lbits.DACON = 0;												// Turn off the CMP module before the initialization
	
	    // Comparator Register settings
	    DACCTRL1L = 0x00; 														// CLKDIV 1:1; DACON disabled; DACSIDL disabled; FCLKDIV 1:1; CLKSEL AFVCO/2 - Auxiliary VCO Clock; 
	    DACCTRL2L = 0x55; 														// TMODTIME 85; 
	    DACCTRL2H = 0x8A; 														// SSTIME 138; 
	    DAC1CONH = 0x00; 														// TMCB 0; 
	    DAC1CONL = 0x8200; 														// CMPPOL Non Inverted; HYSPOL Rising Edge; HYSSEL None; DACEN enabled; FLTREN disabled; CBE disabled; IRQM Interrupts are disabled; INSEL CMP1A; DACOEN enabled; 

	    //Slope Settings
	    SLP1CONH = 0x00; 														// HME disabled; PSE Negative; SLOPEN disabled; TWME disabled; 
	    SLP1CONL = 0x00; 														// HCFSEL None; SLPSTRT None; SLPSTOPB None; SLPSTOPA None; 
	    SLP1DAT = 0x00; 														// SLPDAT 0; 
	    DAC1DATL = 0x00; 														// DACDATL 0; 
	    DAC1DATH = 0x00; 														// DACDATH 0; 
    
		DAC1CONLbits.DACOEN = 1;    											// enable DACOUT pin
        DACCTRL1Lbits.DACON = 1;												// turn on the CMP module
    }
}


void DAC_WriteDCValue( uint_fast16_t val ){
    if ( val < 0x0CD ){
		DAC1DATH = 0x0CD;
		return;
	}
	else if (val > 0xF32 ){
		DAC1DATH = 0xF32;
		return;
	}
	else {
		DAC1DATH = val;
		return;
	}
}


