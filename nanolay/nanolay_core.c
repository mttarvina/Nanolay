#include "nanolay_core.h"


// FSEC
#pragma config BWRP = OFF           //Boot Segment Write-Protect bit->Boot Segment may be written
#pragma config BSS = DISABLED       //Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSEN = OFF           //Boot Segment Control bit->No Boot Segment
#pragma config GWRP = OFF           //General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = DISABLED       //General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF           //Configuration Segment Write-Protect bit->Configuration Segment may be written
#pragma config CSS = DISABLED       //Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = OFF        //Alternate Interrupt Vector Table bit->Disabled AIVT

// FBSLIM
#pragma config BSLIM = 8191         //Boot Segment Flash Page Address Limit bits->8191

// FOSCSEL
#if MASTER_CLK_4MHZ
#pragma config FNOSC = FRCDIVN      //Oscillator Source Selection->Internal Fast RC (FRC) Oscillator with postscaler
#else
#pragma config FNOSC = FRC          //Oscillator Source Selection->FRC
#endif

#pragma config IESO = OFF           //Two-speed Oscillator Start-up Enable bit->Start up with user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE        //Primary Oscillator Mode Select bits->Primary Oscillator disabled

#if CLKOUTEN
#pragma config OSCIOFNC = OFF       //OSC2 Pin Function bit->OSC2 is clock output
#else
#pragma config OSCIOFNC = ON        //OSC2 Pin Function bit->OSC2 is GPIO
#endif

#pragma config FCKSM = CSECME       //Clock Switching Mode bits->Both Clock switching and Fail-safe Clock Monitor are enabled
#pragma config PLLKEN = ON          //PLL Lock Status Control->PLL lock signal will be used to disable PLL clock output if lock is lost
#pragma config XTCFG = G3           //XT Config->24-32 MHz crystals
#pragma config XTBST = ENABLE       //XT Boost->Boost the kick-start

// FWDT
#pragma config RWDTPS = PS1         //Run Mode Watchdog Timer Post Scaler select bits->1:1
#pragma config RCLKSEL = LPRC       //Watchdog Timer Clock Select bits->Always use LPRC
#pragma config WINDIS = OFF         //Watchdog Timer Window Enable bit->Watchdog Timer in Window mode
#pragma config WDTWIN = WIN25       //Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config SWDTPS = PS1         //Sleep Mode Watchdog Timer Post Scaler select bits->1:1
#pragma config FWDTEN = ON_SW       //Watchdog Timer Enable bit->WDT controlled via SW, use WDTCON.ON bit

// FPOR
#pragma config BISTDIS = DISABLED   //Memory BIST Feature Disable->mBIST on reset feature disabled

// FICD
#pragma config ICS = PGD2           //ICD Communication Channel Select bits->Communicate on PGC2 and PGD2
#pragma config JTAGEN = OFF         //JTAG Enable bit->JTAG is disabled
#pragma config NOBTSWP = DISABLED   //BOOTSWP instruction disable bit->BOOTSWP instruction is disabled

// FDMTIVTL
#pragma config DMTIVTL = 0          //Dead Man Timer Interval low word->0

// FDMTIVTH
#pragma config DMTIVTH = 0          //Dead Man Timer Interval high word->0

// FDMTCNTL
#pragma config DMTCNTL = 0          //Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF)->0

// FDMTCNTH
#pragma config DMTCNTH = 0          //Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF)->0

// FDMT
#pragma config DMTDIS = OFF         //Dead Man Timer Disable bit->Dead Man Timer is Disabled and can be enabled by software

// FDEVOPT
#pragma config ALTI2C1 = OFF        //Alternate I2C1 Pin bit->I2C1 mapped to SDA1/SCL1 pins
#pragma config ALTI2C2 = OFF        //Alternate I2C2 Pin bit->I2C2 mapped to SDA2/SCL2 pins
#pragma config ALTI2C3 = OFF        //Alternate I2C3 Pin bit->I2C3 mapped to SDA3/SCL3 pins
#pragma config SMBEN = SMBUS        //SM Bus Enable->SMBus input threshold is enabled
#pragma config SPI2PIN = PPS        //SPI2 Pin Select bit->SPI2 uses I/O remap (PPS) pins

// FALTREG
#pragma config CTXT1 = OFF          //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits->Not Assigned
#pragma config CTXT2 = OFF          //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits->Not Assigned
#pragma config CTXT3 = OFF          //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits->Not Assigned
#pragma config CTXT4 = OFF          //Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits->Not Assigned

// FBTSEQ
#pragma config BSEQ = 4095          //Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active->4095
#pragma config IBSEQ = 4095         //The one's complement of BSEQ; must be calculated by the user and written during device programming.->4095

// FBOOT
#pragma config BTMODE = SINGLE      //Device Boot Mode Configuration->Device is in Single Boot (legacy) mode


void SysInit( void ) {
    GPIOInit();
    ClockInit();
    DisableAllPeripherals();

    if ( CLKOUTEN ) {
        DigitalSetPin(PB1, OUTPUT);
    }
}


void ClockInit( void ){
    if ( MASTER_CLK_4MHZ ){ 
        CLKDIV = 0x3101;            // FRCDIV FRC/2; PLLPRE 1; DOZE 1:8; DOZEN disabled; ROI disabled;
        PLLFBD = 0x96;              // PLLFBDIV 150;
        PLLDIV = 0x141;             // POST1DIV 1:4; VCODIV FVCO/3; POST2DIV 1:1; 
        ACLKCON1 = 0x101;           // APLLEN disabled; FRCSEL FRC; APLLPRE 1:1;
        APLLFBD1 = 0x96;            // APLLFBDIV 150;  
        APLLDIV1 = 0x141;           // APOST1DIV 1:4; APOST2DIV 1:1; AVCODIV FVCO/3;
    }    
    
    OSCTUN = 0x00;                  // TUN Center frequency;  
    REFOCONL = 0x00;                // ROEN disabled; ROSWEN disabled; ROSLP disabled; ROSEL FOSC; ROOUT disabled; ROSIDL disabled;
    REFOCONH = 0x00;                // RODIV 0; 
    REFOTRIMH = 0x00;               // ROTRIM 0; 
    RPCON = 0x00;                   // IOLOCK disabled;
    PMDCON = 0x00;                  // PMDLOCK disabled;

    if ( MASTER_CLK_4MHZ ){
        // CF no clock failure; NOSC FRCDIV; CLKLOCK unlocked; OSWEN Switch is Complete; 
        __builtin_write_OSCCONH((uint8_t) (0x07));
        __builtin_write_OSCCONL((uint8_t) (0x00));
    }
    
    WDTCONLbits.ON = 0;             // Disable Watchdog Timer 
}


void GPIOInit( void ){

    // Setting the Output Latch SFR(s)
    LATA = 0x0000;
    LATB = 0x0000;

    // Setting the GPIO Direction SFR(s)
    TRISA = 0x001F;
    TRISB = 0xFFFF;

    // Setting the Weak Pull Up and Weak Pull Down SFR(s)
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPUA = 0x0000;
    CNPUB = 0x0000;

    // Setting the Open Drain SFR(s)
    ODCA = 0x0000;
    ODCB = 0x0000;

    // Setting the Analog/Digital Configuration SFR(s)
    ANSELA = 0x001F;
    ANSELB = 0x0385;
}


void DisableAllPeripherals( void ) {
    PMD1 = 0xFFFF;
    PMD2 = 0xFFFF;
    PMD3 = 0xFFFF;
    PMD4 = 0xFFFF;
    PMD6 = 0xFFFF;
    PMD7 = 0xFFFF;
    PMD8 = 0xFFFF;
}


void DigitalSetPin( uint8_t pin, bool dir ) {
    switch( pin ){
        case PA0:
            _TRISA0 = dir;
            _ANSELA0 = dir;
            return;
        case PA1:
            _TRISA1 = dir;
            _ANSELA1 = dir;
            return;
        case PA2:
            _TRISA2 = dir;
            _ANSELA2 = dir;
            return;
        case PA3:
            _TRISA3 = dir;
            _ANSELA3 = dir;
            return;
        case PA4:
            _TRISA4 = dir;
            _ANSELA4 = dir;
            return;
        case PB0:
            _TRISB0 = dir;
            _ANSELB0 = dir;
            return;
        case PB1:
            _TRISB1 = dir;
            _ANSELB1 = dir;
            return;
        case PB2:
            _TRISB2 = dir;
            _ANSELB2 = dir;
            return;
        case PB3:
            _TRISB3 = dir;
            _ANSELB3 = dir;
            return;
        case PB4:
            _TRISB4 = dir;
            _ANSELB4 = dir;
            return;
        case PB5:
            _TRISB5 = dir;
            //_ANSELB5 = dir;
            return;
        case PB6:
            _TRISB6 = dir;
            //_ANSELB6 = dir;
            return;
        case PB7:
            _TRISB7 = dir;
            _ANSELB7 = dir;
            return;
        case PB8:
            _TRISB8 = dir;
            _ANSELB8 = dir;
            return;
        case PB9:
            _TRISB9 = dir;
            _ANSELB9 = dir;
            return;
        case PB10:
            _TRISB10 = dir;
            //_ANSELB10 = dir;
            return;
        case PB11:
            _TRISB11 = dir;
            //_ANSELB11 = dir;
            return;
        case PB12:
            _TRISB12 = dir;
            //_ANSELB12 = dir;
            return;
        case PB13:
            _TRISB13 = dir;
            //_ANSELB13 = dir;
            return;
        case PB14:
            _TRISB14 = dir;
            //_ANSELB14 = dir;
            return;
        case PB15:
            _TRISB15 = dir;
            //_ANSELB15 = dir;
            return;
    }
}


void DigitalDrvPin( uint8_t pin, bool state ) {
    switch( pin ){
        case PA0:
            _LATA0 = state;
            return;
        case PA1:
            _LATA1 = state;
            return;
        case PA2:
            _LATA2 = state;
            return;
        case PA3:
            _LATA3 = state;
            return;
        case PA4:
            _LATA4 = state;
            return;
        case PB0:
            _LATB0 = state;
            return;
        case PB1:
            _LATB1 = state;
            return;
        case PB2:
            _LATB2 = state;
            return;
        case PB3:
            _LATB3 = state;
            return;
        case PB4:
            _LATB4 = state;
            return;
        case PB5:
            _LATB5 = state;
            return;
        case PB6:
            _LATB6 = state;
            return;
        case PB7:
            _LATB7 = state;
            return;
        case PB8:
            _LATB8 = state;
            return;
        case PB9:
            _LATB9 = state;
            return;
        case PB10:
            _LATB10 = state;
            return;
        case PB11:
            _LATB11 = state;
            return;
        case PB12:
            _LATB12 = state;
            return;
        case PB13:
            _LATB13 = state;
            return;
        case PB14:
            _LATB14 = state;
            return;
        case PB15:
            _LATB15 = state;
            return;
    }
}


bool DigitalReadPin( uint8_t pin ) {
    switch( pin ){
        case PA0:
            return _RA0;
        case PA1:
            return _RA1;
        case PA2:
            return _RA2;
        case PA3:
            return _RA3;
        case PA4:
            return _RA4;
        case PB0:
            return _RB0;
        case PB1:
            return _RB1;
        case PB2:
            return _RB2;
        case PB3:
            return _RB3;
        case PB4:
            return _RB4;
        case PB5:
            return _RB5;
        case PB6:
            return _RB6;
        case PB7:
            return _RB7;
        case PB8:
            return _RB8;
        case PB9:
            return _RB9;
        case PB10:
            return _RB10;
        case PB11:
            return _RB11;
        case PB12:
            return _RB12;
        case PB13:
            return _RB13;
        case PB14:
            return _RB14;
        case PB15:
            return _RB15;
    }
}