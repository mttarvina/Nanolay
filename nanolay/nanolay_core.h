/* ************************************************************************** */
// Nanolay - Core Library Header File
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    26.Feb.2021
/* ************************************************************************** */


#ifndef _NANOLAY_CORE_H
#define	_NANOLAY_CORE_H

#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


// User needs to set one of these macros to change Fosc
#define MASTER_CLK_4MHZ         true
#define MASTER_CLK_8MHZ         false
#define MASTER_CLK_16MHZ        false
#define MASTER_CLK_20MHZ        false
#define MASTER_CLK_25MHZ        false
#define MASTER_CLK_30MHZ        false
#define MASTER_CLK_40MHZ        false
#define MASTER_CLK_50MHZ        false
#define MASTER_CLK_100MHZ       false

// Enables Fosc/2 output at RB1 pin
#define CLKOUTEN                false            

// GPIO Constants
#define INPUT                   true
#define OUTPUT                  false
#define LOW                     false
#define HIGH                    true
#define PORT_A                  0x1
#define PORT_B                  0x2
#define PA0                     0x01
#define PA1                     0x02
#define PA2                     0x03
#define PA3                     0x04
#define PA4                     0x05
#define PB0                     0x06
#define PB1                     0x07
#define PB2                     0x08
#define PB3                     0x09
#define PB4                     0x0A
#define PB5                     0x0B
#define PB6                     0x0C
#define PB7                     0x0D
#define PB8                     0x0E
#define PB9                     0x0F
#define PB10                    0x10
#define PB11                    0x11
#define PB12                    0x12
#define PB13                    0x13
#define PB14                    0x14
#define PB15                    0x15

// Timer1 constants
#define TMR1_PRIORITY           0x04
#define TMR1_PERIOD_4MHZ        0x07CF  // 1ms
#define TMR1_PERIOD_8MHZ        0x0F9F  // 1ms
#define TMR1_PERIOD_16MHZ       0x1F3F  // 1ms
#define TMR1_PERIOD_20MHZ       0x270F  // 1ms
#define TMR1_PERIOD_25MHZ       0x30D3  // 1ms
#define TMR1_PERIOD_30MHZ       0x3A97  // 1ms
#define TMR1_PERIOD_40MHZ       0x4E1F  // 1ms
#define TMR1_PERIOD_50MHZ       0x61A7  // 1ms
#define TMR1_PERIOD_100MHZ      0xC34F  // 1ms


// custom struct definition for Timer1
typedef struct _TMR1_OBJ_STRUCT{
    volatile uint16_t   counter;
    volatile uint16_t   countmax;
} TMR1_OBJ;


// *****************************************************************************
// @desc:       Initialize system clock, peripherals, and default GPIO states.
//                  Required to call this function at the start of main program
// @args:       None
// @returns:    None
// *****************************************************************************
void SysInit( void );


// *****************************************************************************
// @desc:       Initialize system CLK registers based on MASTER_CLK_xMHZ macro
//                  This is called by SysInit()
// @args:       None
// @returns:    None
// *****************************************************************************
void ClockInit( void );


// *****************************************************************************
// @desc:       Sets the default GPIO setting for all pins at startup
//                  This is called by SysInit()
// @args:       None
// @returns:    None
// *****************************************************************************
void GPIOInit( void );


// *****************************************************************************
// @desc:       Disables all peripherals. User must manually enable a peripheral
//                  in the main program before using peripheral specific
//                  function APIs. This is called by SysInit()
// @args:       None
// @returns:    None
// *****************************************************************************
void DisableAllPeripherals( void );


// *****************************************************************************
// @desc:       Set a pin as Digital OUTPUT or Digital INPUT. This is similar
//                  to Arduino's PinMode() function
// @args:       pin [uint8_t]: PAx or PBx defined in GPIO Macro Definitions
//              dir [bool]: OUTPUT or INPUT
// @returns:    None
// *****************************************************************************
void DigitalSetPin( uint8_t pin, bool dir );


// *****************************************************************************
// @desc:       Set a pin to output HIGH or LOW, provided that it is set as
//                  OUTPUT prior. This is similar to Arduino's DigitalWrite()
//                  function
// @args:       pin [uint8_t]: PAx or PBx defined in GPIO Macro Definitions
//              state [bool]: HIGH or LOW
// @returns:    None
// *****************************************************************************
void DigitalDrvPin( uint8_t pin, bool state );


// *****************************************************************************
// @desc:       Read from a digital pin, provided that it is set as INPUT prior.
//                  This is similar to Arduino's DigitalRead() function
// @args:       pin [uint8_t]: PAx or PBx defined in GPIO Macro Definitions
// @returns:    [bool]: HIGH or LOW (1 or 0)
// *****************************************************************************
bool DigitalReadPin( uint8_t pin );


// *****************************************************************************
// @desc:       Toggle a specific pin, provided it was set as OUTPUT prior
// @args:       None
// @returns:    None
// *****************************************************************************
void Toggle( uint8_t pin );


// *****************************************************************************
// @desc:       Enable Timer1 peripheral, initialize registers and set interval
// @args:       duration_ms [uint16_t]: User defined interrupt interval in ms
// @returns:    None
// *****************************************************************************
void Timer1Init( uint16_t duration_ms );


// *****************************************************************************
// @desc:       Start Timer1, enable interrupt, clear interrupt flag
//                  Internally called by Timer1Init()
// @args:       None
// @returns:    None
// *****************************************************************************
void Timer1Start( void );


// *****************************************************************************
// @desc:       Stop Timer1, disable interrupt
// @args:       None
// @returns:    None
// *****************************************************************************
void Timer1Stop( void );


// *****************************************************************************
// @desc:       Assigns a function defined by the user as interrupt callback
//                  function
// @args:       None
// @returns:    None
// *****************************************************************************
void Timer1SetInterruptHandler(void (* InterruptHandler)(void));


#endif // _NANOLAY_CORE_H