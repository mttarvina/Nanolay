/* ************************************************************************** */
// Nanolay - GPIO Library Header File
//
// Description:     Custom dsPIC33CK library for GPIO functions. Should be
//                  included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    17.Apr.21
/* ************************************************************************** */

#ifndef _NANOLAY_GPIO_H
#define	_NANOLAY_GPIO_H


#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


// GPIO Macros
#define INPUT                   true
#define OUTPUT                  false
#define LOW                     false
#define HIGH                    true
#define RISING                  true
#define FALLING                 false
#define PORT_A                  0x0
#define PORT_B                  0x1

#define PIN0                    0x0001
#define PIN1                    0x0002
#define PIN2                    0x0004
#define PIN3                    0x0008
#define PIN4                    0x0010
#define PIN5                    0x0020
#define PIN6                    0x0040
#define PIN7                    0x0080
#define PIN8                    0x0100
#define PIN9                    0x0200
#define PIN10                   0x0400
#define PIN11                   0x0800
#define PIN12                   0x1000
#define PIN13                   0x2000
#define PIN14                   0x4000
#define PIN15                   0x8000


// *****************************************************************************
// @desc:       Sets the default GPIO setting for all pins, at startup, as INPUT
//                  This is called by SysInit()
// @args:       None
// @returns:    None
// *****************************************************************************
void GPIO_Init( void );


// *****************************************************************************
// @desc:       Set any PORTA pin as INPUT or OUTPUT, opendrain feature
//                  can be enabled individually
// @args:       pin [uint_fast16_t]: PINx
//              dir [bool]: OUTPUT or INPUT
//              opendrain [bool]: True = enable
// @returns:    None
// *****************************************************************************
void GPIO_SetPortAPin( uint_fast16_t pin, bool dir, bool opendrain );


// *****************************************************************************
// @desc:       Set any PORTB pin as INPUT or OUTPUT, opendrain feature
//                  can be enabled individually
// @args:       pin [uint_fast16_t]: PINx
//              dir [bool]: OUTPUT or INPUT
//              opendrain [bool]: True = enable
// @returns:    None
// *****************************************************************************
void GPIO_SetPortBPin( uint_fast16_t pin, bool dir, bool opendrain );


// *****************************************************************************
// @desc:       Set any PORTA pin to output HIGH or LOW. This is similar to
//                  Arduino's DigitalWrite() function
// @args:       pin [uint_fast16_t]: PINx
//              state [bool]: HIGH or LOW
// @returns:    None
// *****************************************************************************
void GPIO_DrivePortAPin( uint_fast16_t pin, bool state );


// *****************************************************************************
// @desc:       Set any PORTB pin to output HIGH or LOW. This is similar to
//                  Arduino's DigitalWrite() function
// @args:       pin [uint_fast16_t]: PINx
//              state [bool]: HIGH or LOW
// @returns:    None
// *****************************************************************************
void GPIO_DrivePortBPin( uint_fast16_t pin, bool state );


// *****************************************************************************
// @desc:       Attach an interrupt to any PORTA pin
// @args:       pin [uint_fast16_t]: PINx
//              edgetype [bool]: RISING or FALLING
//              InterruptHandler [pointer]: An interrupt callback function 
// @returns:    None
// *****************************************************************************
void GPIO_SetPortAInterrupt( uint_fast16_t pin, bool edgetype, void (* InterruptHandler)(void) );


// *****************************************************************************
// @desc:       Attach an interrupt to any PORTB pin
// @args:       pin [uint_fast16_t]: PINx
//              edgetype [bool]: RISING or FALLING
//              InterruptHandler [pointer]: An interrupt callback function 
// @returns:    None
// *****************************************************************************
void GPIO_SetPortBInterrupt( uint_fast16_t pin, bool edgetype, void (* InterruptHandler)(void) );

#endif // _NANOLAY_GPIO_H