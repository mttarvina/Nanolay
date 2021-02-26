#ifndef _NANOLAY_CORE_H
#define	_NANOLAY_CORE_H


#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>


// available MASTER_CLK Frequencies: FOSC
// #define _4MHZ               4000000UL
// #define _8MHZ               8000000UL
// #define _16HZ               16000000UL
// #define _20MHZ              20000000UL
// #define _25MHZ              25000000UL
// #define _30MHZ              30000000UL
// #define _40MHZ              40000000UL
// #define _50MHZ              50000000UL
// #define _100MHZ             100000000UL

// User needs to set one of these macros to change Fosc
#define MASTER_CLK_4MHZ     true    // DEFAULT
#define MASTER_CLK_8MHZ     false
#define MASTER_CLK_16MHZ    false
#define MASTER_CLK_20MHZ    false
#define MASTER_CLK_25MHZ    false
#define MASTER_CLK_30MHZ    false
#define MASTER_CLK_40MHZ    false
#define MASTER_CLK_50MHZ    false
#define MASTER_CLK_100MHZ   false

// Enables Fosc/2 output at RB1 pin
#define CLKOUTEN            true            

// Boolean Macros
#define INPUT               true
#define OUTPUT              false
#define LOW                 false
#define HIGH                true

// Custom GPIO Pin Macros
#define PORT_A              0x1
#define PORT_B              0x2

#define PA0                 0x01
#define PA1                 0x02
#define PA2                 0x03
#define PA3                 0x04
#define PA4                 0x05
#define PB0                 0x06
#define PB1                 0x07
#define PB2                 0x08
#define PB3                 0x09
#define PB4                 0x0A
#define PB5                 0x0B
#define PB6                 0x0C
#define PB7                 0x0D
#define PB8                 0x0E
#define PB9                 0x0F
#define PB10                0x10
#define PB11                0x11
#define PB12                0x12
#define PB13                0x13
#define PB14                0x14
#define PB15                0x15

// *****************************************************************************
// @desc:       Initialize system clock, disables all perihperals,
//                  set default GPIO states
// @args:       None
// @returns:    None
// *****************************************************************************
void SysInit( void );


// *****************************************************************************
// @desc:       Initialize system CLK registers based on MASTER_CLK_xMHZ macro
// @args:       None
// @returns:    None
// *****************************************************************************
void ClockInit( void );


void GPIOInit( void );
void DisableAllPeripherals( void );
void DigitalSetPin( uint8_t pin, bool dir );
void DigitalDrvPin( uint8_t pin, bool state );
bool DigitalReadPin( uint8_t pin );

#endif // _NANOLAY_CORE_H