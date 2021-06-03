/* ************************************************************************** */
// Nanolay - Custom Lquid Crystal (LCD) 16x2 library header for dsPIC33CK
//
// Description:     Custom dsPIC33CK library for 16x2 LCD
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    04.May.21
/* ************************************************************************** */


#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>
#include "nanolay_sccp.h"
#include "nanolay_gpio.h"


// I/D = 1: Increment
// I/D = 0: Decrement
// S = 1: Accompanies display shift
// S/C = 1: Display shift
// S/C = 0: Cursor move
// R/L = 1: Shift to the right
// R/L = 0: Shift to the left
// DL = 1: 8 bits, DL = 0: 4 bits
// N = 1: 2 lines, N = 0: 1 line
// F = 1: 5 × 10 dots, F = 0: 5 × 8 dots
// BF = 1: Internally operating
// BF = 0: Instructions acceptable


// instruction set - general
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRYMODE_CTRL  0x04
#define LCD_DISPLAY_CTRL    0x08
#define LCD_CURSOR_CTRL     0x10
#define LCD_FUNCSET_CTRL    0x20
#define LCD_SETCGRAM        0x40
#define LCD_SETDDRAM        0x80

// instruction set - entry mode settings
#define LCD_INCREMENT_DDRAM 0x02
#define LCD_DECREMENT_DDRAM 0x00
#define LCD_SHIFT_ENABLE    0x01
#define LCD_SHIFT_DISABLE   0x00
#define LCD_SHIFT_LEFT      0x03
#define LCD_SHIFT_RIGHT     0x01

// instruction set - display settings
#define LCD_DISPLAY_ON      0x04
#define LCD_DISPLAY_OFF     0x00
#define LCD_CURSOR_ON       0x02
#define LCD_CURSOR_OFF      0x00
#define LCD_BLINK_ON        0x01
#define LCD_BLINK_OFF       0x00








typedef struct LCD_1602_4BIT {
    uint_fast8_t port;
    uint_fast16_t rs_pin;
    uint_fast16_t en_pin;
    uint_fast16_t data_pins[4];                                                 // [d7, d6, d5, d4]
} LCD_4B_1602;


// *****************************************************************************
// @desc:       Initializes the LCD module. All pins should belong to only one
//                  port register of the uC
// @args:       port [uint_fast8_t]: port register of the pin interface
//              rs [uint_fast16_t]: register select pin
//              en [uint_fast16_t]: enable pin
//              d7 [uint_fast16_t]: bit7 pin
//              d6 [uint_fast16_t]: bit6 pin
//              d5 [uint_fast16_t]: bit5 pin
//              d4 [uint_fast16_t]: bit4 pin
// @returns:    None
// *****************************************************************************
void LCD_4B_1602_Init( uint_fast8_t port, uint_fast16_t rs, uint_fast16_t en, uint_fast16_t d7, uint_fast16_t d6, uint_fast16_t d5, uint_fast16_t d4 );