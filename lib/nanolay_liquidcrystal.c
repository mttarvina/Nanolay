/* ************************************************************************** */
// Nanolay - Custom Lquid Crystal (LCD) 16x2 library source code for dsPIC33CK
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


// LCD performs these steps at power up. Need to do a different reset procedure
// for a 4bit interface. Need also to change the number of lines.
//
// 1. Display clear
// 2. Function set:
//      DL = 1; 8-bit interface data
//      N = 0; 1-line display
//      F = 0; 5 × 8 dot character font
// 3. Display on/off control:
//      D = 0; Display off
//      C = 0; Cursor off
//      B = 0; Blinking off
// 4. Entry mode set:
//      I/D = 1; Increment by 1
//      S = 0; No shift