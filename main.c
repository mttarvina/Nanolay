/* ************************************************************************** */
// Nanolay - Main C File
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    26.Feb.2021
/* ************************************************************************** */
#include "nanolay/nanolay_core.h"


uint8_t LedPin = PB2;


void Timer1CallBack(void);                      // User defined interrupt callback function

int main(void) {
    SysInit();

    Timer1Init(50);                             // Set interrupt interval to 50ms
    Timer1SetInterruptHandler(&Timer1CallBack); // assign callback function as interrupt handler
    Timer1Start();
    
    DigitalSetPin(LedPin, OUTPUT);

    while (true) {
        // do nothing here
    }
    return 0;
}


void __attribute__ ((weak)) Timer1CallBack(void){
    //  interrupt routine, should be kept short
    Toggle(LedPin);
}