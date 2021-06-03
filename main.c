/* ************************************************************************** */
// Nanolay - Main C File
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    17.Mar.2021
/* ************************************************************************** */

#include "nanolay/nanolay.h"

bool state = true;

void __attribute__ ((weak)) Callback( void ){
    state = !state;
    GPIO_DrivePortAPin(PIN4, state);
}


int main(void) {
    Sys_Init(_8MHZ);

    GPIO_SetPortBPin(PIN5, INPUT, false);
    GPIO_SetPortAPin(PIN4, OUTPUT, false);

    GPIO_SetPortBInterrupt(PIN5, FALLING, &Callback);

    while (true) {

    }
    return 0;
}