/* ************************************************************************** */
// Nanolay - Main C File
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    26.Feb.2021
/* ************************************************************************** */

#include "nanolay/nanolay_core.h"


int main(void) {
    SysInit();
    Timer1Init();
    DigitalSetPin(PB2, OUTPUT);
    
    while (true) {
        DigitalDrvPin(PB2, HIGH);
        wait_ms(50);
        DigitalDrvPin(PB2, LOW);
        wait_ms(50);
    }
    return 0;
}