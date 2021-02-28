/* ************************************************************************** */
// Nanolay - Main C File
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    28.Feb.2021
/* ************************************************************************** */
#include "nanolay/nanolay_core.h"

#define TOGGLE_RATE 50              // 50ms
uint8_t LedPin = PB2;
unsigned long long timestamp = 0;

int main(void) {
    SysInit();
 
    DigitalSetPin(LedPin, OUTPUT);
    
    SCCP1Init();
    SCCP1Start();
    
    timestamp = millis();
    
    while (true) {
        if ( millis() - timestamp >= TOGGLE_RATE ){
            timestamp = millis();
            Toggle(LedPin);
        }
    }
    return 0;
}
