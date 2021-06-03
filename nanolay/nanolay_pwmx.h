/* ************************************************************************** */
// Nanolay - High Resolution PWM Library Header File
//
// Description:     Custom dsPIC33CK library for High Resolution PWM functions.
//                  Should be included in the nanolay.h file
//
// Target Device:   dsPIC33CKxxxMP202  
//
// Author:          Mark Angelo Tarvina (mttarvina)
// Email:           mttarvina@gmail.com
// Revision:        1.0
// Last Updated:    02.Jun.21
/* ************************************************************************** */


#ifndef _NANOLAY_PWMX
#define	_NANOLAY_PWMX


#include <p33CK256MP202.h>
#include <stdbool.h>
#include <stdint.h>

// APP Configuration
#define INDEPENDENT_SYS             0x00
#define THREE_PHASE_SYS             0x01

// PWM Modes
#define EDGE_ALIGNED_SINGLE         0x00
#define EDGE_ALIGNED_DUAL           0x01
#define VAR_PHASE                   0x02
#define CENTER_ALIGNED              0x03
#define CENTER_ALIGNED_DUAL_EDGE    0x04

// OUTPUT Modes
#define COMPLEMENTARY_OUTPUT        0x00
#define INDEPENDENT_OUTPUT          0x01
#define PUSH_PULL_OUTPUT            0x02


typedef struct _PWMX_OBJECT{
    uint_fast16_t   period
    
} PWMX_OBJ;

#endif // _NANOLAY_PWMX