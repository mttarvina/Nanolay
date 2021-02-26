# *Nanolay* : Collection of Custom dsPIC33CK Library

| Parameter         | Value                           |
| ----------------- | ------------------------------- |
| Revision          | A                               |
| Target Device     | DSPIC33CK256MP202               |
| Development Board | Nanolay RevA                    |
| Author            | Mark Angelo Tarvina (mttarvina) |
| Email             | mttarvina@gmail.com             |



## Wishlist

* Add function in Core library to enable Reference Oscillator Output and assign it to a specific output pin





## Hardware Overview

#### Pinout

![pinout1](img\pinout1.PNG)

![pinout2](img\pinout2.PNG)



#### Oscillator Subsystem

![osc_diagram](img\osc_diagram.PNG)



#### Primary PLL

The Primary Oscillator and internal FRC Oscillator sources can optionally use an on-chip PLL to obtain higher operating speeds.
For PLL operation, the following requirements must be met at all times without exception:

* The PLL Input Frequency (FPLLI) must be in the range of 8 MHz to 64 MHz
* The PFD Input Frequency (FPFD) must be in the range of 8 MHz to (FVCO/16) MHz
* The VCO Output Frequency (FVCO) must be in the range of 400 MHz to 1600 MHz

![osc_pll_diagram](img\osc_pll_diagram.PNG)



#### Auxiliary PLL

The dsPIC33CK256MP508 device family implements an Auxiliary PLL (APLL) module, which is used to generate various peripheral clock sources independent of the system clock.
For APLL operation, the following requirements must be met at all times without exception:

* The APLL Input Frequency (AFPLLI) must be in the range of 8 MHz to 64 MHz
* The APFD Input Frequency (AFPFD) must be in the range of 8 MHz to (AFVCO/16) MHz
* The AVCO Output Frequency (AFVCO) must be in the range of 400 MHz to 1600 MHz

![osc_apll_diagram](img\osc_apll_diagram.PNG)



## Important Notes

#### Unused I/O

* Unused I/O pins should be configured as outputs and driven to a logic low state.
* Alternatively, connect a 1k to 10k resistor between VSS and unused pins, and drive the output to logic low.



## MCC Settings @ 4MHz

#### SYSTEM MODULE

| Parameter                   | Value                              | Description                                 |
| --------------------------- | ---------------------------------- | ------------------------------------------- |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                      |
| FRC Post-scaler             | Enabled                            | Postscaler = 1:2                            |
| PLL                         | Disabled/Not set                   |                                             |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later     |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug |
| VCO Divider                 | FVCO/3                             |                                             |
| AVCO Divider                | FVCO/3                             |                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                   |
| Enable Clock Switching      | Enabled                            |                                             |
| Enable Fail-Safe Monitor    | Enabled                            |                                             |
| ICD Emulator Pin            | PGC2, PGD2                         |                                             |
| DMT                         | Disabled/Not set                   |                                             |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                             |



## Library: Core

#### Overview

**Core** library features the backbone of all libraries that will be built in the future. This will contain all macro definitions and function API's for a basic microcontroller application.

| Feature                   | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| System Initialization     | Sets the system clock frequency, default GPIO and other peripheral states. Disables all peripheral modules by default |
| GPIO Function APIs        |                                                              |
| Timer1 Function APIs      | Used for basic delay functionalities                         |
| I/O Mapping Function APIs | [ *** to be implemented ]                                    |



#### MASTER CLOCK Macro Settings

Set one of these macro variables to **true** to set the master clock frequency. This corresponds to the value of FOSC. There should only be one of these variables set to **true**.

| Macro Variable    | Value | Description |
| ----------------- | ----- | ----------- |
| MASTER_CLK_4MHZ   | true  | [Default]   |
| MASTER_CLK_8MHZ   | false |             |
| MASTER_CLK_16MHZ  | false |             |
| MASTER_CLK_20MHZ  | false |             |
| MASTER_CLK_25MHZ  | false |             |
| MASTER_CLK_30MHZ  | false |             |
| MASTER_CLK_40MHZ  | false |             |
| MASTER_CLK_50MHZ  | false |             |
| MASTER_CLK_100MHZ | false |             |



#### Boolean Macro Definitions

| Macro Variable | Value | Description                                                  |
| -------------- | ----- | ------------------------------------------------------------ |
| CLKOUTEN       | false | Set this to true if you want to look at the CLKOUT waveform at RB1 pin. Output signal frequency should be equal to FOSC/2 |
| INPUT          | true  | Used in GPIO function APIs                                   |
| OUTPUT         | false | Used in GPIO function APIs                                   |
| LOW            | false | Used in GPIO function APIs                                   |
| HIGH           | true  | Used in GPIO function APIs                                   |



#### GPIO Pin Macro Definitions

| Macro Variable | Value | Description |
| -------------- | ----- | ----------- |
| PORT_A         | 0x01  |             |
| PORT_B         | 0x02  |             |
| PA0            | 0x01  |             |
| PA1            | 0x02  |             |
| PA2            | 0x03  |             |
| PA3            | 0x04  |             |
| PA4            | 0x05  |             |
| PB0            | 0x06  |             |
| PB1            | 0x07  |             |
| PB2            | 0x08  |             |
| PB3            | 0x09  |             |
| PB4            | 0x0A  |             |
| PB5            | 0x0B  |             |
| PB6            | 0x0C  |             |
| PB7            | 0x0D  |             |
| PB8            | 0x0E  |             |
| PB9            | 0x0F  |             |
| PB10           | 0x10  |             |
| PB11           | 0x11  |             |
| PB12           | 0x12  |             |
| PB13           | 0x13  |             |
| PB14           | 0x14  |             |
| PB15           | 0x15  |             |



#### Function APIs

``` C
// *****************************************************************************
// @desc:       Initialize system clock, peripherals, and default GPIO states.
//                  Required to call this function at the start of main program
// @args:       None
// @returns:    None
// *****************************************************************************
void SysInit( void );
```

``` C
// *****************************************************************************
// @desc:       Initialize system CLK registers based on MASTER_CLK_xMHZ macro
//                  This is called by SysInit()
// @args:       None
// @returns:    None
// *****************************************************************************
void ClockInit( void );
```

``` C
// *****************************************************************************
// @desc:       Disables all peripherals. User must manually enable a peripheral
//                  in the main program before using peripheral specific
//                  function APIs. This is called by SysInit()
// @args:       None
// @returns:    None
// *****************************************************************************
void DisableAllPeripherals( void );
```

``` C
// *****************************************************************************
// @desc:       Set a pin as Digital OUTPUT or Digital INPUT. This is similar
//                  to Arduino's PinMode() function
// @args:       pin [uint8_t]: PAx or PBx defined in GPIO Macro Definitions
//              dir [bool]: OUTPUT or INPUT
// @returns:    None
// *****************************************************************************
void DigitalSetPin( uint8_t pin, bool dir );
```

``` C
// *****************************************************************************
// @desc:       Set a pin to output HIGH or LOW, provided that it is set as
//                  OUTPUT prior. This is similar to Arduino's DigitalWrite()
//                  function
// @args:       pin [uint8_t]: PAx or PBx defined in GPIO Macro Definitions
//              state [bool]: HIGH or LOW
// @returns:    None
// *****************************************************************************
void DigitalDrvPin( uint8_t pin, bool state );
```

``` C
// *****************************************************************************
// @desc:       Read from a digital pin, provided that it is set as INPUT prior.
//                  This is similar to Arduino's DigitalRead() function
// @args:       pin [uint8_t]: PAx or PBx defined in GPIO Macro Definitions
// @returns:    [bool]: HIGH or LOW (1 or 0)
// *****************************************************************************
bool DigitalReadPin( uint8_t pin );
```

