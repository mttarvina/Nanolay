# *Nanolay* : Collection of Custom dsPIC33CK Library

| Parameter         | Value                           |
| ----------------- | ------------------------------- |
| Revision          | A                               |
| Target Device     | DSPIC33CK256MP202               |
| Development Board | Nanolay RevA                    |
| Author            | Mark Angelo Tarvina (mttarvina) |
| Email             | mttarvina@gmail.com             |



## Wishlist

* Add function API in Core library to enable Reference Oscillator Output and assign it to a specific output pin
* Add function API in Core library to enable Input Change Notification on a specific GPIO pin





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



#### GPIO Diagram

![gpio_diagram](img\gpio_diagram.PNG)



-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



## Important Notes

#### Unused I/O

* Unused I/O pins should be configured as outputs and driven to a logic low state.
* Alternatively, connect a 1k to 10k resistor between VSS and unused pins, and drive the output to logic low.

#### Measured Current Consumption

This is a series of current measurements using Nanolay RevA board.

| MASTER_CLK | CLKOUTEN | GPIO Default Setting | Current Consumption |
| ---------- | -------- | -------------------- | ------------------- |
| 4MHz       | true     | OUTPUT, Drive LOW    | 22.7mA              |
| 4MHz       | false    | OUTPUT, Drive LOW    | 22.5mA              |
| 4MHz       | false    | INPUT                | 22.4mA              |
| 8MHz       | true     | OUTPUT, Drive LOW    | 23.8mA              |
| 16MHz      | true     | OUTPUT, Drive LOW    | 27.9mA              |
| 20MHz      | true     | OUTPUT, Drive LOW    | 28.3mA              |
| 25MHz      | true     | OUTPUT, Drive LOW    | 28.9mA              |
| 30MHz      | true     | OUTPUT, Drive LOW    | 31.1mA              |
| 40MHz      | true     | OUTPUT, Drive LOW    | 31.1mA              |
| 50MHz      | true     | OUTPUT, Drive LOW    | 32.5mA              |
| 100MHz     | true     | OUTPUT, Drive LOW    | 38.9mA              |



-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



## MCC Settings @ 4MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

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

#### CLKOUT Waveform @ PB1

![4mhz_clkout](img\4mhz_clkout.png)



## MCC Settings @ 8MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                 |
| --------------------------- | ---------------------------------- | ------------------------------------------- |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                      |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                            |
| PLL                         | Disabled/Not set                   |                                             |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later     |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug |
| VCO Divider                 | FVCO/3                             | Target is 400MHz                            |
| AVCO Divider                | FVCO/3                             | Target is 400MHz                            |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                   |
| Enable Clock Switching      | Enabled                            |                                             |
| Enable Fail-Safe Monitor    | Enabled                            |                                             |
| ICD Emulator Pin            | PGC2, PGD2                         |                                             |
| DMT                         | Disabled/Not set                   |                                             |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                             |

#### CLKOUT Waveform @ PB1

![8mhz_clkout](img\8mhz_clkout.png)



## MCC Settings @ 16MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:100, Postscaler1 = 1:5, Postscaler2 = 1:5 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/2                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/2                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![16mhz_clkout](img\16mhz_clkout.png)



## MCC Settings @ 20MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:100, Postscaler1 = 1:5, Postscaler2 = 1:4 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/2                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/2                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![20mhz_clkout](img\20mhz_clkout.png)



## MCC Settings @ 25MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:100, Postscaler1 = 1:4, Postscaler2 = 1:4 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/2                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/2                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![25mhz_clkout](img\25mhz_clkout.png)



## MCC Settings @ 30MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:150, Postscaler1 = 1:5, Postscaler2 = 1:4 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/3                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/3                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![30mhz_clkout](img\30mhz_clkout.png)



## MCC Settings @ 40MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:100, Postscaler1 = 1:5, Postscaler2 = 1:2 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/2                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/2                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![40mhz_clkout](img\40mhz_clkout.png)



## MCC Settings @ 50MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:100, Postscaler1 = 1:4, Postscaler2 = 1:2 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/2                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/2                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![50mhz_clkout](img\50mhz_clkout.png)



## MCC Settings @ 100MHz

#### SYSTEM MODULE

NOTE: System module settings should be adjusted to ensure that most of the peripherals will work at least during 4MHz operation.

| Parameter                   | Value                              | Description                                                  |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ |
| Clock Source                | FRC Oscillator                     | [Default] 8MHz nominal                                       |
| FRC Post-scaler             | Disabled                           | Postscaler = 1:1                                             |
| PLL                         | Enabled                            | Prescaler = 1:1, Feedback = 1:100, Postscaler1 = 1:4, Postscaler2 = 1:1 |
| Auxiliary Clock             | Disabled                           | Default unless otherwise required later, APLL still use the same setting as the PLL, even though it is disabled |
| Clock Output Pin Config     | OSC2 as GPIO                       | [Default] OSC2 as CLK Output for demo/debug                  |
| VCO Divider                 | FVCO/2                             | Target is 400MHz                                             |
| AVCO Divider                | FVCO/2                             | Target is 400MHz                                             |
| Reference Oscillator Output | Disabled/Not Set                   | [Default]                                                    |
| Enable Clock Switching      | Enabled                            |                                                              |
| Enable Fail-Safe Monitor    | Enabled                            |                                                              |
| ICD Emulator Pin            | PGC2, PGD2                         |                                                              |
| DMT                         | Disabled/Not set                   |                                                              |
| Watchdog: Enable Mode       | Software controlled: WDTCON.ON bit |                                                              |

#### CLKOUT Waveform @ PB1

![100mhz_clkout](img\100mhz_clkout.png)



-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



## Library: Core

#### Overview

**Core** library features the backbone of all libraries that will be built in the future. This will contain all macro definitions and function API's for a basic microcontroller application.

| Feature                   | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| System Initialization     | Sets the system clock frequency, default GPIO and other peripheral states. Disables all peripheral modules by default |
| GPIO Function APIs        |                                                              |
| Timer1 Function APIs      | Used to implement a default timer based delay function       |
| I/O Mapping Function APIs | [ *** to be implemented ]                                    |



#### MASTER CLOCK Macro Definitions

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



#### GPIO Registers

* The Data Direction register (TRISx) determines whether the pin is an input or an output. If the data direction bit is a ‘1’, then the pin is an input.
* All port pins are defined as inputs after a Reset. Reads from the latch (LATx), read the latch. Writes to the latch, write the latch. Reads from the port (PORTx), read the port pins, while writes to the port pins, write the latch.
* When a pin is shared with another peripheral or function that is defined as an input only, it is nevertheless regarded as a dedicated port because there is no other competing source of outputs.
* Port pins can also be individually configured for either digital or open-drain output. This is controlled by the Open-Drain Enable for PORTx register, ODCx, associated with each port. Setting any of the bits configures the corresponding pin to act as an open-drain output. 
* The port pins that are to function as analog inputs or outputs must have their corresponding ANSELx and TRISx bits set
* In order to use port pins for I/O functionality with digital modules, such as timers, UARTs, etc., the corresponding ANSELx bit must be cleared.
* If the TRISx bit is cleared (output) while the ANSELx bit is set, the digital output level (VOH or VOL) is converted by an analog peripheral, such as the ADC module or comparator module.
* When the PORTx register is read, all pins configured as analog input channels are read as cleared (a low level)
* Pins configured as digital inputs do not convert an analog input. Analog levels on any pin, defined as a digital input (including the ANx pins), can cause the input buffer to consume current that exceeds the device specifications.

| Register | Description                         | 0 Value                                      | 1 Value                                      |
| -------- | ----------------------------------- | -------------------------------------------- | -------------------------------------------- |
| TRISx    | Output enable for port register     | OUTPUT                                       | INPUT                                        |
| ANSELx   | Analog select for port register.    | Analog input DISABLED, Digital input ENABLED | Analog input ENABLED, Digital input DISABLED |
| PORTx    | Input data for port register        | n/a                                          | n/a                                          |
| LATx     | Output data for port register       | LOW                                          | HIGH                                         |
| ODCx     | Open-drain enable for port register | DISABLED                                     | ENABLED                                      |
| CNPUx    | Pull-up enable for port register    | DISABLED                                     | ENABLED                                      |
| CNPDx    | Pull-down enable for port register  | DISABLED                                     | ENABLED                                      |



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

