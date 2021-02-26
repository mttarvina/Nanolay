# *Nanolay* : Collection of Custom dsPIC33CK Library

| Parameter     | Value                           |
| ------------- | ------------------------------- |
| Revision      | A                               |
| Target Device | DSPIC33CK256MP202               |
| Author        | Mark Angelo Tarvina (mttarvina) |
| Email         | mttarvina@gmail.com             |



## Wishlist

* Add function in Core library to enable Reference Oscillator Output and assign it to a specific output pin





## Hardware Overview

#### Pinout

![pinout1](img\pinout1.PNG)

![pinout2](img\pinout2.PNG)



#### Oscillator Subsystem

![osc_diagram](C:\Users\mttar\Documents\Software\Platforms\MPLAB\Projects\docs\img\osc_diagram.PNG)



#### Primary PLL

The Primary Oscillator and internal FRC Oscillator sources can optionally use an on-chip PLL to obtain higher operating speeds.
For PLL operation, the following requirements must be met at all times without exception:

* The PLL Input Frequency (FPLLI) must be in the range of 8 MHz to 64 MHz
* The PFD Input Frequency (FPFD) must be in the range of 8 MHz to (FVCO/16) MHz
* The VCO Output Frequency (FVCO) must be in the range of 400 MHz to 1600 MHz

![osc_pll_diagram](C:\Users\mttar\Documents\Software\Platforms\MPLAB\Projects\docs\img\osc_pll_diagram.PNG)



#### Auxiliary PLL

The dsPIC33CK256MP508 device family implements an Auxiliary PLL (APLL) module, which is used to generate various peripheral clock sources independent of the system clock.
For APLL operation, the following requirements must be met at all times without exception:

* The APLL Input Frequency (AFPLLI) must be in the range of 8 MHz to 64 MHz
* The APFD Input Frequency (AFPFD) must be in the range of 8 MHz to (AFVCO/16) MHz
* The AVCO Output Frequency (AFVCO) must be in the range of 400 MHz to 1600 MHz

![osc_apll_diagram](C:\Users\mttar\Documents\Software\Platforms\MPLAB\Projects\docs\img\osc_apll_diagram.PNG)



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
| I/O Mapping Function APIs |                                                              |



#### Frequency Macro Definitions

| Macro Variable | Value       | Description |
| -------------- | ----------- | ----------- |
| _4MHZ          | 4000000UL   | [Default]   |
| _8MHZ          | 8000000UL   |             |
| _16MHZ         | 16000000UL  |             |
| _20MHZ         | 20000000UL  |             |
| _25MHZ         | 25000000UL  |             |
| _30MHZ         | 30000000UL  |             |
| _40MHZ         | 40000000UL  |             |
| _50MHZ         | 50000000UL  |             |
| _100MHZ        | 100000000UL |             |

#### Boolean Macro Definitions

| Macro Variable | Value | Description |
| -------------- | ----- | ----------- |
| INPUT          | true  |             |
| OUTPUT         | false |             |
| LOW            | false |             |
| HIGH           | true  |             |



