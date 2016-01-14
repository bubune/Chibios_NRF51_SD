# nrf51 Chibios Port (2.6.5) With ANT+ HRM belt
Chibios port on the NRF51422 with Softdevice S210. HRM demo

[Orgainisation folder]:

Chibios => The RTOS folder without modification, base on Chibios 2.6.5

ported => specific ported code for NRF51

[Before compiling]

1. tested with arm-none-eabi-gcc-4.9.3
2. adapt your board and the NRF51 pin mapping, see ported/board.c and board.h

[Commands]

To flash softdevice:
make flash-softdevice (see Makefile for details)

To compile:
make clean
make all

To flash:
make flash

To Dedub:
make clean
make all
make debug

[Demo]

the application run 2 threads defined in main.c.
-1st thread run a blink led
-2nd thread
    * open an ant+ channel to an ant+ HRM heart belt
    * catch data
    * display HRM bpm on the UART (115200,8,1)


