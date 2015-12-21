# nrf51 Chibios Port (2.6.5) With ANT+ HRM belt
Chibios port on the NRF51422 with Softdevice S210. HRM demo

-> THE PORT IS NOT FUNCTIONAL, see [issue section and main.c for explanation <-

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

[Issue]

when uart and softdevice are enabled after a few sec the application stay blocked, probably due to an interruption.
Sometimes It goes in the hardfault handler. Something interresting, I get 0xDEADBEEF in mcu register and for sure I've never used this value. If I disable the uart int, and use the a led to check that ant+ data comes, it never blocks

A part of an email that I' wrote to the Nordic semi support

"Can you see also my thread at:
https://devzone.nordicsemi.com/question/30492/softdevice-s310-and-interrupt/
I'm pretty sure it's realted. You will see in mu code I've tried to replace cmsis function softdevice equivalent function but it's not working. Maybe I miss something."

