# STM32Blinky #

Some simple STM32 ARM test programs to verify toolchain, programmer and chip.

The programs are in C and may be compiled with GCC on Linux.

## Hardware Needed ##

Apart from the STM32 development boards, you'll need some LEDs.
For full effect, use two RGB LEDs with suitable resistors.
Connect one of them to the GPIO pins which are digitally controlled
and the other to the pins which are driven by PWM from Timer 3
(T3C1, T3C2 and T3C3).

You'll also need up to three serial-to-USB converter cables (such as FTDI cables,
CH340 cables, or Silicon Labs cables).
The STM32 UARTs are set up on most of the development boards,
and UART1 is used to accept a limited repertoire of commands.
The other UARTs simply send an identifying message.
All UART I/O is via interrupt driven ring buffers and is at 9600 baud.

## Chips Supported ##

At present, there's support for the STM32F103 on the "Blue Pill" development board,
the STM32F411 on the "Black Pill" board,
the STM32F407 on the STM32F4-Discovery board,
and the STM32L152 on the STM32L-Discovery board.
Note that my Discovery boards are old ones and have the smaller STM32 chips on them.
Edit the 'Makefile' if you have a newer board, to compile for the bigger chips.

## ARM Toolchain ##

A recent version of GCC for ARM, such as that installed by the Arduino "stm32duino" toolchain.

You'll also need some files from the STMicroelectronics "STM32CubeF1",
"STM32CubeF4",
and/or "STM32CubeL1" repo on GitHub,
such as the linker script.
These repos are assumed to be installed locally and are referred to by relative paths in that 'Makefile'.
Edit the 'Makefile' to suit your setup.

