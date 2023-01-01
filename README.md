# STM32Blinky #

Some simple STM32 ARM test programs to verify toolchain, programmer and chip.

The programs are in C and may be compiled with GCC on Linux.

## Chips Supported ##

At present, there's support for the STM32F103 on the "Blue Pill" development board
and the STM32L152 on the STM32L-Discovery board.
Note that my Discovery board is an old one and has the smaller STM32L152RBT6 on it.
Edit the 'Makefile' if you have a newer board, to compile for the bigger chip.

## ARM Toolchain ##

A recent version of GCC for ARM, such as that installed by the Arduino "stm32duino" toolchain.

You'll also need some files from the STMicroelectronics "STM32CubeF1" and/or "STM32CubeL1" repo on GitHub,
such as the linker script.

