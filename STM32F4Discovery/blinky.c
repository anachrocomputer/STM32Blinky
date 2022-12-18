/* blinky --- blink LEDs on STM32F4Discovery board          2022-12-18 */

#include <stm32f4xx.h>

#include <stdint.h>


volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;


/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
   return (Milliseconds);
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;      // Enable clock to GPIO D peripherals on AHB1 bus
   
   // Configure the GPIO pin with the LEDs
   GPIOD->MODER |= GPIO_MODER_MODER12_0;     // Configure PD12 as output, green LED
   GPIOD->MODER |= GPIO_MODER_MODER13_0;     // Configure PD13 as output, amber LED
   GPIOD->MODER |= GPIO_MODER_MODER14_0;     // Configure PD14 as output, red LED
   GPIOD->MODER |= GPIO_MODER_MODER15_0;     // Configure PD15 as output, blue LED
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
}


int main(void)
{
   volatile int dally;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initPWM();
   initMillisecondTimer();
   
   while (1) {
      GPIOD->BSRR = GPIO_BSRR_BR12; // GPIO pin PD12 LOW, green LED off
      GPIOD->BSRR = GPIO_BSRR_BS13; // GPIO pin PD13 HIGH, amber LED on
      GPIOD->BSRR = GPIO_BSRR_BR14; // GPIO pin PD14 LOW, red LED off
      GPIOD->BSRR = GPIO_BSRR_BS15; // GPIO pin PD15 HIGH, blue LED on
      
      for (dally = 0; dally < 400000; dally++)
         ;
         
      GPIOD->BSRR = GPIO_BSRR_BS12; // GPIO pin PD12 HIGH, green LED on
      GPIOD->BSRR = GPIO_BSRR_BR13; // GPIO pin PD13 LOW, amber LED off
      GPIOD->BSRR = GPIO_BSRR_BS14; // GPIO pin PD14 HIGH, red LED on
      GPIOD->BSRR = GPIO_BSRR_BR15; // GPIO pin PD15 LOW, blue LED off
      
      for (dally = 0; dally < 400000; dally++)
         ;
   }
}

