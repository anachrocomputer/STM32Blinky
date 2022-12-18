/* blinky --- blink LED on PC13 of Blue Pill STM32 board    2022-12-17 */

#include <stm32f1xx.h>

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
   RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                    // Enable clock to GPIO peripherals on APB2 bus
   
   // Configure PC13, the GPIO pin with the LED
   GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // Set PC13 to push-pull output mode
   GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1); // Configure PC13 as output, 50MHz
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
      GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW, LED on
      
      for (dally = 0; dally < 400000; dally++)
         ;
         
      GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH, LED off
      
      for (dally = 0; dally < 400000; dally++)
         ;
   }
}

