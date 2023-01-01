/* blinky --- blink LEDs on STM32L-Discovery board          2023-01-01 */

#include "stm32l1xx.h"


uint32_t SavedRccCsr = 0u;
volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;

/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
   return (Milliseconds);
}


/* SysTick_Handler --- ISR for System Timer overflow, used for 1ms ticker */

void SysTick_Handler(void)
{
   static uint8_t flag = 0;
   
   Milliseconds++;
   Tick = 1;
   
   // DEBUG: 500Hz on PC14 pin
   if (flag)
      GPIOC->BSRR = GPIO_BSRR_BR_14; // GPIO pin PC14 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS_14; // GPIO pin PC14 HIGH
      
   flag = !flag;
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   RCC->CR |= RCC_CR_HSION;       // Switch on High Speed Internal clock (16MHz on the STM32L152RBT6)
   
   // Wait for HSI to start up
   while ((RCC->CR & RCC_CR_HSIRDY) == 0)
      ;
   
   RCC->CFGR = RCC_CFGR_SW_HSI;   // Select HSI as system clock
   
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
      ;

   RCC->CSR |= 1<<0;    		    // Switch the LSI on, too- we need it for the LCD
   
   while (!(RCC->CSR & 0x2))      // Wait for it to become stable and available
      ;
   
   SystemCoreClockUpdate();
   
   SavedRccCsr = RCC->CSR;
   RCC->CSR |= RCC_CSR_RMVF;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->AHBENR |= RCC_AHBENR_GPIOBEN;        // Enable clock to GPIO B peripherals on AHB bus
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN;        // Enable clock to GPIO C peripherals on AHB bus
   
   // Configure the GPIO pins with the LEDs
   GPIOB->MODER |= GPIO_MODER_MODER7_0;     // Configure PB7 as output, green LED
   GPIOB->MODER |= GPIO_MODER_MODER6_0;     // Configure PB6 as output, blue LED
   
   // Configure PC14, the GPIO pin with 500Hz square wave
   GPIOC->MODER |= GPIO_MODER_MODER14_0;    // Configure PC14 as output
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
   if (SysTick_Config(16000)) { // 16MHz divided by 1000  (SystemCoreClock / 1000)
      while (1)
         ;
   }
}


int main(void)
{
   volatile int dally;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initPWM();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   while (1) {
      GPIOB->BSRR = GPIO_BSRR_BS_6;        // Blue led ON
      GPIOB->BSRR = GPIO_BSRR_BR_7;        // Green led OFF
      
      for (dally = 0; dally < 400000; dally++)
         ;
      
      GPIOB->BSRR = GPIO_BSRR_BR_6;        // Blue led OFF
      GPIOB->BSRR = GPIO_BSRR_BS_7;        // Green led ON
      
      for (dally = 0; dally < 400000; dally++)
         ;
   }
}
