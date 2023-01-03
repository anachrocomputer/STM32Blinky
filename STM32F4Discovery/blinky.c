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


/* SysTick_Handler --- ISR for System Timer overflow, used for 1ms ticker */

void SysTick_Handler(void)
{
   static uint8_t flag = 0;
   
   Milliseconds++;
   Tick = 1;
   
   // DEBUG: 500Hz on PC13 pin
   if (flag)
      GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH
      
   flag = !flag;
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   RCC->CR      = 0x00000083;
   RCC->CFGR    = 0x00000000;
   RCC->PLLCFGR = 0x24003010;
   
   FLASH->ACR |= FLASH_ACR_LATENCY_5WS;   // Set Flash latency to 5 wait states
   FLASH->ACR |= FLASH_ACR_ICEN;          // Cache enable
   FLASH->ACR |= FLASH_ACR_DCEN;
   FLASH->ACR |= FLASH_ACR_PRFTEN;        // Prefetch enable

   RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;    // Set APB1 bus to not exceed 42MHz
   RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;    // Set APB2 bus to not exceed 84MHz

   // Enable power interface clock
   RCC->APB1ENR |= RCC_APB1ENR_PWREN;

   /* Set voltage scale to 1 for max frequency (PWR_CR:bit 14)
    * (0b0) scale 2 for fCLK <= 144 MHz
    * (0b1) scale 1 for 144 MHz < fCLK <= 168 MHz
    */
   PWR->CR |= PWR_CR_VOS;

   // Switch on MCO1 for debugging
   //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;         // Enable clock to GPIO A peripherals on AHB1 bus

   //GPIOA->MODER |= GPIO_MODER_MODER8_0;       // Configure PA8 as output, MCO1
   //GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1;

   // Configure PA8, the GPIO pin with alternative function MCO1
   //GPIOA->MODER |= GPIO_MODER_MODER8_1;          // PA8 in Alternative Function mode

   //RCC->CFGR |= RCC_CFGR_MCO1_1;                 // Send HSE to MCO1
   //RCC->CFGR |= RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2; // Prescale divide-by-4

   RCC->CR |= RCC_CR_HSEON;    // Switch on High Speed External clock (8MHz on the STM32F4Discovery)

   // Wait for HSE to start up
   while ((RCC->CR & RCC_CR_HSERDY) == 0)
      ;
   
   // Make sure PLL is off before we start to configure it
   RCC->CR &= ~RCC_CR_PLLON;
   
   // Configure PLL
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_0;   // Set Q to divide-by 7

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;            // P = divide-by-2

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
   RCC->PLLCFGR |= 168 << RCC_PLLCFGR_PLLN_Pos;   // N = multiply-by-168

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
   RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLM_Pos;      // M = divide-by-4

   RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;    // Select HSE as PLL input
   
   RCC->CR |= RCC_CR_PLLON;             // Switch on the PLL

   // Wait for PLL to start up
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
      ;
   
   uint32_t reg = RCC->CFGR;
   reg &= ~RCC_CFGR_SW;
   reg |= RCC_CFGR_SW_PLL;          // Select PLL as system clock (168MHz)
   RCC->CFGR = reg;
   
   // Wait for PLL to select
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
      ;
   
   SystemCoreClockUpdate();
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;      // Enable clock to GPIO C peripherals on AHB1 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;      // Enable clock to GPIO D peripherals on AHB1 bus
   
   // Configure PC13, the GPIO pin with 500Hz square wave
   GPIOC->MODER |= GPIO_MODER_MODER13_0;     // Configure PC13 as output, 500Hz square wave
   
   // Configure the GPIO pins with the LEDs
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
   if (SysTick_Config(168000)) { // 168MHz divided by 1000  (SystemCoreClock / 1000)
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
      GPIOD->BSRR = GPIO_BSRR_BR12; // GPIO pin PD12 LOW, green LED off
      GPIOD->BSRR = GPIO_BSRR_BS13; // GPIO pin PD13 HIGH, amber LED on
      GPIOD->BSRR = GPIO_BSRR_BR14; // GPIO pin PD14 LOW, red LED off
      GPIOD->BSRR = GPIO_BSRR_BS15; // GPIO pin PD15 HIGH, blue LED on
      
      for (dally = 0; dally < 4000000; dally++)
         ;
         
      GPIOD->BSRR = GPIO_BSRR_BS12; // GPIO pin PD12 HIGH, green LED on
      GPIOD->BSRR = GPIO_BSRR_BR13; // GPIO pin PD13 LOW, amber LED off
      GPIOD->BSRR = GPIO_BSRR_BS14; // GPIO pin PD14 HIGH, red LED on
      GPIOD->BSRR = GPIO_BSRR_BR15; // GPIO pin PD15 LOW, blue LED off
      
      for (dally = 0; dally < 4000000; dally++)
         ;
   }
}

