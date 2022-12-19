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


/* SysTick_Handler --- ISR for System Timer overflow, used for 1ms ticker */

void SysTick_Handler(void)
{
   static uint8_t flag = 0;
   
   Milliseconds++;
   Tick = 1;
   
   // DEBUG: 500Hz on PC14 pin
   if (flag)
      GPIOC->BSRR = GPIO_BSRR_BR14; // GPIO pin PC14 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS14; // GPIO pin PC14 HIGH
      
   flag = !flag;
}


/* t1ou --- send a single character on UART1 by polling */

static void t1ou(const int ch)
{
   // Wait for TX ready
   while ((USART1->SR & 1<<7) == 0)
      ;
  
   // Send byte
   USART1->DR = ch;
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
   
   // Configure PC14, the GPIO pin with 500Hz square wave
   GPIOC->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);     // Set PC14 to push-pull output mode
   GPIOC->CRH |= (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1); // Configure PC14 as output, 50MHz
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   // TODO: set alternate mode on pins PA9 and PA10
   
   // Configure the UART - defaults are 1 start bit, 8 data bits, 1 stop bit,
   // no parity, so I'll leave those as they are
   RCC->APB2ENR |= 1<<14;         // Enable USART1 clock
   USART1->CR1 |= 1<<13;          // Switch on the UART
   USART1->BRR |= 0x683;          // Set for 9600 baud (reference manual page 528)
   USART1->CR1 |= 1<<3;           // Enable transmitter (sends a junk character)
// USART1->CR1 |= 1<<2;           // Enable receiver
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
   if (SysTick_Config (8000)) { // 8MHz divided by 1000  (SystemCoreClock / 1000)
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
      GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW, LED on
      
      for (dally = 0; dally < 400000; dally++)
         ;
      
      t1ou('-');
      
      GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH, LED off
      
      for (dally = 0; dally < 400000; dally++)
         ;
      
      t1ou('*');
   }
}

