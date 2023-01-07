/* blinky --- blink LEDs on STM32F4Discovery board          2022-12-18 */

#include <stm32f4xx.h>

#include <stdio.h>
#include <stdint.h>

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
struct UART_BUFFER U2Buf;  // Only UART2 and UART3 are useful on the STM32F4Discovery
struct UART_BUFFER U3Buf;

uint32_t SavedRccCsr = 0u;
volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;
volatile uint8_t RtcTick = 0;
volatile uint8_t Hour = 0;
volatile uint8_t Minute = 0;
volatile uint8_t Second = 0;


/* USART2_IRQHandler --- ISR for USART2, used for Rx and Tx */

void USART2_IRQHandler(void)
{
   if (USART2->SR & USART_SR_RXNE) {
      const uint8_t tmphead = (U2Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
      const uint8_t ch = USART2->DR;  // Read received byte from UART
      
      if (tmphead == U2Buf.rx.tail)   // Is receive buffer full?
      {
          // Buffer is full; discard new byte
      }
      else
      {
         U2Buf.rx.head = tmphead;
         U2Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
      }
   }
   
   if (USART2->SR & USART_SR_TXE) {
      if (U2Buf.tx.head != U2Buf.tx.tail) // Is there anything to send?
      {
         const uint8_t tmptail = (U2Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
         
         U2Buf.tx.tail = tmptail;

         USART2->DR = U2Buf.tx.buf[tmptail];    // Transmit one byte
      }
      else
      {
         USART2->CR1 &= ~USART_CR1_TXEIE; // Nothing left to send; disable Tx Empty interrupt
      }
   }
}


/* USART3_IRQHandler --- ISR for USART3, used for Rx and Tx */

void USART3_IRQHandler(void)
{
   if (USART3->SR & USART_SR_RXNE) {
      const uint8_t tmphead = (U3Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
      const uint8_t ch = USART3->DR;  // Read received byte from UART
      
      if (tmphead == U3Buf.rx.tail)   // Is receive buffer full?
      {
          // Buffer is full; discard new byte
      }
      else
      {
         U3Buf.rx.head = tmphead;
         U3Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
      }
   }
   
   if (USART3->SR & USART_SR_TXE) {
      if (U3Buf.tx.head != U3Buf.tx.tail) // Is there anything to send?
      {
         const uint8_t tmptail = (U3Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
         
         U3Buf.tx.tail = tmptail;

         USART3->DR = U3Buf.tx.buf[tmptail];    // Transmit one byte
      }
      else
      {
         USART3->CR1 &= ~USART_CR1_TXEIE; // Nothing left to send; disable Tx Empty interrupt
      }
   }
}


/* TIM7_IRQHandler --- ISR for TIM7, used for one-second real-time clock */

void TIM7_IRQHandler(void)
{
   TIM7->SR &= ~TIM_SR_UIF;   // Clear timer interrupt flag
   
   if (Second >= 59) {
      if (Minute >= 59) {
         if (Hour >= 23)
            Hour = 0;
         else
            Hour++;
            
         Minute = 0;
      }
      else
         Minute++;
      
      Second = 0;
   }
   else
      Second++;
   
   RtcTick = 1;
}


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


/* UART2RxByte --- read one character from UART2 via the circular buffer */

uint8_t UART2RxByte(void)
{
   const uint8_t tmptail = (U2Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U2Buf.rx.head == U2Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U2Buf.rx.tail = tmptail;
   
   return (U2Buf.rx.buf[tmptail]);
}


/* UART2RxAvailable --- return true if a byte is available in UART2 circular buffer */

int UART2RxAvailable(void)
{
   return (U2Buf.rx.head != U2Buf.rx.tail);
}


/* UART2TxByte --- send one character to UART2 via the circular buffer */

void UART2TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U2Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U2Buf.tx.tail)   // Wait, if buffer is full
       ;

   U2Buf.tx.buf[tmphead] = data;
   U2Buf.tx.head = tmphead;

   USART2->CR1 |= USART_CR1_TXEIE;   // Enable UART2 Tx Empty interrupt
}


/* UART3TxByte --- send one character to UART3 via the circular buffer */

void UART3TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U3Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U3Buf.tx.tail)   // Wait, if buffer is full
       ;

   U3Buf.tx.buf[tmphead] = data;
   U3Buf.tx.head = tmphead;

   USART3->CR1 |= USART_CR1_TXEIE;   // Enable UART3 Tx Empty interrupt
}


/* _write --- connect stdio functions to UART2 */

int _write(const int fd, const char *ptr, const int len)
{
   int i;

   for (i = 0; i < len; i++) {
      if (*ptr == '\n')
         UART2TxByte('\r');
      
      UART2TxByte(*ptr++);
   }
  
   return (len);
}


/* printDeviceID --- print the Device ID bytes as read from SCB and DBGMCU */

void printDeviceID(void)
{
   const uint16_t *const flashSize = (const uint16_t *const)FLASHSIZE_BASE;
   
   const uint32_t devId = (DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID_Msk) >> DBGMCU_IDCODE_DEV_ID_Pos;
   const uint32_t revId = (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos;
   const uint32_t cpuId = SCB->CPUID;
   
   printf("Device ID = %x rev %x (%08x)\n", devId, revId, DBGMCU->IDCODE);
   
   // CPUID in the System Control Block
   printf("CPUID =  %08x r%dp%d ", cpuId, ((cpuId >> 20) & 0x0F), (cpuId & 0x0F));
   
   switch ((cpuId & 0x0000FFF0) >> 4) {
   case 0xC20:
      printf("Cortex M0\n");
      break;
   case 0xC60:
      printf("Cortex M0+\n");
      break;
   case 0xC21:
      printf("Cortex M1\n");
      break;
   case 0xC23:
      printf("Cortex M3\n");
      break;
   case 0xC24:
      printf("Cortex M4\n");
      break;
   case 0xC27:
      printf("Cortex M7\n");
      break;
   default:
      printf("Unknown CORE\n");
      break;
   }
   
   printf("Flash size: %ukB\n", flashSize[0]);
}


/* printSerialNumber --- print the chip's unique serial number */

void printSerialNumber(void)
{
   // See STM32F103xx Reference Manual RM0008, section 30.2
   const uint32_t *const id = (const uint32_t *const)UID_BASE;
   
   printf("Serial Number = %08x %08x %08x\n", id[0], id[1], id[2]);
}


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
   printf("RCC_CSR = 0x%08x ", SavedRccCsr);
   
   if (SavedRccCsr & RCC_CSR_LPWRRSTF)
      printf("LPWRR ");
   
   if (SavedRccCsr & RCC_CSR_WWDGRSTF)
      printf("WWDGR ");
   
   if (SavedRccCsr & RCC_CSR_IWDGRSTF)
      printf("IWDGR ");
   
   if (SavedRccCsr & RCC_CSR_SFTRSTF)
      printf("SFTR ");
   
   if (SavedRccCsr & RCC_CSR_PORRSTF)
      printf("PORR ");
   
   if (SavedRccCsr & RCC_CSR_PINRSTF)
      printf("PINR ");
   
   printf("\n");
}


/* printRandomNumber --- print a 32-bit random number from the chip's harware RNG */

void printRandomNumber(void)
{
   uint32_t rnd;

   if (RNG->SR & RNG_SR_SEIS) {
      printf("RNG_SR_SEIS\n");
      return;
   }

   if (RNG->SR & RNG_SR_CEIS) {
      printf("RNG_SR_CEIS\n");
      return;
   }

   while ((RNG->SR & RNG_SR_DRDY) == 0)
      ;

   rnd = RNG->DR;

   printf("Random number is: 0x%08x\n", rnd);
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
   
   SavedRccCsr = RCC->CSR;
   RCC->CSR |= RCC_CSR_RMVF;
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
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        // Enable clock to GPIO A peripherals on AHB1 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;        // Enable clock to GPIO D peripherals on AHB1 bus
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;       // Enable USART2 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART3EN;       // Enable USART3 clock
   
   // UART1 TxD on pin PA9 is connected to the green LED LD7 and USB Vbus
   // UART1 RxD on pin PA10 is connected to USB ID
   
   // Set up UART2 and associated circular buffers
   U2Buf.tx.head = 0;
   U2Buf.tx.tail = 0;
   U2Buf.rx.head = 0;
   U2Buf.rx.tail = 0;

   // Configure PA2, the GPIO pin with alternative function TxD2
   GPIOA->MODER |= GPIO_MODER_MODER2_1;        // PA2 in Alternative Function mode
   GPIOA->AFR[0] |= 7 << 8;                    // Configure PA2 as alternate function, AF7, UART2
  
   // Configure PA3, the GPIO pin with alternative function RxD2
   GPIOA->MODER |= GPIO_MODER_MODER3_1;        // PA3 in Alternative Function mode
   GPIOA->AFR[0] |= 7 << 12;                   // Configure PA3 as alternate function, AF7, UART2
  
   // Select high speed for UART2 pins
   GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;
  
   // Configure UART2 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART2->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART2->BRR |= (273<<4) | 7;           // Set for 9600 baud (reference manual page 1010) 42000000 / (16 * 9600)
   USART2->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART2->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART2->CR1 |= USART_CR1_RE;           // Enable receiver

   NVIC_EnableIRQ(USART2_IRQn);
      
   // Set up UART3 and associated circular buffers
   U3Buf.tx.head = 0;
   U3Buf.tx.tail = 0;
   U3Buf.rx.head = 0;
   U3Buf.rx.tail = 0;
      
   // Configure PD8, the GPIO pin with alternative function TxD3
   GPIOD->MODER |= GPIO_MODER_MODER8_1;        // PD8 in Alternative Function mode
   GPIOD->AFR[1] |= 7 << 0;                    // Configure PD8 as alternate function, AF7, UART3
   
   // Configure PD9, the GPIO pin with alternative function RxD3
   GPIOD->MODER |= GPIO_MODER_MODER9_1;        // PD9 in Alternative Function mode
   GPIOD->AFR[1] |= 7 << 4;                    // Configure PD9 as alternate function, AF7, UART3
   
   // Configure UART3 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART3->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART3->BRR |= (273<<4) | 7;           // Set for 9600 baud (reference manual page 1010) 42000000 / (16 * 9600)
   USART3->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART3->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART3->CR1 |= USART_CR1_RE;           // Enable receiver

   NVIC_EnableIRQ(USART3_IRQn);
}


/* initTimers --- set up timer for regular 1Hz interrupts */

static void initTimers(void)
{
   // The two Basic Timers, 6 and 7, are sufficient to generate regular
   // interrupts. They have a 16-bit prescaler and a 16-bit counter register
   
   RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;        // Enable Timer 7 clock
   
   TIM7->CR1 = 0;               // Start with default CR1 and CR2
   TIM7->CR2 = 0;
   TIM7->PSC = 8400 - 1;        // Prescaler: 84MHz, divide-by-8400 to give 10kHz
   TIM7->ARR = 10000 - 1;       // Auto-reload: 10000 to give interrupts at 1Hz
   TIM7->CNT = 0;               // Counter: 0
   TIM7->DIER |= TIM_DIER_UIE;  // Enable interrupt
   TIM7->CR1 |= TIM_CR1_CEN;    // Enable counter
   
   NVIC_EnableIRQ(TIM7_IRQn);
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
}


/* initRNG --- set up the random number generator */

static void initRNG(void)
{
   RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;      // Enable clock to random number generator

   RNG->CR |= RNG_CR_RNGEN;            // Enable the random number generator
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
   uint32_t end;
   uint8_t flag = 0;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initTimers();
   initPWM();
   initRNG();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   printf("\nHello from the STM%dF%d\n", 32, 407);
   
   while (1) {
      if (Tick) {
         if (millis() >= end) {
            end = millis() + 500u;
            
            if (flag) {
               GPIOD->BSRR = GPIO_BSRR_BR12; // GPIO pin PD12 LOW, green LED off
               GPIOD->BSRR = GPIO_BSRR_BS13; // GPIO pin PD13 HIGH, amber LED on
               GPIOD->BSRR = GPIO_BSRR_BR14; // GPIO pin PD14 LOW, red LED off
               GPIOD->BSRR = GPIO_BSRR_BS15; // GPIO pin PD15 HIGH, blue LED on
            }
            else {
               GPIOD->BSRR = GPIO_BSRR_BS12; // GPIO pin PD12 HIGH, green LED on
               GPIOD->BSRR = GPIO_BSRR_BR13; // GPIO pin PD13 LOW, amber LED off
               GPIOD->BSRR = GPIO_BSRR_BS14; // GPIO pin PD14 HIGH, red LED on
               GPIOD->BSRR = GPIO_BSRR_BR15; // GPIO pin PD15 LOW, blue LED off
            }
            
            flag = !flag;
            
            UART3TxByte('U');
            UART3TxByte('3');
            UART3TxByte(' ');
            UART3TxByte('S');
            UART3TxByte('T');
            UART3TxByte('M');
            UART3TxByte('3');
            UART3TxByte('2');
            UART3TxByte(' ');
            
            printf("millis() = %ld\n", millis());
         }
         
         Tick = 0;
      }
      
      if (RtcTick) {
         printf("RTC: %02d:%02d:%02d\n", Hour, Minute, Second);
         
         RtcTick = 0;
      }
      
      if (UART2RxAvailable()) {
         const uint8_t ch = UART2RxByte();
         
         printf("UART2: %02x\n", ch);
         switch (ch) {
         case 'i':
         case 'I':
            printDeviceID();
            break;
         case 'n':
         case 'N':
            printSerialNumber();
            break;
         case 'r':
         case 'R':
            printResetReason();
            break;
         case '#':
            printRandomNumber();
            break;
         }
      }
   }
}

