/* blinky --- blink LED on PC13 of Black Pill STM32 board   2023-01-22 */

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
struct UART_BUFFER U1Buf;
struct UART_BUFFER U2Buf;
struct UART_BUFFER U3Buf;

uint32_t SavedRccCsr = 0u;
volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;


/* USART1_IRQHandler --- ISR for USART1, used for Rx and Tx */

void USART1_IRQHandler(void)
{
   if (USART1->SR & USART_SR_RXNE) {
      const uint8_t tmphead = (U1Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
      const uint8_t ch = USART1->DR;  // Read received byte from UART
      
      if (tmphead == U1Buf.rx.tail)   // Is receive buffer full?
      {
          // Buffer is full; discard new byte
      }
      else
      {
         U1Buf.rx.head = tmphead;
         U1Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
      }
   }
   
   if (USART1->SR & USART_SR_TXE) {
      if (U1Buf.tx.head != U1Buf.tx.tail) // Is there anything to send?
      {
         const uint8_t tmptail = (U1Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
         
         U1Buf.tx.tail = tmptail;

         USART1->DR = U1Buf.tx.buf[tmptail];    // Transmit one byte
      }
      else
      {
         USART1->CR1 &= ~USART_CR1_TXEIE; // Nothing left to send; disable Tx Empty interrupt
      }
   }
}


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


/* USART6_IRQHandler --- ISR for USART6, used for Rx and Tx */

void USART6_IRQHandler(void)
{
   if (USART6->SR & USART_SR_RXNE) {
      const uint8_t tmphead = (U3Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
      const uint8_t ch = USART6->DR;  // Read received byte from UART
      
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
   
   if (USART6->SR & USART_SR_TXE) {
      if (U3Buf.tx.head != U3Buf.tx.tail) // Is there anything to send?
      {
         const uint8_t tmptail = (U3Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
         
         U3Buf.tx.tail = tmptail;

         USART6->DR = U3Buf.tx.buf[tmptail];    // Transmit one byte
      }
      else
      {
         USART6->CR1 &= ~USART_CR1_TXEIE; // Nothing left to send; disable Tx Empty interrupt
      }
   }
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
   
   // DEBUG: 500Hz on PC14 pin
   if (flag)
      GPIOC->BSRR = GPIO_BSRR_BR14; // GPIO pin PC14 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS14; // GPIO pin PC14 HIGH
      
   flag = !flag;
}


/* UART1RxByte --- read one character from UART1 via the circular buffer */

uint8_t UART1RxByte(void)
{
   const uint8_t tmptail = (U1Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U1Buf.rx.head == U1Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U1Buf.rx.tail = tmptail;
   
   return (U1Buf.rx.buf[tmptail]);
}


/* UART1RxAvailable --- return true if a byte is available in UART1 circular buffer */

int UART1RxAvailable(void)
{
   return (U1Buf.rx.head != U1Buf.rx.tail);
}


/* UART1TxByte --- send one character to UART1 via the circular buffer */

void UART1TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U1Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U1Buf.tx.tail)   // Wait, if buffer is full
       ;

   U1Buf.tx.buf[tmphead] = data;
   U1Buf.tx.head = tmphead;

   USART1->CR1 |= USART_CR1_TXEIE;   // Enable UART1 Tx Empty interrupt
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


/* UART6TxByte --- send one character to UART6 via the circular buffer */

void UART6TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U3Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U3Buf.tx.tail)   // Wait, if buffer is full
       ;

   U3Buf.tx.buf[tmphead] = data;
   U3Buf.tx.head = tmphead;

   USART6->CR1 |= USART_CR1_TXEIE;   // Enable UART6 Tx Empty interrupt
}


/* _write --- connect stdio functions to UART1 */

int _write(const int fd, const char *ptr, const int len)
{
   int i;

   for (i = 0; i < len; i++) {
      if (*ptr == '\n')
         UART1TxByte('\r');
      
      UART1TxByte(*ptr++);
   }
  
   return (len);
}


/* setRGBLed --- control two RGB LEDs connected to PORT B and PWM */

void setRGBLed(const int state, const uint8_t fade)
{
   switch (state) {
   case 0:                    // Red fading up, blue on
      TIM3->CCR1 = fade;
      TIM3->CCR2 = 0;
      TIM3->CCR3 = 255;
      GPIOB->BSRR = GPIO_BSRR_BS12; // GPIO pin PB12 HIGH, red LED on
      GPIOB->BSRR = GPIO_BSRR_BR13; // GPIO pin PB13 LOW, green LED off
      GPIOB->BSRR = GPIO_BSRR_BR14; // GPIO pin PB14 LOW, blue LED off
      break;
   case 1:                    // Red on, blue fading down
      TIM3->CCR1 = 255;
      TIM3->CCR2 = 0;
      TIM3->CCR3 = 255 - fade;
      GPIOB->BSRR = GPIO_BSRR_BS12; // GPIO pin PB12 HIGH, red LED on
      GPIOB->BSRR = GPIO_BSRR_BS13; // GPIO pin PB13 HIGH, green LED on
      GPIOB->BSRR = GPIO_BSRR_BR14; // GPIO pin PB14 LOW, blue LED off
      break;
   case 2:                    // Red on, green fading up
      TIM3->CCR1 = 255;
      TIM3->CCR2 = fade;
      TIM3->CCR3 = 0;
      GPIOB->BSRR = GPIO_BSRR_BR12; // GPIO pin PB12 LOW, red LED off
      GPIOB->BSRR = GPIO_BSRR_BS13; // GPIO pin PB13 HIGH, green LED on
      GPIOB->BSRR = GPIO_BSRR_BR14; // GPIO pin PB14 LOW, blue LED off
      break;
   case 3:                    // Red fading down, green on
      TIM3->CCR1 = 255 - fade;
      TIM3->CCR2 = 255;
      TIM3->CCR3 = 0;
      GPIOB->BSRR = GPIO_BSRR_BR12; // GPIO pin PB12 LOW, red LED off
      GPIOB->BSRR = GPIO_BSRR_BS13; // GPIO pin PB13 HIGH, green LED on
      GPIOB->BSRR = GPIO_BSRR_BS14; // GPIO pin PB14 HIGH, blue LED on
      break;
   case 4:                    // Green on, blue fading up
      TIM3->CCR1 = 0;
      TIM3->CCR2 = 255;
      TIM3->CCR3 = fade;
      GPIOB->BSRR = GPIO_BSRR_BR12; // GPIO pin PB12 LOW, red LED off
      GPIOB->BSRR = GPIO_BSRR_BR13; // GPIO pin PB13 LOW, green LED off
      GPIOB->BSRR = GPIO_BSRR_BS14; // GPIO pin PB14 HIGH, blue LED on
      break;
   case 5:                    // Green fading down, blue on
      TIM3->CCR1 = 0;
      TIM3->CCR2 = 255 - fade;
      TIM3->CCR3 = 255;
      GPIOB->BSRR = GPIO_BSRR_BS12; // GPIO pin PB12 HIGH, red LED on
      GPIOB->BSRR = GPIO_BSRR_BR13; // GPIO pin PB13 LOW, green LED off
      GPIOB->BSRR = GPIO_BSRR_BS14; // GPIO pin PB14 HIGH, blue LED on
      break;
   }
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


/* softwareReset --- reset the microcontroller */

void softwareReset(void)
{
   printf("Software RESET...\n");
   
   // Delay about 40ms so that the message can get out
   uint32_t end = millis() + 40u;
   
   while (millis() < end)
      ;
   
   NVIC_SystemReset();
}


/* testWatchdog --- enter an infinite loop to test the watchdog */

void testWatchdog(void)
{
   int second;
   
   printf("Watchdog test...\n");
   
   for (second = 0; ; second++) {
      printf("%d ", second);
      fflush(stdout);
      
      uint32_t end = millis() + 1000u;
      
      while (millis() < end)
         ;
   }
}


/* nudgeWatchdog --- reset the watchdog counter */

void nudgeWatchdog(void)
{
   IWDG->KR = (uint16_t)0xAAAA;
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   RCC->CR      = 0x00000081;
   RCC->CFGR    = 0x00000000;
   RCC->PLLCFGR = 0x24003010;
   
   FLASH->ACR |= FLASH_ACR_LATENCY_2WS;   // Set Flash latency to 2 wait states
   FLASH->ACR |= FLASH_ACR_ICEN;          // Cache enable
   FLASH->ACR |= FLASH_ACR_DCEN;
   FLASH->ACR |= FLASH_ACR_PRFTEN;        // Prefetch enable
   
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // Set APB1 bus to not exceed 50MHz
   //RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;    // Set APB2 bus to not exceed 100MHz
   
#if 0
   // Switch on MCO1 for debugging
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;         // Enable clock to GPIO A peripherals on AHB1 bus

   // Configure PA8, the GPIO pin with alternative function MCO1
   GPIOA->MODER |= GPIO_MODER_MODER8_1;          // PA8 in Alternative Function mode
   GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR8_0;

   RCC->CFGR |= RCC_CFGR_MCO1_1 | RCC_CFGR_MCO1_0;                 // Send PLL to MCO1
   RCC->CFGR |= RCC_CFGR_MCO1PRE_0 | RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2; // Prescale divide-by-5
#endif
   
   RCC->CR |= RCC_CR_HSEON;    // Switch on High Speed External clock (25MHz on the Black Pill)

   // Wait for HSE to start up
   while ((RCC->CR & RCC_CR_HSERDY) == 0)
      ;
   
   // Configure PLL
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_0;   // Set Q to divide-by 7

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;            // P = divide-by-2

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
   RCC->PLLCFGR |= 200 << RCC_PLLCFGR_PLLN_Pos;    // N = multiply-by-200

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
   RCC->PLLCFGR |= 25 << RCC_PLLCFGR_PLLM_Pos;     // M = divide-by-25 for 1MHz PLL input

   RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;    // Select HSE as PLL input
   
   RCC->CR |= RCC_CR_PLLON;             // Switch on the PLL

   // Wait for PLL to start up
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
      ;

   uint32_t reg = RCC->CFGR;
   reg &= ~RCC_CFGR_SW;
   reg |= RCC_CFGR_SW_PLL;          // Select PLL as system clock (100MHz)
   RCC->CFGR = reg;
   
   // Wait for PLL to select
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
      ;
   
   SavedRccCsr = RCC->CSR;
   RCC->CSR |= RCC_CSR_RMVF;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;      // Enable clock to GPIO B peripherals on AHB1 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;      // Enable clock to GPIO C peripherals on AHB1 bus
   
   // Configure PB12, the GPIO pin with the red LED
   GPIOB->MODER |= GPIO_MODER_MODER12_0;     // Configure PB12 as output for the red LED
   
   // Configure PB13, the GPIO pin with the green LED
   GPIOB->MODER |= GPIO_MODER_MODER13_0;     // Configure PB13 as output for the green LED
   
   // Configure PB14, the GPIO pin with the blue LED
   GPIOB->MODER |= GPIO_MODER_MODER14_0;     // Configure PB14 as output for the blue LED
   
   // Configure PC13, the GPIO pin with the on-board blue LED
   GPIOC->MODER |= GPIO_MODER_MODER13_0;     // Configure PC13 as output for the on-board LED
   
   // Configure PC14, the GPIO pin with 500Hz square wave
   GPIOC->MODER |= GPIO_MODER_MODER14_0;     // Configure PC14 as output, 500Hz square wave
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        // Enable clock to GPIO A peripherals on AHB1 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;        // Enable clock to GPIO D peripherals on AHB1 bus
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;       // Enable USART1 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;       // Enable USART2 clock
   RCC->APB2ENR |= RCC_APB2ENR_USART6EN;       // Enable USART6 clock
   
   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;
   
   // Configure PA9, the GPIO pin with alternative function TxD2
   GPIOA->MODER |= GPIO_MODER_MODER9_1;        // PA9 in Alternative Function mode
   GPIOA->AFR[1] |= 7 << 4;                    // Configure PA9 as alternate function, AF7, UART1
  
   // Configure PA10, the GPIO pin with alternative function RxD2
   GPIOA->MODER |= GPIO_MODER_MODER10_1;       // PA10 in Alternative Function mode
   GPIOA->AFR[1] |= 7 << 8;                    // Configure PA10 as alternate function, AF7, UART1
   
   // Configure UART1 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART1->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= (651<<4) | 1;           // Set for 9600 baud (reference manual page 518) 100000000 / (16 * 9600)
   USART1->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART1->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART1->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART1_IRQn);
   
   // Set up UART2 and associated circular buffers
   U2Buf.tx.head = 0;
   U2Buf.tx.tail = 0;
   U2Buf.rx.head = 0;
   U2Buf.rx.tail = 0;
#if 0
   // Configure PA2, the GPIO pin with alternative function TxD2
   GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);       // Clear configuration bits for PA2
   GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1;     // Configure PA2 as alternate function, push-pull
   
   // Configure PA3, the GPIO pin with alternative function RxD2
   GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);       // Clear configuration bits for PA3
   GPIOA->CRL |= GPIO_CRL_CNF3_1;                         // Configure PA3 as alternate function, floating input
#endif
   // Configure UART2 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART2->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART2->BRR |= (234<<4) | 6;           // Set for 9600 baud (reference manual page 799) 36000000 / (16 * 9600)
   USART2->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART2->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART2->CR1 |= USART_CR1_RE;           // Enable receiver

   //NVIC_EnableIRQ(USART2_IRQn);
   
   // Set up UART6 and associated circular buffers
   U3Buf.tx.head = 0;
   U3Buf.tx.tail = 0;
   U3Buf.rx.head = 0;
   U3Buf.rx.tail = 0;
#if 0
   // Configure PB10, the GPIO pin with alternative function TxD6
   GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // Clear configuration bits for PB10
   GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1;  // Configure PB10 as alternate function, push-pull
   
   // Configure PB11, the GPIO pin with alternative function RxD6
   GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);     // Clear configuration bits for PB11
   GPIOB->CRH |= GPIO_CRH_CNF11_1;                        // Configure PB11 as alternate function, floating input
#endif
// Configure UART6 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART6->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= (651<<4) | 1;           // Set for 9600 baud (reference manual page 518) 100000000 / (16 * 9600)
   USART6->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART6->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART6->CR1 |= USART_CR1_RE;           // Enable receiver
   
   //NVIC_EnableIRQ(USART6_IRQn);
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
#if 0
   // Configure Reset and Clock Control
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                    // Enable clock to TIM3 peripheral on APB1 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                    // Enable clock to GPIO A peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   
   // Configure PA6, the GPIO pin with alternative function T3C1
   GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);       // Clear configuration bits for PA6
   GPIOA->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1;     // Configure PA6 as alternate function, push-pull
   
   // Configure PA7, the GPIO pin with alternative function T3C2
   GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);       // Clear configuration bits for PA7
   GPIOA->CRL |= GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1;     // Configure PA6 as alternate function, push-pull
   
   // Configure PB0, the GPIO pin with alternative function T3C3
   GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);       // Clear configuration bits for PB0
   GPIOB->CRL |= GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1;     // Configure PB0 as alternate function, push-pull
   
   // Don't configure PB1, the GPIO pin with alternative function T3C4, not needed
#endif
   // Configure Timer 3 for triple PWM generation
   TIM3->CCMR1 =  TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // Timer output compare mode PWM for T3C1
   TIM3->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // Timer output compare mode PWM for T3C2
   TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // Timer output compare mode PWM for T3C3
   TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable counter, auto-reload to 0 at the end of a count
   TIM3->ARR = 0xFF;              // Auto-Reload Register: count to 255 (no point counting higher)
   TIM3->CCR1 = 0;                // Set T3C1 PWM to zero (0% duty cycle)
   TIM3->CCR2 = 0;                // Set T3C2 PWM to zero (0% duty cycle)
   TIM3->CCR3 = 0;                // Set T3C3 PWM to zero (0% duty cycle)
   TIM3->EGR |= TIM_EGR_UG;       // Event Generation Register: update generation ON
   TIM3->CCER |= TIM_CCER_CC1E;   // Output T3C1 enabled
   TIM3->CCER |= TIM_CCER_CC2E;   // Output T3C2 enabled
   TIM3->CCER |= TIM_CCER_CC3E;   // Output T3C3 enabled
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
   if (SysTick_Config(100000)) { // 100MHz divided by 1000  (SystemCoreClock / 1000)
      while (1)
         ;
   }
}


/* initWatchdog --- set up the watchdog timer */

static void initWatchdog(void)
{
   IWDG->KR = (uint16_t)0x5555;
   IWDG->PR = IWDG_PR_PR_2;      // Prescaler divide-by-64 gives 8 seconds
   IWDG->KR = (uint16_t)0xAAAA;
   
   IWDG->KR = (uint16_t)0xCCCC;  // Start the watchdog
}


int main(void)
{
   int ledState = 0;
   uint8_t fade = 0;
   uint32_t end;
   uint8_t flag = 0;
   
   initMCU();
   initGPIOs();
   initUARTs();
   //initPWM();
   initMillisecondTimer();
   initWatchdog();
   
   __enable_irq();   // Enable all interrupts

   printf("\nHello from the STM%dF%d\n", 32, 411);
   
   printResetReason();
   printDeviceID();
   printSerialNumber();
   
   end = millis() + 500u;
   
   while (1) {
      if (Tick) {
         if (fade == 255) {
            fade = 0;

            if (ledState == 5)
               ledState = 0;
            else
               ledState++;
         }
         else
            fade++;
            
         //setRGBLed(ledState, fade);
         
         if (millis() >= end) {
            end = millis() + 500u;
            
            if (flag) {
               GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW, LED on
            }
            else {
               GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH, LED off
            }
            
            flag = !flag;
#if 0 
            UART2TxByte('U');
            UART2TxByte('2');
            UART2TxByte(' ');
            UART2TxByte('S');
            UART2TxByte('T');
            UART2TxByte('M');
            UART2TxByte('3');
            UART2TxByte('2');
            UART2TxByte(' ');

            UART3TxByte('U');
            UART3TxByte('3');
            UART3TxByte(' ');
            UART3TxByte('S');
            UART3TxByte('T');
            UART3TxByte('M');
            UART3TxByte('3');
            UART3TxByte('2');
            UART3TxByte(' ');
#endif
            printf("millis() = %ld\n", millis());
         }
         
         nudgeWatchdog();
         Tick = 0;
      }
      
      if (UART1RxAvailable()) {
         const uint8_t ch = UART1RxByte();
         
         printf("UART1: %02x\n", ch);
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
         case 's':
         case 'S':
            softwareReset();
            break;
         case 'w':
         case 'W':
            testWatchdog();
            break;
         }
      }
   }
}

