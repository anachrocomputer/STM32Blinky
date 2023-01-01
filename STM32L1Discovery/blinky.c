/* blinky --- blink LEDs on STM32L-Discovery board          2023-01-01 */

#include "stm32l1xx.h"

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
      GPIOC->BSRR = GPIO_BSRR_BR_13; // GPIO pin PC13 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS_13; // GPIO pin PC13 HIGH
   
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
   // See STM32L152xx Reference Manual RM0038, section 31.2
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
   
   RCC->APB1ENR |= RCC_APB1ENR_PWREN;      // Enable clock to PWR
  
   // Reset PWR so that it's ready to accept DBP bit
   RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST;
   RCC->APB1RSTR &= ~(RCC_APB1RSTR_PWRRST);

   // Wait for Voltage Scaling Select Flag to be cleared
   while ((PWR->CSR & (PWR_CSR_VOSF)) != 0)
      ;
    
   PWR->CR = PWR_CR_VOS_0;    // VOS = range 1 (1.8V)
  
   // Wait for Voltage Scaling Select Flag to be cleared
   while ((PWR->CSR & (PWR_CSR_VOSF)) != 0)
      ;
   
   // Set up Flash wait state
   FLASH->ACR |= FLASH_ACR_ACC64;     // Enable 64-bit mode (must be set separately from LATENCY and PRFTEN bits)
  
   while ((FLASH->ACR & (FLASH_ACR_ACC64)) == 0)
      ;
  
   FLASH->ACR |= FLASH_ACR_PRFTEN;    // Enable pre-fetch
  
   while ((FLASH->ACR & (FLASH_ACR_PRFTEN)) == 0)
      ;
  
   FLASH->ACR |= FLASH_ACR_LATENCY;   // Select one wait state
  
   while ((FLASH->ACR & (FLASH_ACR_LATENCY)) == 0)
      ;
   
   // Set up PLL to give us 32MHz
   RCC->CFGR |= RCC_CFGR_PLLMUL4;     // PLLMUL x4 = (16Mhz x 4) = 64MHz
   RCC->CFGR |= RCC_CFGR_PLLDIV2;     // PLLDIV /2 = (64MHz / 2) = 32MHz
   
   RCC->CR |= RCC_CR_PLLON;           // Enable PLL
   
   while ((RCC->CR & (RCC_CR_PLLRDY)) == 0)   // Wait for PLL to become ready
      ;
   
   RCC->CFGR |= RCC_CFGR_SW_PLL;    // Switch to PLL clock source
   
   while ((RCC->CFGR & (RCC_CFGR_SWS)) != RCC_CFGR_SWS_PLL)   // Wait for PLL to be selected
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
   
   // Configure PC13, the GPIO pin with 500Hz square wave
   GPIOC->MODER |= GPIO_MODER_MODER13_0;    // Configure PC13 as output
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;        // Enable clock to GPIO A peripherals on AHB bus
   RCC->AHBENR |= RCC_AHBENR_GPIOBEN;        // Enable clock to GPIO B peripherals on AHB bus
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;     // Enable USART1 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;     // Enable USART2 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART3EN;     // Enable USART3 clock
   
   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;
   
   // Configure PA9, the GPIO pin with alternative function TxD1
   GPIOA->MODER |= GPIO_MODER_MODER9_1;          // PA9 in Alternative Function mode
   GPIOA->AFR[1] |= 7 << 4;                      // Configure PA9 as alternate function, AF7, UART1
   
   // Configure PA10, the GPIO pin with alternative function RxD1
   GPIOA->MODER |= GPIO_MODER_MODER10_1;         // PA10 in Alternative Function mode
   GPIOA->AFR[1] |= 7 << 8;                      // Configure PA10 as alternate function, AF7, UART1
   
   // Select high speed for UART1 pins
   GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR10_1;
   
   // Configure UART1 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART1->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= 0xD06;                  // Set for 9600 baud (reference manual page 736)
   USART1->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART1->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART1->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART1_IRQn);
   
   // Set up UART2 and associated circular buffers
   U2Buf.tx.head = 0;
   U2Buf.tx.tail = 0;
   U2Buf.rx.head = 0;
   U2Buf.rx.tail = 0;
   
   // Configure PA2, the GPIO pin with alternative function TxD2
   GPIOA->MODER |= GPIO_MODER_MODER2_1;          // PA2 in Alternative Function mode
   GPIOA->AFR[0] |= 7 << 8;                      // Configure PA2 as alternate function, AF7, UART2
   
   // Configure PA3, the GPIO pin with alternative function RxD2
   GPIOA->MODER |= GPIO_MODER_MODER3_1;         // PA3 in Alternative Function mode
   GPIOA->AFR[0] |= 7 << 12;                    // Configure PA3 as alternate function, AF7, UART2
   
   // Select high speed for UART2 pins
   GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;
   
   // Configure UART2 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART2->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART2->BRR |= 0xD06;                  // Set for 9600 baud (reference manual page 736)
   USART2->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART2->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART2->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART2_IRQn);
   
   // Set up UART3 and associated circular buffers
   U3Buf.tx.head = 0;
   U3Buf.tx.tail = 0;
   U3Buf.rx.head = 0;
   U3Buf.rx.tail = 0;
   
   // Configure PB10, the GPIO pin with alternative function TxD3
   GPIOB->MODER |= GPIO_MODER_MODER10_1;         // PB10 in Alternative Function mode
   GPIOB->AFR[1] |= 7 << 8;                      // Configure PB10 as alternate function, AF7, UART3
   
   // Configure PB11, the GPIO pin with alternative function RxD3
   GPIOB->MODER |= GPIO_MODER_MODER11_1;        // PB11 in Alternative Function mode
   GPIOB->AFR[1] |= 7 << 12;                    // Configure PB11 as alternate function, AF7, UART3
   
   // Select high speed for UART3 pins
   GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR11_1;
   
   // Configure UART3 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART3->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART3->BRR |= 0xD06;                  // Set for 9600 baud (reference manual page 736)
   USART3->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART3->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART3->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART3_IRQn);
}


/* initDAC --- set up the Digital-to-Analog Converter */

static void initDAC(void)
{
   // Channel 0 is on PA4 and channel 1 is on PA5
   // PA4 is in use on STM32L-Discovery for Idd sensing
   
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;        // Enable clock to GPIO A peripherals on AHB bus
   RCC->APB1ENR |= RCC_APB1ENR_DACEN;        // Enable DAC clock
   
   //GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER4_0;    // Mode bits 11 on PA4 for DAC analog out
   GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER5_0;    // Mode bits 11 on PA5 for DAC analog out
   
   //DAC->CR |= DAC_CR_EN1;	// Switch the DAC channel 1 on
   DAC->CR |= DAC_CR_EN2;	// Switch the DAC channel 2 on
   
   //DAC->CR |= DAC_CR_BOFF1;  // Switch off buffer amp to remove clipping
   DAC->CR |= DAC_CR_BOFF2;  // Switch off buffer amp to remove clipping
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
   if (SysTick_Config(32000)) { // 32MHz divided by 1000  (SystemCoreClock / 1000)
      while (1)
         ;
   }
}


int main(void)
{
   uint32_t end;
   uint8_t flag = 0;
   uint16_t dac = 0;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initDAC();
   initPWM();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   printf("\nHello from the STM%dL%d\n", 32, 152);
   
   end = millis() + 500u;
   
   DAC->DHR12R2 = 0x0;
   
   while (1) {
      if (Tick) {
         DAC->DHR12R2 = dac;  // Generate a voltage ramp on PA5
         
         if (dac >= 4095)
            dac = 0;
         else
            dac++;
         
         if (millis() >= end) {
            end = millis() + 500u;
            
            if (flag) {
               GPIOB->BSRR = GPIO_BSRR_BS_6;        // Blue led ON
               GPIOB->BSRR = GPIO_BSRR_BR_7;        // Green led OFF
            }
            else {
               GPIOB->BSRR = GPIO_BSRR_BR_6;        // Blue led OFF
               GPIOB->BSRR = GPIO_BSRR_BS_7;        // Green led ON
            }
            
            flag = !flag;
            
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
            
            printf("millis() = %ld\n", millis());
         }
         
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
         }
      }
   }
}
