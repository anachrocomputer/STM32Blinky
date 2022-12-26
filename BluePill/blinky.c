/* blinky --- blink LED on PC13 of Blue Pill STM32 board    2022-12-17 */

#include <stm32f1xx.h>

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
   
   // Check Tx interrupt here
   if (USART1->SR & USART_SR_TXE) {
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


/* t1ou --- send a single character on UART1 by polling */

static void t1ou(const int ch)
{
   // Wait for TX empty
   while ((USART1->SR & USART_SR_TXE) == 0)
      ;
  
   // Send byte
   USART1->DR = ch;
}


/* t1ou2 --- send a single character on UART2 by polling */

static void t1ou2(const int ch)
{
   // Wait for TX empty
   while ((USART2->SR & USART_SR_TXE) == 0)
      ;
  
   // Send byte
   USART2->DR = ch;
}


/* t1ou3 --- send a single character on UART3 by polling */

static void t1ou3(const int ch)
{
   // Wait for TX empty
   while ((USART3->SR & USART_SR_TXE) == 0)
      ;
  
   // Send byte
   USART3->DR = ch;
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


/* _write --- connect stdio functions to UART1 */

int _write(const int fd, const char *ptr, const int len)
{
   int i;

   for (i = 0; i < len; i++) {
      if (*ptr == '\n')
         t1ou('\r');
      
      t1ou(*ptr++);
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
}


/* printSerialNumber --- print the chip's unique serial number */

void printSerialNumber(void)
{
   // See STM32F103xx Reference Manual RM0008, section 30.2
   uint32_t *id = (uint32_t *)0x1FFFF7E8;
   
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
   FLASH->ACR |= FLASH_ACR_LATENCY_2;  // Set Flash latency
   
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // Set APB1 bus to not exceed 36MHz
   
   RCC->CR |= RCC_CR_HSEON;   // Switch on High Speed External clock (8MHz on the Blue Pill)
   
   // Wait for HSE to start up
   while ((RCC->CR & RCC_CR_HSERDY) == 0)
      ;
   
   RCC->CFGR |= RCC_CFGR_PLLSRC;       // Select HSE as input to PLL
   RCC->CFGR |= RCC_CFGR_PLLMULL9;     // Select multiply-by-9 to go from 8MHz to 72MHz
   
   RCC->CR |= RCC_CR_PLLON;            // Switch on PLL
   
   // Wait for PLL to start up
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
      ;
   
   RCC->CFGR |= RCC_CFGR_SW_PLL;       // Select PLL as system clock (72MHz)
   
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
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                    // Enable clock to GPIO C peripherals on APB2 bus
   
   // Configure PB12, the GPIO pin with the red LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);     // Set PB12 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1); // Configure PB12 as output, 50MHz
   
   // Configure PB13, the GPIO pin with the green LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // Set PB13 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1); // Configure PB13 as output, 50MHz
   
   // Configure PB14, the GPIO pin with the blue LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);     // Set PB14 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1); // Configure PB14 as output, 50MHz
   
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
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                    // Enable clock to GPIO A peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                  // Enable USART1 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;                  // Enable USART2 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART3EN;                  // Enable USART3 clock
   
   // Configure PA9, the GPIO pin with alternative function TxD1
   GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);       // Clear configuration bits for PA9
   GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;     // Configure PA9 as alternate function, push-pull
   
   // Configure PA10, the GPIO pin with alternative function RxD1
   GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // Clear configuration bits for PA10
   GPIOA->CRH |= GPIO_CRH_CNF10_1;                        // Configure PA10 as alternate function, floating input
   
   // Configure PA2, the GPIO pin with alternative function TxD2
   GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);       // Clear configuration bits for PA2
   GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1;     // Configure PA2 as alternate function, push-pull
   
   // Configure PA3, the GPIO pin with alternative function RxD2
   GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);       // Clear configuration bits for PA3
   GPIOA->CRL |= GPIO_CRL_CNF3_1;                         // Configure PA3 as alternate function, floating input
   
   // Configure PB10, the GPIO pin with alternative function TxD3
   GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // Clear configuration bits for PB10
   GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1;  // Configure PB10 as alternate function, push-pull
   
   // Configure PB11, the GPIO pin with alternative function RxD3
   GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);     // Clear configuration bits for PB11
   GPIOB->CRH |= GPIO_CRH_CNF11_1;                        // Configure PB11 as alternate function, floating input
   
   // Configure UART1 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART1->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= (467<<4) | 12;          // Set for 9600 baud (reference manual page 799) 72000000 / (16 * 9600)
   USART1->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART1->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART1->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART1_IRQn);
   
   // Configure UART2 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART2->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART2->BRR |= (234<<4) | 6;           // Set for 9600 baud (reference manual page 799) 36000000 / (16 * 9600)
   USART2->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
// USART2->CR1 |= USART_CR1_RE;           // Enable receiver

// Configure UART3 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART3->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART3->BRR |= (234<<4) | 6;           // Set for 9600 baud (reference manual page 799) 36000000 / (16 * 9600)
   USART3->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
// USART3->CR1 |= USART_CR1_RE;           // Enable receiver
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
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
   if (SysTick_Config(72000)) { // 72MHz divided by 1000  (SystemCoreClock / 1000)
      while (1)
         ;
   }
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
   initPWM();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   printf("\nHello from the STM%dF%d\n", 32, 103);
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
            
         setRGBLed(ledState, fade);
         
         if (millis() >= end) {
            end = millis() + 500u;
            
            if (flag) {
               GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW, LED on
               
               t1ou2('A');
               t1ou3('X');
            }
            else {
               GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH, LED off
               
               t1ou2('B');
               t1ou3('Y');
            }
            
            flag = !flag;
            
            //UART1TxByte('U');
            //UART1TxByte('1');
            //UART1TxByte(' ');
            //UART1TxByte('4');
            //UART1TxByte('8');
            //UART1TxByte('0');
            //UART1TxByte('9');
            //UART1TxByte(' ');

            //UART2TxByte('U');
            //UART2TxByte('2');
            //UART2TxByte(' ');
            //UART2TxByte('4');
            //UART2TxByte('8');
            //UART2TxByte('0');
            //UART2TxByte('9');
            //UART2TxByte(' ');

            //UART3TxByte('U');
            //UART3TxByte('3');
            //UART3TxByte(' ');
            //UART3TxByte('4');
            //UART3TxByte('8');
            //UART3TxByte('0');
            //UART3TxByte('9');
            //UART3TxByte(' ');
            
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

