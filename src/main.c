#include "stm32g0xx.h"

#define I2C_ADDRESS 0x55

uint32_t i2c_buffer[2] = {0};
uint16_t spi_buffer[5] = {0};

uint8_t ready = 0;
uint8_t state[3] = {0};
uint8_t button[3] = {0};

const uint8_t map[83] =
{
  // Control Outputs (13, 14, 15, 16, 17, 18, 19)
  0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
  // Dither, Random (2C, 2D)
  0x2C, 0x2D,
  // RX Antenna (29, 2A, 2B)
  0x29, 0x2A, 0x2B,
  // TX Antenna (1A, 1B, 1C)
  0x1A, 0x1C, 0x1C,
  // Bypass all HPFs (2E)
  0x2E,
  // 6m low noise amplifier (2F)
  0x2F,
  // Disable T/R relay (32)
  0x32,
  // ATT1 (PA3, PA4, PA5, PA6, PA7)
  0x53, 0x54, 0x55, 0x56, 0x57,
  // ATT2 (PA15, PB3, PB4, PB5, PB6)
  0x5F, 0x63, 0x64, 0x65, 0x66,
  // BCD code for RX1 band (34, 35, 36, 37)
  0x34, 0x35, 0x36, 0x37,
  // BCD code for RX2 band (38, 39, 3A, 3B)
  0x38, 0x39, 0x3A, 0x3B,
  // BPF TX (3C, 3D, 3E, 3F, 40, 41, 42, 43, 44, 45, 46, 47)
  0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
  // BPF RX1 (00, 01, 02, 03, 04, 05, 06, 07, 08, 09, 0A, 0B)
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
  // BPF RX2 (1D, 1E, 1F, 20, 21, 22, 23, 24, 25, 26, 27, 28)
  0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
  // LPF TX (0C, 0D, 0E, 0F, 10, 11, 12)
  0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
  // TX band != RX1 band (33)
  0x33,
  // Relays (30, 31)
  0x30, 0x31,
  // PTT (48)
  0x48
};

void input()
{
  int32_t i;

  state[0] = (state[0] << 1) | ((GPIOC->IDR & GPIO_IDR_ID6_Msk) == GPIO_IDR_ID6_Msk);
  state[1] = (state[1] << 1) | ((GPIOC->IDR & GPIO_IDR_ID14_Msk) == GPIO_IDR_ID14_Msk);
  state[2] = (state[2] << 1) | ((GPIOC->IDR & GPIO_IDR_ID15_Msk) == GPIO_IDR_ID15_Msk);

  for(i = 0; i < 3; ++i)
  {
    if(state[i] == 0x80)
    {
      ready = 1;
      button[i] = 1;
    }
    else if(state[i] == 0x7F)
    {
      ready = 1;
      button[i] = 0;
    }
  }
}

void output()
{
  int32_t i, tx, lpf, ptt;
  uint32_t code[3], data[3];
  uint16_t bits[7];

  tx = i2c_buffer[0] & 0x1;
  ptt = (i2c_buffer[0] >> 1) & 0x1;

  code[2] = i2c_buffer[1] >> 4 & 0xF;
  code[1] = i2c_buffer[1] & 0xF;
  code[0] = tx ? code[2] : code[1];

  lpf = 0;
  if(code[0] == 2) lpf = 1;
  else if(code[0] == 3 || code[0] == 4) lpf = 2;
  else if(code[0] == 5 || code[0] == 6) lpf = 3;
  else if(code[0] == 7 || code[0] == 8) lpf = 4;
  else if(code[0] == 9 || code[0] == 10) lpf = 5;
  else if(code[0] == 11 || code[0] == 0) lpf = 6;

  data[0] = code[1] << 28 | i2c_buffer[0] >> 2;

  data[1] = code[2];
  data[1] |= 1 << (code[0] + 4);
  data[1] |= 1 << (code[1] + 16);
  data[1] |= 1 << (code[2] + 28);

  data[2] = (code[2] > 3) ? 1 << (code[2] - 4) : 0;
  data[2] |= lpf << 8;
  data[2] |= tx << 15;
  data[2] |= (ptt && tx == 0) << 16;
  data[2] |= (ptt && tx == 1) << 17;
  data[2] |= ptt << 18;

  for(i = 0; i < 7; ++i)
  {
    bits[i] = 0;
  }

  for(i = 0; i < 83; ++i)
  {
    bits[map[i] >> 4] |= (data[i >> 5] >> (i & 0x1F) & 1) << (map[i] & 0xF);
  }

  for(i = 0; i < 5; ++i)
  {
    spi_buffer[i] = bits[4 - i];
  }

  // enable DMA transfers
  DMA1_Channel2->CCR &= ~DMA_CCR_EN_Msk;
  DMA1_Channel2->CNDTR = 5;
  DMA1_Channel2->CCR |= DMA_CCR_EN_Msk;

  // wait for the end of the transfer
  while(SPI1->SR & SPI_SR_FTLVL_Msk);
  while(SPI1->SR & SPI_SR_BSY_Msk);

  // generate strobe pulse
  GPIOA->ODR |= GPIO_ODR_OD0_Msk;
  GPIOA->ODR &= ~GPIO_ODR_OD0_Msk;

  // output ATT1 and ATT2 signals
  GPIOA->ODR = (GPIOA->ODR & 0xFFFF7F07) | bits[5];
  GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF87) | bits[6];
}

int main()
{
  // enable GPIOA, GPIOB and GPIOC
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN_Msk | RCC_IOPENR_GPIOBEN_Msk | RCC_IOPENR_GPIOAEN_Msk;

  // configure PA as output
  GPIOB->MODER &= 0x7FFF557F;

  // configure PB as output
  GPIOB->MODER &= 0xFFFFD57F;

  // configure PC as input
  GPIOC->MODER &= 0x0FFFCFFF;

  // configure PA0 as output
  GPIOA->MODER = ~GPIO_MODER_MODE0_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_1;

  // configure PA1 as SCK
  GPIOA->MODER = ~GPIO_MODER_MODE1_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1;

  // configure PA2 as MOSI
  GPIOA->MODER = ~GPIO_MODER_MODE2_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_1;

  // configure PB8 as SCL
  GPIOB->MODER = ~GPIO_MODER_MODE8_0;
  GPIOB->OTYPER |= GPIO_OTYPER_OT8_Msk;
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_0;
  GPIOB->AFR[1] |= GPIO_AFRH_AFSEL8_2 | GPIO_AFRH_AFSEL8_1;

  // configure PB7 as SDA
  GPIOB->MODER = ~GPIO_MODER_MODE7_0;
  GPIOB->OTYPER |= GPIO_OTYPER_OT7_Msk;
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_1;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_1;

  // enable clock for DMA1
  RCC->AHBENR |= RCC_AHBENR_DMA1EN_Msk;

  // configure DMA for I2C
  DMAMUX1_Channel0->CCR |= DMAMUX_CxCR_DMAREQ_ID_3 | DMAMUX_CxCR_DMAREQ_ID_1;
  DMA1_Channel1->CCR |= DMA_CCR_MINC_Msk;
  DMA1_Channel1->CPAR = (uint32_t)&I2C1->RXDR;
  DMA1_Channel1->CMAR = (uint32_t)i2c_buffer;

  // configure DMA for SPI
  DMAMUX1_Channel1->CCR |= DMAMUX_CxCR_DMAREQ_ID_4 | DMAMUX_CxCR_DMAREQ_ID_0;
  DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC_Msk | DMA_CCR_DIR_Msk;
  DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel2->CMAR = (uint32_t)spi_buffer;

  // enable LSI
  RCC->CSR |= RCC_CSR_LSION_Msk;

  // set LPTIM1 clock source to LSI
  // set I2C1 clock source to HSI16
  RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0 | RCC_CCIPR_I2C1SEL_1;

  // enable clock for I2C1, PWR and LPTIM1
  RCC->APBENR1 |= RCC_APBENR1_LPTIM1EN_Msk | RCC_APBENR1_PWREN_Msk | RCC_APBENR1_I2C1EN_Msk;

  // enable I2C1 interrupts, DMA and wakeup from Stop 1 mode
  I2C1->CR1 |= I2C_CR1_WUPEN_Msk | I2C_CR1_RXDMAEN_Msk | I2C_CR1_STOPIE_Msk | I2C_CR1_ADDRIE_Msk;

  // enable I2C1
  I2C1->CR1 |= I2C_CR1_PE_Msk;
  // set I2C1 address 1
  I2C1->OAR1 |= I2C_ADDRESS << 1;
  // enable I2C1 address 1
  I2C1->OAR1 |= I2C_OAR1_OA1EN_Msk;

  // enable clock for SPI1
  RCC->APBENR2 |= RCC_APBENR2_SPI1EN_Msk;

  // configure SPI1
  SPI1->CR1 |= SPI_CR1_SSM_Msk | SPI_CR1_SSI_Msk | SPI_CR1_BR_0 | SPI_CR1_MSTR_Msk;
  SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_TXDMAEN_Msk;
  // enable SPI1
  SPI1->CR1 |= SPI_CR1_SPE_Msk;

  // enable ARRM interrupt
  LPTIM1->IER |= LPTIM_IER_ARRMIE_Msk;

  // enable LPTIM1
  LPTIM1->CR |= LPTIM_CR_ENABLE_Msk;

  // enable Stop 1 mode
  PWR->CR1 |= PWR_CR1_LPMS_0;
  SCB->SCR |= SCB_SCR_SEVONPEND_Msk | SCB_SCR_SLEEPDEEP_Msk;

  // set LPTIM1 autoreload register
  LPTIM1->ARR = 160;

  // start LPTIM1 counter
  LPTIM1->CR |= LPTIM_CR_CNTSTRT_Msk;

  while(1)
  {
    __WFE();
    if(I2C1->ISR & I2C_ISR_ADDR_Msk)
    {
      I2C1->ICR |= I2C_ICR_ADDRCF_Msk;
      i2c_buffer[0] = 0;
      i2c_buffer[1] = 0;
      // enable DMA transfers
      DMA1_Channel1->CCR &= ~DMA_CCR_EN_Msk;
      DMA1_Channel1->CNDTR = 5;
      DMA1_Channel1->CCR |= DMA_CCR_EN_Msk;
      // disable Stop 1 mode
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
      // clear interrupt pending bit
      NVIC_ClearPendingIRQ(I2C1_IRQn);
    }
    else if(I2C1->ISR & I2C_ISR_STOPF_Msk)
    {
      I2C1->ICR |= I2C_ICR_STOPCF_Msk;
      ready = 1;
      // enable Stop 1 mode
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      // clear interrupt pending bit
      NVIC_ClearPendingIRQ(I2C1_IRQn);
    }
    if(LPTIM1->ISR & LPTIM_ISR_ARRM_Msk)
    {
      LPTIM1->ICR |= LPTIM_ICR_ARRMCF_Msk;
      input();
      // clear interrupt pending bit
      NVIC_ClearPendingIRQ(TIM6_DAC_LPTIM1_IRQn);
    }
    if(ready)
    {
      ready = 0;
      output();
    }
  }
}
