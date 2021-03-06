#include "stm32g0xx.h"

#define I2C_ADDRESS 0x55

uint32_t i2c_buffer[2] = {0};
uint16_t spi_buffer[5] = {0};

uint8_t ready = 0;
uint8_t state[3] = {0};
uint8_t button[3] = {0};

const uint32_t bcd[12] = {15, 1, 2, 0, 3, 4, 5, 6, 7, 8, 9, 10};
const uint32_t ant[16] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1};

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
  int32_t i;
  uint32_t tx, lpf, ptt, misc;
  uint32_t code[3], data[3];
  uint16_t bits[7];
  uint8_t att[2];

  tx = i2c_buffer[0] & 0x1;
  ptt = (i2c_buffer[0] >> 1) & 0x1;
  misc = ((i2c_buffer[0] >> 7) & 0xC) | (i2c_buffer[0] & 0x3);
  att[0] = i2c_buffer[0] >> 20 & 0x1F;
  att[1] = i2c_buffer[0] >> 25 & 0x1F;

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

  if(button[2]) code[2] = 0;
  if(button[1]) code[1] = 0;
  if(button[0]) code[0] = 0;

  data[0] = bcd[code[1]] << 28 | i2c_buffer[0] >> 2;

  data[1] = bcd[code[2]];
  data[1] |= 1 << (code[0] + 4);
  data[1] |= 1 << (code[1] + 16);
  data[1] |= 1 << (code[2] + 28);

  data[2] = (code[2] > 3) ? 1 << (code[2] - 4) : 0;
  data[2] |= 1 << (lpf + 8);
  data[2] |= tx << 15;
  data[2] |= (ptt && tx == 0) << 16;
  data[2] |= (ptt && tx == 1) << 17;
  data[2] |= ptt << 18;
  data[2] |= (att[0] == 0) << 19;
  data[2] |= (att[1] == 0) << 20;
  data[2] |= ant[misc] << 21;

  bits[0] = ((data[1] >> 6) & 0x0400) | ((data[1] >> 8) & 0x0200) | ((data[1] >> 10) & 0x0100) | ((data[1] >> 15) & 0x00f0) | ((data[1] >> 16) & 0x0800) | ((data[1] >> 20) & 0x0008) | ((data[1] >> 22) & 0x0004) | ((data[1] >> 24) & 0x0002) | ((data[1] >> 26) & 0x0001) | ((data[2] << 4) & 0xf000);
  bits[1] = ((data[0] << 3) & 0x03f8) | ((data[0] << 1) & 0x8000) | ((data[0] >> 2) & 0x0c00) | ((data[2] << 9) & 0x7000) | ((data[2] >> 12) & 0x0007);
  bits[2] = ((data[0] << 5) & 0x3000) | ((data[0] >> 0) & 0x0e00) | ((data[0] >> 1) & 0xc000) | ((data[1] >> 20) & 0x0100) | ((data[1] >> 23) & 0x00c0) | ((data[1] >> 29) & 0x0004) | ((data[2] << 3) & 0x0008) | ((data[2] >> 0) & 0x0002) | ((data[2] >> 1) & 0x0020) | ((data[2] >> 2) & 0x0001) | ((data[2] >> 3) & 0x0010);
  bits[3] = ((data[0] >> 15) & 0x0004) | ((data[0] >> 24) & 0x00f0) | ((data[1] << 8) & 0x0f00) | ((data[1] << 2) & 0xf000) | ((data[2] >> 12) & 0x0008) | ((data[2] >> 16) & 0x0003);
  bits[4] = ((data[1] << 2) & 0x0080) | ((data[1] >> 0) & 0x0010) | ((data[1] >> 4) & 0x000c) | ((data[1] >> 7) & 0x0002) | ((data[1] >> 8) & 0x0040) | ((data[1] >> 9) & 0x0001) | ((data[1] >> 10) & 0x0020) | ((data[2] >> 10) & 0x1f00);
  bits[5] = ((data[0] >> 8) & 0x8000) | ((data[0] >> 15) & 0x00f8);
  bits[6] = ((data[0] >> 21) & 0x0078);

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
  GPIOA->MODER &= 0x7FFF557F;

  // configure PB as output
  GPIOB->MODER &= 0xFFFFD57F;

  // configure PC as input
  GPIOC->MODER &= 0x0FFFCFFF;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD15_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD6_0;

  // configure PA0 as output
  GPIOA->MODER &= ~GPIO_MODER_MODE0_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_1;

  // configure PA1 as SCK
  GPIOA->MODER &= ~GPIO_MODER_MODE1_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1;

  // configure PA2 as MOSI
  GPIOA->MODER &= ~GPIO_MODER_MODE2_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_1;

  // configure PB8 as SCL
  GPIOB->MODER &= ~GPIO_MODER_MODE8_0;
  GPIOB->OTYPER |= GPIO_OTYPER_OT8_Msk;
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_0;
  GPIOB->AFR[1] |= GPIO_AFRH_AFSEL8_2 | GPIO_AFRH_AFSEL8_1;

  // configure PB7 as SDA
  GPIOB->MODER &= ~GPIO_MODER_MODE7_0;
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

  i2c_buffer[0] = 0x02100000;
  output();

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
