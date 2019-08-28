#include "DSpi.h"
#include "stm32l4xx.h"


DSpi::DSpi()
{
}


void DSpi::Init()
{
    // Enable clocks to SPI1 and GPIOA/B
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // SPI1 CONFIGURATION
    //
    // CR1: BR = 000 (4-MHz pclk / 2), MSTR = 1 (master mode), CPOL=0, CPHA=1,
    //      SSM=SSI=1 (software slave-select mode)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI;
    // CR2: DS = 0111 (8-bit data), FRXTH=1 (RXNE asserted on each rx byte)
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;

    // PIN CONFIGURATION
    //
    // Pinmux:
    //   PA0  (GPIO, IRQ input)
    //   PA5  (SPI1_SCK), AF5
    //   PA6  (SPI1_MISO), AF5
    //   PA7  (SPI1_MOSI), AF5
    //   PB6  (GPIO, slave-select output)
    GPIOA->AFR[0] = (GPIOA->AFR[0] & 0x000fffff) | (5u << 5*4) | (5u << 6*4) | (5u << 7*4);

    // Set medium-speed drive level
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & 0xffff03ff) | (1u << 5*2) | (1u << 6*2) | (1u << 7*2);
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & 0xffffcfff) | (1u << 6*2);

    // Init slave-select line to high (de-asserted)
    GPIOB->ODR = (GPIOB->ODR & 0xffffffbf) | (1u << 6);

    // Set general-purpose output mode for slave-select, input mode for IRQ
    GPIOA->MODER = (GPIOA->MODER & 0xfffffffc) | (0u << 0*2);
    GPIOB->MODER = (GPIOB->MODER & 0xffffcfff) | (1u << 6*2);
//    SPIMode();  // alt-func mode for SPI1 pins

    // Enable pull-downs on PA0 (IRQ) and PA6 (MISO)
    GPIOA->PUPDR = (GPIOA->PUPDR & 0xffffcffc) | (2u << 0*2) | (2u << 6*2);

    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}
