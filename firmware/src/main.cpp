#include "DADC.h"
#include "DCIC.h"
#include "DGPIO.h"
#include "DMotor.h"
#include "DTimers.h"
#include "Demod.h"
#include "UsbDebug.h"
#include "usb.h"
#include "stm32l4xx.h"

//#include <stdio.h>
//#include <stdlib.h>


void ClockInit();
void SetInterruptPriorities();


// Driver and module instantiation
DADC g_adc;
DCIC g_cic;
DGPIO g_gpio;
DMotor g_motor;
DTimers g_timers;
Demod g_demod;
UsbDebug g_usb;


void main()
{
    // Configure PLL and switch to high-speed clock
    ClockInit();
    
    // Enable cycle counter
    ITM->LAR = 0xC5ACCE55;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Configure interrupts
    SetInterruptPriorities();
    
    // Initialize drivers
    g_timers.Init();
    g_motor.Init();
    g_gpio.Init();
    g_cic.Init();
    g_adc.Init();
    
    // Initialize USB debug
    g_usb.cdc_init_usbd();
    
    while (1)
    {
#if defined(CDC_USE_IRQ)
        // __WFI();
#else
        g_usb.Poll();
#endif
        
//        DWT->CYCCNT = 0;
//        while (DWT->CYCCNT < 144000000u)
//        { }
//        g_motor.SetSpeed(0.2f);
//        DWT->CYCCNT = 0;
//        while (DWT->CYCCNT < 144000000u)
//        { }
//        g_motor.SetSpeed(1.0f);
//        DWT->CYCCNT = 0;
//        while (DWT->CYCCNT < 144000000u)
//        { }
//        g_motor.SetSpeed(0.0f);
    }
}


void ClockInit()
{
//    _BST(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);
//    /* Set power Range 1 */
//    _BMD(PWR->CR1, PWR_CR1_VOS, PWR_CR1_VOS_0);
//    _WBC(PWR->SR2, PWR_SR2_VOSF);
//    /* Adjust Flash latency */
//    _BMD(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS);
//    /* set clock 48Mhz MSI */
//    _BMD(RCC->CR, RCC_CR_MSIRANGE, RCC_CR_MSIRANGE_11 | RCC_CR_MSIRGSEL);
//    /* set MSI as 48MHz USB */
//    _BMD(RCC->CCIPR, RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1);
//    /* enable GPIOA clock */
//    _BST(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);
//    /* set GP11 and GP12 as USB data pins AF10 */
//    _BST(GPIOA->AFR[1], (0x0A << 12) | (0x0A << 16));
//    _BMD(GPIOA->MODER, (0x03 << 22) | (0x03 << 24), (0x02 << 22) | (0x02 << 24));

    // Enable PWR clock, set voltage range 1, and wait for stabilization.
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    __NOP();
    __NOP();
    __NOP();
    PWR->CR1 = PWR_CR1_VOS_0;
    while (PWR->SR2 & PWR_SR2_VOSF)
    {
    }
  
    // At this point, the CPU clock source is still internal (MSI).  We will
    // enable the HSE to get an 8-MHz clock source from the external crystal,
    // then configure the main PLL for 72-MHz SYSCLK and HCLK, and 48-MHz USB
    // clock, and finally switch over to the PLL clock after it locks.

    // Enable HSE and wait for confirmation.
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
    {
    }

    // Configure and start the main PLL, running off of HSE
    // N/M=36 (N=36, M=1)
    // R=4 (72 MHz to SYSCLK), Q=6 (48 MHz to USB)
    RCC->PLLCFGR = RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLREN |
                   RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQEN |
                   (36 << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
    }

    // Flash latency must be increased before increasing the system clock frequency.
    // ST says to use 0 up to 16 MHz, 1 up to 32, 2 up to 48, 3 up to 64, and 4 up to 80 MHz (all assuming voltage range 1).
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS; // | FLASH_ACR_PRFTEN; <-- to enable prefetch (increases pwr consumption?)

    // Set clock-tree dividers:
    //   HCLK (AHB clock) = SYSCLK (which is still the default MSI oscillator)
    //   PCLK1 = HCLK
    //   PCLK2 = HCLK
    RCC->CFGR = RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_HPRE_DIV1;

    // Switch SYSCLK over to PLL and wait for confirmation.
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
    {
    }
    
    // Use main PLL "Q" output for 48-MHz USB clock, main SYSCLK for ADCs, and
    // PCLK (default) for LPTIM1, I2C2, and DFSDM1.
    RCC->CCIPR = RCC_CCIPR_ADCSEL_1 | RCC_CCIPR_ADCSEL_0 |
                 RCC_CCIPR_CLK48SEL_1;
      
    // Disable MSI, which is no longer needed.
    RCC->CR &= ~RCC_CR_MSION;

    // Set the CMSIS-standard SystemCoreClock global variable.
    //SystemCoreClock = 72000000u;
}


void SetInterruptPriorities()
{
    // All priority bits are "group priority" aka preempt priority (none for subpriority).
    NVIC_SetPriorityGrouping(0U);

    // DFSDM DMA half-transfer and transfer-complete interrupt for incoming IQ data.
    NVIC_SetPriority(DMA1_Channel4_IRQn, 5);
    
    // DFSDM DMA half-transfer and transfer-complete interrupt for processed
    // data to final FIR filter.
    //NVIC_SetPriority(DMA1_Channel6_IRQn, 5);

    // USB debug output
    NVIC_SetPriority(OTG_FS_IRQn, 10);

    // SysTick merely increments a global variable, nothing more.
    NVIC_SetPriority(SysTick_IRQn, 13);

    // PendSV is lowest priority; it is used to hand-off the DSP processing
    // work from the DFSDM DMA interrupt handler.
    NVIC_SetPriority(PendSV_IRQn, 15);
}
