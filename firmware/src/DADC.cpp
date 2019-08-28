#include "DADC.h"
#include "stm32l4xx.h"


DADC::DADC()
{
}


void DADC::Init()
{
    // 3-5 MHz / 25 sinc^4 : cal+demod @ 120-200 kHz / 4-13 sinc^5 / 8 FIR (144-tap polyphase)
    //
    // Try (a) 3.6 MSPS with 6.5-cycle sampling time (~90 ns) / 25 sinc^4 : cal+demod @ 144 kHz
    //  or (b) 2.4 MSPS with 12.5-cycle sampling time (~170 ns) / 25 sinc^4 : cal+demod @ 96 kHz
    //
    // (With 72-MHz clock, max f_samp = 72/19 = 3.789 MHz @ 6.5-cyc t_samp, 72/25 = 2.88 MHz @ 12.5-cyc t_samp)

    // Enable clocks to GPIO ports used for analog inputs, the ADCs
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    
    // PIN CONFIGURATION
    //
    // [MCU pin]    [signal]  [ADC] [channel #]
    // PA0_ADC12_5    Q+        2       5
    // PA1_ADC12_6    Q-        2       5
    // PC0_ADC123_1   batt+     3       1
    // PC1_ADC123_2   batt-     3       1
    // PC2_ADC123_3   I+        1       3
    // PC3_ADC123_4   I-        1       3
    GPIOA->MODER = (GPIOA->MODER & 0xffffffff) | (3 << 0*2) | (3 << 1*2);
    GPIOC->MODER = (GPIOC->MODER & 0xffffffff) | (3 << 0*2) | (3 << 1*2) | (3 << 2*2) | (3 << 3*2);
      
    // ADC COMMON CONFIGURATION
    // 
    // CCR: CKMODE=01 (synchronous clock = 72-MHz HCLK), PRESC=0000 (no prescaling)
    ADC123_COMMON->CCR = ADC_CCR_CKMODE_0;

    // PER-ADC CONFIGURATION
    //
    // DEEPPWD=0 to exit deep sleep, ADVREGEN=1 to enable ADC voltage regulator,
    // then wait for at least T_adcVreg_setup = 20 us
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    ADC2->CR &= ~ADC_CR_DEEPPWD;
    ADC2->CR |= ADC_CR_ADVREGEN;
    ADC3->CR &= ~ADC_CR_DEEPPWD;
    ADC3->CR |= ADC_CR_ADVREGEN;
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < 100*72)  // 100 us @ 72-MHz clock
    { }

    // Calibrate each ADC for differential inputs (ADEN still = 0)
    ADC1->CR |= ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL)
    { }
    ADC2->CR |= ADC_CR_ADCALDIF;
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL)
    { }
    ADC3->CR |= ADC_CR_ADCALDIF;
    ADC3->CR |= ADC_CR_ADCAL;
    while (ADC3->CR & ADC_CR_ADCAL)
    { }
    // Wait >= 4 ADC clock cycles

    // DIFSEL: set all channels to differential (bits 15:1 = 1)
    ADC1->DIFSEL = 0x0000fffe;
    ADC2->DIFSEL = 0x0000fffe;
    ADC3->DIFSEL = 0x0000fffe;

    // Enable each ADC and wait for them to be ready
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
    ADC3->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    { }
    ADC1->ISR = ADC_ISR_ADRDY;  // write 1 to clear
    while (!(ADC2->ISR & ADC_ISR_ADRDY))
    { }
    ADC2->ISR = ADC_ISR_ADRDY;  // write 1 to clear
    while (!(ADC3->ISR & ADC_ISR_ADRDY))
    { }
    ADC3->ISR = ADC_ISR_ADRDY;  // write 1 to clear

    // Configure channels: ADC1 and ADC2 (I/Q data)
    //
    // CFGR: JQDIS=1 (reset default), CONT=0 (single conversion per trigger),
    //       EXTEN=11 (trigger on both rising/falling edges),
    //       EXTSEL=011 (trigger is TIM2_CH2), DFSDMCFG=1 (use DFSDM)
    ADC1->CFGR = (3 << ADC_CFGR_EXTEN_Pos) | (3 << ADC_CFGR_EXTSEL_Pos) |
                 ADC_CFGR_JQDIS | ADC_CFGR_DFSDMCFG;
    ADC2->CFGR = (3 << ADC_CFGR_EXTEN_Pos) | (3 << ADC_CFGR_EXTSEL_Pos) |
                 ADC_CFGR_JQDIS | ADC_CFGR_DFSDMCFG;
    // CFGR2: defaults (not using the ADC oversampling module)
    // SMPRx: all SMPx[2:0]=010 (sample for 12.5 ADC clock cycles)
    // (or 001 (sample for 6.5 ADC clock cycles) -- commented out)
    ADC1->SMPR1 = 0x12492492;  // 0001 0010 0100 1001 0010 0100 1001 0010
    //ADC1->SMPR1 = 0x09249249;  // 0000 1001 0010 0100 1001 0010 0100 1001
    ADC1->SMPR2 = 0x02492492;  // 0000 0010 0100 1001 0010 0100 1001 0010
    //ADC1->SMPR2 = 0x01249249;  // 0000 0001 0010 0100 1001 0010 0100 1001
    ADC2->SMPR1 = 0x12492492;  // 0001 0010 0100 1001 0010 0100 1001 0010
    ADC2->SMPR2 = 0x02492492;  // 0000 0010 0100 1001 0010 0100 1001 0010
    // SQR1: L[3:0]=0 (# conversions - 1), SQ1[4:0]=3 and 5 (one chan in regular seq)
    ADC1->SQR1 = (0 << ADC_SQR1_L_Pos) | (3 << ADC_SQR1_SQ1_Pos);
    ADC2->SQR1 = (0 << ADC_SQR1_L_Pos) | (5 << ADC_SQR1_SQ1_Pos);
    // OFRy: OFFSETy_EN=1, OFFSETy_CH[4:0]=1 , OFFSETy[11:0]=0x800
    //       (sign-extended int16 data format, required for DFSDM per
    //       reference manual page 550)
    ADC1->OFR1 = ADC_OFR1_OFFSET1_EN | (3 << ADC_OFR1_OFFSET1_CH_Pos) |
                 (0x800 << ADC_OFR1_OFFSET1_Pos);
    ADC2->OFR1 = ADC_OFR1_OFFSET1_EN | (5 << ADC_OFR1_OFFSET1_CH_Pos) |
                 (0x800 << ADC_OFR1_OFFSET1_Pos);

    // Configure channels: ADC3 (battery monitor)
    //
    // CFGR: defaults
    // CFGR2: ROVSE=1, OVSR=011, OVSS=0000 (16x oversampling, no shift)
    ADC3->CFGR2 = (3 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_ROVSE;
    // SMPRx: all SMPx[2:0]=111 (sample for 640.5 ADC clock cycles, longest possible)
    ADC3->SMPR1 = 0x3fffffff;  // 00111....
    ADC3->SMPR2 = 0x07ffffff;  // 00000111...
    // SQR1: L[3:0]=1, SQ1[4:0]=1 (one channel in regular sequence)
    ADC3->SQR1 = (0 << ADC_SQR1_L_Pos) | (1 << ADC_SQR1_SQ1_Pos);
    
    // "Start" ADCs.  Setting this bit should cause conversions to happen
    // as soon as the external trigger(s) arrive.
    ADC1->CR |= ADC_CR_ADSTART;
    ADC2->CR |= ADC_CR_ADSTART;
    //ADC3->CR |= ADC_CR_ADSTART;
}

