// STM32L4 DFSDM (cascaded integrator comb filter) driver
#include "DCIC.h"
#include "Demod.h"
#include "stm32l4xx.h"
#include <math.h>


DCIC::DCIC()
{
}


void DCIC::Init()
{
    // Enable clock to DFSDM1, DMA1
    RCC->APB2ENR |= RCC_APB2ENR_DFSDM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Get IQ-data DMA buffers from Demod component.
    g_demod.GetBuffers(&I1, &Q1);

    // DFSDM CONFIGURATION
    //
    // Global enable
    DFSDM1_Channel0->CHCFGR1 = DFSDM_CHCFGR1_DFSDMEN;
    // First-stage config:
    //
    // CHyCFGR1: DATMPX=01 (input from ADC) or 10 (input from CPU register write)
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DATMPX_0;
    DFSDM1_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_DATMPX_0;
    // CHyCFGR2: DTRBS=x to right-shift the result by x bits.  CIC bit growth is N*log2(R)
    //           where N=filter order and R=decim factor.  The output register is 24 bits
    //           of which we will keep only the high 16 MSBs, and the data from the ADCs
    //           are in the 12 LSBs of the input path.  We want a net bit growth of 12 bits,
    //           so we right shift by N*log2(R) - 12.  (Similar reasoning applies to the
    //           second-stage CIC, except now the input is a full 16 bits.)
    DFSDM1_Channel0->CHCFGR2 = (static_cast<int>(ceilf(4.0f*log2f(static_cast<float>(DECIM_FACTOR_1)))) - 12) << DFSDM_CHCFGR2_DTRBS_Pos;
    DFSDM1_Channel1->CHCFGR2 = (static_cast<int>(ceilf(4.0f*log2f(static_cast<float>(DECIM_FACTOR_1)))) - 12) << DFSDM_CHCFGR2_DTRBS_Pos;
    // FLTxCR1: RCH=0/1/2/3 (channel0 -> filter0, etc.), RDMAEN=1 (output DMA enabled),
    //          FAST=1 (correct for streaming data), RCONT=1 (continuous mode)
    DFSDM1_Filter0->FLTCR1 = DFSDM_FLTCR1_FAST | (0 << DFSDM_FLTCR1_RCH_Pos) |
                             DFSDM_FLTCR1_RCONT | DFSDM_FLTCR1_RDMAEN;
    DFSDM1_Filter1->FLTCR1 = DFSDM_FLTCR1_FAST | (1 << DFSDM_FLTCR1_RCH_Pos) |
                             DFSDM_FLTCR1_RCONT | DFSDM_FLTCR1_RDMAEN;
    // FLTxFCR: FORD=4 (sinc^4), FOSR=DECIM_FACTOR_1, IOSR=0 (no extra integration)
    DFSDM1_Filter0->FLTFCR = (4 << DFSDM_FLTFCR_FORD_Pos) | 
                             ((DECIM_FACTOR_1 - 1) << DFSDM_FLTFCR_FOSR_Pos);
    DFSDM1_Filter1->FLTFCR = (4 << DFSDM_FLTFCR_FORD_Pos) | 
                             ((DECIM_FACTOR_1 - 1) << DFSDM_FLTFCR_FOSR_Pos);
 
    
    // DMA CONFIGURATION
    //
    // CSELR: C4S=C5S=C6S=C7S=0x00 (map DFSDM1_FLT0/1/2/3 to channels 4/5/6/7)
    DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~(DMA_CSELR_C4S | DMA_CSELR_C5S));
    // CPAR: DFSDM data register, upper 16 bits (for half-word access)
    DMA1_Channel4->CPAR = ((uint32_t) &(DFSDM1_Filter0->FLTRDATAR)) + 2;
    DMA1_Channel5->CPAR = ((uint32_t) &(DFSDM1_Filter1->FLTRDATAR)) + 2;
    // CMAR: Output buffer
    DMA1_Channel4->CMAR = (uint32_t) I1;
    DMA1_Channel5->CMAR = (uint32_t) Q1;
    // CNDTR: Length of half of output buffer
    DMA1_Channel4->CNDTR = DEMOD_BUFFER_SAMPLES;
    DMA1_Channel5->CNDTR = DEMOD_BUFFER_SAMPLES;
    // CCR: PL=xx (priority), MSIZE=PSIZE=01 (16-bit), DIR=0 (periph->mem),
    //      MINC=1 and PINC=0 (inc memory addr but not peripheral addr),
    //      no interrupts, CIRC=0 (stop when CNDTR reaches 0)
    DMA1_Channel4->CCR = (2 << DMA_CCR_PL_Pos) | (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos)
                       | DMA_CCR_MINC;
    DMA1_Channel5->CCR = (2 << DMA_CCR_PL_Pos) | (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos)
                       | DMA_CCR_MINC;
    // Enable DMA
    DMA1_Channel4->CCR |= DMA_CCR_EN;
    DMA1_Channel5->CCR |= DMA_CCR_EN;


    // Enable DFSDM filters and channels
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_DFEN;
    DFSDM1_Filter1->FLTCR1 |= DFSDM_FLTCR1_DFEN;
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
    DFSDM1_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;
    DFSDM1_Filter1->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;
}
