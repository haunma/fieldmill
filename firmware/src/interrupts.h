#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H

extern "C" {

void SysTick_Handler();
void DMA1_Channel4_IRQHandler();
//void DMA1_Channel1_IRQHandler();
void PendSV_Handler();

}

extern volatile uint32_t g_sysTicks;

#endif  // __INTERRUPTS_H defined