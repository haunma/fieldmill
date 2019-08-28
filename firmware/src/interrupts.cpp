// C-linkage interrupt-handler stubs to call the actual handlers inside C++ classes

#include "DCIC.h"
#include "Demod.h"
#include "UsbDebug.h"
#include "stm32l4xx.h"
#include <stdint.h>

volatile uint32_t g_sysTicks = 0;


extern "C" {

void SysTick_Handler()
{
    g_sysTicks++;
}


#if defined(CDC_USE_IRQ)
// USB interrupt handler used by libusb_stm32 for debug/test port
void OTG_FS_IRQHandler(void) {
    g_usb.Poll();
}
#endif


// DFSDM Filter 0 DMA interrupts on half transfer (HT) and transfer complete (TC) (DMA auto-repeats)
//void DMA1_Channel4_IRQHandler()
//{
//    static uint32_t count = 0;
//    // When we arrive here, not only the channel 4 flags (I channel), but also the
//    // channel 5 flags (Q channel) should be set.
//    uint32_t status = DMA1->ISR;
//    DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
//
//    if ((status & 0x0000f000) == (DMA_ISR_GIF4 | DMA_ISR_HTIF4))
//    {
//        g_cic.IQDataReady(true);        // first half-buffer is ready
//    }
//    else if ((status & 0x0000f000) == (DMA_ISR_GIF4 | DMA_ISR_TCIF4))
//    {
//        g_cic.IQDataReady(false);       // second half-buffer is ready
//    }
//    else
//    {
//        count++;
//        g_cic.IQDataReady(false);       // second half-buffer is ready
////        while(1) {}     // FIXME TRAP
//    }
//}

//void DMA1_Channel1_IRQHandler()
//{
//    // When we arrive here, not only the channel 4 flags (I channel), but also the
//    // channel 5 flags (Q channel) should be set.
//    uint32_t status = DMA1->ISR;
//    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
//
//    if ((status & 0x000000ff) == (DMA_ISR_GIF1 | DMA_ISR_HTIF1 | DMA_ISR_GIF2 | DMA_ISR_HTIF2))
//    {
//        g_cic.IQDataReady(true);        // first half-buffer is ready
//    }
//    else if ((status & 0x000000ff) == (DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_GIF2 | DMA_ISR_TCIF2))
//    {
//        g_cic.IQDataReady(false);       // second half-buffer is ready
//    }
//    else
//    {
//        while(1) {}     // FIXME TRAP
//    }
//}


// Called on TIM3 rollover or FG (motor tach) input capture, CH4.
// (Rollovers used to extend the range of the 16-bit counter)
void TIM3_IRQHandler()
{
    static uint16_t rollovers = 0, lastCapture = 0;
    
    uint32_t status = TIM3->SR;
    
    if (status & TIM_SR_UIF)
    {   // keep track of counter rollovers
        TIM3->SR = status & (~TIM_SR_UIF);  // clear UIF flag
        rollovers++;
    }

    if (status & TIM_SR_CC4IF)
    {   // rising/falling edge detected and captured
        uint16_t capture = TIM3->CCR4;  // this clears CC4IF flag
        g_demod.TachEdge((rollovers << 16) + capture - lastCapture);
        lastCapture = capture;
        rollovers = 0;
    }
}


// Called on TIM4 rollover or VCNT2020 (rotor index reflector) input capture, CH3.
// (Rollovers used to extend the range of the 16-bit counter)
void TIM4_IRQHandler()
{
    static uint16_t rollovers = 0, lastCapture = 0;
    
    uint32_t status = TIM4->SR;
    
    if (status & TIM_SR_UIF)
    {   // keep track of counter rollovers
        TIM4->SR = status & (~TIM_SR_UIF);  // clear UIF flag
        rollovers++;
    }

    if (status & TIM_SR_CC3IF)
    {   // falling edge detected and captured
        uint16_t capture = TIM4->CCR3;  // this clears CC3IF flag
        g_demod.RotorIndex((rollovers << 16) + capture - lastCapture);
        lastCapture = capture;
        rollovers = 0;
    }
}


void PendSV_Handler()
{
    g_demod.ProcessIQ();
}


} // extern "C"
