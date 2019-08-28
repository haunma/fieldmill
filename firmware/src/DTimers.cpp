#include "DTimers.h"
#include "DADC.h"
#include "DMotor.h"

#include "stm32l4xx.h"


DTimers::DTimers()
{
}


void DTimers::Init()
{
    // Timer assignments:
    //   TIM2   ADC1/2 conversion clock, on rising and falling edge of CH2
    //   TIM3   DRV10975 motor driver tachometer capture (on CH4)
    //   TIM4   Reflective object sensor edge capture (on CH3)
    //   LPTIM1 PWM speed-control output to DRV10975 brushless motor driver

    // Enable clocks to TIM2/3/4 and LPTIM1
    RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN | RCC_APB1ENR1_LPTIM1EN);


    // TIM2 CONFIGURATION

    // TIM2 provides the ADC1/2 sample clock for I/Q mill signals.  Conversions
    // are triggered on both the rising and falling edges of TIM2_CH2.
    //
    // CR1: defaults (up-counter, counter disabled)
    // CR2: defaults
    // SMCR: defaults (no slave mode)
    // CCMR1: CC2S=00 (channel 2 is an output),
    //        OC2M=0011 (toggle OC1REF on compare match),
    //        OC2PE=1 (CCRx registers become buffered)
    // CCMR2: defaults
    TIM2->CCMR1 = TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    // CCER: CC2P=0 (OC1 active high), CC2E=1 (OC2 output activated)
    TIM2->CCER = TIM_CCER_CC2E;
    // PSC = 0 (don't prescale 72-MHz input clock)
    TIM2->PSC = 0;
    // ARR = 72e6 / (target sampling rate in Hz) - 1
    TIM2->ARR = (72000000 / DADC::SAMPLE_FREQ_HZ) - 1;
    TIM2->CCR2 = 0; // output toggles each time the counter hits this value
    // Generate an update event to force these values into the active registers
    TIM2->EGR = TIM_EGR_UG;


    // TIM3 CONFIGURATION

    // TIM3 is connected (via the Si8605 isolator) to the DRV10975 FG output,
    // which, for our hard-drive motor, completes four cycles per physical
    // revolution of the shutter, or 4/3 cycle per cycle of the IQ data.  We
    // use the capture functionality of the timer to time-stamp both rising-
    // and falling-edge transitions.  Because TIM3 is a 16-bit counter, we
    // hook into the update interrupt as well as the capture interrupt, in
    // order to count multiple rollovers between captures.
    //
    // CR1, CR2, SMCR, PSC, ARR: defaults (up-counter, counter disabled, no slave
    //                           mode, full 72-MHz count rate, full 16-bit count)
    // CCMR2: CC4S=01 (CH4->TI4 capture), IC4PSC=00 (no prescale), IC4F=0011 (glitch filter)
    TIM3->CCMR2 = TIM_CCMR2_CC4S_0 | (3 << TIM_CCMR2_IC4F_Pos);
    // CCER: CC4P=CC4NP=1 (capture both rising/falling edges), CC4E=1 (capture enabled)
    TIM3->CCER = TIM_CCER_CC4P | TIM_CCER_CC4NP | TIM_CCER_CC4E;
    // DIER: CC4IE=1 (enable interrupt on capture or counter rollover)
    TIM3->DIER = TIM_DIER_CC4IE | TIM_DIER_UIE;


    // TIM4 CONFIGURATION

    // TIM4 is connected to the VCNT2020 reflective object sensor.  The signal
    // is normally high, with a low pulse on each revolution of the shutter.
    // We use the capture functionality of the timer to time-stamp each falling
    // edge.  Because TIM3 is a 16-bit counter, we hook into the update
    // interrupt as well as the capture interrupt, in order to count multiple
    // rollovers between captures.
    //
    // CR1, CR2, SMCR, PSC, ARR: defaults (up-counter, counter disabled, no slave
    //                           mode, full 72-MHz count rate, full 16-bit count)
    // CCMR2: CC3S=01 (CH3->TI3 capture), IC3PSC=00 (no prescale), IC3F=0011 (glitch filter)
    TIM4->CCMR2 = TIM_CCMR2_CC3S_0 | (3 << TIM_CCMR2_IC3F_Pos);
    // CCER: CC3P=1 (capture falling edges only), CC3E=1 (capture enabled)
    TIM4->CCER = TIM_CCER_CC3P | TIM_CCER_CC3E;
    // DIER: CC3IE=1 (enable interrupt on capture or counter rollover)
    TIM4->DIER = TIM_DIER_CC3IE | TIM_DIER_UIE;


    // LPTIM1 CONFIGURATION

    // LPTIM1 can provide a ~ 10-kHz PWM signal to the DRV10975 SPEED input,
    // for motor speed control.  This also doubles as a motor on/off control
    // because the DRV10975 shuts off the motor when SPEED=0.
    // (LPTIM1 is managed by DMotor due to the requirement of stopping/starting
    // the counter when setting 0% or 100% duty cycle.)

    
    // Enable interrupts for input capture
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
    
    // Enable counters (except LPTIM1, which is controlled by DMotor::SetSpeed())
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
}
