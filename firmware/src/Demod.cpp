// Field-mill IQ data DSP routines
#include "Demod.h"
#include "DCIC.h"
#include "DGPIO.h"
#include "UsbDebug.h"
#include "stm32l4xx.h"
#include <arm_math.h>


static const float DC_BLOCK_COEF = 0.9999f;

// First half of 144-tap symmetric FIR 8x decimation / CIC compensation filter.
// After cascade with 4th-order CIC, ripple = +/- 0.2 dB, -6 dB point @ 0.44,
// -22 dB @ 0.50, stopband <= -99 dB @ f >= 0.58.  (See cic_comp.m.)
static const float FIRdec[DCIC::FIR_DECIMATOR_TAPS/2] =
{
     3.24989172e-05,
     4.57196656e-05,
     1.03148000e-04,
     1.61410123e-04,
     2.57180949e-04,
     3.62806757e-04,
     4.94898633e-04,
     6.27202299e-04,
     7.60177039e-04,
     8.63943776e-04,
     9.27487202e-04,
     9.20165658e-04,
     8.29039946e-04,
     6.32399944e-04,
     3.29068491e-04,
    -8.12980161e-05,
    -5.75829874e-04,
    -1.12560449e-03,
    -1.68065073e-03,
    -2.18738201e-03,
    -2.58078083e-03,
    -2.80305701e-03,
    -2.80093909e-03,
    -2.54362969e-03,
    -2.02132339e-03,
    -1.25858144e-03,
    -3.09153050e-04,
     7.39029820e-04,
     1.77612284e-03,
     2.67503395e-03,
     3.31051509e-03,
     3.57034553e-03,
     3.37620032e-03,
     2.69415614e-03,
     1.54927878e-03,
     2.67320071e-05,
    -1.72738714e-03,
    -3.51991477e-03,
    -5.12592875e-03,
    -6.31632891e-03,
    -6.88376967e-03,
    -6.67463101e-03,
    -5.61304524e-03,
    -3.72294069e-03,
    -1.13561120e-03,
     1.91103621e-03,
     5.09076208e-03,
     8.01680515e-03,
     1.02841297e-02,
     1.15157679e-02,
     1.14144482e-02,
     9.80790525e-03,
     6.68735314e-03,
     2.22871531e-03,
    -3.20270945e-03,
    -9.07082069e-03,
    -1.47062549e-02,
    -1.93643086e-02,
    -2.22947451e-02,
    -2.28201770e-02,
    -2.04118723e-02,
    -1.47575357e-02,
    -5.81058459e-03,
     6.18301015e-03,
     2.06865960e-02,
     3.69027839e-02,
     5.38313246e-02,
     7.03487138e-02,
     8.53033906e-02,
     9.76163709e-02,
     1.06378123e-01,
     1.10931301e-01
};


Demod::Demod()
  : Iptr(0), Qptr(0), Istore(0.0f), Qstore(0.0f), NIQSamples(0), Half(false),
    FirIndex(0), FirOffset(0), OutIndex(0), TachPtr(0), IndexPtr(0)
{
    // make invMag here, copying from flash or using some kind of calibration routine?
    for (int k = 0 ; k < DEMOD_TABLE_SAMPLES; k++)
    {
        invMag[k] = 1.0f;
    }
}


// Provide the IQ data buffers to the CIC (DFSDM) driver.
void Demod::GetBuffers(int16_t **i1, int16_t **q1)
{
    if (i1 != 0) *i1 = I1;
    if (q1 != 0) *q1 = Q1;
}



// This is the main DSP worker code, called from the PendSV exception handler
// in interrupts.cpp.
void Demod::ProcessIQ(void)
{
static int tptr = 0, iptr = 0;

    g_gpio.Set(DGPIO::DEBUG1);

    float tmpA[2*DCIC::DEMOD_BUFFER_SAMPLES], tmpB[DCIC::DEMOD_BUFFER_SAMPLES];
    float iirTmp;
    
    for (int k = 0 ; k < DCIC::DEMOD_BUFFER_SAMPLES ; k++)
    {
        // Remove I and Q DC bias using a first-order IIR HPF
        iirTmp = (static_cast<float>(*(Iptr+k)) / 32768.0f) + (DC_BLOCK_COEF * Istore);
        tmpA[2*k] = iirTmp - Istore;
        Istore = iirTmp;
        
        iirTmp = (static_cast<float>(*(Qptr+k)) / 32768.0f) + (DC_BLOCK_COEF * Qstore);
        tmpA[2*k+1] = iirTmp - Qstore;
        Qstore = iirTmp;
    }
    
    // Efficiently calculate complex magnitude.
    arm_cmplx_mag_f32(tmpA, tmpB, NIQSamples);
    arm_mult_f32(invMag, tmpB, tmpA, NIQSamples);
    
    
int tmp2 = OutIndex;
    // Apply the decimating FIR compensatoNIQSamplesr.
    int start, end, lenToWrap;
    for (int j = 0 ; j < NIQSamples ; j++)
    {
        Out[OutIndex] = 0;
        
        start = FirOffset + (j * DCIC::DECIM_FACTOR_FIR);
        lenToWrap = (DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES) - start;
        if (lenToWrap <= 0)
        {
            start -= DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES;
            lenToWrap += DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES;
        }

        if (lenToWrap < DCIC::FIR_DECIMATOR_TAPS / 2)
        {
            end = start + DCIC::FIR_DECIMATOR_TAPS - 1 - (DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES);

            // start wraps, end does not
            for (int k = 0 ; k < lenToWrap ; k++)
            {
                Out[OutIndex] += FIRdec[k] * (tmpA[start] + tmpA[end]);
                start++, end--;
            }
            start -= (DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES);
            for (int k = lenToWrap ; k < DCIC::FIR_DECIMATOR_TAPS / 2 ; k++)
            {
                Out[OutIndex] += FIRdec[k] * (tmpA[start] + tmpA[end]);
                start++, end--;
            }
        }
        else if (lenToWrap < DCIC::FIR_DECIMATOR_TAPS)
        {
            end = start + DCIC::FIR_DECIMATOR_TAPS - 1 - (DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES);

            // end wraps, start does not
            for (int k = 0 ; k < DCIC::FIR_DECIMATOR_TAPS - lenToWrap ; k++)
            {
                Out[OutIndex] += FIRdec[k] * (tmpA[start] + tmpA[end]);
                start++, end--;
            }
            end += (DCIC::FIR_BUFFER_CHUNKS * DCIC::FIR_BUFFER_SAMPLES);
            for (int k = DCIC::FIR_DECIMATOR_TAPS - lenToWrap ; k < DCIC::FIR_DECIMATOR_TAPS / 2 ; k++)
            {
                Out[OutIndex] += FIRdec[k] * (tmpA[start] + tmpA[end]);
                start++, end--;
            }
        }
        else
        {
            end = start + DCIC::FIR_DECIMATOR_TAPS - 1;

            // neither start nor end wrap
            for (int k = 0 ; k < DCIC::FIR_DECIMATOR_TAPS / 2 ; k++)
            {
                Out[OutIndex] += FIRdec[k] * (tmpA[start] + tmpA[end]);
                start++, end--;
            }
        }
        
        if (++OutIndex >= OUT_BUFFER_SAMPLES)
        {
            OutIndex = 0;
        }
    }
    
g_usb.Write(&Out[tmp2], DCIC::OUT_BUFFER_SAMPLES*sizeof(Out[0]));

LatestIntervals[0] = 0;
LatestIntervals[1] = 0;
LatestIntervals[2] = 0;
LatestIntervals[3] = 0;
LatestIntervals[4] = 0;
LatestIntervals[5] = 0;
LatestIntervals[6] = 0;
LatestIntervals[7] = 0;
int p = 0;
while ((iptr != IndexPtr) && (p < 8))
{
    LatestIntervals[p++] = IndexPeriod[iptr++];
    if (iptr >= 1024)
    {
        iptr = 0;
    }
}
iptr = IndexPtr;

g_usb.Write(LatestIntervals, 8*sizeof(uint32_t));

    //
    // TODO figure out who to hand off the output buffer to... USB debug,
    // optical transmission, etc.
    //

    g_gpio.Clear(DGPIO::DEBUG1);
}


void Demod::TachEdge(uint32_t interval)
{
    TachPeriod[TachPtr] = interval;
    if (++TachPtr >= 1024)
    {
        TachPtr = 0;
    }
}


// Called by timer capture interrupt on each full mechanical rotation of
// the rotor.  When this happens we kick off processing on the last
// rotation, swap buffers, and restart DMA for the next rotation.
void Demod::RotorIndex(uint32_t interval)
{
    IndexPeriod[IndexPtr] = interval;
    if (++IndexPtr >= 1024)
    {
        IndexPtr = 0;
    }
    
    // Reset DMA and point to the other buffers.
    //
    // To ensure that I and Q remain synchronized (same number of samples in each),
    // wait here until the number of samples in each buffers are a multiple of
    // the FIR decimation factor, and the last sample has just been DMAed; then,
    // halt DMA and do the buffer swap in the time between samples---typically 5-10 us.
    unsigned count = DMA1_Channel4->CNDTR;
    if (count > 0)
    {
        unsigned targetCount = DCIC::DEMOD_BUFFER_SAMPLES - (DCIC::DECIM_FACTOR_FIR * (1 + (DCIC::DEMOD_BUFFER_SAMPLES - count) / DCIC::DECIM_FACTOR_FIR));
        while (DMA1_Channel4->CNDTR != targetCount)
        { }
        while (DMA1_Channel5->CNDTR != targetCount)
        { }
        NIQSamples = DCIC::DEMOD_BUFFER_SAMPLES - targetCount; // guaranteed multiple of DECIM_FACTOR_FIR
    }
    
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CTEIF4 | DMA_IFCR_CTEIF5;  // clear error flags just in case
    
    if (count > 0)
    {
        if (Half)
        {   // First half of ping-pong filled; point DSP to start of first half (IQ)
            // and oldest needed history in FIR input buffer.
            Half = false;
            Iptr = &I1[0];
            Qptr = &Q1[0];
            DMA1_Channel4->CMAR = (uint32_t) &I1[DCIC::DEMOD_BUFFER_SAMPLES];
            DMA1_Channel5->CMAR = (uint32_t) &Q1[DCIC::DEMOD_BUFFER_SAMPLES];
        }
        else
        {   // Second half of ping-pong filled; point DSP to start of second half (IQ)
            // and oldest needed history in FIR input buffer.
            Half = true;
            Iptr = &I1[DCIC::DEMOD_BUFFER_SAMPLES];
            Qptr = &Q1[DCIC::DEMOD_BUFFER_SAMPLES];
            DMA1_Channel4->CMAR = (uint32_t) &I1[0];
            DMA1_Channel5->CMAR = (uint32_t) &Q1[0];
        }
    }

    DMA1_Channel4->CNDTR = DCIC::DEMOD_BUFFER_SAMPLES;
    DMA1_Channel5->CNDTR = DCIC::DEMOD_BUFFER_SAMPLES;

    DMA1_Channel4->CCR |= DMA_CCR_EN;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    if (count > 0)
    {
        // FirOffset points to the oldest sample needed by the FIR decimator in its
        // input buffer.
//FIXME what is FirIndex???  might be a bug here somewhere.  refer to older version of Demod.
        FirOffset = FirIndex + DCIC::DECIM_FACTOR_FIR - DCIC::FIR_DECIMATOR_TAPS;
        if (FirOffset < 0)
        {
            FirOffset += DCIC::FIR_BUFFER_CHUNKS*DCIC::FIR_BUFFER_SAMPLES;
        }
        
        // Increment FIR input-buffer index to next chunk, circularly.
        FirIndex += DCIC::FIR_BUFFER_SAMPLES;
        if (FirIndex >= DCIC::FIR_BUFFER_CHUNKS*DCIC::FIR_BUFFER_SAMPLES)
        {
            FirIndex = 0;
        }

        // Tail-chain into low-priority PendSV interrupt for DSP work.
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}
