#ifndef __DEMOD_H
#define __DEMOD_H

#include "Demod.h"
#include "DADC.h"
#include "DCIC.h"
#include "DMotor.h"
#include <stdint.h>


class Demod
{
public:
    enum
    {
        // Size the demod wavetable for one [mechanical] rotation of the motor
        //DEMOD_TABLE_SIZE = (int) DADC::SAMPLE_FREQ_HZ / (int) DCIC::DECIM_FACTOR_1 * 60 / (int) DMotor::MOTOR_SPEED_RPM,
        DEMOD_TABLE_SAMPLES = DCIC::DEMOD_BUFFER_SAMPLES,
        
        // Size the output buffer according to the needs of data transmission, etc.
        OUT_BUFFER_CHUNKS = 2,
        OUT_BUFFER_SAMPLES = OUT_BUFFER_CHUNKS * DCIC::OUT_BUFFER_SAMPLES,
    };
    
    Demod();
    virtual ~Demod() {};

    // Public API
    void GetBuffers(int16_t **i1, int16_t **q1);
    void ProcessIQ(void);  // called from PendSV handler after new IQ data become available
    void TachEdge(uint32_t interval);    // called from TIM3 IRQ handler on FG rising/falling edge
    void RotorIndex(uint32_t interval);  // called from TIM4 IRQ handler on rotor index falling edge

private:
    float invMag[DEMOD_TABLE_SAMPLES];
    int16_t I1[2*DCIC::DEMOD_BUFFER_SAMPLES],     // after stage-1 decimation
            Q1[2*DCIC::DEMOD_BUFFER_SAMPLES];
    int16_t *Iptr, *Qptr;
    float Istore, Qstore;
    unsigned NIQSamples;
    bool Half;
    int FirIndex, FirOffset;
    
    float Out[OUT_BUFFER_SAMPLES];
    int OutIndex;

    uint32_t TachPeriod[1024];
    uint32_t IndexPeriod[1024];
    int TachPtr, IndexPtr;
    
uint32_t LatestIntervals[8];

    Demod(const Demod&);
    void operator=(const Demod&);
};

extern Demod g_demod;

#endif  // __DEMOD_H defined