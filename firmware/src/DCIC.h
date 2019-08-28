#ifndef __DCIC_H
#define __DCIC_H

#include "DADC.h"
#include "DMotor.h"
#include <stdint.h>

class DCIC
{
public:
    enum
    {
        // NOTE: f_adc must be an integer multiple of
        // DECIM_FACTOR_1 * DECIM_FACTOR_FIR * (1000 / PROCESSING_INTERVAL_MS) !!
        // (This multiple is the number of output samples per processing interval.)
        DECIM_FACTOR_1 = 25,  // sinc^4 filter, limited to 26x
        DECIM_FACTOR_FIR = 8, // decimating / CIC-droop-compensating FIR

        PROCESSING_INTERVAL_MS = 50,  // *maximum* time for one mechanical rotation

        // Sizes of one processing chunk at (1) output of initial CIC decimation,
        // (2) input to FIR (same as 1 for this iteration), and (3) final output
        DEMOD_BUFFER_SAMPLES = DADC::SAMPLE_FREQ_HZ / DECIM_FACTOR_1 * PROCESSING_INTERVAL_MS / 1000,
        FIR_BUFFER_SAMPLES = DEMOD_BUFFER_SAMPLES,
        OUT_BUFFER_SAMPLES = FIR_BUFFER_SAMPLES / DECIM_FACTOR_FIR,
        
        // Length of the symmetric FIR compensating/decimating filter in Demod.cpp
        FIR_DECIMATOR_TAPS = 144,    // must be a multiple of DECIM_FACTOR_FIR
        
        // Make the FIR input buffer length a multiple of FIR_BUFFER_SAMPLES
        // (so the circular DMA will work), but long enough so that there
        // is always at least FIR_DECIMATOR_TAPS of past history in addition
        // to the most latest chunk of FIR_BUFFER_SAMPLES.
        FIR_BUFFER_CHUNKS = 2 + (FIR_DECIMATOR_TAPS / FIR_BUFFER_SAMPLES),
    };
    

    DCIC();
    virtual ~DCIC() {};

    // Public API
    void Init();       // Call during system init
    
private:
    int16_t *I1, *Q1;  // IQ data buffers which DFSDM DMAs to
    
    DCIC(const DCIC&);
    void operator=(const DCIC&);
};

extern DCIC g_cic;

#endif  // __DCIC_H defined