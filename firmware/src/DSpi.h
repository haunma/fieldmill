#ifndef __DSPI_H
#define __DSPI_H

#include <stdint.h>

class DSpi
{
public:
//    enum
//    {
//        ADC_CNV_WIDTH_NS     = 200,
//        ADC_CNV_OFFSET_NS    = 200,
//        ADC_SAMPLE_PERIOD_NS = 4800, //2400,
//        SPI_CLOCK_MHZ        = 40,
//        TCXO_PWM_KHZ         = 50,
//    };

    // Constructor/destructor/init
    DSpi();
    virtual ~DSpi() {};

    // Public API
    void Init();       // Call during system init

private:
    DSpi(const DSpi&);
    void operator=(const DSpi&);
};

extern DSpi g_spi;

#endif // __DSPI_H defined