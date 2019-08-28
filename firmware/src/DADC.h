#ifndef __DADC_H
#define __DADC_H


class DADC
{
public:
    enum
    {
        SAMPLE_FREQ_HZ = 2400000,
    };

    
    DADC();
    virtual ~DADC() {};

    // Public API
    void Init();

private:
    DADC(const DADC&);
    void operator=(const DADC&);
};

extern DADC g_adc;

#endif  // __DADC_H defined