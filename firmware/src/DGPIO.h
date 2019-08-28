#ifndef __DGPIO_H
#define __DGPIO_H

class DGPIO
{
public:
    enum GPIOName
    {
        IR_LED_0 = 0,
        IR_LED_1,
        DEBUG1,
        NUM_GPIONAMES,
    };

    enum GPIOPort
    {
        PortA = 0,
        PortB,
        PortC,
        PortD,
        PortE,
        PortF,
        PortG,
        PortH,
        NUM_GPIOPORTS
    };

    enum GPIOIO
    {
        Input,
        Output,
        NUM_GPIOIOS
    };

    enum GPIOPUPD
    {
        PU,
        PD,
        Float,
        NUM_GPIOPUPDS
    };
        
    struct GPIOTable
    {
        enum GPIOName name : 8;
        unsigned pin       : 8;
        enum GPIOPort port : 4;
        enum GPIOIO io     : 4;
        enum GPIOPUPD pupd : 4;
        unsigned init      : 4;
    };

    static const GPIOTable gpios[];

    DGPIO();
    virtual ~DGPIO (){};

    // Public API
    void Init();

    // Client API
    bool Status(GPIOName name);
    void Set(GPIOName name);
    void Clear(GPIOName name);
    void Toggle(GPIOName name);

private:
    DGPIO(const DGPIO&);
    void operator=(const DGPIO&);
};

extern DGPIO g_gpio;

#endif // __DGPIO_H defined
