// GPIO pin driver

#include "DGPIO.h"
#include "stm32l4xx.h"
#include <assert.h>

// Master list of GPIO lines
// NOTE: order must match exactly with the GPIOName enum in DGPIO.h!
// (This is checked at runtime if assertions are enabled.)
const DGPIO::GPIOTable DGPIO::gpios[] = {
    //  name        pin    port     i/o  pu/pd  init
    { IR_LED_0,       7,  PortB,  Input, Float,    0 },
    { IR_LED_1,       6,  PortB, Output, Float,    1 },
    { DEBUG1,        12,  PortB, Output, Float,    0 },
};


DGPIO::DGPIO()
{
}


void DGPIO::Init()
{
    GPIO_TypeDef *port;

    for (unsigned k = 0 ; k < sizeof(gpios)/sizeof(gpios[0]) ; k++)
    {
        assert(k == gpios[k].name);  // GPIO table rows must be in same order as GPIO enum!

        // Verify clock is enabled to this port (low 11 bits of reg are A-K)
        if (!(RCC->AHB2ENR & (1 << gpios[k].port)))
        {
            RCC->AHB2ENR |= (1 << gpios[k].port);
        }

        // Calculate port base address, for accessing MODER, OTYPER, etc. regs
        port = (GPIO_TypeDef *)(GPIOA_BASE + gpios[k].port * 0x0400);

        // Everything that depends on input vs output
        if (gpios[k].io == Output)
        {
            // Initial value
            port->ODR = (port->ODR & ~(1 << gpios[k].pin)) | (gpios[k].init << gpios[k].pin);
            // Set direction to output
            port->MODER = (port->MODER & ~(3 << 2*gpios[k].pin)) | (1 << 2*gpios[k].pin);
        }
        else
        {
            // Set direction to input
            port->MODER = (port->MODER & ~(3 << 2*gpios[k].pin));
        }

        // Choose pull-up or pull-down
        unsigned tmp = 0;
        if (gpios[k].pupd == PU)
        {
            tmp = 1;
        }
        else if (gpios[k].pupd == PD)
        {
            tmp = 2;
        }
        port->PUPDR = (port->PUPDR & ~(3 << 2*gpios[k].pin)) | (tmp << 2*gpios[k].pin);

        // OTYPER unused (no open-drain outputs)
        // OSPEEDR unused (all outputs are low speed)
    }
}

// Read the named GPIO and return true if it is a 1, false if 0.
bool DGPIO::Status(GPIOName name)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)(GPIOA_BASE + gpios[name].port * 0x0400);
    return (port->IDR & (1 << gpios[name].pin)) ? true : false;
}

// Set the named GPIO.
void DGPIO::Set(GPIOName name)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)(GPIOA_BASE + gpios[name].port * 0x0400);
    port->BSRR = (1 << gpios[name].pin);
}

// Clear the named GPIO.
void DGPIO::Clear(GPIOName name)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)(GPIOA_BASE + gpios[name].port * 0x0400);
    port->BSRR = (1 << (16 + gpios[name].pin));
}

// Toggle the named GPIO.
void DGPIO::Toggle(GPIOName name)
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)(GPIOA_BASE + gpios[name].port * 0x0400);
    port->ODR ^= (1 << gpios[name].pin);
}
