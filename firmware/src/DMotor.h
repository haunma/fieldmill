#ifndef __DMOTOR_H
#define __DMOTOR_H

#include <stdint.h>


class DMotor
{
public:
    enum
    {
        DRV10975_I2C_ADDRESS = 0x52,
        MOTOR_SPEED_RPM = 1800,
        MOTOR_SPEED_PWM_FREQ_HZ = 10000,
        MOTOR_SPEED_PWM_MAX_VALUE = 72000000 / MOTOR_SPEED_PWM_FREQ_HZ,
    };


    DMotor();
    virtual ~DMotor() {};

    // Public API
    void Init();       // Call during system init
    void SetSpeed(float pwmFraction);  // Update the motor speed

private:
    // Helpers
    void ConfigMotor();
    void WriteI2C(uint8_t reg, uint8_t val);

    DMotor(const DMotor&);
    void operator=(const DMotor&);
};

extern DMotor g_motor;

#endif // __DMOTOR_H