// I2C driver and motor control logic for the DRV10975 BLDC controller
#include "DMotor.h"
#include "stm32l4xx.h"
#include <math.h>

DMotor::DMotor()
{
}


void DMotor::Init()
{
    // Enable clocks to DRV10975-connected I2C peripheral, relevant GPIO ports
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    
    // CONFIGURE PINS
    //
    // [MCU]              [DRV10975]
    // PB10 (I2C2 SCL)   --> SCL
    // PB11 (I2C2 SDA)   <-> SDA
    // PB2  (LPTIM1_OUT) --> SPEED
    // PB1  (TIM3_CH4)   <-- FG
    // Also set PB8 to TIM4_CH3 function for the VCNT2020 rotor index signal.
    //
    // Set alternate functions
    // PB1, PB8: AF2 (TIM3/TIM4)
    // PB2: AF1 (LPTIM1)
    // PB10, PB11: AF4 (I2C2)
    GPIOB->AFR[0] = (GPIOB->AFR[0] & 0xfffff00f) | (2 << 1*4) | (1 << 2*4);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & 0xffff00f0) | (2 << (8-8)*4) | (4 << (10-8)*4) | (4 << (11-8)*4);

    // Set open-drain output on I2C2 SCL/SDA (PB10, PB11)
    GPIOB->OTYPER = (GPIOB->OTYPER & 0xfffff3ff) | (1 << 10) | (1 << 11);

    // Set low-speed drive for I2C2 and LPTIM1 outputs (PB2, PB10, PB11)
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & 0xff0fffcf);

    // Set pull-ups on I2C2 (PB10, PB11)
    GPIOB->PUPDR = (GPIOB->PUPDR & 0xff0fffff) | (1 << 10*2) | (1 << 11*2);

    // Set alt-function mode on PB1/2/8/10/11
    GPIOB->MODER = (GPIOB->MODER & 0xff0cffc3) | (2 << 1*2) | (2 << 2*2) | (2 << 8*2)
                                               | (2 << 10*2) | (2 << 11*2);

    // CONFIGURE I2C
    //
    // CR2: AUTOEND=1 (automatically send STOP), SADD[7:1]=device address,
    //      NBYTES=2 (8-bit reg addr + 8-bit data), direction=write
    I2C2->CR2 = I2C_CR2_AUTOEND | (2 << I2C_CR2_NBYTES_Pos) | (DRV10975_I2C_ADDRESS << 1);
    // PRESC=8 (prescaled clock = 72/9 = 8 MHz), SCLL = 5 us => 39,
    // SCLH = 4 us => 31, SDADEL = 500 ns => 3, SCLDEL = 1250 ns => 9
    // (These numbers for 100-kHz bus; see ref. guide table 232, p. 1296)
    I2C2->TIMINGR = (8 << I2C_TIMINGR_PRESC_Pos) |
                    (39 << I2C_TIMINGR_SCLL_Pos) | (31 << I2C_TIMINGR_SCLH_Pos) |
                    (3 << I2C_TIMINGR_SDADEL_Pos) | (9 << I2C_TIMINGR_SCLDEL_Pos);
//// 10-kHz
//    I2C2->TIMINGR = (15 << I2C_TIMINGR_PRESC_Pos) |
//                    (0xe1 << I2C_TIMINGR_SCLL_Pos) | (0xdd << I2C_TIMINGR_SCLH_Pos) |
//                    (0x02 << I2C_TIMINGR_SDADEL_Pos) | (0x05 << I2C_TIMINGR_SCLDEL_Pos);
    I2C2->CR1 = I2C_CR1_PE;  // enable last

    // CONFIGURE DRV10975 BLDC MOTOR DRIVER
    SetSpeed(1.0f);  // first set SPEED line high to keep from entering sleep mode
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < 7200000u)  // wait 100 ms
    { }
    ConfigMotor();
    SetSpeed(0.28f); // 0.35f
//SetSpeed(0.0f);
}


// Update the motor speed using the SPEED PWM signal to the DRV10975.  This
// adjusts the duty cycle between 0.0 (always low) and 1.0 (always high).
void DMotor::SetSpeed(float pwmFraction)
{
    if (pwmFraction > 0.0f && pwmFraction < 1.0f)
    {
        // CFGR: PRELOAD=1, WAVE=0 (PWM output), WAVPOL=1, PRESC=000 (/1)
        LPTIM1->CFGR = LPTIM_CFGR_PRELOAD | LPTIM_CFGR_WAVPOL;
        LPTIM1->CR = LPTIM_CR_ENABLE;
        // CMP: controls PWM duty cycle from 1 (min) to MOTOR_SPEED_PWM_MAX_VALUE-1 (max)
        LPTIM1->CMP = static_cast<uint32_t>(floorf(
                          pwmFraction * static_cast<float>(MOTOR_SPEED_PWM_MAX_VALUE) + 0.5f
                      ));
        // ARR: controls PWM frequency; 72 MHz clk / 10 kHz f_pwm = 7200 (see enum)
        LPTIM1->ARR = MOTOR_SPEED_PWM_MAX_VALUE;
        LPTIM1->CR |= LPTIM_CR_CNTSTRT;
    }
    else if (pwmFraction == 0.0f)
    {   // With the LPTIMs, we seem to only get duty cycles of 0% and 100% when
        // the timer is stopped.  The output polarity can be changed with the
        // WAVPOL bit to achieve constant 1 or 0.
        LPTIM1->CR = 0;
        LPTIM1->CFGR = LPTIM_CFGR_PRELOAD;
    }
    else
    {   // (See comment above)
        LPTIM1->CR = 0;
        LPTIM1->CFGR = LPTIM_CFGR_PRELOAD | LPTIM_CFGR_WAVPOL;
    }
}


// Perform I2C writes to set up the DRV10975 brushless motor controller to
// configure it for our HDD motor.
void DMotor::ConfigMotor()
{
    // EECtrl: SIdata=1 (enable writing to config registers), sleepDis=1 (prevent sleep mode)
    // TODO: figure out sleep mode later...
    WriteI2C(0x03, 0xc0);

    // The following motor-control parameters must be re-written after every
    // power-on or sleep/standby, because the DRV10975 loads its own defaults
    // from EEPROM at those times.

    // Motor Param1: DoubleFreq=0 (default 25-kHz PWM),
    //               Rm[6:0] motor phase-to-common resistance computed as
    //               R_ph_ct = X * .00735 ohms, X = Rm[3:0] << Rm[6:4]
    //                 => Rm[3:0]=15, Rm[6:4]=3 for R_ph_ct = 0.882 ohms
    WriteI2C(0x20, 0x3f);
    // Motor Param2: AdjMode=0 (full-cycle adj), Kt[6:0] back EMF constant computed as
    //               Kt = X / (1442 Hz/V), X = Kt[3:0] << Kt[6:4]
    //                 => Kt[3:0]=11, Kt[6:4]=1 for Kt = .0153 V/Hz
    WriteI2C(0x21, 0x1b);
    // Motor Param3: CtrlAdvMd=0 (fixed time), Tdelay[6:0] LR time constant computed as
    //               tau = X * 2.5 us, X = Tdelay[3:0] << Tdelay[6:4]
    //                 => Tdelay[3:0]=8, Tdelay[6:4]=0 for tau = 20 us
    WriteI2C(0x22, 0x2f);
    // SysOpt1: ISDThr=00 (tbd ISD stationary judgment threshold 6 Hz),
    //          IPDAdvcAgl=00 (tbd advancing angle after inductive sense 30 deg),
    //          ISDen=0 (disable Initial Speed Detect),
    //          RvsDrEn=0 (disable reverse drive),
    //          RvsDrThr=00 (tbd reverse-drive threshold 6.3 Hz)
    WriteI2C(0x23, 0x00);
    // SysOpt2: OpenLCurr=01 (open-loop current 0.4 A),
    //          OpLCurrRt=010 (open-loop current ramp-up 1.5 VCC/s),
    //          BrkDoneThr=000 (brake-done threshold: no brake)
    WriteI2C(0x24, 0x50);
    // SysOpt3: CtrlCoef=11 (control coef = 1.0),
    //          StAccel2=111 (tbd 0.22 Hz/s^2 2nd-order start-up accel),
    //          StAccel=011 (9.2 Hz/s 1st-order start-up accel)
    WriteI2C(0x25, 0xfb);
    // SysOpt4: Op2ClsThr=10011 (51.2-Hz open-to-closed-loop speed threshold),
    //          AlignTime=001 (2.7s)
    WriteI2C(0x26, 0x99);
    // SysOpt5: LockEn[3]=0 (disable no-motor fault),
    //          LockEn[2]=0 (disable abnormal Kt fault),
    //          LockEn[1]=0 (disable abnormal speed fault),
    //          LockEn[0]=0 (disable lock-detection current limit),
    //          AVSIndEn=0 (disable inductive AVS),
    //          AVSMEn=0 (disable mechanical AVS),
    //          AVSMMd=0 (AVS to Vcc)
    //          IPDRlsMd=0 (brake when inductive release)
    WriteI2C(0x27, 0x00);
    // SysOpt6: SWILimitThr=0010 (0.2A*2 accel current limit threshold),
    //          HWILimitThr=001 (0.4A*2 lock detection current limit threshold)
    WriteI2C(0x28, 0x22);
    // SysOpt7: LockEn[5]=0 (disable stuck-in-closed-loop fault),
    //          ClsLpAccel=100 (0.37 Vcc/s), Deadtime=1001 (40ns*10)
    WriteI2C(0x29, 0x49);
    // SysOpt8: IPDCurrThr=0000 (no IPD),
    //          LockEn[4]=0 (disable stuck-in-open-loop fault),
    //          VregSel=0 (5V), IPDClk=00 (tbd)
    WriteI2C(0x2a, 0x00);
    // SysOpt9: FGOLsel=00 (FG outputs in both open/closed loop),
    //          FGcycle=00 (one pulse per electrical cycle---best resolution)
    //          KtLckThr=11 (Kt_hi=2Kt, Kt_lo=0.5Kt)
    //          SpdCtrlMd=1 (SPEED pin takes PWM input)
    //          CLoopDis=0 (enable closed loop)
    WriteI2C(0x2b, 0x0e);
}       


// Perform an I2C write to the DRV10975 at slave address DRV10975_I2C_ADDRESS.
void DMotor::WriteI2C(uint8_t reg, uint8_t val)
{
    // Send address and wait for ACK
    I2C2->CR2 |= I2C_CR2_START;
    while (!(I2C2->ISR & I2C_ISR_TXIS))
    { }

    // Send register address byte
    I2C2->TXDR = reg;
    while (!(I2C2->ISR & I2C_ISR_TXIS))
    { }

    // Send data byte
    I2C2->TXDR = val;
    
    // That was the last byte in the write, so wait for and then clear STOP
    while (!(I2C2->ISR & I2C_ISR_STOPF))
    { }
    I2C2->ICR = I2C_ICR_STOPCF;
}
