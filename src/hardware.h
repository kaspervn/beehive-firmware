#ifndef BEEHIVE_MCU_FW_HARDWARE_H
#define BEEHIVE_MCU_FW_HARDWARE_H

#include <cstdint>

#define PWM_OUT_PERIOD (0x0FFF)             // The number of clock cycles of the PWM frequency
#define PWM_OUT_LIMIT (PWM_OUT_PERIOD/3)    // The maximum duty cycle. Can be somewhere between 0 and PWM_OUT_PERIOD
#define PWM_OUT_MAX PWM_OUT_LIMIT           // The max value you can give to hardware_set_pwm_out()

//Call this in the setup of your main.cpp
void hardware_init();

//Set the coil PWM output
//n specifies which output. Valid values are 0,1,2
//val specifies the pwm duty cycle. Valid values are between 0 and PWM_OUT_MAX
void hardware_set_pwm_out(unsigned int n, uint16_t val);

#endif
