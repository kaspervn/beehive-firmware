#ifndef BEEHIVE_MCU_FW_HARDWARE_H
#define BEEHIVE_MCU_FW_HARDWARE_H

#include <cstdint>

#define PWM_OUT_PERIOD (0x0FFF)             // The number of clock cycles of the PWM frequency
#define PWM_OUT_LIMIT (PWM_OUT_PERIOD/3)    // The maximum duty cycle. Can be somewhere between 0 and PWM_OUT_PERIOD
#define PWM_OUT_MAX PWM_OUT_LIMIT           // The max value you can give to hardware_set_coil_power()

typedef struct {
    uint32_t coil_pwm_period[3];
    uint32_t coil_pwm_pulse_width[3];
} hardware_state_t;

//Call this in the setup of your main.cpp
void hardware_setup();

//Call this in the loop of your main.cpp
hardware_state_t hardware_loop();

//Set the coil PWM output and phase (according to the sign of val
//n specifies which output. Valid values are 0,1,2. 0 is coil A, 1 is coil B, and 2 is coil C
//val specifies the pwm duty cycle. Valid values are between 0 and PWM_OUT_MAX
void hardware_set_coil_power(unsigned int n, int32_t val);

#endif
