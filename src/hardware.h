#ifndef BEEHIVE_MCU_FW_HARDWARE_H
#define BEEHIVE_MCU_FW_HARDWARE_H

#include <cstdint>

void hardware_init();
void hardware_set_pwm_out(unsigned int n, uint16_t val);

#endif
