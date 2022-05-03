#include <Arduino.h>
#include "hardware.h"

void setup() {
    hardware_init();
}

void loop()
{
    delay(10);
    int32_t t = millis();
    hardware_set_pwm_out(0, (t % 10000) * 0xFFFF / 10000);
    hardware_set_pwm_out(1, ((t+5000) % 10000) * 0xFFFF / 10000);
    hardware_set_pwm_out(2, ((t+2500) % 10000) * 0xFFFF / 10000);
}