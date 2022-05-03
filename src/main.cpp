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
}