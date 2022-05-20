#include <Arduino.h>
#include "hardware.h"
#include <cmath>

// ========== Settings
float g_test_coil_wave_freq = 1; // in Hz
float g_test_coil_wave_amplitude = 0.8; // 0.0 - 1.0
float g_test_coil_wave_phase_shift_1 = TWO_PI / 3.0f;
float g_test_coil_wave_phase_shift_2 = 2.0f * TWO_PI / 3.0f;


// ========== Global variables
int64_t g_loop_previous_loop_t = -1; // last known time of the start of the loop function.


// ========== Functions
//input: freq in hz, t in milliseconds, ampl in the range of 0-1
//output: sinus waveform between 0 and PWM_MAX
static int32_t pwm_sin(int32_t t, const float freq, const float ampl, const float phase_shift) {
    return (int32_t)(
            sinf(freq * t / 1000.0f * (float)TWO_PI + phase_shift)
            * ampl
            * (float)(PWM_OUT_MAX));
}


// ========== Arduino stuff
void setup() {
    Serial.begin(9600);
    Serial.write("Start\r\n");
    hardware_setup();

}

void loop() {
    // Used for loop performance calculation
    int64_t t_now_us = micros();
    int64_t last_loop_duration_us = t_now_us - g_loop_previous_loop_t;
    g_loop_previous_loop_t = t_now_us;

    hardware_state_t hardware_state = hardware_loop();

    int32_t duty0 = 1e9*hardware_state.coil_pwm_pulse_width[0]/hardware_state.coil_pwm_period[0];
    int32_t duty1 = 1e9*hardware_state.coil_pwm_pulse_width[1]/hardware_state.coil_pwm_period[1];
    int32_t duty2 = 1e9*hardware_state.coil_pwm_pulse_width[2]/hardware_state.coil_pwm_period[2];

    //Calculate the coil pwm output
    int32_t t_now = millis();

    int32_t pwm0 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, 0.0f);
    int32_t pwm1 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, g_test_coil_wave_phase_shift_1);
    int32_t pwm2 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, g_test_coil_wave_phase_shift_2);

    hardware_set_coil_power(0, pwm0);
    hardware_set_coil_power(1, pwm1);
    hardware_set_coil_power(2, pwm2);

    //Prints the coil pwm outputs in percentage of max effort, and the loop duration time in microseconds
    Serial.printf("%d, %d, %d, %d, %d, %d, %lu\r\n", 100*pwm0/PWM_OUT_MAX, 100*pwm1/PWM_OUT_MAX, 100*pwm2/PWM_OUT_MAX, duty0, duty1, duty2,(uint32_t)last_loop_duration_us);
}
