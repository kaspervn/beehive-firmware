#include <Arduino.h>
#include "hardware.h"
#include <cmath>

// ========== Settings Initialize
float g_test_coil_wave_freq = 1;            // in Hz
float g_test_coil_wave_amplitude = 0.8;     // 0.0 - 1.0
float g_test_coil_wave_phase_shift_1 = TWO_PI / 3.0f;
float g_test_coil_wave_phase_shift_2 = 2.0f * TWO_PI / 3.0f;

// ========== Constants for cycle and measurements
const int32_t N = 2301;                     // Number of values for duty cycle (Corresponded to 0.01% steps between 25 and 48 % Duty cycle)
float duty_Low = 25;                        // Duty cycle range lowest value
float duty_High = 48;                       // Duty cycle range highest value
int32_t K = 333;                            // Gain (Please note its /1000 in the code to allow it to be an int)
int32_t pwm_range[N], disp_range[N];

// ========== Global variables
int64_t g_loop_previous_loop_t = -1; // last known time of the start of the loop function.

// ========== Initial Constants
int32_t pwm0_count = 0;
int32_t disp_Min0 = 0;
int32_t disp_Min1 = 0;
int32_t disp_Min2 = 0;
int32_t disp_Max0 = 0;
int32_t disp_Max1 = 0;
int32_t disp_Max2 = 0;

// ========== Functions
//input: freq in hz, t in milliseconds, ampl in the range of 0-1
//output: sinus waveform between 0 and PWM_MAX
static int32_t pwm_sin(int32_t t, const float freq, const float ampl, const float phase_shift) {
    return (int32_t)(
            sinf(freq * t / 1000.0f * (float)TWO_PI + phase_shift)
            * ampl
            * (float)(PWM_OUT_MAX));
}

// Runs to create lookup table for the disp functions
void disp_lookup_Generate() {
    // 5th order polynomial fit values from Instron
    float fit_a = -6.381217309199334;
    float fit_b = 0.3472;
    float fit_c = -0.0064;
    float fit_d = 6.4509e-05;
    float fit_e = -2.6059e-07;
    int32_t i;

    // Create lookup arrays with N values in specified duty cycle range
    for ( i=0; i<N; i++) {
        pwm_range[i] = duty_Low*1e7+i*((duty_High-duty_Low)*1e7/N); // create range between 25% and 48%  
        int32_t Field_st = ((100-pwm_range[i]/1e7)-50)/0.3;         // Convert to field strength (mT) to have correct fit...
        disp_range[i] = (fit_a+fit_b*Field_st+fit_c*pow(Field_st,float(2))+fit_d*pow(Field_st,float(3))+fit_e*pow(Field_st,float(4)))*1e3; // displacement in micrometers
    }
}

//input: Duty cycle in (0-1)*1e9
//output: Position (um) based on Instron Fit
static int32_t disp(int32_t duty) {
    return (int32_t)(
            disp_range[int32_t((duty-25e7)*1e-5)]); // Calculate the closest integer  
}

//input: Duty cycle in (0-1)*1e9, minimum displacement (um) and maximum displacement (um)
//output: Position (um) based on Instron Fit and Centered between min and max values (Endstops)
static int32_t corr_disp(int32_t duty, int32_t disp_Min, int32_t disp_Max) {
    return (int32_t)(
            disp_range[int32_t((duty-25e7)*1e-5)]) - disp_Min -(disp_Max-disp_Min)/2; // Calculate the closest integer  
}

void initialize()
{
// Run an initial sine sweep to find the edges
    int32_t pwm0_count = 0;

    static int32_t pwm0_old = 0;
    while (pwm0_count<3) { 
        hardware_state_t hardware_state = hardware_loop();

        // Measure the duty cycle
        int32_t duty0 = 1e9*hardware_state.coil_pwm_pulse_width[0]/hardware_state.coil_pwm_period[0];
        int32_t duty1 = 1e9*hardware_state.coil_pwm_pulse_width[1]/hardware_state.coil_pwm_period[1];
        int32_t duty2 = 1e9*hardware_state.coil_pwm_pulse_width[2]/hardware_state.coil_pwm_period[2];

        // Calculate the coil pwm output
        int32_t t_now = millis();
        int32_t pwm0 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, 0.0f);
        int32_t pwm1 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, g_test_coil_wave_phase_shift_1);
        int32_t pwm2 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, g_test_coil_wave_phase_shift_2);

        if (100*pwm0/PWM_OUT_MAX==0 && 100*pwm0_old/PWM_OUT_MAX!=0) {pwm0_count++;}     // Detects 0 crossing of the setpoints

        // Set the coil PWM valuse
        hardware_set_coil_power(0, pwm0);
        hardware_set_coil_power(1, pwm1);
        hardware_set_coil_power(2, pwm2);

        // Find min and max values and store them when higher than previous values
        // In this way we find the endstop values (For the disp_corr function later)
        int32_t disp0 = disp(duty0);
        if (disp0 < disp_Min0 && disp0>-4000){disp_Min0 = disp0;}
        if (disp0 > disp_Max0 && disp0< 4000){disp_Max0 = disp0;}

        int32_t disp1 = disp(duty1);    
        if (disp1 < disp_Min1 && disp1>-4000){disp_Min1 = disp1;}
        if (disp1 > disp_Max1 && disp1< 4000){disp_Max1 = disp1;}

        int32_t disp2 = disp(duty2);
        if (disp2 < disp_Min2 && disp2>-4000){disp_Min2 = disp2;}
        if (disp2 > disp_Max2 && disp2< 4000){disp_Max2 = disp2;}

        //Prints the coil pwm outputs in percentage of max effort, and the loop duration time in microseconds
        Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", 100*pwm0/PWM_OUT_MAX, 100*pwm1/PWM_OUT_MAX, 100*pwm2/PWM_OUT_MAX, duty0, duty1, duty2,disp0,disp1,disp2);
         
        pwm0_old = pwm0; // Store old PWM Value
    }

    Serial.printf("Calibration completed, endstop values are found: \r\n");
    Serial.printf("%d, %d, %d, %d, %d, %d\r\n", disp_Min0, disp_Min1, disp_Min2, disp_Max0, disp_Max1, disp_Max2);

}

// ========== Arduino stuff
void setup() {
    delay(2000);
    Serial.begin(9600);
    Serial.write("Start.");
    delay(2000);
    Serial.write("..Initializing...");
    hardware_setup();
    disp_lookup_Generate(); // Create the lookup table
    initialize(); // Run sine wave as initialization of setup
    Serial.write("\r\n");
    Serial.write("Going to main loop...\r\n");
    delay(1500);
}

void loop() {
    
    // Used for loop performance calculation
    int64_t t_now_us = micros();
    int64_t last_loop_duration_us = t_now_us - g_loop_previous_loop_t;
    g_loop_previous_loop_t = t_now_us;

    hardware_state_t hardware_state = hardware_loop();

    // Get the duty cycles from the Hall sensors
    int32_t duty0 = 1e9*hardware_state.coil_pwm_pulse_width[0]/hardware_state.coil_pwm_period[0];
    int32_t duty1 = 1e9*hardware_state.coil_pwm_pulse_width[1]/hardware_state.coil_pwm_period[1];
    int32_t duty2 = 1e9*hardware_state.coil_pwm_pulse_width[2]/hardware_state.coil_pwm_period[2];

    // Calculate the corrected positions of the actuators
    int32_t disp0 = corr_disp(duty0,disp_Min0,disp_Max0);
    int32_t disp1 = corr_disp(duty1,disp_Min1,disp_Max1);
    int32_t disp2 = corr_disp(duty2,disp_Min2,disp_Max2);

    // Calculation of odd forces
    int32_t pwm0 = (disp1-disp2)*K/1000;
    int32_t pwm1 = (disp2-disp0)*K/1000;
    int32_t pwm2 = (disp0-disp1)*K/1000;

    // Set those forces to a PWM value
    hardware_set_coil_power(0, pwm0);
    hardware_set_coil_power(1, pwm1);
    hardware_set_coil_power(2, pwm2);
    
    //Prints the coil pwm outputs in percentage of max effort, the positions [um], the time [us] and the loop duration time [us]
    Serial.printf("%d, %d, %d, %d, %d, %d, %lu, %lu\r\n",  100*pwm0/PWM_OUT_MAX, 100*pwm1/PWM_OUT_MAX, 100*pwm2/PWM_OUT_MAX, disp0, disp1, disp2,(uint32_t)t_now_us,(uint32_t)last_loop_duration_us);
}
