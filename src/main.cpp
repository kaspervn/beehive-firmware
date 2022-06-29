#include <Arduino.h>
#include "hardware.h"
#include <cmath>

// ========== Settings Initialize
float g_test_coil_wave_freq = 0.25;                                 // in Hz
float g_test_coil_wave_amplitude = 0.8;                             // 0.0 - 1.0  => For the initialization Cycle
float g_test_coil_wave_phase_shift_1 = TWO_PI / 3.0f;               // Phase shift for Actuator 1 (Coil B)
float g_test_coil_wave_phase_shift_2 = 2.0f * TWO_PI / 3.0f;        // Phase shift for Actuator 2 (Coil C)

// ========== Constants for cycle and measurements
float pwm_max = 0.9*2048;                   // Max PWM (on top of hardware.h limit. 100% is 2.5 V.) ranges from 0 to fff (2048 both ways), so 0.5*2048 caps the pwm value of the cycle at max 50%
const int32_t N = 2301;                     // Number of values for duty cycle (Corresponded to 0.01% steps between 25 and 48 % Duty cycle)
float duty_Low = 17;                        // Hall effect measurement Duty cycle range lowest value (0-50)
float duty_High = 47;                       // Hall effect measurement Duty cycle range highest value (0-50)
int32_t K = 0;                              // Start Gain (Please note its /1000 in the code to allow it to be an int) Antidamp , K = 6, Odd: K = ~250
int32_t K_gain = 25;                        // For the gain sweep, how much gets added (or subtracted)
int32_t K_range = 350;                      // Maximum gain for the gain sweep (Sweeps from K => K_range with steps of K_Gain and then K => -K_range with same step size)
int32_t pwm_range[N], disp_range[N];        // Arrays for creating the hall effect lookup table PWM => Displacement

// ========== Global variables
int64_t g_loop_previous_loop_t = -1; // last known time of the start of the loop function.

// ========== Initial Constants (Glbbal, changed elsewhere in code)
int32_t pwm0_count = 0;
int32_t disp_Min0 = 0;
int32_t disp_Min1 = 0;
int32_t disp_Min2 = 0;
int32_t disp_Max0 = 0;
int32_t disp_Max1 = 0;
int32_t disp_Max2 = 0;
int32_t disp0 = 0;
int32_t disp1 = 0;
int32_t disp2 = 0;
int32_t t_switch_us = 0;
static bool coils_work = false;

// ========== Functions

//input: freq in hz, t in milliseconds, ampl in the range of 0-1
//output: sinus waveform between 0 and PWM_MAX
static int32_t pwm_sin(int32_t t, const float freq, const float ampl, const float phase_shift) {
    return (int32_t)(
            sinf(freq * t / 1000.0f * (float)TWO_PI + phase_shift)
            * ampl
            * (float)(PWM_OUT_MAX));
}

//input: integer of which Coil to test
//output: Changes boolean whether current is high enough through Coil (Sends 100% duty cycle, 5V for 20 ms and measures the current)
static bool test_coil(int n)
{
    float current_sense_result = sense_max_current(n % 3);
    bool ok = current_sense_result > 1.5f && current_sense_result < 2.5f;
    Serial.printf("coil %s: %s    max current: %d mA\r\n",
                  (n % 3 == 0) ? "A" : (n % 3 == 1 ? "B" : "C"),
                  ok ? "OK " : "BAD",
                  (int)(1000*current_sense_result));
    return ok;
}

// Runs to create lookup table for the disp functions
void disp_lookup_Generate() {
    // 5th order polynomial fit values from Instron data
    float fit_a = -8.4988;
    float fit_b = -0.486563976082542;
    float fit_c = -0.010265137615431;
    float fit_d = -1.272934888373004e-04;
    float fit_e = -8.369762396404029e-07;
    float fit_f = -2.302169255399056e-09;

    
    int32_t i;
    // Create lookup arrays with N values in specified duty cycle range
    for ( i=0; i<N; i++) {
        pwm_range[i] = duty_Low*1e7+i*((duty_High-duty_Low)*1e7/N);         // create range between duty_Low [%] and duty_High [%]  
        float Field_st = ((pwm_range[i]/1e7)-50)/0.3;                       // Convert to field strength (mT) (50% is 0 mT, then 0.3% per mT)
        
        // Calculate displacement in micrometers from the polynomial
        disp_range[i] = (fit_a+fit_b*Field_st+fit_c*pow(Field_st,float(2))+fit_d*pow(Field_st,float(3))+fit_e*pow(Field_st,float(4))+fit_f*pow(Field_st,float(5)))*1e3; 
        }
    Serial.printf("PWM range is %d, %d\r\n", pwm_range[0],pwm_range[2300]);
    Serial.printf("Measurement range is %d, %d\r\n", disp_range[0],disp_range[2300]);
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

// Runs some tests to find the state of the system
void initialize()
{
    int32_t pwm0_count = 0;
    uint64_t t_now_us = micros();

    // Check if all 3 coils work (do they have a sufficient current through them?)
    coils_work = true;
    for(int n = 0; n < 3; n++) {
        if(!test_coil(n))
        {
            coils_work = false;
        }
        delay(500);
    }
    Serial.write("\r\n");

    if(coils_work=true){set_led_color(50, 25, 0);}      // Set coil to orange/yellowish if the coils work

    // Run a sine wave profile (Settings above in the Settings initialize tab) Does this for 3 cycles
    static int32_t pwm0_old = 0;
    while (pwm0_count<3) { 
        t_now_us = micros();
        hardware_state_t hardware_state = hardware_loop();

        // Measure the duty cycle (and clip at readings that are out of bounds)
        int32_t duty0 = 1e9*hardware_state.coil_pwm_pulse_width[0]/hardware_state.coil_pwm_period[0]; // in ppb
        if (duty0 < duty_Low*1e7){duty0 = duty_Low*1e7;}            // Clipping of min measurement
        if (duty0 > duty_High*1e7){duty0 = duty_High*1e7;}          // Clipping of max measurement

        int32_t duty1 = 1e9*hardware_state.coil_pwm_pulse_width[1]/hardware_state.coil_pwm_period[1]; // in ppb
        if (duty1 < duty_Low*1e7){duty1 = duty_Low*1e7;}            // Clipping of min measurement
        if (duty1 > duty_High*1e7){duty1 = duty_High*1e7;}          // Clipping of max measurement

        int32_t duty2 = 1e9*hardware_state.coil_pwm_pulse_width[2]/hardware_state.coil_pwm_period[2]; // in ppb
        if (duty2 < duty_Low*1e7){duty2 = duty_Low*1e7;}            // Clipping of min measurement
        if (duty2 > duty_High*1e7){duty2 = duty_High*1e7;}          // Clipping of max measurement        

        // Calculate the coil pwm output from the timed sine waves
        int32_t t_now = millis();
        int32_t pwm0 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, 0.0f);
        int32_t pwm1 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, g_test_coil_wave_phase_shift_1);
        int32_t pwm2 = pwm_sin(t_now, g_test_coil_wave_freq, g_test_coil_wave_amplitude, g_test_coil_wave_phase_shift_2);

        if (100*pwm0/PWM_OUT_MAX==0 && 100*pwm0_old/PWM_OUT_MAX!=0) {pwm0_count++;}     // Detects 0 crossing of the setpoints (could be better...)
        pwm0_old = pwm0; // Store old PWM Value for detecting the 0 edge in next step

        // Set the coil PWM valuse
        hardware_set_coil_power(0, pwm0);
        hardware_set_coil_power(1, pwm1);
        hardware_set_coil_power(2, pwm2);

        // Find min and max values and store them when higher than previous values
        // In this way we find the endstop values (For the disp_corr function later)
        int32_t disp0 = disp(duty0); // From lookup table
        if (disp0 < disp_Min0 && disp0>-4000){disp_Min0 = disp0;} // store lowest disp
        if (disp0 > disp_Max0 && disp0< 4000){disp_Max0 = disp0;} // store highest disp

        int32_t disp1 = disp(duty1); // From lookup table    
        if (disp1 < disp_Min1 && disp1>-4000){disp_Min1 = disp1;} // store lowest disp
        if (disp1 > disp_Max1 && disp1< 4000){disp_Max1 = disp1;} // store highest disp

        int32_t disp2 = disp(duty2); // From lookup table
        if (disp2 < disp_Min2 && disp2>-4000){disp_Min2 = disp2;} // store lowest disp
        if (disp2 > disp_Max2 && disp2< 4000){disp_Max2 = disp2;} // store highest disp

        //Prints the coil pwm outputs in percentage of max effort, the (uncorrected) displacements and the time in microseconds
        Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %lu\r\n", 1000*pwm0/PWM_OUT_MAX, 1000*pwm1/PWM_OUT_MAX, 1000*pwm2/PWM_OUT_MAX, duty0, duty1, duty2,disp0,disp1,disp2,(uint32_t)t_now_us);
         
    }

    // Relax the actuators
    hardware_set_coil_power(0, 0);
    hardware_set_coil_power(1, 0);
    hardware_set_coil_power(2, 0);

    //Prints the endstop values it found, can be usefull for example to see if it actually hit the endstops (or to detect weird readings)
    Serial.printf("Calibration completed, endstop values are found: \r\n");
    Serial.printf("%d, %d, %d, %d, %d, %d\r\n", disp_Min0, disp_Min1, disp_Min2, disp_Max0, disp_Max1, disp_Max2);

}

// Perturbs the system by a programmable input (currently rising pwm0 for 0.01 s)
void perturb(){
    // Used for loop performance calculation
    uint64_t t_now_us = micros();
    uint64_t t_end_us = micros()+.01e6;
    uint32_t idx = 0;

    // Run loop for t_now until t_end => Should be 0.01s
    while (t_now_us<t_end_us){
        t_now_us = micros();
        uint64_t last_loop_duration_us = t_now_us - g_loop_previous_loop_t;
        g_loop_previous_loop_t = t_now_us;

         // Get the hardware functions
        hardware_state_t hardware_state = hardware_loop();

        // Get the duty cycles from the Hall sensors
        int32_t duty0 = 1e9*hardware_state.coil_pwm_pulse_width[0]/hardware_state.coil_pwm_period[0];
        if (duty0 < duty_Low*1e7){duty0 = duty_Low*1e7;}        // Clipping of min measurement
        if (duty0 > duty_High*1e7){duty0 = duty_High*1e7;}        //Clipping of max measurement

        int32_t duty1 = 1e9*hardware_state.coil_pwm_pulse_width[1]/hardware_state.coil_pwm_period[1];
        if (duty1 < duty_Low*1e7){duty1 = duty_Low*1e7;}        // Clipping of min measurement
        if (duty1 > duty_High*1e7){duty1 = duty_High*1e7;}        //Clipping of max measurement

        int32_t duty2 = 1e9*hardware_state.coil_pwm_pulse_width[2]/hardware_state.coil_pwm_period[2];
        if (duty2 < duty_Low*1e7){duty2 = duty_Low*1e7;}        // Clipping of min measurement
        if (duty2 > duty_High*1e7){duty2 = duty_High*1e7;}        //Clipping of max measurement 

        // Calculate the corrected positions of the actuators
        disp0 = corr_disp(duty0,disp_Min0,disp_Max0);
        disp1 = corr_disp(duty1,disp_Min1,disp_Max1);
        disp2 = corr_disp(duty2,disp_Min2,disp_Max2);

        // Ramp for pwm0 (a bit sketchy, but basically by knowing the frequency of the loop, I tuned it so it rises to 2000 in the set time. Loops 4 or 5x in the 0.01s) 
        int32_t pwm0 = idx*500;
        int32_t pwm1 = 0;
        int32_t pwm2 = 0;
        
        idx++;

        // Set those forces to a PWM value
        hardware_set_coil_power(0, pwm0);
        hardware_set_coil_power(1, pwm1);
        hardware_set_coil_power(2, pwm2);

        //Prints the coil pwm outputs in percentage of max effort, the positions [um], the time [us] and the loop duration time [us]
        Serial.printf("%d, %d, %d, %d, %d, %d, %lu, %lu\r\n",  1000*pwm0/PWM_OUT_MAX, 1000*pwm1/PWM_OUT_MAX, 1000*pwm2/PWM_OUT_MAX, disp0, disp1, disp2,(uint32_t)t_now_us,(uint32_t)last_loop_duration_us);
    }
}

// ========== Arduino style code, setup and then loop
#ifndef HARDWARE_TESTER
void setup() {
    // Begin serial communication at set baud rate and send some messages (give some time for serial port to start logging hence the delays)
    Serial.begin(19200);                                
    Serial.write("Start.");                   
    delay(2000);
    Serial.write("..Initializing...");
    Serial.printf("Gain = %d\r\n",K);

    hardware_setup();               // run the harware setup script from Kasper
    disp_lookup_Generate();         // Create the hall effect lookup table
    initialize();                   // Run the initialization (coil and hall sensor test)
    Serial.write("\r\n");
    Serial.write("Going to main loop...\r\n");
    set_led_color(0, 50, 0);

    // Good to go!
    delay(500);
    // perturb();                   // Perturb the system, only used when not cycling through gains, otherwise duplicate
    uint64_t t_end_us = micros()+5e6; // Not sure why this is here? 
}

void loop() {
    // Used for loop performance calculation
    uint64_t t_now_us = micros();
    uint64_t last_loop_duration_us = t_now_us - g_loop_previous_loop_t;
    g_loop_previous_loop_t = t_now_us;

    if (t_now_us > t_switch_us){
        t_switch_us = t_now_us+1.5e6;         // Window of 1.5 second including perturb and the delay at the end
        
        // Update the gain value
        K = K+K_gain;
        if ((K > K_range)|(K<-K_range)){K = 0; K_gain = -K_gain;}
        Serial.printf("Gain = %d \r\n",K); 

        // Relax the coils
        hardware_set_coil_power(0, 0);
        hardware_set_coil_power(1, 0);
        hardware_set_coil_power(2, 0);
        delay(500);
        perturb();
    }
    
    // Get the hardware functions
    hardware_state_t hardware_state = hardware_loop();

    // Get the duty cycles from the Hall sensors
    int32_t duty0 = 1e9*hardware_state.coil_pwm_pulse_width[0]/hardware_state.coil_pwm_period[0];
    if (duty0 < duty_Low*1e7){duty0 = duty_Low*1e7;}            // Clipping of min measurement
    if (duty0 > duty_High*1e7){duty0 = duty_High*1e7;}          // Clipping of max measurement

    int32_t duty1 = 1e9*hardware_state.coil_pwm_pulse_width[1]/hardware_state.coil_pwm_period[1];
    if (duty1 < duty_Low*1e7){duty1 = duty_Low*1e7;}            // Clipping of min measurement
    if (duty1 > duty_High*1e7){duty1 = duty_High*1e7;}          // Clipping of max measurement

    int32_t duty2 = 1e9*hardware_state.coil_pwm_pulse_width[2]/hardware_state.coil_pwm_period[2];
    if (duty2 < duty_Low*1e7){duty2 = duty_Low*1e7;}            // Clipping of min measurement
    if (duty2 > duty_High*1e7){duty2 = duty_High*1e7;}          // Clipping of max measurement 

    // Store old values for velocity calculation dx/dt = (x_i + x_i-1)/dt
    int32_t disp0_old = disp0;
    int32_t disp1_old = disp1;
    int32_t disp2_old = disp2;

    // Calculate the corrected positions of the actuators
    disp0 = corr_disp(duty0,disp_Min0,disp_Max0);
    disp1 = corr_disp(duty1,disp_Min1,disp_Max1);
    disp2 = corr_disp(duty2,disp_Min2,disp_Max2);

    // Calculate Velocity
    int32_t vel0 = (disp0-disp0_old)/(1e-6*last_loop_duration_us); // Velocity in um/s
    int32_t vel1 = (disp1-disp1_old)/(1e-6*last_loop_duration_us); // Velocity in um/s
    int32_t vel2 = (disp2-disp2_old)/(1e-6*last_loop_duration_us); // Velocity in um/s

    //=================== Control function ============================================

    // 2 Options, odd coupling or the antidamping. Can basically do anything here as long as the output makes sense
    // Reminder: pwm range: [-2048:2048], 2048 is 2.5V, which means 2.5/3.2 = 0.78A 
    // Coil can handle 0.3 A (~1V = 820) continuous, 0.6A RMS (~2V = 1640) for 30s  (for the real nerd: I_max = 3000e^(-10.6t))

    // Calculation of odd forces 
    int32_t pwm0 = (disp1-disp2)*K/1000;
    int32_t pwm1 = (disp2-disp0)*K/1000;
    int32_t pwm2 = (disp0-disp1)*K/1000;

    // AntiDamping
    // int32_t pwm0 = K*vel0/3000;
    // int32_t pwm1 = K*vel1/3000;
    // int32_t pwm2 = K*vel2/3000;

    //================== Set coil output ==============================================

    // Some soft coded Protection (safety first)
    if (pwm0>pwm_max){pwm0 = pwm_max;}
    if (pwm0<-pwm_max){pwm0 = -pwm_max;}
    if (pwm1>pwm_max){pwm1= pwm_max;}
    if (pwm1<-pwm_max){pwm1 = -pwm_max;}
    if (pwm2>pwm_max){pwm2 = pwm_max;}
    if (pwm2<-pwm_max){pwm2 = -pwm_max;}

    // Set those forces to a PWM value
    hardware_set_coil_power(0, pwm0);
    hardware_set_coil_power(1, pwm1);
    hardware_set_coil_power(2, pwm2);
    
    //Prints the coil pwm outputs in promilage of max effort (2.5V), the positions [um], the time [us] and the loop duration time [us]
    Serial.printf("%d, %d, %d, %d, %d, %d, %lu, %lu\r\n",  1000*pwm0/PWM_OUT_MAX, 1000*pwm1/PWM_OUT_MAX, 1000*pwm2/PWM_OUT_MAX, disp0, disp1, disp2,(uint32_t)t_now_us,(uint32_t)last_loop_duration_us);
}
#else

void setup() {
    Serial.begin(9600);
    hardware_setup();


    set_led_color(50, 0, 0);
    delay(250);
    set_led_color(0, 50, 0);
    delay(250);
    set_led_color(0, 0, 50);
    delay(250);
    set_led_color(50, 0, 50);
    delay(250);
    set_led_color(50, 50, 0);
    delay(250);
    set_led_color(0, 0, 0);
}

static bool test_coil(int n)
{
    float current_sense_result = sense_max_current(n % 3);
    bool ok = current_sense_result > 1.5f && current_sense_result < 2.5f;
    Serial.printf("coil %s: %s    max current: %d mA\r\n",
                  (n % 3 == 0) ? "A" : (n % 3 == 1 ? "B" : "C"),
                  ok ? "OK " : "BAD",
                  (int)(1000*current_sense_result));
    return ok;
}

static bool test_hall_readout(int n)
{
    try_read_pwm_in_res res = try_read_pwm_in(n);
    bool ok = res.timeout_count == 0 &&
              res.last_period > 23000 &&
              res.last_period < 25000 &&
              res.last_pulse_width > (int)(res.last_period * 0.1f) &&
              res.last_pulse_width < (int)(res.last_period * 0.9f);
    Serial.printf("hall %s: %s    %d timeouts out of %d tries. Last cycle: %d/%d\r\n",
                  (n % 3 == 0) ? "A" : (n % 3 == 1 ? "B" : "C"),
                  ok ? "OK " : "BAD",
                  res.timeout_count, res.try_count, res.last_pulse_width, res.last_period);
    return ok;
}

static bool coils_work = false;
static bool hall_sensors_work = false;

void loop() {

    coils_work = true;
    for(int n = 0; n < 3; n++) {
        if(!test_coil(n))
        {
            coils_work = false;
        }
        delay(1000);
    }
    Serial.write("\r\n");

    hall_sensors_work = true;
    for(int n = 0; n < 3; n++) {
        if(!test_hall_readout(n))
        {
            hall_sensors_work = false;
        }
        delay(1000);
    }
    Serial.write("\r\n\r\n");

    if(coils_work && hall_sensors_work)
    {
        set_led_color(0, 50, 0);
    }
    else if(coils_work && !hall_sensors_work)
    {
        set_led_color(50, 50, 0);
    }
    else if(!coils_work && hall_sensors_work)
    {
        set_led_color(50, 0, 50);
    }
    else {
        set_led_color(50, 0, 0);
    }
}

#endif