#include "Arduino.h"
#include "hardware.h"
#include <vector>
#include "sam.h"
#include "samd21e18a.h"
#include <tcc.h>
#include <cstdint>
#include <Adafruit_TinyUSB_API.h>
#include <extint.h>
#include <system_interrupt.h>
#include "events.h"


#include "hardware.h"

//microcontroller: SAMD21E18A-F

struct events_resource pwm_in_event_1, pwm_in_event_2, pwm_in_event_3;

//Coil A hall sensor in
#define PWM_IN_1_2_TCC_NR 1
#define PWM_IN_1_2_TCC TCC1
#define PWM_IN_1_2_EVSYS_USR EVSYS_ID_USER_TCC1_EV_1
#define PWM_IN_1_EXTINT_PIN PIN_PA31A_EIC_EXTINT11
#define PWM_IN_1_EXTINT_MUX MUX_PA31A_EIC_EXTINT11
#define PWM_IN_1_EXTINT_NR 11
#define PWM_IN_1_EVSYS_GEN EVSYS_ID_GEN_EIC_EXTINT_11

//Coil B hall sensor in
#define PWM_IN_2_EXTINT_PIN PIN_PA16A_EIC_EXTINT0
#define PWM_IN_2_EXTINT_MUX MUX_PA16A_EIC_EXTINT0
#define PWM_IN_2_EXTINT_NR 0
#define PWM_IN_2_EVSYS_GEN EVSYS_ID_GEN_EIC_EXTINT_0

//Coil C hall sensor in
#define PWM_IN_3_TCC_NR 2
#define PWM_IN_3_TCC TCC2
#define PWM_IN_3_EVSYS_USR EVSYS_ID_USER_TCC2_EV_1
#define PWM_IN_3_EXTINT_PIN PIN_PA07A_EIC_EXTINT7
#define PWM_IN_3_EXTINT_MUX MUX_PA07A_EIC_EXTINT7
#define PWM_IN_3_EXTINT_NR 7
#define PWM_IN_3_EVSYS_GEN EVSYS_ID_GEN_EIC_EXTINT_7

static tcc_module tcc_modules[3];
static Tcc* tcc_hardware[3] = {TCC0, TCC1, TCC2};

typedef struct {
    Tcc* TCC;
    uint32_t channel, output, pin, pin_mux;
    tcc_module* module;
} pwm_config_t;


//Not every channel can use every WOx. Please refer to 31.6.3.7 "Waveform Extension" of the SAM D21/DA1 datasheet
//By default:
//  channel 0: output WO0 and WO4
//  channel 1: output WO1 and WO5
//  channel 2: output WO2 and WO6  (only available on TCC0)
//  channel 3: output WO3 and WO7  (only available on TCC0)
static std::vector<pwm_config_t> pwm_configurations = {
        {.TCC = TCC0, .channel = 0, .output = 0, .pin = PIN_PA04E_TCC0_WO0, .pin_mux = PINMUX_PA04E_TCC0_WO0}, // Coil A
        {.TCC = TCC0, .channel = 3, .output = 3, .pin = PIN_PA19F_TCC0_WO3, .pin_mux = PINMUX_PA19F_TCC0_WO3}, // Coil B
        {.TCC = TCC0, .channel = 1, .output = 1, .pin = PIN_PA09E_TCC0_WO1, .pin_mux = PINMUX_PA09E_TCC0_WO1}, // Coil C
        };


#define GPIO_COIL_A_PHASE PIN_PA01
#define GPIO_COIL_B_PHASE PIN_PA27
#define GPIO_COIL_C_PHASE PIN_PA08
static uint8_t gpio_coil_phase_pins[3] = {1, 27, 8};

static void init_output_pwm() {

    for(int tcc_idx = 0; tcc_idx < 3; tcc_idx++) {

        bool enable_tcc = false;
        for(int n = 0; n < pwm_configurations.size(); n++)
        {
            if(pwm_configurations[n].TCC == tcc_hardware[tcc_idx]) {
                enable_tcc = true;
                break;
            }
        }

        if(enable_tcc) {
            struct tcc_config config_tcc;
            tcc_get_config_defaults(&config_tcc, tcc_hardware[tcc_idx]);
            config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
            config_tcc.counter.period = PWM_OUT_PERIOD;
            config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

            for(int n = 0; n < pwm_configurations.size(); n++)
            {
                if(pwm_configurations[n].TCC == tcc_hardware[tcc_idx]) {
                    config_tcc.compare.match[pwm_configurations[n].channel] = 0;
                    config_tcc.pins.enable_wave_out_pin[pwm_configurations[n].output] = true;
                    config_tcc.pins.wave_out_pin[pwm_configurations[n].output] = pwm_configurations[n].pin;
                    config_tcc.pins.wave_out_pin_mux[pwm_configurations[n].output] = pwm_configurations[n].pin_mux;
                }
            }

            tcc_init(&tcc_modules[tcc_idx], tcc_hardware[tcc_idx], &config_tcc);

            for(int n = 0; n < pwm_configurations.size(); n++)
            {
                if(pwm_configurations[n].TCC == tcc_hardware[tcc_idx]) {
                    pwm_configurations[n].module = &tcc_modules[tcc_idx];
                }
            }

            tcc_enable(&tcc_modules[tcc_idx]);
        }
    }
}

void hardware_set_coil_power(unsigned int n, int32_t val)
{
    //gpio_pin_set_output_level(GPIO_COIL_PHASE_PINS[n], sign > 0);
    if(val > 0) {
        PORT->Group[0].OUTCLR.reg = (1ul << gpio_coil_phase_pins[n]);
    } else {
        PORT->Group[0].OUTSET.reg = (1ul << gpio_coil_phase_pins[n]);
    }

    uint32_t val_abs = abs(val);
    uint32_t real_pwm = std::min(val_abs, (uint32_t)(PWM_OUT_MAX-1));
    tcc_set_compare_value(pwm_configurations[n].module,
                          static_cast<const tcc_match_capture_channel>(pwm_configurations[n].channel), real_pwm);
}

static void init_coil_phase_pins()
{
    for(int n = 0; n < 3; n++)
        PORT->Group[0].DIRSET.reg = (1ul << gpio_coil_phase_pins[n]);
}

static void init_pwm_in()
{
    struct tcc_config config_tcc;
    tcc_get_config_defaults(&config_tcc, PWM_IN_1_2_TCC);
    config_tcc.counter.clock_source         = GCLK_GENERATOR_0;
    config_tcc.counter.clock_prescaler      = TCC_CLOCK_PRESCALER_DIV1;
    config_tcc.compare.channel_function[0]  = TCC_CHANNEL_FUNCTION_CAPTURE;
    config_tcc.compare.channel_function[1]  = TCC_CHANNEL_FUNCTION_CAPTURE;
    config_tcc.double_buffering_enabled     = false;
    tcc_init(&tcc_modules[PWM_IN_1_2_TCC_NR], PWM_IN_1_2_TCC, &config_tcc);
    tcc_get_config_defaults(&config_tcc, PWM_IN_3_TCC);
    config_tcc.counter.clock_source         = GCLK_GENERATOR_0;
    config_tcc.counter.clock_prescaler      = TCC_CLOCK_PRESCALER_DIV1;
    config_tcc.compare.channel_function[0]  = TCC_CHANNEL_FUNCTION_CAPTURE;
    config_tcc.compare.channel_function[1]  = TCC_CHANNEL_FUNCTION_CAPTURE;
    config_tcc.double_buffering_enabled     = false;
    tcc_init(&tcc_modules[PWM_IN_3_TCC_NR], PWM_IN_3_TCC, &config_tcc);

    struct tcc_events events_tcc;
    events_tcc.input_config[0].modify_action      = false;
    events_tcc.input_config[1].modify_action      = true;
    events_tcc.on_input_event_perform_action[1]   = true;
    events_tcc.input_config[1].action             = TCC_EVENT_ACTION_PULSE_WIDTH_PERIOD_CAPTURE;
    tcc_enable_events(&tcc_modules[PWM_IN_1_2_TCC_NR], &events_tcc);
    tcc_enable(&tcc_modules[PWM_IN_1_2_TCC_NR]);
    tcc_enable_events(&tcc_modules[PWM_IN_3_TCC_NR], &events_tcc);
    tcc_enable(&tcc_modules[PWM_IN_3_TCC_NR]);

    struct extint_chan_conf config_extint_chan;
    extint_chan_get_config_defaults(&config_extint_chan);
    config_extint_chan.gpio_pin           = PWM_IN_1_EXTINT_PIN;
    config_extint_chan.gpio_pin_mux       = PWM_IN_1_EXTINT_MUX;
    config_extint_chan.gpio_pin_pull      = EXTINT_PULL_NONE;
    config_extint_chan.detection_criteria = EXTINT_DETECT_HIGH;
    extint_chan_set_config(PWM_IN_1_EXTINT_NR, &config_extint_chan);
    config_extint_chan.gpio_pin           = PWM_IN_2_EXTINT_PIN;
    config_extint_chan.gpio_pin_mux       = PWM_IN_2_EXTINT_MUX;
    extint_chan_set_config(PWM_IN_2_EXTINT_NR, &config_extint_chan);
    config_extint_chan.gpio_pin           = PWM_IN_3_EXTINT_PIN;
    config_extint_chan.gpio_pin_mux       = PWM_IN_3_EXTINT_MUX;
    extint_chan_set_config(PWM_IN_3_EXTINT_NR, &config_extint_chan);

    struct extint_events config_events = {0};
    config_events.generate_event_on_detect[PWM_IN_1_EXTINT_NR] = true;
    config_events.generate_event_on_detect[PWM_IN_2_EXTINT_NR] = true;
    config_events.generate_event_on_detect[PWM_IN_3_EXTINT_NR] = true;
    extint_enable_events(&config_events);

    struct events_config config;
    events_get_config_defaults(&config);
    config.clock_source = GCLK_GENERATOR_0;
    config.generator    = PWM_IN_1_EVSYS_GEN;
    config.path         = EVENTS_PATH_ASYNCHRONOUS;
    config.edge_detect  = EVENTS_EDGE_DETECT_BOTH;
    events_allocate(&pwm_in_event_1, &config);
    config.generator    = PWM_IN_2_EVSYS_GEN;
    events_allocate(&pwm_in_event_2, &config);
    config.generator    = PWM_IN_3_EVSYS_GEN;
    events_allocate(&pwm_in_event_3, &config);

    events_attach_user(&pwm_in_event_1, PWM_IN_1_2_EVSYS_USR);
    events_attach_user(&pwm_in_event_2, PWM_IN_1_2_EVSYS_USR);
    events_attach_user(&pwm_in_event_3, PWM_IN_3_EVSYS_USR);
}

void hardware_setup()
{
    system_extint_init();
    system_events_init();
    system_interrupt_enable_global();

    init_coil_phase_pins();
    init_output_pwm();
    init_pwm_in();

}

hardware_state_t hardware_loop()
{
    hardware_state_t hardware_state;

    //flush the buffer to remove an invalid value from the previous cycle
    while(!(PWM_IN_1_2_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_1_2_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);

    while(!(PWM_IN_1_2_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_1_2_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    hardware_state.coil_pwm_period[1] = tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    hardware_state.coil_pwm_pulse_width[1] = tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);

    //switch to channel 1
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(pwm_in_event_1.channel + 1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1);

    //flush the buffer to remove an invalid value from the previous cycle
    while(!(PWM_IN_1_2_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_1_2_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);
    while(!(PWM_IN_1_2_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_1_2_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    hardware_state.coil_pwm_period[0] = tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    hardware_state.coil_pwm_pulse_width[0] = tcc_get_capture_value(&tcc_modules[PWM_IN_1_2_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);

    //switch to channel 2 for the next round
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(pwm_in_event_2.channel + 1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1);

    while(!(PWM_IN_3_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_3_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    hardware_state.coil_pwm_period[2] = tcc_get_capture_value(&tcc_modules[PWM_IN_3_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    hardware_state.coil_pwm_pulse_width[2] = tcc_get_capture_value(&tcc_modules[PWM_IN_3_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);

    return hardware_state;
}