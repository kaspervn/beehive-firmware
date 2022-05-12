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

struct events_resource pwm_in_event_rsrc1, pwm_in_event_rsrc2;
#define PWM_IN_TCC_NR 1
#define PWM_IN_TCC TCC1

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
//  channel 4: output WO3 and WO7  (only available on TCC0)
static std::vector<pwm_config_t> pwm_configurations = {
        {.TCC = TCC0, .channel = 0, .output = 0, .pin = PIN_PA04E_TCC0_WO0, .pin_mux = PINMUX_PA04E_TCC0_WO0}, // Coil A
        {.TCC = TCC0, .channel = 1, .output = 5, .pin = PIN_PA23F_TCC0_WO5, .pin_mux = PINMUX_PA23F_TCC0_WO5}, // Coil B
        {.TCC = TCC0, .channel = 3, .output = 1, .pin = PIN_PA09E_TCC0_WO1, .pin_mux = PINMUX_PA09E_TCC0_WO1}, // Coil C
        };


#define GPIO_COIL_A_PHASE PIN_PA01
#define GPIO_COIL_B_PHASE PIN_PA27
#define GPIO_COIL_C_PHASE PIN_PA08
#define GPIO_COIL_PHASE_PINS {GPIO_COIL_A_PHASE, GPIO_COIL_B_PHASE, GPIO_COIL_C_PHASE}
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

static void inin_pwm_in_tcc()
{
    struct tcc_config config_tcc;
    tcc_get_config_defaults(&config_tcc, PWM_IN_TCC);
    config_tcc.counter.clock_source         = GCLK_GENERATOR_0;
    config_tcc.counter.clock_prescaler      = TCC_CLOCK_PRESCALER_DIV1;
    config_tcc.compare.channel_function[0]  = TCC_CHANNEL_FUNCTION_CAPTURE;
    config_tcc.compare.channel_function[1]  = TCC_CHANNEL_FUNCTION_CAPTURE;
    config_tcc.double_buffering_enabled     = false;
    tcc_init(&tcc_modules[PWM_IN_TCC_NR], PWM_IN_TCC, &config_tcc);

    struct tcc_events events_tcc;
    events_tcc.input_config[0].modify_action      = false;
    events_tcc.input_config[1].modify_action      = true;
    events_tcc.on_input_event_perform_action[1]   = true;
    events_tcc.input_config[1].action             = TCC_EVENT_ACTION_PULSE_WIDTH_PERIOD_CAPTURE;
    tcc_enable_events(&tcc_modules[PWM_IN_TCC_NR], &events_tcc);
    tcc_enable(&tcc_modules[PWM_IN_TCC_NR]);

}

static void init_pwm_in()
{
    inin_pwm_in_tcc();

    struct extint_chan_conf config_extint_chan;
    extint_chan_get_config_defaults(&config_extint_chan);
    config_extint_chan.gpio_pin           = PIN_PA31A_EIC_EXTINT11;
    config_extint_chan.gpio_pin_mux       = MUX_PA31A_EIC_EXTINT11;
    config_extint_chan.gpio_pin_pull      = EXTINT_PULL_NONE;
    config_extint_chan.detection_criteria = EXTINT_DETECT_HIGH;
    extint_chan_set_config(11, &config_extint_chan);
    // ##########
    config_extint_chan.gpio_pin           = PIN_PA16A_EIC_EXTINT0;
    config_extint_chan.gpio_pin_mux       = MUX_PA16A_EIC_EXTINT0;
    extint_chan_set_config(0, &config_extint_chan);
    // ##########
    struct extint_events config_events = {0};
    config_events.generate_event_on_detect[3] = true;
    config_events.generate_event_on_detect[5] = true;
    extint_enable_events(&config_events);

    struct events_config config;
    events_get_config_defaults(&config);
    config.clock_source = GCLK_GENERATOR_0;
    config.generator    = EVSYS_ID_GEN_EIC_EXTINT_11;
    config.path         = EVENTS_PATH_ASYNCHRONOUS;
    config.edge_detect  = EVENTS_EDGE_DETECT_BOTH;
    events_allocate(&pwm_in_event_rsrc1, &config);
    config.generator    = EVSYS_ID_GEN_EIC_EXTINT_0;
    events_allocate(&pwm_in_event_rsrc2, &config);

    events_attach_user(&pwm_in_event_rsrc1, EVSYS_ID_USER_TCC1_EV_1);
    events_attach_user(&pwm_in_event_rsrc2, EVSYS_ID_USER_TCC1_EV_1);
    //EVSYS->CHANNEL.reg = pwm_in_event_rsrc1.channel_reg;
}

void hardware_setup()
{
    system_extint_init();
    system_events_init();
    system_interrupt_enable_global();


    // init_coil_phase_pins();
    // init_output_pwm();
    init_pwm_in();

//    PORT->Group[0].CTRL.reg = 0xFFFF; //continous sampling on all pins
//    PORT->Group[0].DIRCLR.reg = (1ul << 3);
//    PORT->Group[0].PINCFG[3].reg = PORT_PINCFG_INEN;
}

//void hardware_loop()
//{
//    static int n = 0;
//
//    uint8_t* input_reg = (uint8_t*)&(PORT->Group[0].IN.reg);
//#define PIN_HIGH ((*input_reg) && (1ul << 3))
//
//    while( !PIN_HIGH ) {}
//    while( PIN_HIGH ) {}
//    int64_t t0 = SysTick->VAL;
//    while( !PIN_HIGH ) {}
//    int64_t t1 = SysTick->VAL;
//    while( PIN_HIGH ) {}
//    int64_t t2 = SysTick->VAL;
//
//    if(n++ % 10 == 0) {
//        //int64_t pcnt = (int64_t)100 * (t1 - t0) / (t2 - t0);
//        //Serial.printf("%d\r\n", (int32_t)pcnt);
//        hardware_set_coil_power(0, (int64_t)PWM_OUT_MAX * (t1 - t0) / (t2 - t0));
//    }
//    //Serial.printf("%u %u %u %d\r\n", t0, t1, t2,  );
//    //Serial.printf("%d\r\n", PIN_HIGH);
//}



void hardware_loop()
{
    while(!(PWM_IN_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);
    while(!(PWM_IN_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    uint32_t period1 = tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    uint32_t pulse_width1 = tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);


    EVSYS->USER.reg = EVSYS_USER_CHANNEL(pwm_in_event_rsrc1.channel + 1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1);

    while(!(PWM_IN_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);
    while(!(PWM_IN_TCC->INTFLAG.bit.MC1)) { }
    PWM_IN_TCC->INTFLAG.reg |= TCC_INTFLAG_MC1;
    uint32_t period2 = tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_1);
    uint32_t pulse_width2 = tcc_get_capture_value(&tcc_modules[PWM_IN_TCC_NR], TCC_MATCH_CAPTURE_CHANNEL_0);

    EVSYS->USER.reg = EVSYS_USER_CHANNEL(pwm_in_event_rsrc2.channel + 1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1);

    Serial.printf("period=%ld , pulse width =%ld, ", period1 , pulse_width1);
    Serial.printf("period=%ld , pulse width =%ld \r\n", period2 , pulse_width2);

    delay(500);
}