#include "hardware.h"
#include <vector>
#include "sam.h"
#include "samd21e18a.h"
#include <tcc.h>
#include <cstdint>

#include "hardware.h"

//microcontroller: SAMD21E18A-F

static const uint16_t PWM_OUT_PERIOD = 0xFFFF;

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
        {.TCC = TCC0, .channel = 0, .output = 0, .pin = PIN_PA04E_TCC0_WO0, .pin_mux = PINMUX_PA04E_TCC0_WO0},
        {.TCC = TCC0, .channel = 3, .output = 7, .pin = PIN_PA17F_TCC0_WO7, .pin_mux = PINMUX_PA17F_TCC0_WO7},
        {.TCC = TCC0, .channel = 1, .output = 5, .pin = PIN_PA23F_TCC0_WO5, .pin_mux = PINMUX_PA23F_TCC0_WO5},
        };

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
                    config_tcc.compare.match[pwm_configurations[n].channel] = (PWM_OUT_PERIOD / 2);
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

void hardware_set_pwm_out(unsigned int n, uint16_t val)
{
    tcc_set_compare_value(pwm_configurations[n].module,
                          static_cast<const tcc_match_capture_channel>(pwm_configurations[n].channel), val);
}

void hardware_init()
{
    init_output_pwm();
}