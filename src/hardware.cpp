#include "hardware.h"
#include <vector>
#include "sam.h"
#include "samd21e18a.h"
#include <tcc.h>
#include <cstdint>

#include "hardware.h"

//microcontroller: SAMD21E18A-F

static const uint16_t PWM_OUT_PERIOD = 0xFFFF;

typedef struct {
    Tcc* TCC;
    uint32_t channel, output, pin, pin_mux;
    tcc_module module;
} pwm_config_t;

static std::vector<pwm_config_t> pwm_configurations = {
        {.TCC = TCC0, .channel = 0, .output = 0, .pin = PIN_PA04E_TCC0_WO0, .pin_mux = PINMUX_PA04E_TCC0_WO0},
        };

static void init_output_pwm() {

    for(int n = 0; n < pwm_configurations.size(); n++)
    {
        struct tcc_config config_tcc;
        tcc_get_config_defaults(&config_tcc, pwm_configurations[n].TCC);
        config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
        config_tcc.counter.period = PWM_OUT_PERIOD;
        config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
        config_tcc.compare.match[pwm_configurations[n].channel] = (PWM_OUT_PERIOD / 2);
        config_tcc.pins.enable_wave_out_pin[pwm_configurations[n].channel] = true;
        config_tcc.pins.wave_out_pin[pwm_configurations[n].output] = pwm_configurations[n].pin;
        config_tcc.pins.wave_out_pin_mux[pwm_configurations[n].output] = pwm_configurations[n].pin_mux;

        tcc_init(&pwm_configurations[n].module, pwm_configurations[n].TCC, &config_tcc);
        tcc_enable(&pwm_configurations[n].module);
    }
}

void hardware_set_pwm_out(unsigned int n, uint16_t val)
{
    tcc_set_compare_value(&pwm_configurations[n].module,
                          static_cast<const tcc_match_capture_channel>(pwm_configurations[n].channel), val);
}

void hardware_init()
{
    init_output_pwm();
}