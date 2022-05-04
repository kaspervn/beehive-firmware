#include "hardware.h"
#include <vector>
#include "sam.h"
#include "samd21e18a.h"
#include <tcc.h>
#include <cstdint>

#include "hardware.h"

//microcontroller: SAMD21E18A-F

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

void hardware_init()
{
    init_coil_phase_pins();
    init_output_pwm();
}