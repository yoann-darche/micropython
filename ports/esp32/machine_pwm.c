/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016-2021 Damien P. George
 * Copyright (c) 2018 Alan Dragomirecky
 * Copyright (c) 2020 Antoine Aubert
 * Copyright (c) 2021 Ihor Nehrutsa
 * Copyright (c) 2024 Yoann Darche
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// This file is never compiled standalone, it's included directly from
// extmod/machine_pwm.c via MICROPY_PY_MACHINE_PWM_INCLUDEFILE.

#include <math.h>
#include "py/mphal.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_clk_tree.h"
// #include "soc/clk_tree_defs.h"
#include "soc/gpio_sig_map.h"


#define PWM_DBG(...)
// #define PWM_DBG(...) mp_printf(&mp_plat_print, __VA_ARGS__); mp_printf(&mp_plat_print, "\n");

// Total number of channels
#define PWM_CHANNEL_MAX (LEDC_SPEED_MODE_MAX * LEDC_CHANNEL_MAX)

typedef struct _chan_t {
    // Which channel has which GPIO pin assigned?
    // (-1 if not assigned)
    gpio_num_t pin;
    // Which channel has which timer assigned?
    // (-1 if not assigned)
    int timer_idx;
    // Is light sleep enable has been set for this pin
    bool lightsleepenabled;
} chan_t;

// List of PWM channels
static chan_t chans[PWM_CHANNEL_MAX];

// channel_idx is an index (end-to-end sequential numbering) for all channels
// available on the chip and described in chans[]
#if SOC_LEDC_SUPPORT_HS_MODE
    #define CHANNEL_IDX(mode, channel) ((mode == LEDC_HIGH_SPEED_MODE) ? LEDC_CHANNEL_MAX + channel : channel)
    #define CHANNEL_IDX_TO_MODE(channel_idx) ((channel_idx > (LEDC_CHANNEL_MAX - 1)) ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE)
#else
    #define CHANNEL_IDX(mode, channel) (channel)
    #define CHANNEL_IDX_TO_MODE(channel_idx) (LEDC_LOW_SPEED_MODE)
#endif
#define CHANNEL_IDX_TO_CHANNEL(channel_idx) (channel_idx % LEDC_CHANNEL_MAX)

// Total number of timers
#define PWM_TIMER_MAX (LEDC_SPEED_MODE_MAX * LEDC_TIMER_MAX)

// List of timer configs
static ledc_timer_config_t timers[PWM_TIMER_MAX];

// timer_idx is an index (end-to-end sequential numbering) for all timers
// available on the chip and configured in timers[]

#if SOC_LEDC_SUPPORT_HS_MODE
    #define TIMER_IDX_TO_MODE(timer_idx) (timer_idx > (LEDC_TIMER_MAX - 1)  ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE)
    #define TIMER_IDX(mode, timer) ((mode == LEDC_HIGH_SPEED_MODE) ? LEDC_TIMER_MAX + timer : timer)
#else
    #define TIMER_IDX_TO_MODE(timer_idx) (LEDC_LOW_SPEED_MODE)
    #define TIMER_IDX(mode, timer) (timer)
#endif

#define TIMER_IDX_TO_TIMER(timer_idx) (timer_idx % LEDC_TIMER_MAX)

// Params for PWM operation
// 5khz is default frequency
#define PWM_FREQ (5000)

// 10-bit resolution (compatible with esp8266 PWM)
#define PWM_RES_10_BIT (LEDC_TIMER_10_BIT)

// Maximum duty value on 10-bit resolution
#define MAX_DUTY_U10 ((1 << PWM_RES_10_BIT) - 1)
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html#supported-range-of-frequency-and-duty-resolutions
// duty() uses 10-bit resolution or less
// duty_u16() and duty_ns() use 16-bit resolution or less

// Possible highest resolution in device (16bit or less)
#if ((LEDC_TIMER_BIT_MAX - 1) < 16)
#define HIGHEST_PWM_RES (LEDC_TIMER_BIT_MAX - 1)
#else
#define HIGHEST_PWM_RES (LEDC_TIMER_16_BIT) // 20 bit for ESP32, but 16 bit is used
#endif
// Duty resolution of user interface in `duty_u16()` and `duty_u16` parameter in constructor/initializer
#define UI_RES_16_BIT (16)
// Maximum duty value on highest user interface resolution
#define UI_MAX_DUTY ((1 << UI_RES_16_BIT) - 1)
// How much to shift from the HIGHEST_PWM_RES duty resolution to the user interface duty resolution UI_RES_16_BIT
#define UI_RES_SHIFT (UI_RES_16_BIT - HIGHEST_PWM_RES) // 0 for ESP32, 2 for S2, S3, C3

#if SOC_LEDC_SUPPORT_REF_TICK
// If the PWM frequency is less than EMPIRIC_FREQ, then LEDC_REF_CLK_HZ(1 MHz) source is used, else LEDC_APB_CLK_HZ(80 MHz) source is used
#define EMPIRIC_FREQ (10) // Hz
#endif

// Config of timer upon which we run all PWM'ed GPIO pins
static bool pwm_inited = false;

// MicroPython PWM object struct
typedef struct _machine_pwm_obj_t {
    mp_obj_base_t base;
    gpio_num_t pin;
    bool active;
    bool lightsleepenabled;
    ledc_mode_t mode;
    int channel;
    int timer;
    int duty_x; // PWM_RES_10_BIT if duty(), HIGHEST_PWM_RES if duty_u16(), -HIGHEST_PWM_RES if duty_ns()
    int duty_u10; // stored values from previous duty setters
    int duty_u16; // - / -
    int duty_ns; // - / -
} machine_pwm_obj_t;

static bool is_timer_in_use(int current_channel_idx, int timer_idx);
static void set_duty_u16(machine_pwm_obj_t *self, int duty);
static void set_duty_u10(machine_pwm_obj_t *self, int duty);
static void set_duty_ns(machine_pwm_obj_t *self, int ns);

static void pwm_init(void) {

    // Initial condition: no channels assigned
    for (int i = 0; i < PWM_CHANNEL_MAX; ++i) {
        chans[i].pin = -1;
        chans[i].timer_idx = -1;
        chans[i].lightsleepenabled = false;
    }

    // Prepare all timers config
    // Initial condition: no timers assigned
    for (int i = 0; i < PWM_TIMER_MAX; ++i) {
        timers[i].duty_resolution = HIGHEST_PWM_RES;
        // unset timer is -1
        timers[i].freq_hz = -1;
        timers[i].speed_mode = TIMER_IDX_TO_MODE(i);
        timers[i].timer_num = TIMER_IDX_TO_TIMER(i);
        timers[i].clk_cfg = LEDC_AUTO_CLK; // will reinstall later
    }
}

// Deinit channel and timer if the timer is unused
static void pwm_deinit(int channel_idx) {
    // Valid channel?
    if ((channel_idx >= 0) && (channel_idx < PWM_CHANNEL_MAX)) {

        // Stop the channel
        int pin = chans[channel_idx].pin;
        if (pin != -1) {
            ledc_mode_t mode = CHANNEL_IDX_TO_MODE(channel_idx);
            int channel = CHANNEL_IDX_TO_CHANNEL(channel_idx);

            if (chans[channel_idx].lightsleepenabled) {
                // Enable SLP_SEL to change GPIO status automantically in lightsleep.
                check_esp_err(gpio_sleep_sel_en(pin));
            }

            // Mark it unused, and tell the hardware to stop routing
            check_esp_err(ledc_stop(mode, channel, 0));
            // Disable ledc signal for the pin
            // esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
            if (mode == LEDC_LOW_SPEED_MODE) {
                esp_rom_gpio_connect_out_signal(pin, LEDC_LS_SIG_OUT0_IDX + channel, false, true);
            } else {
                #if LEDC_SPEED_MODE_MAX > 1
                #if CONFIG_IDF_TARGET_ESP32
                esp_rom_gpio_connect_out_signal(pin, LEDC_HS_SIG_OUT0_IDX + channel, false, true);
                #else
                #error Add supported CONFIG_IDF_TARGET_ESP32_xxx
                #endif
                #endif
            }
        }
        chans[channel_idx].pin = -1;
        chans[channel_idx].lightsleepenabled = false;

        // Clean up timer if necessary
        int timer_idx = chans[channel_idx].timer_idx;
        if (timer_idx != -1) {
            if (!is_timer_in_use(channel_idx, timer_idx)) {
                // Pause the timer
                check_esp_err(ledc_timer_pause(TIMER_IDX_TO_MODE(timer_idx), TIMER_IDX_TO_TIMER(timer_idx)));
                // Set flag to deconfigure
                timers[timer_idx].deconfigure = true;
                // Flag it unused
                timers[timer_idx].freq_hz = -1;
                timers[timer_idx].clk_cfg = LEDC_AUTO_CLK;
                // run deconfiguration
                check_esp_err(ledc_timer_config(&timers[timer_idx]));
            }
        }

        chans[channel_idx].timer_idx = -1;

    }
}

// This called from Ctrl-D soft reboot
void machine_pwm_deinit_all(void) {
    if (pwm_inited) {
        for (int channel_idx = 0; channel_idx < PWM_CHANNEL_MAX; ++channel_idx) {
            pwm_deinit(channel_idx);
        }
        pwm_inited = false;
    }
}

static void configure_channel(machine_pwm_obj_t *self) {
    ledc_channel_config_t cfg = {
        .channel = self->channel,
        .duty = (1 << (timers[TIMER_IDX(self->mode, self->timer)].duty_resolution)) / 2,
        .gpio_num = self->pin,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = self->mode,
        .timer_sel = self->timer
    };
    if (ledc_channel_config(&cfg) != ESP_OK) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("PWM not supported on Pin(%d)"), self->pin);
    }

    if (self->lightsleepenabled) {
        // Disable SLP_SEL to change GPIO status automantically in lightsleep.
        check_esp_err(gpio_sleep_sel_dis(self->pin));
    }

}

// led_src_clock = LEDC_USE_APB_CLK, LEDC_USE_RC_FAST_CLK ...
static void set_freq(machine_pwm_obj_t *self, unsigned int freq, ledc_timer_config_t *timer, ledc_clk_cfg_t led_src_clock) {
    if (freq != timer->freq_hz) {

        uint32_t src_clock_freq;

        ledc_clk_cfg_t old_clk = timer->clk_cfg;  // YDE:: To remove !

        if (esp_clk_tree_src_get_freq_hz(led_src_clock, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX, &src_clock_freq) != ESP_OK) {
            mp_raise_ValueError(MP_ERROR_TEXT("Error on getting reference clock frequency (FREQ_PRECISION_APPROX)."));
        }

        uint32_t res = ledc_find_suitable_duty_resolution(src_clock_freq, freq);
        /*   if (res > HIGHEST_PWM_RES) {
               res = HIGHEST_PWM_RES;
           }*/

        // Configure the new resolution and frequency
/*/        #if SOC_LEDC_SUPPORT_HS_MODE
        timer->speed_mode = self->mode;
