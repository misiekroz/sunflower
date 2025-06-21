#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdatomic.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"
#include "state/state_controller.h"
#include "hal/gpio_types.h"
#include <led_strip_types.h>
#include <led_strip_rmt.h>
#include <led_strip.h>

#include "config/config_main.h"
#include "state/state_controller.h"

    /**
     * @brief Initializes led indicator and starts a led indicator task
     *
     */
    void led_indicator_init();

#ifdef __cplusplus
}
#endif

#endif // LED_INDICATOR_H