#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdatomic.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "../config/config_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"
#include "state/state_controller.h"

    extern atomic_bool stop_triggered;

    // a flag that is set when stop_triggered was true, but state was != STOP or ERROR
    extern atomic_bool stop_state_error;

    bool is_stop_triggered();

    /**
     * @brief Waits for STOP to be released
     * IMPORTANT: this method does not contain any form of timeout. Return before
     * safety button is released would be misleading and would increase code complexity.
     * @param timeout_ms
     */
    void wait_stop_release();

    void safety_init();

    void set_stop_triggered();

    bool safety_reset();

    int safety_get_last_error_flags();

#ifdef __cplusplus
}
#endif

#endif // SAFETY_CONTROLLER_H