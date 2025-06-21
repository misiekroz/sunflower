#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdbool.h>
#include "../config/config_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"
#include "freertos/semphr.h"
#include <config/config_main.h>

    /**
     * @brief Get the current state
     * Gets current state of the tracker. Due to the nature of WARN state, this method
     *  does not wait for mutex release. Otherwise, WARN state would never be returned.
     *
     * See set_state for more information
     * @return tracker_states
     */
    tracker_states get_state();

    /**
     * @brief Returns if state == IDLE
     * Most of the tasks will be able to start only if state == IDLE,
     *  so a method is provided for simplification
     *
     * @return true
     * @return false
     */
    bool state_is_idle();

    /**
     * @brief Set the current state
     * Set current state, guarded by semaphore. State setting rules:
     *  - if ERROR or STOP state is set, state cannot be changed
     *  - WARN is a monostable state. When setting it, the method will wait N seconds, then reset
     *           the previous state.
     * @return true - state was set correctly
     * @return false - either timeout occurred when waiting for semaphore, or setting requested
     *                  state would violate state rules
     */
    bool set_state(tracker_states state);

    /**
     * @brief Same as set_state, but will change stop_state
     *
     * @return true - state was set correctly
     * @return false - either timeout occurred when waiting for semaphore, or setting requested
     *                  state would violate state rules
     */
    bool set_state_with_stop_override(tracker_states state);

#ifdef __cplusplus
}
#endif

#endif // STATE_CONTROLLER_H