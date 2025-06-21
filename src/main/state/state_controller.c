#ifdef __cplusplus
extern "C"
{
#endif

#include "state/state_controller.h"

    static SemaphoreHandle_t xSemaphore = NULL;
    static tracker_states current_state = UNKNOWN;

    static bool set_state_helper(tracker_states state, bool override_stop);

    static void state_controller_init()
    {
        ESP_LOGD(LOG_TAG_RUNTIME, "Initializing state_controller...");

        xSemaphore = xSemaphoreCreateMutex();
    }

    tracker_states get_state()
    {
        return current_state;
    }

    bool state_is_idle()
    {
        return get_state() == IDLE;
    }

    bool set_state(tracker_states state)
    {
        return set_state_helper(state, false);
    }

    bool set_state_with_stop_override(tracker_states state)
    {
        return set_state_helper(state, true);
    }

    static bool set_state_helper(tracker_states state, bool override_stop)
    {
        if (xSemaphore == NULL)
        {
            state_controller_init();
        }

        bool new_state_is_valid = (!override_stop && current_state != STOP && current_state != ERROR) || (override_stop && current_state != ERROR);

        // if selected state is valid
        if (new_state_is_valid)
        {
            // and can take the semaphore
            if (xSemaphoreTake(xSemaphore, (TickType_t)pdMS_TO_TICKS(TRACKER_STATE_TIMEOUT_MS)) == pdTRUE)
            {
                // do the change, remembering to reset if new state is WARN
                tracker_states old_state = current_state;
                current_state = state;

                ESP_LOGD(LOG_TAG_RUNTIME, "Setting state to: %i. Previous state: %i.", state, old_state);

                if (state == WARNING)
                {
                    // wait for set lock time
                    vTaskDelay(pdMS_TO_TICKS(WARNING_LOCK_TIME_MS));
                    // reset the state
                    current_state = state;
                }
                xSemaphoreGive(xSemaphore);
                // then return correct state setting result
                return true;
            }
        }
        // in any other case, something went wrong with state setting
        return false;
    }

#ifdef __cplusplus
}
#endif