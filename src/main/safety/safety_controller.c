#ifdef __cplusplus
extern "C"
{
#endif

#include "safety/safety_controller.h"

    atomic_bool stop_triggered = true;
    atomic_bool stop_state_error = false;

    static QueueHandle_t isr_queue = NULL;
    static int last_error_flags = 0;

    static void safety_controller_task();

    static void IRAM_ATTR gpio_isr(void *arg)
    {
        uint32_t gpio_num = (uint32_t)arg;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Send the GPIO number to the queue
        xQueueSendFromISR(isr_queue, &gpio_num, &xHigherPriorityTaskWoken);

        // Yield to a higher priority task if needed
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }

    static void interrupt_handler()
    {
        uint32_t gpio_num;

        while (1)
        {
            if (xQueueReceive(isr_queue, &gpio_num, portMAX_DELAY))
            {
                ESP_LOGI(LOG_TAG_SAFETY, "Interrupt detected on GPIO [ %" PRIu32 " ]", gpio_num);

                // decide which interrupt was triggered and take action
                if (gpio_num == INPUT_ESTOP_SIGNAL)
                {
                    ESP_LOGD(LOG_TAG_SAFETY, "ISR for E-STOP triggered");
                    set_stop_triggered();
                }
                else if (gpio_num == INPUT_SAFETY_RESET_SIGNAL)
                {
                    ESP_LOGD(LOG_TAG_SAFETY, "ISR for safety reset triggered");
                    safety_reset();
                }
            }
        }
    }

    bool is_stop_triggered()
    {
        return atomic_load(&stop_triggered);
    }

    void wait_stop_release()
    {
        while (is_stop_triggered())
        {
            vTaskDelay(pdTICKS_TO_MS(SAFETY_WAIT_LOOP_DELAY));
        }
    }

    void safety_init()
    {
        // Register ISR with the highest priority
        gpio_isr_handler_add(INPUT_ESTOP_SIGNAL, gpio_isr, (void *)INPUT_ESTOP_SIGNAL);
        gpio_isr_handler_add(INPUT_SAFETY_RESET_SIGNAL, gpio_isr, (void *)INPUT_SAFETY_RESET_SIGNAL);

        // Create a queue to handle GPIO events
        isr_queue = xQueueCreate(10, sizeof(uint32_t));

        // Ensure queue creation succeeded
        if (isr_queue == NULL)
        {
            ESP_LOGE(LOG_TAG_SAFETY, "Failed to create ISR queue!");
            return;
        }

        TaskHandle_t xHandleSafetyController = NULL,
                     xHandleInterruptHandler = NULL;

        // set state to STOP
        set_state(STOP);

        xTaskCreate(safety_controller_task,
                    "sfty_ctr",
                    4096,
                    NULL,
                    1,
                    &xHandleSafetyController);

        xTaskCreate(interrupt_handler,
                    "intr_cnt",
                    4096,
                    NULL,
                    4,
                    &xHandleInterruptHandler);
    }

    static void safety_controller_task()
    {
        ESP_LOGD(LOG_TAG_SAFETY, "Starting safety task");

        while (1)
        {
            // a note for future me:
            // why skip checking last_error_flags during STOP state?
            // STOP by design is a safe state, in which the device cannot harm its surroundings
            // therefore raising ERROR could be misleading

            if (!is_stop_triggered())
            {
                //  first check error-state issues
                // TBD
                last_error_flags = 0x00;
                // add here checks for various states NOTE: make sure to add logs about the thing that caused the error
                // example use:
                last_error_flags += atomic_load(&stop_state_error);
                last_error_flags = (last_error_flags << 1);

                // if any critical error occured, enter endless error state
                if (last_error_flags != 0)
                {
                    ESP_LOGE(LOG_TAG_SAFETY, "Safety controller locked the device. Please shut down, resolve the issue and restart the device.");
                    ESP_LOGE(LOG_TAG_SAFETY, "Lock reason: %x", last_error_flags);
                    while (1 == 1)
                    {
                        set_state(ERROR);
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    }
                }

                // then check warning states and call warning if needed
                uint16_t warnings = false;
                if (warnings != 0)
                {
                    ESP_LOGW(LOG_TAG_SAFETY, "Safety controller found malfunctions. Code: %x", warnings);
                    ESP_LOGW(LOG_TAG_SAFETY, "Sending warning message...");

                    // retry to warn user
                    while (!set_state(WARNING))
                    {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        ESP_LOGW(LOG_TAG_SAFETY, ".");
                    }
                }
            }
            else
            {
                // if despite STOP being called, the state is not STOP or ERROR,
                // raise an error about incorrect state
                tracker_states state = get_state();
                if (state != STOP && state != ERROR && state != UNKNOWN)
                {
                    ESP_LOGE(LOG_TAG_SAFETY, "Safety found mismatch in state and is_stop_triggered. Raising error.");
                    atomic_store(&stop_state_error, true);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(SAFETY_LOOP_DELAY_MS));
        }
    }

    /** Set Stop Triggered
     *
     * Sets stop_triggered, only if is not set already
     */
    void set_stop_triggered()
    {
        ESP_LOGD(LOG_TAG_SAFETY, "Setting safety state");

        if (!is_stop_triggered())
        {
            atomic_store(&stop_triggered, true);
            set_state(STOP);
        }
    }

    /** Safety Reset
     *
     * Resets the state of stop_triggered, but only if E-STOP
     * pin is SET (i.e. E-STOP is released).
     *
     * Returns stop_triggered.
     */
    bool safety_reset()
    {
        ESP_LOGD(LOG_TAG_SAFETY, "Entering Safety Reset Routine");

        if (!is_stop_triggered())
        {
            ESP_LOGD(LOG_TAG_SAFETY, "device not in safety state, returning");
        }

        else
        {
            bool estop_released = gpio_get_level(INPUT_ESTOP_SIGNAL);

            if (estop_released)
            {
                ESP_LOGI(LOG_TAG_SAFETY, "Releasing safety state");

                atomic_store(&stop_triggered, false);
                set_state_with_stop_override(IDLE);
            }

            else
            {
                ESP_LOGI(LOG_TAG_SAFETY, "E-STOP still latched, safety state not released");
            }
        }

        return is_stop_triggered();
    }

    int safety_get_last_error_flags()
    {
        return last_error_flags;
    }

#ifdef __cplusplus
}
#endif
