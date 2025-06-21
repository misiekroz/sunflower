#ifdef __cplusplus
extern "C"
{
#endif

#include "state/led_indicator/led_indicator.h"

    static led_strip_handle_t led_strip;
    static TaskHandle_t xHandleLedIndicatorDriver = NULL;

    static void led_indicator_task();

    void led_indicator_init()
    {
        // init the LED strip (with single LED)
        /// LED strip common configuration
        led_strip_config_t strip_config = {
            .strip_gpio_num = OUTPUT_STATUS_LED_PIN,                     // The GPIO that connected to the LED strip's data line
            .max_leds = 1,                                               // The number of LEDs in the strip,
            .led_model = LED_MODEL_WS2812,                               // LED strip model, it determines the bit timing
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
            .flags = {
                .invert_out = false, // don't invert the output signal

            }};

        /// RMT backend specific configuration
        led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
            .resolution_hz = 10 * 1000 * 1000, // RMT ocunter clock frequency: 10MHz
            .mem_block_symbols = 64,           // the memory size of each RMT channel, in words (4 bytes)
            .flags = {
                .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
            }};
        /// Create the LED strip object
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

        xTaskCreate(led_indicator_task,
                    "led_task",
                    4096,
                    NULL,
                    1,
                    &xHandleLedIndicatorDriver);
    }

    /**
     * Based on current tracker state, drives a WS2812 RGB LED indicator
     *
     */

    static void led_indicator_task()
    {
        while (1)
        {
            // ESP_LOGD(LOG_TAG_RUNTIME, "setting led");
            // Clamp STATUS to the valid range
            int current_status = get_state() % NUM_STATUS_PATTERNS;

            // Retrieve the configuration for the current status
            float hue = STATUS_PATTERNS_HSV[current_status][0];
            float saturation = STATUS_PATTERNS_HSV[current_status][1];
            float value = STATUS_PATTERNS_HSV[current_status][2];
            float frequency = STATUS_PATTERNS_HSV[current_status][3];

            // Calculate on/off times based on frequency
            float period = 1.0 / frequency,
                  on_time = 0.3 * period,
                  off_time = 0.7 * period;

            // Turn on the LED
            led_strip_set_pixel_hsv(led_strip, 0, hue, saturation, value);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(on_time * 1000));

            // Turn off the LED
            led_strip_clear(led_strip);
            vTaskDelay(pdMS_TO_TICKS(off_time * 1000));
        }
    }

#ifdef __cplusplus
}
#endif
