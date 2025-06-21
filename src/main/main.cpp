#include <stdio.h>
#include <stdbool.h>
#include <esp_log.h>
#include "config/config_main.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"
#include <led_strip_types.h>
#include <led_strip_rmt.h>
#include <led_strip.h>
#include "esp_intr_alloc.h"
#include "motion/motor_controller.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"

#include "safety/safety_controller.h"
#include "state/led_indicator/led_indicator.h"
#include "state/state_controller.h"
#include <driver/ledc.h>
#include "serial/serial_service.h"
#include "tracking_controller/tracking_controller.h"
#include "wifi/wifi_connect.h"
#include "sdcard/sd_card_manager.h"

#ifdef __cplusplus
extern "C"
{ // Ensure C++ symbols can be linked in a C file
#endif

    uint32_t init_error_flags = 0x00;

    void log_motor();

    void pin_config()
    {
        ESP_LOGI(LOG_TAG_INIT, "Configuring GPIOs");

        gpio_config_t isr_conf_falling = {
            .pin_bit_mask = (1ULL << INPUT_ESTOP_SIGNAL) | (1ULL << INPUT_SAFETY_RESET_SIGNAL),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE // falling edge interrupt
        };
        gpio_config(&isr_conf_falling);

        gpio_config_t isr_conf_change = {
            .pin_bit_mask = (1ULL << MOTOR_R_ENC) | (1ULL << MOTOR_L_ENC),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_ANYEDGE // any edge interrupt
        };
        gpio_config(&isr_conf_change);

        // enable per-pin interrupts
        ESP_LOGD(LOG_TAG_INIT, "Initializing ISR pins");
        init_error_flags <<= 1;
        init_error_flags += gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2) != ESP_OK;

        // configure ledc for pwm output
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_MODE,
            .duty_resolution = LEDC_DUTY_RES,
            .timer_num = LEDC_TIMER,
            .freq_hz = LEDC_FREQUENCY,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false};
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        // Configure the LEDC channel
        ledc_channel_config_t ledc_channel_l_cw = {
            .gpio_num = MOTOR_L_CW_PWM,
            .speed_mode = LEDC_MODE,
            .channel = MOTOR_L_CW_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0, // Start with 0% duty cycle
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
            .flags = 0};
        ledc_channel_config_t ledc_channel_l_ccw = {
            .gpio_num = MOTOR_L_CCW_PWM,
            .speed_mode = LEDC_MODE,
            .channel = MOTOR_L_CCW_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0, // Start with 0% duty cycle
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
            .flags = 0};
        ledc_channel_config_t ledc_channel_r_cw = {
            .gpio_num = MOTOR_R_CW_PWM,
            .speed_mode = LEDC_MODE,
            .channel = MOTOR_R_CW_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0, // Start with 0% duty cycle
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
            .flags = 0};
        ledc_channel_config_t ledc_channel_r_ccw = {
            .gpio_num = MOTOR_R_CCW_PWM,
            .speed_mode = LEDC_MODE,
            .channel = MOTOR_R_CCW_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0, // Start with 0% duty cycle
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
            .flags = 0};

        // enable per-pin interrupts
        ESP_LOGD(LOG_TAG_INIT, "Initializing LEDC pins.");
        init_error_flags <<= 1;
        init_error_flags += ledc_channel_config(&ledc_channel_l_cw) != ESP_OK;
        init_error_flags <<= 1;
        init_error_flags += ledc_channel_config(&ledc_channel_l_ccw) != ESP_OK;
        init_error_flags <<= 1;
        init_error_flags += ledc_channel_config(&ledc_channel_r_cw) != ESP_OK;
        init_error_flags <<= 1;
        init_error_flags += ledc_channel_config(&ledc_channel_r_ccw) != ESP_OK;
    }

    void time_sync_notification_cb(struct timeval *tv)
    {
        ESP_LOGI(LOG_TAG_RUNTIME, "Notification of a time synchronization event");
    }

    static void check_safety_and_set_state(tracker_states state)
    {
        wait_stop_release();

        bool result = set_state(state);
        if (result)
            vTaskDelay(pdMS_TO_TICKS(10000));
        else
            ESP_LOGE(LOG_TAG_RUNTIME, "Error setting state %i", state);
    }

    void try_connect_wifi()
    {
        esp_err_t conn_result = wifi_init_sta();
        init_error_flags <<= 1;
        init_error_flags += conn_result != ESP_OK;

        // perform hard lock when wifi conenction fails
        if (conn_result != ESP_OK)
        {
            ESP_LOGE(LOG_TAG_INIT, "Unable to connect WiFi, error %i", conn_result);
            // set_state(ERROR);
            // set_stop_triggered();
        }
    }

    void obtain_sntp_time_init()
    {
    }

    void obtain_sntp_time()
    {
        ESP_LOGI(LOG_TAG_INIT, "Initializing and starting SNTP");
        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        config.sync_cb = time_sync_notification_cb; // optional
        esp_netif_sntp_init(&config);               // <-- this sets the server
        init_error_flags <<= 1;
        init_error_flags += esp_netif_sntp_start() != ESP_OK;
        // wait for time to be set
        time_t now = 0;
        struct tm timeinfo = {0};
        int retry = 0;
        const int retry_count = 15;
        while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count)
        {
            ESP_LOGI(LOG_TAG_INIT, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        }
        time(&now);
        localtime_r(&now, &timeinfo);
        setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
        tzset();
        esp_netif_sntp_deinit();

        char out_buf[20];
        SDCardManager::get_current_datetime_str(out_buf, 20);
        ESP_LOGI(LOG_TAG_INIT, "Got system time of: %s", out_buf);
    }

    /*
        Performs initial checks and setup required for correct tracker operation
    */
    void init()
    {
        pin_config();
        safety_init();
        led_indicator_init();

        init_error_flags <<= 1;
        init_error_flags += nvs_flash_init() != ESP_OK;
        // ESP_ERROR_CHECK(nvs_flash_init());

        obtain_sntp_time_init();
        try_connect_wifi();
        obtain_sntp_time();

        init_error_flags <<= 1;
        init_error_flags += SerialService::init() != ESP_OK;
        init_error_flags <<= 1;
        init_error_flags += TrackingController::init() != ESP_OK;

        // TrackingController::change_mode(UART_TRACKING);

        if (init_error_flags != 0)
        {
            ESP_LOGE(LOG_TAG_INIT, "Initialization failed with error flags: %" PRIu32 "", init_error_flags);
        }

        // wait for user to release STOP button
        ESP_LOGI(LOG_TAG_INIT, "Waiting for STOP release... ");
    }

    /*
        Ensures all axes reach the target position
    */
    void home_axes()
    {
        // send homing signal to the motors, wait for return

        // as a placeholder, some delay is in place
        ESP_LOGI("HOMING", "Placeholder: homing");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    int hz_to_ticks(float period_hz)
    {
        return pdMS_TO_TICKS(1000.0 / period_hz);
    }

    void debug_logger(void *ptr)
    {
        // used only when log verbosity is set top verbose
        while (1)
        {
#if LOG_DEFAULT_LEVEL == ESP_LOG_DEBUG
            float temperature = -1.0f, humidity = -1.0f;
            int out_adc_readings[ADC_NUM_CHANNELS] = {-1}, out_num_channels = -1;
            char current_time[20];

            // ESP_ERROR_CHECK(TrackingController::get_temperature_humidity(temperature, humidity));
            esp_err_t adc_result = TrackingController::get_adc_readings(out_adc_readings, &out_num_channels);
            SDCardManager::get_current_datetime_str(current_time, 20);

            ESP_LOGD(LOG_TAG_RUNTIME, "=====================");
            ESP_LOGD(LOG_TAG_RUNTIME, "===== DEBUG LOG =====");
            ESP_LOGD(LOG_TAG_RUNTIME, "=====================");
            ESP_LOGD(LOG_TAG_RUNTIME, "System time: %s", current_time);
            ESP_LOGD(LOG_TAG_RUNTIME, "Current tracker state: %i", get_state());
            ESP_LOGD(LOG_TAG_RUNTIME, "Temperature: %f, Humidity: %f", temperature, humidity);
            ESP_LOGD(LOG_TAG_RUNTIME, "Foto readings [%sd]", esp_err_to_name(adc_result));
            ESP_LOGD(LOG_TAG_RUNTIME, "[TL]\t[TR]\t %i\t\t  %i", out_adc_readings[0], out_adc_readings[1]);
            ESP_LOGD(LOG_TAG_RUNTIME, "\t[C]\t %i\t\t", out_adc_readings[4]);
            ESP_LOGD(LOG_TAG_RUNTIME, "[BL]\t[BR]\t %i\t\t  %i", out_adc_readings[2], out_adc_readings[3]);
            log_motor();
            ESP_LOGD(LOG_TAG_RUNTIME, "Last fuzzy decision: %f", TrackingController::ai_last_result);
            ESP_LOGD(LOG_TAG_RUNTIME, "TrackingController state: %i", TrackingController::get_mode());
            ESP_LOGD(LOG_TAG_RUNTIME, "TrackingController motor errors: %i", TrackingController::get_motor_errors());
            ESP_LOGD(LOG_TAG_RUNTIME, "=====================");
#endif

            vTaskDelay(hz_to_ticks(LOG_VERBOSE_FREQUENCY));
        }
    }

    void data_logger(void *ptr);

    void app_main(void)
    {
        // // Set global logging level to LOG_VERBOSITY
        // esp_log_level_set("*", LOG_VERBOSITY);
        init();

        TaskHandle_t xHandleVerboseLogger = NULL,
                     xHandleDataLogger = NULL;

        xTaskCreate(debug_logger,
                    "vlog_task",
                    8192,
                    NULL,
                    1,
                    &xHandleVerboseLogger);

        xTaskCreate(data_logger,
                    "dlog_task",
                    8192,
                    NULL,
                    1,
                    &xHandleDataLogger);

        wait_stop_release();                     // wait for stop button to be released
        TrackingController::start_home_motors(); // attempt home motors
        // TODO: homing on init logic should be more robust
    }

#ifdef __cplusplus
}
#endif

void log_motor()
{
    TrackingController::log_motors();
}

void data_logger(void *ptr)
{
    // TODO: pin assignments
    SDCardManager sd_manager = SDCardManager(SD_SPI_MISO, SD_SPI_MOSI, SD_SPI_CLK, SD_SPI_CS);

    while (1 == 1)
    {
        // obtain data
        motor_setpoint_t current_pos = TrackingController::get_motors_position();
        int motors_moving = TrackingController::get_motors_moving();
        int motors_homing = TrackingController::get_motors_homing();
        int errors = safety_get_last_error_flags();
        // NOTE: to minimize changes, temperature is used as a placeholder for AI result
        float temperature = TrackingController::ai_last_result*1000, humidity = -1.0f;

        int out_adc_readings[ADC_NUM_CHANNELS] = {-1}, out_num_channels = -1;

        // TODO replace ESP_ERROR_CHECK with error handling
        esp_err_t adc_result = TrackingController::get_adc_readings(out_adc_readings, &out_num_channels);
        // ESP_ERROR_CHECK(TrackingController::get_temperature_humidity(temperature, humidity));

        if (out_num_channels != ADC_NUM_CHANNELS)
        {
            ESP_LOGE(LOG_TAG_RUNTIME, "Error getting ADC readings, expected %d channels, got %d", ADC_NUM_CHANNELS, out_num_channels);
            continue;
        }
        if (adc_result != ESP_OK)
        {
            ESP_LOGE(LOG_TAG_RUNTIME, "Error getting ADC readings: %s", esp_err_to_name(adc_result));
            continue;
        }

        sd_manager.log_line(
            current_pos.target_position_R,
            current_pos.target_position_L,
            out_adc_readings[0],
            out_adc_readings[1],
            out_adc_readings[2],
            out_adc_readings[3],
            out_adc_readings[4],
            temperature,
            -1,
            -1,
            humidity,
            errors,
            motors_homing,
            motors_moving);
        vTaskDelay(pdMS_TO_TICKS(LOG_SD_DLY_MS));
    }
}