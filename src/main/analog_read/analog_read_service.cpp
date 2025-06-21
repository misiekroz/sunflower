#include "analog_read/analog_read_service.h"

AnalogReadService::AnalogReadService(adc_channel_t *gpio_list,
                                     int num_channels,
                                     int frequency,
                                     int average_samples) : gpio_conf(gpio_list),
                                                            num_channels(num_channels),
                                                            frequency(frequency),
                                                            average_samples(average_samples)
{
    adc_readings = new int[num_channels]();
}

esp_err_t AnalogReadService::init_adc_reading_service(void)
{
    TaskHandle_t xHandleReadingTask = NULL;
    esp_err_t result = ESP_FAIL;

    // Init ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_USED,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    result = adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    if (result != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "ADC unit initialization failed: %s", esp_err_to_name(result));
        return result;
    }

    // Configure all channels
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };

    for (int i = 0; i < num_channels; i++) {
        result = adc_oneshot_config_channel(adc_handle, gpio_conf[i], &chan_cfg);
        if (result != ESP_OK)
        {
            ESP_LOGE(LOG_TAG_INIT, "ADC channel [%i] configuration failed: %s", i, esp_err_to_name(result));
            return result;
        }
    }

    // Calibration setup
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_USED,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    last_adc_readings_err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);

    if (last_adc_readings_err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "ADC calibration failed: %s", esp_err_to_name(last_adc_readings_err));
        return result;
    }


    xTaskCreate(&AnalogReadService::reading_task_wrapper,
                "aread_task",
                4096,
                this,
                2,
                &xHandleReadingTask);

    return ESP_OK;
}

void AnalogReadService::reading_task_wrapper(void *pvParameter)
{
    AnalogReadService *instance = static_cast<AnalogReadService *>(pvParameter);
    instance->reading_task(nullptr); // Call the actual non-static method
}

void AnalogReadService::reading_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;

    while (true)
    {
        if (last_adc_readings_err != ESP_OK)
        {
            for (int ch = 0; ch < num_channels; ch++)
            {
                int raw_sum = 0;
                for (int i = 0; i < average_samples; i++)
                {
                    int raw = 0;
                    last_adc_readings_err = adc_oneshot_read(adc_handle, gpio_conf[ch], &raw);
                    raw_sum += raw;
                    vTaskDelay(pdMS_TO_TICKS(2)); // Optional: ADC settling
                }

                int avg_raw = raw_sum / average_samples;

                int voltage = 0;
                last_adc_readings_err = adc_cali_raw_to_voltage(cali_handle, avg_raw, &voltage);
                adc_readings[ch] = voltage; // Store the voltage reading
            }
        }
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / frequency)); // Ensure consistent loop timing based on frequency
        if (xWasDelayed == pdTRUE)
        {
            last_adc_readings_err = ESP_ERR_TIMEOUT; // Set error if task was delayed
        }
    }
}

esp_err_t AnalogReadService::get_last_readings(int *out_adc_readings, int *out_adc_readings_len)
{
    // Set the number of channels in the output parameter
    *out_adc_readings_len = num_channels;

    if (last_adc_readings_err == ESP_OK)
    {
        // Copy the ADC readings to the output array
        for (int i = 0; i < num_channels; i++)
        {
            out_adc_readings[i] = adc_readings[i];
        }
        return ESP_OK; // Return success
    }
    else
    {
        return last_adc_readings_err; // Return the last error encountered
    }
}
