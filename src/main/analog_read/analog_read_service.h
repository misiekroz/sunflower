#ifndef ANALOG_READ_SERVICE_H
#define ANALOG_READ_SERVICE_H

#include <stdio.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"
#include "esp_err.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_adc_cal.h>
#include <esp_err.h>


#include <config/config_main.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifdef __cplusplus
    class AnalogReadService
    {
    private:
        adc_channel_t *gpio_conf;
        int num_channels,
            frequency,
            average_samples;

        int *adc_readings;
        esp_err_t last_adc_readings_err;

        adc_oneshot_unit_handle_t adc_handle;
        adc_cali_handle_t cali_handle;

        void reading_task(void *arg);

    public:
        AnalogReadService(adc_channel_t *gpio_conf, int num_channels, int frequency, int average_samples);

        esp_err_t init_adc_reading_service(void);

        esp_err_t get_last_readings(int *out_adc_readings, int *out_adc_readings_len);

        static void reading_task_wrapper(void *pvParameter);
    };

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // ANALOG_READ_SERVICE_H
