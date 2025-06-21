#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#include <stdio.h>
#include <stdbool.h>
#include <stdatomic.h>

#include <driver/ledc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
// #include "sd_test_io.h"
#include <time.h>

#include <config/config_main.h>
#include <tracking_controller/tracking_controller.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

class SDCardManager
{
private:
    char log_file_path[64];

public:
    SDCardManager(gpio_num_t MOSI, gpio_num_t MISO, gpio_num_t CLK, gpio_num_t CS);

    esp_err_t log_line(int R_pos,
                       int L_pos,
                       int A1,
                       int A2,
                       int A3,
                       int A4,
                       int A5,
                       int T0,
                       int T1,
                       int T2,
                       int humidity,
                       int errors,
                       int motors_homing,
                       int motors_moving);

    esp_err_t get_next_target_position(motor_setpoint_t &result_setpoint);

    static void get_current_date_str(char *out_buf, size_t len);
    static void get_current_datetime_str(char *out_buf, size_t len);
};

#endif // SD_CARD_MANAGER_H