#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H

#include <stdio.h>
#include <stdbool.h>
#include <stdatomic.h>

#include <driver/ledc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"
#include "freertos/semphr.h"

#include <config/config_main.h>
#include "safety/safety_controller.h"
#include "motion/motor_controller.h"
#include <serial/serial_service.h>

#include "esp_err.h"
#include "analog_read/analog_read_service.h"

// #include "fuzzy_controller/codegen_simulink.h"

extern "C" {
    #include "fuzzy_controller/codegen_simulink.h"
}

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef TRACKING_MODES
#define TRACKING_MODES

    typedef enum tracking_modes_t
    {
        UART_TRACKING,        // cntrl via UART communication
        FULL_SENSOR_TRACKING, // cntrl by photoressistor readings
        TIMED_TRACKING,       //  cntrl based on fixed tables
        HYBRID_TRACKING,      //  cntrl including both timed and measured tracking
        AI_TRACKING           // TBD
    } tracking_modes_t;
#endif // TRACKING_MODES

#ifndef TRACKING_LOOOP_DLY
#define TRACKING_LOOP_DLY 1000
#endif

#ifndef MIXED_TRACKING_RANGE
#define MIXED_TRACKING_RANGE 75
#endif

// C++ classes are used only when compiled with C++
#ifdef __cplusplus
    typedef struct motor_setpoint_t
    {
        int target_position_R;
        int target_position_L;
        bool request_homing = false;
        bool request_night_pos = false;
    };

    class TrackingController
    {
    private:
        static tracking_modes_t current_mode;
        static MotorController *motor_r, *motor_l;
        static AnalogReadService *analog_read_service;
        static motor_setpoint_t current_setpoint;

        static adc_channel_t *adc_channels;

        static int const_tracker_positions[19][4], hybrid_tracker_positions[5][3],
                    timed_tracking_iterator, hybrid_tracking_iterator, ai_tracking_iterator;
                    
        static uint64_t ai_time_from_last, ai_consider_time, ai_last_timestamp;

        static TickType_t ai_last_wait_time;


        static void main_loop(void *pvParameter);

        /* Following methods are responsible for calculating single iterations of tracking loop */

        /**
         * @brief Gets first available queued position sent via UART
         *
         */
        static motor_setpoint_t iterate_uart_tracking();

        /**
         * @brief Approximates next position based on quad sensor array
         *
         */
        static motor_setpoint_t iterate_full_sensor_tracking(bool reduce_step = false);

        /**
         * @brief Gets the position based on date and time, from constant array
         *
         */
        static motor_setpoint_t iterate_timed_tracking();

        /**
         * @brief TBD; Intention is to set the overall curve via array of constants, but index it by
         * light intensity, not hour of the day
         *
         */
        static motor_setpoint_t iterate_hybrid_tracking();

        /**
         * @brief TBD
         *
         */
        static motor_setpoint_t iterate_AI_tracking();

        static bool set_night_position();

    public:
        static esp_err_t init();
        static void change_mode(tracking_modes_t mode_to_set);
        static tracking_modes_t get_mode();

        static motor_setpoint_t get_motors_target();
        static motor_setpoint_t get_motors_position();

        static float ai_last_result;

        /**
         * @brief Get the motors moving per motor
         *
         * @return uint8_t - contains binary data of each motor b(000000LR)
         */
        static uint8_t get_motors_moving();
        /**
         * @brief Get the motors homing per motor
         *
         * @return uint8_t - contains binary data of each motor b(000000LR)
         */
        static uint8_t get_motors_homing();

        /**
         * @brief Get the errors per motor. In order of:has_timeout, PI_missed_step
         *
         * @return uint8_t - contains binary data of each motor b(0000LLRR)
         */
        static uint8_t get_motor_errors();

        /**
         * @brief Calls log function on both motors
         *
         */
        static void log_motors();

        static esp_err_t get_adc_readings(int *out_adc_readings, int *out_adc_readings_len);

        static esp_err_t get_temperature_humidity(float &out_temp, float &out_humidity);

        static bool start_home_motors();
    };

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // TRACKING_CONTROLLER_H
