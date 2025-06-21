#ifndef SERIAL_SERVICE_H
#define SERIAL_SERVICE_H

#include <stdio.h>
#include <stdbool.h>
#include <stdatomic.h>

#include <driver/ledc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "hal/gpio_types.h"
#include "freertos/semphr.h"

#include <driver/uart.h>
#include <string.h>

#include "esp_sleep.h"

#include <config/config_main.h>
#include "safety/safety_controller.h"
#include <tracking_controller/tracking_controller.h>

#include "esp_err.h"

#define MSG_START_SEQ_0 0xA5
#define MSG_START_SEQ_1 0xAA
#define MSG_START_SEQ_2 0xAA
#define MSG_SIZE 9

#ifdef __cplusplus
extern "C"
{
#endif

// C++ classes are used only when compiled with C++
#ifdef __cplusplus

    // Available commands
    enum class frame_command : uint8_t
    {
        OK = 0x00,          // confirmation or PING command
        STOP,               // sets the device into STOP state
        RESET_STOP,         // attempts to reset stop (valid only if physical STOP is not pressed)
        GET_STATE,          // gets current device state (IDLE, STOP, ERROR itp)
        HALT_TRACKING,      // halts tracking to allow full configuration of setpoint
        START_TRACKING,     // starts tracking after halt
        HOME,               // starts homing sequence (not implemented yet)
        SET_TRACKING_MODE,  // sets tracking mode. Device will keep responding  to UART frames, but will not  react to motor commands
        SET_MOTOR_R,        // target position for motor R
        SET_MOTOR_L,        // target position for motor L
        GET_MOTOR_POS_R,    // gets current position of motor R
        GET_MOTOR_POS_L,    // gets current position of motor L
        GET_MOTOR_TARGET_R, // gets current target of right motor
        GET_MOTOR_TARGET_L, // gets current target of left motor
        GET_MOTORS_MOVING,  // gets information if any motor is moving
        GET_IS_HOMING,      // gets information if tracking controller is homing
        GET_READING_1,      // gets reading from photoresisstor 1
        GET_READING_2,      // gets reading from photoresisstor 2
        GET_READING_3,      // gets reading from photoresisstor 3
        GET_READING_4,      // gets reading from photoresisstor 4
        ERROR = 0xFF        // error - either in interpreting the CMD, execution or other

    };

    enum class uart_ok_messages : uint32_t
    {
        OK = 0x00,       // standard OK, no meaning
        MOTORS_STOPPED,  // motors stopped
        HOMING_FINISHED, // homing stopped
    };

    // Plus error codes that are sent as values in Error message
    enum class uart_errors : uint32_t
    {
        UNKNOWN,
        NOT_IMPLEMENTED,
        CHECKSUM_MISMATCH,
        MOTORS_OUT_OF_RAGE,
        SAFETY_BUTTON_NOT_RELEASED,
        HOMING_NOT_STARTED,
    };

    // Frame structure
    typedef struct
    {
        uint8_t start_seq[3];
        frame_command command;
        uint32_t data;
        uint8_t checksum;
    } __attribute__((packed)) uart_frame_t;

    class SerialService
    {
    private:
        static void rx_loop(void *pvParameter);

        static uint8_t calculate_checksum(frame_command command, uint32_t data);

        /**
         * @brief Takes action based on the passed frame and frame_command enum.
         *
         * @param frame frame to analyze
         * @return uart_frame_t response frame (OK or ERROR)
         */
        static uart_frame_t handle_frame(uart_frame_t frame);

    public:
        static int last_motor_L_cmd, last_motor_R_cmd;
        static atomic_bool halt_tracking, start_homing;

        static esp_err_t init();
        static esp_err_t uart_send_frame(frame_command command, uint32_t data);
        static esp_err_t uart_send_frame(uart_frame_t frame);
        static bool get_homing_flag();
        static void reset_homing_flag();
    };

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // SERIAL_SERVICE_H
