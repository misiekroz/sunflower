#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

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

#include "esp_err.h"

#include "esp_timer.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum target_pos_result
    {
        TARGET_OK,              // target pos set correctly
        TARGET_OUTSIDE_BOUNDS,  // target pos was outside max pos, position set to max
        TARGET_DELTA_TOO_SMALL, // target pos not set, due to it being within deadzone from current position
        TARGET_MOTOR_IS_MOVING, // target pos not set, due to motor still moving
    } target_pos_result;

// C++ classes are used only when compiled with C++
#ifdef __cplusplus

/**
 * @brief Simple deadzone to avoid oscillations after reaching target
 */
#ifndef MOTOR_DEADZONE_PLS
#define MOTOR_DEADZONE_PLS 7
#endif

/**
 * @brief Maximum acceleration that motor should reach during ramping
 *
 */
#ifndef MOTOR_MAX_ACCELERATION_PLS_PSS
#define MOTOR_MAX_ACCELERATION_PLS_PSS 10
#endif

#ifndef MOTOR_WAIT_LOOP_DELAY
#define MOTOR_WAIT_LOOP_DELAY 100
#endif

#ifndef MOTOR_TIMEOUT
#define MOTOR_TIMEOUT 10000
#endif

#ifndef MOTOR_PI_LOOP_FREQ
#define MOTOR_PI_LOOP_FREQ 50 // Hz
#endif

#ifndef ENCODER_SOFT_DEBOUNCE_TRSH_US
#define ENCODER_SOFT_DEBOUNCE_TRSH_US 10000
#endif

    class MotorController
    {
    private:
        std::atomic<int> setpoint;        // current setpoint, includes ramp and other factors
        std::atomic<int> target_position; // target position that actuator will reach
        int max_travel;                   // max travel that the actuator can do [pulses]
        std::atomic<int> current_pos;     // current position of the actuator, updated by encoder_isr

        gpio_num_t motor_encoder_pin; // encoder input pin

        ledc_channel_t ledc_channel_cw, ledc_channel_ccw;

        float Kp,     // proportional term of internal PI controller
            Ki,       // integral term of internal PI controller
            PWM_val,  // current PWM value of the controller
            integral; // stores the integral (sum) of past values

        volatile bool current_direction; // stores information about current motor direction (true for extending)

        volatile uint64_t last_isr_time_us; // needed for software debouncing

        atomic_bool is_homing,
            is_moving,
            has_timeout,
            *stop_flag,
            PI_missed_step;

        /**
         * @brief Watches the motor for movement if is_moving flag is true.
         * Raises has_timeout if no movement was detected after set timeout
         *
         */
        void watchdog_task(void);
        /**
         * @brief The main PI control loop. Watches the safety state ond other states in order
         *          to ensure
         *
         */
        void motor_loop_task(void);

        /**
         * @brief Counts encoder ticks. Direction is determined by current_direction flag.
         *
         */
        void IRAM_ATTR encoder_isr();

        /**
         * @brief Based on current parameters, update the setpoint
         *
         * @return true - setpoint updated
         * @return false - setpoint did not change
         */
        bool update_setpoint();

        /**
         * @brief Computes PWM duty cycle based on set target position and current position
         *
         */
        float compute_PI();

        /**
         * @brief Static wrapper for watchdog task
         *
         * @param pvParameter
         */
        static void watchdog_task_wrapper(void *pvParameter);

        /**
         * @brief Static wrapper for control loop task
         *
         * @param pvParameter
         */
        static void motor_loop_task_wrapper(void *pvParameter);

        /**
         * @brief Static wrapper for control encoder isr
         *
         * @param pvParameter
         */
        static IRAM_ATTR void encoder_isr_wrapper(void *pvParameter);

        /**
         * @brief Static wrapper for homing task
         *
         * @param pvParameter
         */
        static IRAM_ATTR void homing_wrapper(void *pvParameter);

    public:
        /**
         * @brief Construct a new Motor Controller:: Motor Controller object
         *
         * @param motor_pwm_pin
         * @param motor_dir_pin
         * @param motor_encoder_pin
         * @param max_travel
         * @param Kp
         * @param Ki
         */
        MotorController(
            ledc_channel_t ledc_channel_cw,
            ledc_channel_t ledc_channel_ccw,
            gpio_num_t motor_encoder_pin,
            int max_travel,
            float Kp,
            float Ki);

        /**
         * @brief Initializes tasks on the motor controller
         *
         */
        esp_err_t init_controller();

        /**
         * @brief Set the target position
         * Prevents setting position when motor is moving, or the new setpoint is not outside
         * deadzone in relation to current position
         *
         * @param setpoint target position value in encoder ticks
         * @return setting result, see target_pos_result for more info
         */
        target_pos_result set_target_position(int setpoint);

        /**
         * @brief returns current position value in encoder ticks
         *
         * @return int
         */
        int get_position(void);

        /**
         * @brief returns target position in encoder ticks
         *
         * @return int
         */
        int get_target_position(void);

        /**
         * @brief Returns setpoint in encoder ticks
         * NOTE: setpoint is the value that is set internally based on ramping and other factors
         *
         * @return int
         */
        int get_setpoint(void);

        /**
         * @brief Returns is_moving state
         *
         * @return true
         * @return false
         */
        bool get_is_moving(void);

        /**
         * @brief Returns is_homing state
         *
         * @return true
         * @return false
         */
        bool get_is_homing(void);

        /**
         * @brief Returns has_timeout state
         *
         * @return true
         * @return false
         */
        bool get_has_timeout(void);

        /**
         * @brief Returns PI_missed_step state
         *
         * @return true
         * @return false
         */
        bool get_PI_missed_step(void);

        /**
         * @brief Aborts motor movement by simply setting target_position to current setpoint
         * NOTE: homing sequence, once started cannot be interrupted in any way other than STOP
         *
         * @return true
         * @return false
         */
        bool abort_movement(void);

        bool home_blocking();
        void home_async();

        /**
         * @brief Blocks until motor is_moving is set to false. No timeout available
         *
         */
        void wait_motor_stopped(void);

        /**
         * @brief Using ESP_LOGD, logs current state of the motor controller
         *
         */
        void log_current_state();
    };

#endif // __cplusplus

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROLLER_H
