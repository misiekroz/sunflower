#include "motion/motor_controller.h"
#include <soc/gpio_struct.h>

//***PUBLIC MEMBERS***//
MotorController::MotorController(ledc_channel_t ledc_channel_cw,
                                 ledc_channel_t ledc_channel_ccw,
                                 gpio_num_t motor_encoder_pin,
                                 int max_travel,
                                 float Kp,
                                 float Ki) : setpoint(0),
                                             target_position(0),
                                             max_travel(max_travel),
                                             current_pos(0),
                                             motor_encoder_pin(motor_encoder_pin),
                                             ledc_channel_cw(ledc_channel_cw),
                                             ledc_channel_ccw(ledc_channel_ccw),
                                             Kp(Kp),
                                             Ki(Ki),
                                             PWM_val(0),
                                             current_direction(false),
                                             last_isr_time_us(0)
{
    atomic_store(&is_homing, false);
    atomic_store(&is_moving, false);
    atomic_store(&has_timeout, false);
    atomic_store(&PI_missed_step, false);
}

esp_err_t MotorController::init_controller()
{
    TaskHandle_t xHandleWatchdogtask = NULL,
                 xHandleLoopTask = NULL;

    xTaskCreate(&MotorController::watchdog_task_wrapper,
                "wdog_task",
                4096,
                this,
                3,
                &xHandleWatchdogtask);

    xTaskCreate(&MotorController::motor_loop_task_wrapper,
                "loop_task",
                4096,
                this,
                3,
                &xHandleLoopTask);

    return gpio_isr_handler_add(motor_encoder_pin, MotorController::encoder_isr_wrapper, this);
}

target_pos_result MotorController::set_target_position(int new_target_position)
{
    // target position can be changed only if:
    // motor is not homing
    // motor is not moving
    // the passed position is outside motor deadzone in relation to current position
    // TODO: change return type to int, return 1 on success, 0 on max pos exceeded and -1 on invalid pos
    // + add check for max stroke
    if (get_is_homing() || get_is_moving())
        return TARGET_MOTOR_IS_MOVING;
    else if (abs(new_target_position - target_position.load()) < MOTOR_DEADZONE_PLS)
        return TARGET_DELTA_TOO_SMALL;
    else if (new_target_position > max_travel)
    {
        target_position.store(max_travel);
        return TARGET_OUTSIDE_BOUNDS;
    }
    else if (new_target_position < 0)
    {
        target_position.store(0);
        return TARGET_OUTSIDE_BOUNDS;
    }

    else
    {
        target_position.store(new_target_position);
        return TARGET_OK;
    }
}

int MotorController::get_position(void)
{
    return current_pos.load();
}

int MotorController::get_target_position(void)
{
    return target_position.load();
}

int MotorController::get_setpoint(void)
{
    return setpoint.load();
}

bool MotorController::get_is_moving(void)
{
    return atomic_load(&is_moving);
}

bool MotorController::get_is_homing(void)
{
    return atomic_load(&is_homing);
}

bool MotorController::get_has_timeout(void)
{
    return atomic_load(&has_timeout);
}

bool MotorController::get_PI_missed_step(void)
{
    return atomic_load(&PI_missed_step);
}

bool MotorController::abort_movement(void)
{
    if (atomic_load(&is_moving))
    {
        target_position.store(setpoint.load());
        return true;
    }
    return false;
}

bool MotorController::home_blocking(void)
{
    // if is already homing, return - the outcome is still that the motor is homing correctly
    if (get_is_homing())
    {
        return false;
    }

    // set the homing flag
    atomic_store(&is_homing, true);

    // homing is arbitrary over movement, so stop if needed
    if (get_is_moving())
    {
        abort_movement();
    }

    // NOTE: this order of aciton ensures correct PI working, even if it is not entirely correct way of setting the setpoint
    // command the move to find 0 position
    target_position.store(0);

    // override current pos to trick the controller into moving untill hard stop
    current_pos.store(2.0 * max_travel);

    ESP_LOGD(LOG_TAG_HOMING, "In homing, params are: %i, %i", target_position.load(), current_pos.load());

    // wait untill position does not change anymore
    // TODO: add wait timeout AND raise a flag if error occured
    int last_pos = 0; // current_pos.load();

    // NOTE: this piece of code does not care about stop state -> if STOP is triggered, motor will not move anyways
    do
    {
        last_pos = current_pos.load();
        ESP_LOGD(LOG_TAG_HOMING, "last pos: %i", last_pos);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } while (abs(last_pos - current_pos.load()) > MOTOR_DEADZONE_PLS);

    // abort_movement();
    ESP_LOGD(LOG_TAG_HOMING, "resetting current pos");
    current_pos.store(0);

    // move some pulses up, to prevent hitting hard stop during work
    ESP_LOGD(LOG_TAG_HOMING, "moing to 30");
    target_position.store(30);

    ESP_LOGD(LOG_TAG_HOMING, "waiting for move to 30");
    // wait for the motor to stop again, this
    wait_motor_stopped();

    ESP_LOGD(LOG_TAG_HOMING, "resetting current pos again");
    // zero out the current position once again
    current_pos.store(0);
    target_position.store(0);

    ESP_LOGD(LOG_TAG_HOMING, "homing done!");
    // reset homing flag and return
    atomic_store(&is_homing, false);
    return true;
}

void MotorController::home_async()
{
    TaskHandle_t xHandleHomeAsync = NULL;
    xTaskCreate(&MotorController::homing_wrapper,
                "homing_task",
                4096,
                this,
                2,
                &xHandleHomeAsync);
}

void MotorController::wait_motor_stopped(void)
{
    // a note on no-timeouts strategy:
    // returning
    do
    {
        // ESP_LOGD(LOG_TAG_MOTORS, "Cur pos: %i", current_pos.load());
        vTaskDelay(pdTICKS_TO_MS(MOTOR_WAIT_LOOP_DELAY));
    } while (get_is_moving());
}

//***PRIVATE MEMBERS***//
void MotorController::watchdog_task(void)
{
    int timer = -1, last_recorded_position = 0;
    while (1)
    {
        // if is moving, check timer
        if (get_is_moving() || get_is_homing())
        {
            // if motor just started moving, init the timer
            if (timer < 0)
            {
                timer = 0;
            }
            // if motor already exceeded timeout, raise the flag
            else if (timer >= MOTOR_TIMEOUT)
            {
                // NOTE: has timeout is never, and should never be reset to faLse.
                atomic_store(&has_timeout, true);
            }
            else
            {
                // in all other cases, either increment the timer if no movement is detected
                if (abs(current_pos.load() - last_recorded_position) <= MOTOR_DEADZONE_PLS)
                {
                    timer += MOTOR_WAIT_LOOP_DELAY;
                }
                // or zero out the timer
                else
                {
                    timer = 0;
                }
            }
        }
        else
        {
            // reset the timer value
            timer = -1;
        }

        if (get_has_timeout() || get_PI_missed_step())
        {
            set_stop_triggered();
            set_state(ERROR);
            ESP_LOGE(LOG_TAG_MOTORS, "Motor timeout or missed step detected!");
            log_current_state();
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_WAIT_LOOP_DELAY));
    }
}

void IRAM_ATTR MotorController::encoder_isr()
{
    // get current time
    uint64_t current_time_us = esp_timer_get_time();

    // and accept only pulses longer than debounce treshold
    if ((current_time_us - last_isr_time_us) > ENCODER_SOFT_DEBOUNCE_TRSH_US)
    {
        // then count pulse up, but only on high state (to avoid double counting)
        if (gpio_get_level(motor_encoder_pin) == 1)
        {
            (current_direction ? current_pos.fetch_add(1, std::memory_order_relaxed)
                               : current_pos.fetch_sub(1, std::memory_order_relaxed));
            last_isr_time_us = current_time_us; // update last ISR time
        }
    }
}

void MotorController::motor_loop_task(void)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    bool first = true;

    while (1)
    {
        // first, do anything only if stop is not triggered
        if (!is_stop_triggered())
        {
            // atomic_store(&is_moving, true);
            // apply ramps whenever needed
            bool requires_PI_reset = update_setpoint();
            // then, process PID, but only if it is needed and the PID parameters do not need to be
            // reset
            if (abs(current_pos.load() - setpoint.load()) > MOTOR_DEADZONE_PLS && !requires_PI_reset)
            {
                if (!atomic_load(&is_moving))
                {
                    atomic_store(&is_moving, true);
                }
                // compute PWM output based on motor state
                float computed_duty, duty_normalized;
                computed_duty = compute_PI();
                duty_normalized = abs(computed_duty);
                // ESP_LOGD(LOG_TAG_MOTORS, "Duty is is: %f", computed_duty);

                // update classes variables
                PWM_val = duty_normalized;
                current_direction = computed_duty >= 0;
            }
            // otherwise shut down the motor completely and reset integral
            else
            {
                atomic_store(&is_moving, false);
                integral = 0;
                PWM_val = 0;
            }

            int duty_to_set;
            duty_to_set = (int)(PWM_val * 1023.0);

            // control cw or ccw output based on the direction of turning
            if (current_direction)
            {
                // TODO: add handling and possibly another error
                ledc_set_duty(LEDC_MODE, ledc_channel_cw, duty_to_set);
                ledc_update_duty(LEDC_MODE, ledc_channel_cw);
                ledc_set_duty(LEDC_MODE, ledc_channel_ccw, 0);
                ledc_update_duty(LEDC_MODE, ledc_channel_ccw);
            }
            else
            {
                ledc_set_duty(LEDC_MODE, ledc_channel_cw, 0);
                ledc_update_duty(LEDC_MODE, ledc_channel_cw);
                ledc_set_duty(LEDC_MODE, ledc_channel_ccw, duty_to_set);
                ledc_update_duty(LEDC_MODE, ledc_channel_ccw);
            }
        }
        else
        {
            // stop motor by resetting all pins
            ledc_set_duty(LEDC_MODE, ledc_channel_cw, 0);
            ledc_update_duty(LEDC_MODE, ledc_channel_cw);
            ledc_set_duty(LEDC_MODE, ledc_channel_ccw, 0);
            ledc_update_duty(LEDC_MODE, ledc_channel_ccw);
            atomic_store(&is_moving, false);
        }

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / MOTOR_PI_LOOP_FREQ));

        // xTaskDelayUntil reports about a step being missed
        set an error flag to report the issue.if (!first)
        {
            if (xWasDelayed == pdFALSE)
                atomic_store(&PI_missed_step, true);
        }
        else first = false;
    }
}

bool MotorController::update_setpoint()
{
    // TODO: currently this function is a placeholder
    // if tests suggest that acceleration and deceleration have to be limited
    // then this function would  act accordingly.
    if (setpoint.load() != target_position.load())
    {
        setpoint.store(target_position);
        return true;
    }
    else
    {
        return false;
    }
}

float MotorController::compute_PI()
{
    float error = ((float)setpoint.load() - (float)current_pos.load()) / (float)max_travel, result = 0;
    // ESP_LOGD(LOG_TAG_MOTORS, "Error is: %f", error);

    // anti-windup
    if (PWM_val > -1.0 && PWM_val < 1.0)
    {
        integral += error;
    }

    result = Kp * error + Ki * integral;

    if (result < -1.0)
    {
        return -1.0;
    }
    else if (result > -0.4 && result < 0)
    {
        return -0.4;
    }
    else if (result > 0 && result < 0.4)
    {
        return 0.4;
    }
    else if (result > 1.0)
    {
        return 1.0;
    }
    else
    {
        return result;
    }
}

void MotorController::log_current_state()
{
    ESP_LOGD(LOG_TAG_MOTORS, "Target position: %i; setpoint %i; current position: %i, direction %i, PWM duty: %f, integral: %f, is_homing: %i",
             target_position.load(), setpoint.load(), current_pos.load(), current_direction, PWM_val, integral, get_is_homing());
}

/*
 * Static members
 */

void MotorController::watchdog_task_wrapper(void *pvParameter)
{
    MotorController *instance = static_cast<MotorController *>(pvParameter);
    instance->watchdog_task(); // Call the actual non-static method
}

void MotorController::motor_loop_task_wrapper(void *pvParameter)
{
    MotorController *instance = static_cast<MotorController *>(pvParameter);
    instance->motor_loop_task(); // Call the actual non-static method
}

void IRAM_ATTR MotorController::encoder_isr_wrapper(void *pvParameter)
{

    MotorController *instance = static_cast<MotorController *>(pvParameter);
    instance->encoder_isr(); // Call the actual non-static method
}

void IRAM_ATTR MotorController::homing_wrapper(void *pvParameter)
{

    MotorController *instance = static_cast<MotorController *>(pvParameter);
    instance->home_blocking(); // Call the actual non-static method
    vTaskDelete(NULL);
}