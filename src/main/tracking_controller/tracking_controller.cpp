#include "tracking_controller/tracking_controller.h"

MotorController *TrackingController::motor_r = NULL;
MotorController *TrackingController::motor_l = NULL;
tracking_modes_t TrackingController::current_mode = AI_TRACKING;
adc_channel_t *TrackingController::adc_channels = NULL;
AnalogReadService *TrackingController::analog_read_service = NULL;
uint64_t TrackingController::ai_time_from_last = 0;
uint64_t TrackingController::ai_consider_time = 0;
uint64_t TrackingController::ai_last_timestamp = 0;
int TrackingController::ai_tracking_iterator = 0;
float TrackingController::ai_last_result = 0.0f;
TickType_t TrackingController::ai_last_wait_time = 0;

int TrackingController::const_tracker_positions[19][4] =
    {
        // Place your motor config here, example:
        // timestamp, L, R, Home?
        // {1746918000, 0, 800, 0},
        // {1746952200, 450, 450, 0},
        // {1746966600, 800, 0, 0},
};

int TrackingController::hybrid_tracker_positions[5][3] =
    {
        // Place your motor positions config here, example:
        // HHMM, L, R
        // {100, 0, 800},
        // {1000, 0, 600},
        // {1130, 300, 300},
        // {1330, 600, 0},
        // {1500, 800, 0},
};

int TrackingController::timed_tracking_iterator = 1;
int TrackingController::hybrid_tracking_iterator = 0;

void TrackingController::main_loop(void *pvParameter)
{
    tracker_states state_to_set = IDLE;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(TRACKING_LOOP_DLY));

        if (!is_stop_triggered())
        {
            if (get_state() == IDLE || get_state() == NIGHT)
            {

                motor_setpoint_t new_setpoint;

                switch (current_mode)
                {
                case UART_TRACKING:
                    new_setpoint = iterate_uart_tracking();
                    break;
                case FULL_SENSOR_TRACKING:
                    new_setpoint = iterate_full_sensor_tracking();
                    break;
                case TIMED_TRACKING:
                    new_setpoint = iterate_timed_tracking();
                    break;
                case HYBRID_TRACKING:
                    new_setpoint = iterate_hybrid_tracking();
                    break;
                case AI_TRACKING:
                    new_setpoint = iterate_AI_tracking();
                    break;
                }

                // before commanding motor move, check if homing was requested
                if (new_setpoint.request_homing)
                {
                    if (start_home_motors() && current_mode == UART_TRACKING)
                    {
                        // reset homing flag (and night in the future) in the serial service
                        // not too sure if that is a good practice, but is the simplest to synchronize the tasks correctly
                        SerialService::reset_homing_flag();
                    }

                    continue;
                }
                else if (new_setpoint.request_night_pos)
                {
                    if (get_state() != NIGHT)
                    {
                        start_home_motors();
                        set_state(NIGHT);
                        continue;
                    }
                    else
                    {
                        state_to_set = NIGHT;
                        new_setpoint.target_position_L = 0;
                        new_setpoint.target_position_R = 750;
                    }
                }
                else
                {
                    state_to_set = TRACKING;
                }

                // TBD
                // else if (new_setpoint.request_night_pos)
                // {
                //     set_night_position();
                //     continue;
                // }

                target_pos_result target_result_r, target_result_l;
                target_result_r = motor_r->set_target_position(new_setpoint.target_position_R);
                target_result_l = motor_l->set_target_position(new_setpoint.target_position_L);

                if (target_result_l == TARGET_OUTSIDE_BOUNDS || target_result_r == TARGET_OUTSIDE_BOUNDS)
                {
                    ESP_LOGW(LOG_TAG_MOTORS, "At least one motor was set outside the bounds. Result R: %i, L: %i", target_result_l,
                             target_result_r);
                    // use locking method to warn user about the error
                    set_state(WARNING);
                }
                else if (target_result_l == TARGET_MOTOR_IS_MOVING || target_result_r == TARGET_MOTOR_IS_MOVING)
                {
                    ESP_LOGW(LOG_TAG_MOTORS, "At least one motor was moving when it was not expected. Skipping tracking cycle. Result R: %i, L: %i",
                             target_result_l,
                             target_result_r);
                    continue;
                }
                else if (target_result_l == TARGET_DELTA_TOO_SMALL && target_result_r == TARGET_DELTA_TOO_SMALL)
                {
                    ESP_LOGW(LOG_TAG_MOTORS, "Delta set for the motors was too small. Skipping tracking cycle. Result R: %i, L: %i",
                             target_result_l,
                             target_result_r);

                    continue;
                }

                if (!set_state(state_to_set))
                {
                    ESP_LOGW(LOG_TAG_RUNTIME, "Unable to set tracking state. Stoppping motors and skipping tracking cycle.");

                    motor_l->abort_movement();
                    motor_r->abort_movement();

                    continue;
                }

                // for now, waiting is done here - but that may change in order to keep loop time intact
                do
                {
                    // ESP_LOGD(LOG_TAG_MAIN, "Got motors: %i", get_motors_moving());
                    vTaskDelay(pdMS_TO_TICKS(500));
                } while (get_motors_moving() != 0);

                if (state_to_set != NIGHT)
                {
                    set_state(IDLE);
                }
            }

            else if (get_state() == HOMING)
            {
                if (get_motors_homing() == 0)
                {
                    ESP_LOGD(LOG_TAG_RUNTIME, "Motors finished homing!");
                    // signal end of homing
                    set_state(IDLE);
                }
            }
        }
    }
}

esp_err_t TrackingController::init()
{
    esp_err_t result = ESP_FAIL;
    // init motors
    motor_l = new MotorController(MOTOR_L_CW_LEDC_CHANNEL, MOTOR_L_CCW_LEDC_CHANNEL, MOTOR_L_ENC, MOTOR_L_MAX_TRAVEL, MOTOR_PI_Kp, MOTOR_PI_Ki);
    motor_r = new MotorController(MOTOR_R_CW_LEDC_CHANNEL, MOTOR_R_CCW_LEDC_CHANNEL, MOTOR_R_ENC, MOTOR_R_MAX_TRAVEL, MOTOR_PI_Kp, MOTOR_PI_Ki);

    // init motor R and return if failed
    result = motor_r->init_controller();
    if (result != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_RUNTIME, "Failed to initialize motor R: %s", esp_err_to_name(result));
        return result;
    }
    // init motor L and return if failed
    result = motor_l->init_controller();
    if (result != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_RUNTIME, "Failed to initialize motor L: %s", esp_err_to_name(result));
        return result;
    }

    // init ADC reading
    adc_channels = new adc_channel_t[]{ADC_FOTO_TL,
                                       ADC_FOTO_TR,
                                       ADC_FOTO_BL,
                                       ADC_FOTO_BR,
                                       ADC_FOTO_NEUTRAL};

    analog_read_service = new AnalogReadService(adc_channels, ADC_NUM_CHANNELS, ADC_FREQUENCY, ADC_AVERAGE_SAMPLES);
    analog_read_service->init_adc_reading_service();

    // finally, start the task's loop
    TaskHandle_t xHandleTrackingLoop = NULL;

    xTaskCreate(&TrackingController::main_loop,
                "trloop_task",
                8192,
                NULL,
                2,
                &xHandleTrackingLoop);

    codegen_simulink_initialize();

    return ESP_OK;
}

void TrackingController::change_mode(tracking_modes_t mode_to_set)
{
    // this probably is not that prone to multithreading-related issues (a set and forget thing)
    // but maybe some multithreading locks will have to be implemented later on
    current_mode = mode_to_set;
}

tracking_modes_t TrackingController::get_mode()
{
    return current_mode;
}

motor_setpoint_t TrackingController::iterate_uart_tracking()
{
    motor_setpoint_t result;
    // if halt was called, return current position
    if (atomic_load(&SerialService::halt_tracking))
    {
        motor_setpoint_t current_position = get_motors_position();
        result.target_position_L = current_position.target_position_L;
        result.target_position_R = current_position.target_position_R;
        result.request_homing = false;
        result.request_night_pos = false;
    }
    // else return requested position
    else
    {
        result.target_position_L = SerialService::last_motor_L_cmd;
        result.target_position_R = SerialService::last_motor_R_cmd;
        result.request_homing = SerialService::get_homing_flag();
        result.request_night_pos = false;
    }

    return result;
}

motor_setpoint_t TrackingController::iterate_full_sensor_tracking(bool reduce_step)
{
    // all results of this method rely on tracker's current position, so get it
    motor_setpoint_t result = get_motors_target();

    // get ADC readings from all five sensors
    int out_adc_readings[ADC_NUM_CHANNELS] = {-1}, out_num_channels = -1;

    // TODO replace ESP_ERROR_CHECK with error handling
    esp_err_t adc_result = TrackingController::get_adc_readings(out_adc_readings, &out_num_channels);

    // if reading ADC's failed, stop the tracker and log the error
    if (adc_result != ESP_OK)
    {
        // set_stop_triggered();
        ESP_LOGE(LOG_TAG_RUNTIME, "Error occurred when reading ADC's in tracking, returning.");
        return result;
    }

    // logic is simple:
    // calculate Left, Right, Top and Bottom values (average of sensors)
    // check if night pos should be set
    // compare L<>R, if any is larger, add some constant value to one motor and subtract from the other
    // then compare T<>R, if any is larger, add/subtract a value from both motors
    // return the result and wait for another iteration
    // NOTE: caller should handle timing, i.e. call this method onlyu every X minutes, plus prevent too long repositioning

    // convert int values from array to named float values
    float TL = out_adc_readings[0] / 1000.0f,
          TR = out_adc_readings[1] / 1000.0f,
          BL = out_adc_readings[2] / 1000.0f,
          _BR = out_adc_readings[3] / 1000.0f; // NOTE BR is some system constant, so _ was added
    float top = (TL + TR) / 2,
          bottom = (BL + _BR) / 2,
          left = (TL + BL) / 2,
          right = (TR + _BR) / 2;
    float vertical = bottom - top,
          horizontal = right - left;
    float average = (TL + TR + BL + _BR) / 4;

    float step = reduce_step ? TRACKING_STEP_REDUCED : TRACKING_STEP;

    // check night condition
    if (average <= TRACKING_NIGHT_TRSH)
    {
        result.request_night_pos = true;
        return result;
    }

    else
    {
        // vertical check
        if (abs(vertical) <= TRACKING_DEADZONE_VALUE)
        {
            ESP_LOGI(LOG_TAG_RUNTIME, "Vertical value is too small, not moving motors. Value: %f", horizontal);

            // do not modify result
        }
        else if (vertical > 0)
        {
            result.target_position_R += step;
            result.target_position_L += step;
        }
        else
        { // vertical < 0
            result.target_position_R -= step;
            result.target_position_L -= step;

            // return result;
        }

        //
        if (result.target_position_R > MOTOR_R_MAX_TRAVEL)
        {
            result.target_position_R = MOTOR_R_MAX_TRAVEL;
        }
        else if (result.target_position_R < 0)
        {
            result.target_position_R = 0;
        }

        if (result.target_position_L > MOTOR_L_MAX_TRAVEL)
        {
            result.target_position_L = MOTOR_L_MAX_TRAVEL;
        }
        else if (result.target_position_L < 0)
        {
            result.target_position_L = 0;
        }

        // horizontal check
        if (abs(horizontal) <= TRACKING_DEADZONE_VALUE)
        {
            ESP_LOGI(LOG_TAG_RUNTIME, "Horizontal value is too small, not moving motors. Value: %f", horizontal);
            // do not modify result
        }
        else if (horizontal > 0)
        {
            result.target_position_R = result.target_position_R - step;
            result.target_position_L = result.target_position_L + step;
        }
        else
        { // horizontal < 0
            result.target_position_R = result.target_position_R + step;
            result.target_position_L = result.target_position_L - step;
        }
    }

    ESP_LOGI(LOG_TAG_RUNTIME, "setting target positions: L: %i, R: %i", result.target_position_L, result.target_position_R);

    return result;
}
motor_setpoint_t TrackingController::iterate_timed_tracking()
{
    // ESP_LOGD(LOG_TAG_RUNTIME, "Selected TIMED_TRACKING mode, which is not supported yet");

    motor_setpoint_t result = get_motors_position();
    int timestamp = time(nullptr); // / 1e10; // get current time in seconds since epoch

    while (timed_tracking_iterator < 19 && timestamp > const_tracker_positions[timed_tracking_iterator][0])
    {
        // increment iterator
        timed_tracking_iterator += 1;
        // Set homing only once, on position change
        result.request_homing = const_tracker_positions[timed_tracking_iterator - 1][3];
    }

    // if end of table was reached, change mode to full sensor tracking
    if (timed_tracking_iterator >= 19)
    {
        change_mode(tracking_modes_t::FULL_SENSOR_TRACKING);
    }

    ESP_LOGI(LOG_TAG_RUNTIME, "Current time: %i, iterator: %i", timestamp, timed_tracking_iterator);

    result.target_position_L = const_tracker_positions[timed_tracking_iterator - 1][1];
    result.target_position_R = const_tracker_positions[timed_tracking_iterator - 1][2];

    return result;
}
motor_setpoint_t TrackingController::iterate_hybrid_tracking()
{
    // calculate time substitute value (timestamp)
    // based on the timesamp, find base position for that time
    // if position was changed, return base value
    // if position was not changed, iterate full sensor tracking with decreased step
    // limit to local min/max values
    time_t timestamp = time(nullptr);
    tm *timeinfo = localtime(&timestamp);
    motor_setpoint_t result = get_motors_position(), base_result;

    int custom_timestamp = timeinfo->tm_hour * 100 + timeinfo->tm_min;

    int next_iterator = hybrid_tracking_iterator == 4 ? 0 : hybrid_tracking_iterator + 1,
        start_iterator = hybrid_tracking_iterator;

    // if current timestamp is
    while ((hybrid_tracking_iterator != 4 && custom_timestamp > hybrid_tracker_positions[next_iterator][0]) || ((hybrid_tracking_iterator == 4) && custom_timestamp < 50))
    {
        // hybrid iterator is increased, but not more than 5
        // next iterator is always 1 more, but also not exceeding 5
        hybrid_tracking_iterator = hybrid_tracking_iterator == 4 ? 0 : hybrid_tracking_iterator + 1;
        next_iterator = hybrid_tracking_iterator == 4 ? 0 : hybrid_tracking_iterator + 1;
        ESP_LOGI(LOG_TAG_RUNTIME, "Current time: %i, iterator: %i, next iterator: %i", custom_timestamp, hybrid_tracking_iterator, next_iterator);
    }
    ESP_LOGI(LOG_TAG_RUNTIME, "Current time: %i; settled on: iterator: %i, greater? %i; is last step? %i",
             custom_timestamp,
             hybrid_tracking_iterator,
             (hybrid_tracking_iterator != 4 && custom_timestamp > hybrid_tracker_positions[next_iterator][0]),
             (hybrid_tracking_iterator == 4 && custom_timestamp < 50));

    base_result.target_position_L = hybrid_tracker_positions[hybrid_tracking_iterator][1];
    base_result.target_position_R = hybrid_tracker_positions[hybrid_tracking_iterator][2];

    // if iterator was changed, return base position for that iterator
    if (start_iterator != hybrid_tracking_iterator)
    {
        ESP_LOGI(LOG_TAG_RUNTIME, "Hybrid tracking: setting target positions: L: %i, R: %i", result.target_position_L, result.target_position_R);
        return base_result;
    }
    else
    {
        // get calculated position from full sensor tracking
        result = iterate_full_sensor_tracking(true);
        if (result.request_night_pos)
        {
            return result; // if night position was requested, return it
        }
        else
        {
            // else limit the local base position
            if (result.target_position_R > base_result.target_position_R + MIXED_TRACKING_RANGE)
            {
                result.target_position_R = base_result.target_position_R + MIXED_TRACKING_RANGE;
            }
            else if (result.target_position_R < base_result.target_position_R - MIXED_TRACKING_RANGE)
            {
                result.target_position_R = base_result.target_position_R - MIXED_TRACKING_RANGE;
            }

            if (result.target_position_L > base_result.target_position_L + MIXED_TRACKING_RANGE)
            {
                result.target_position_L = base_result.target_position_L + MIXED_TRACKING_RANGE;
            }
            else if (result.target_position_L < base_result.target_position_L - MIXED_TRACKING_RANGE)
            {
                result.target_position_L = base_result.target_position_L - MIXED_TRACKING_RANGE;
            }

            // then to min/max values
            if (result.target_position_R > MOTOR_R_MAX_TRAVEL)
            {
                result.target_position_R = MOTOR_R_MAX_TRAVEL;
            }
            else if (result.target_position_R < 0)
            {
                result.target_position_R = 0;
            }

            if (result.target_position_L > MOTOR_L_MAX_TRAVEL)
            {
                result.target_position_L = MOTOR_L_MAX_TRAVEL;
            }
            else if (result.target_position_L < 0)
            {
                result.target_position_L = 0;
            }

            // then return the result
            ESP_LOGI(LOG_TAG_RUNTIME, "Hybrid tracking: setting target positions: L: %i, R: %i", result.target_position_L, result.target_position_R);
            return result;
        }
    }
}
motor_setpoint_t TrackingController::iterate_AI_tracking()
{
    motor_setpoint_t result = get_motors_position();

    // ensure 1 minute delay between AI iterations
    for (int t = 0; t < 60; t++)
        vTaskDelayUntil(&ai_last_wait_time, pdMS_TO_TICKS(1000));

    // Get light sensor values
    // get ADC readings from all five sensors
    int out_adc_readings[ADC_NUM_CHANNELS] = {-1}, out_num_channels = -1;

    // TODO replace ESP_ERROR_CHECK with error handling
    esp_err_t adc_result = TrackingController::get_adc_readings(out_adc_readings, &out_num_channels);

    // if reading ADC's failed, stop the tracker and log the error
    if (adc_result != ESP_OK)
    {
        // set_stop_triggered();
        ESP_LOGE(LOG_TAG_RUNTIME, "Error occurred when reading ADC's in tracking, returning.");
        return result;
    }

    codegen_simulink_U.Light_Diff = abs(((float)out_adc_readings[0] + (float)out_adc_readings[1] - (float)out_adc_readings[2] - (float)out_adc_readings[3]) / 2); // (TL + TR - Bl - BR)/2
    codegen_simulink_U.Sun_Strength = (float)out_adc_readings[4];                                                                                                 // central sensor
    codegen_simulink_U.Time_From_Last = ai_time_from_last;
    codegen_simulink_U.Consider_Time = ai_consider_time;

    codegen_simulink_step();

    float decision = codegen_simulink_Y.Move_Decision;
    ESP_LOGI(LOG_TAG_RUNTIME, "AI decision: %f for inputs: diff %f, strength: %f, time from last: %f, consider: %f",
             decision,
             codegen_simulink_U.Light_Diff,
             codegen_simulink_U.Sun_Strength,
             codegen_simulink_U.Time_From_Last,
             codegen_simulink_U.Consider_Time);

    time_t datetime = time(nullptr);
    uint64_t timestamp = datetime, time_delta = timestamp - ai_last_timestamp;
    tm *timeinfo = localtime(&datetime);
    int custom_timestamp = timeinfo->tm_hour * 100 + timeinfo->tm_min;

    ai_time_from_last += time_delta; // update time from last movement
    ai_consider_time += time_delta;  // update consideration time

    ai_last_result = decision;

    // decide on the result
    if (decision > 0.7f)
    {
        // reset both timers
        ai_consider_time = 0;
        ai_time_from_last = 0;
        // Trigger movement

        // NOTE: for clarity, the hybrid tracker positions are used here
        // and iterated manually, but it could be done using aready exisiting methods

        int next_iterator = ai_tracking_iterator == 4 ? 0 : ai_tracking_iterator + 1,
            start_iterator = ai_tracking_iterator;

        // if current timestamp is
        while ((ai_tracking_iterator != 4 && custom_timestamp > hybrid_tracker_positions[next_iterator][0]) || ((ai_tracking_iterator == 4) && custom_timestamp < 50))
        {
            // hybrid iterator is increased, but not more than 4
            // next iterator is always 1 more, but also not exceeding 4
            ai_tracking_iterator = ai_tracking_iterator == 4 ? 0 : ai_tracking_iterator + 1;
            next_iterator = ai_tracking_iterator == 4 ? 0 : ai_tracking_iterator + 1;
            ESP_LOGI(LOG_TAG_RUNTIME, "Current time: %i, iterator: %i, next iterator: %i", custom_timestamp, ai_tracking_iterator, next_iterator);
        }
        ESP_LOGI(LOG_TAG_RUNTIME, "[Fuzzy] Current time: %i; settled on: iterator: %i, greater? %i; is last step? %i",
                 custom_timestamp,
                 ai_tracking_iterator,
                 (ai_tracking_iterator != 4 && custom_timestamp > hybrid_tracker_positions[next_iterator][0]),
                 (ai_tracking_iterator == 4 && custom_timestamp < 50));

        result.target_position_L = hybrid_tracker_positions[ai_tracking_iterator][1];
        result.target_position_R = hybrid_tracker_positions[ai_tracking_iterator][2];
    }
    else if (decision > 0.4f)
    {
        // consideration state
        // do not reset any timer
    }
    else
    {
        // Stay idle
        // reset consider time
        ai_consider_time = 0;
    }

    return result;
}

motor_setpoint_t TrackingController::get_motors_target()
{
    motor_setpoint_t result{
        .target_position_R = motor_r->get_target_position(),
        .target_position_L = motor_l->get_target_position(),
    };
    return result;
}

motor_setpoint_t TrackingController::get_motors_position()
{
    motor_setpoint_t result{
        .target_position_R = motor_r->get_position(),
        .target_position_L = motor_l->get_position(),
    };
    return result;
}

uint8_t TrackingController::get_motors_moving()
{
    uint8_t result = (uint8_t)(motor_l->get_is_moving() << 1) | (uint8_t)(motor_r->get_is_moving()); // add last byte

    // uint8_t result = 0;
    // result += motor_l->get_is_moving() & 0x01; // add last byte
    // result <<= 1;                              // lshift
    // result += motor_r->get_is_moving() & 0X01; // add last byte
    return result;
}

uint8_t TrackingController::get_motors_homing()
{
    // uint8_t result = 0;
    uint8_t result = (uint8_t)(motor_l->get_is_homing() << 1) | (uint8_t)(motor_r->get_is_homing()); // add last byte
    // result <<= 1;                                    // lshift
    // result += (uint8_t)(motor_r->get_has_timeout());   // add last byte
    return result;
}

void TrackingController::log_motors()
{
    if (motor_l != NULL && motor_r != NULL)
    {
        ESP_LOGD(LOG_TAG_MOTORS, "motor_l");
        motor_l->log_current_state();
        ESP_LOGD(LOG_TAG_MOTORS, "motor_r");
        motor_r->log_current_state();
    }
}

bool TrackingController::start_home_motors()
{
    // try to set the state
    int retries = 0;
    while (1 == 1)
    {
        if (retries >= 5)
        {
            ESP_LOGW(LOG_TAG_HOMING, "Unable to set state, returning");
            return false;
        }
        else if (set_state(HOMING))
        {
            break;
        }
        retries += 1;
        vTaskDelay(pdTICKS_TO_MS(0.25));
    }

    motor_l->home_async();
    motor_r->home_async();

    return true;
}

esp_err_t TrackingController::get_adc_readings(int *out_adc_readings, int *out_adc_readings_len)
{
    if (analog_read_service != nullptr)
    {
        esp_err_t result = analog_read_service->get_last_readings(out_adc_readings, out_adc_readings_len);

        if (result != ESP_OK)
        {
            ESP_LOGE(LOG_TAG_RUNTIME, "Failed to get ADC readings: %s.Setting safety!", esp_err_to_name(result));
            set_stop_triggered();
            set_state(ERROR);
        }

        return result;
    }
    else
    {
        return ESP_FAIL;
    }
}

esp_err_t TrackingController::get_temperature_humidity(float &out_temp, float &out_humidity)
{
    esp_err_t result;
    result = ESP_FAIL;
    // result = SHT30::read(out_temp, out_humidity);

    if (result != ESP_OK)
    {
        out_temp = -99.0;
        out_humidity = -1.0;
    }

    return result;
}

uint8_t TrackingController::get_motor_errors()
{
    return (uint8_t)(motor_l->get_has_timeout() << 3) |
           (uint8_t)(motor_l->get_PI_missed_step() << 2) |
           (uint8_t)(motor_r->get_has_timeout() << 1) |
           (uint8_t)(motor_r->get_PI_missed_step());
}
