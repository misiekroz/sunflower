#include "serial/serial_service.h"

int SerialService::last_motor_L_cmd = 0;
int SerialService::last_motor_R_cmd = 0;
atomic_bool SerialService::halt_tracking = false;
atomic_bool SerialService::start_homing = false;

void SerialService::rx_loop(void *pvParameter)
{
    uint8_t byte;
    uart_frame_t frame;
    int state = 0;
    uint8_t buffer[MSG_SIZE];
    int index = 0;

    while (1)
    {
        if (uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(100)) == 1)
        {
            switch (state)
            {
            case 0:
                if (byte == MSG_START_SEQ_0)
                {
                    // ESP_LOGD(LOG_TAG_RUNTIME, "found sequence start");
                    state = 1;
                }
                break;
            case 1:
                state = (byte == MSG_START_SEQ_1) ? 2 : 0;
                break;
            case 2:
                state = (byte == MSG_START_SEQ_2) ? 3 : 0;
                index = 0;
                break;
            case 3:
                buffer[index++] = byte;

                // ESP_LOGD(LOG_TAG_RUNTIME, "reading message");

                if (index >= 6)
                { // 1 cmd + 4 data + 1 checksum = 6 bytes
                    memcpy(&frame.command, &buffer[0], 1);
                    memcpy(&frame.data, &buffer[1], 4);
                    frame.checksum = buffer[5];

                    uart_frame_t response;

                    uint8_t calculated_checksum = calculate_checksum(frame.command, frame.data);
                    if (calculated_checksum == frame.checksum)
                    {
                        // ESP_LOGI(LOG_TAG_RUNTIME, "Valid Frame: CMD=0x%02X DATA=[ %" PRIu32 " ]", (uint8_t)(frame.command), frame.data);
                        response = handle_frame(frame);
                    }
                    else
                    {
                        ESP_LOGE(LOG_TAG_RUNTIME, "Checksum mismatch! Expected 0x%02X, got 0x%02X", calculated_checksum, (uint8_t)(frame.checksum));
                        response.command = frame_command::ERROR;
                        response.data = (uint32_t)uart_errors::CHECKSUM_MISMATCH;
                    }

                    uart_send_frame(response);
                    state = 0;
                }
                break;
            default:
                state = 0;
                break;
            }
        }
    }
}

uint8_t SerialService::calculate_checksum(frame_command command, uint32_t data)
{
    uint8_t command_cast = (uint8_t)(command);
    return command_cast ^
           ((data >> 24) & 0xFF) ^
           ((data >> 16) & 0xFF) ^
           ((data >> 8) & 0xFF) ^
           (data & 0xFF);
}

esp_err_t SerialService::init()
{
    esp_err_t ret = ESP_OK;
    last_motor_L_cmd = 0;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_BITS,
        .parity = UART_PAIRITY,
        .stop_bits = UART_STOP_BITS,
        .flow_ctrl = UART_FLOWCTRL,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_CLOCK,
    };

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_UART);
    ret = uart_set_wakeup_threshold(UART_NUM_1, 3);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "Failed to disable wakeup source: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    TaskHandle_t xHandleRXLoop = NULL;

    xTaskCreate(&SerialService::rx_loop,
                "rxloop_task",
                4096,
                NULL,
                2,
                &xHandleRXLoop);

    return ESP_OK;
}

esp_err_t SerialService::uart_send_frame(frame_command command, uint32_t data)
{
    uart_frame_t frame;
    frame.start_seq[0] = MSG_START_SEQ_0;
    frame.start_seq[1] = MSG_START_SEQ_1;
    frame.start_seq[2] = MSG_START_SEQ_2;
    frame.command = command;
    frame.data = data;
    frame.checksum = calculate_checksum(command, data);

    int tx_bytes = uart_write_bytes(UART_NUM, (const char *)&frame, MSG_SIZE);
    return (tx_bytes == MSG_SIZE) ? ESP_OK : ESP_FAIL;
}

esp_err_t SerialService::uart_send_frame(uart_frame_t frame)
{
    return uart_send_frame(frame.command, frame.data);
}

uart_frame_t SerialService::handle_frame(uart_frame_t frame)
{
    uart_frame_t response;
    // ESP_LOGD(LOG_TAG_RUNTIME, "Handling command [ %" PRIu32 " ], data=[ %" PRIu32 " ]", (uint32_t)(frame.command), frame.data);

    switch (frame.command)
    {
    // receiving OK is perceived as PING
    // in future, maybe some serial number could be returned
    case frame_command::OK:
    {
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        break;
    }

    // sets the global STOP flag
    case frame_command::STOP:
    {
        set_stop_triggered();
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        break;
    }

    // attempts to reset the stop flag
    case frame_command::RESET_STOP:
    {
        bool result = safety_reset();
        if (result)
        {
            response.command = frame_command::OK;
            response.data = (uint32_t)uart_ok_messages::OK;
        }
        else
        {
            response.command = frame_command::ERROR;
            response.data = (uint32_t)uart_errors::SAFETY_BUTTON_NOT_RELEASED;
        }
        break;
    }

    case frame_command::GET_STATE:
    {
        response.command = frame_command::OK;
        response.data = (uint32_t)get_state();
        break;
    }

    case frame_command::HALT_TRACKING:
    {
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        atomic_store(&halt_tracking, true);
        break;
    }

    case frame_command::START_TRACKING:
    {
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        atomic_store(&halt_tracking, false);
        break;
    }
    case frame_command::HOME:
    {
        // NOTE: this command only attempts to set homing flag (if not already set)
        // It does not inherently ensure motors start the homing move (eg. device might be not in UART mode)
        if (!atomic_load(&start_homing))
        {
            atomic_store(&start_homing, true);
            response.command = frame_command::OK;
            response.data = (uint32_t)uart_ok_messages::OK;
        }
        else
        {
            response.command = frame_command::ERROR;
            response.data = (uint32_t)uart_errors::HOMING_NOT_STARTED;
        }
        break;
    }
    case frame_command::SET_TRACKING_MODE:
    {
        TrackingController::change_mode((tracking_modes_t)frame.data);
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        break;
    }
    case frame_command::SET_MOTOR_R:
    {
        last_motor_R_cmd = frame.data;
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        break;
    }
    case frame_command::SET_MOTOR_L:
    {
        last_motor_L_cmd = frame.data;
        response.command = frame_command::OK;
        response.data = (uint32_t)uart_ok_messages::OK;
        break;
    }
    case frame_command::GET_MOTOR_POS_R:
    {
        motor_setpoint_t tracker_position = TrackingController::get_motors_position();
        response.command = frame_command::OK;
        response.data = (uint32_t)tracker_position.target_position_R;
        break;
    }
    case frame_command::GET_MOTOR_POS_L:
    {
        motor_setpoint_t tracker_position = TrackingController::get_motors_position();
        response.command = frame_command::OK;
        response.data = (uint32_t)tracker_position.target_position_L;
        break;
    }
    case frame_command::GET_MOTOR_TARGET_R:
    {
        motor_setpoint_t tracker_target = TrackingController::get_motors_target();
        response.command = frame_command::OK;
        response.data = (uint32_t)tracker_target.target_position_R;
        break;
    }
    case frame_command::GET_MOTOR_TARGET_L:
    {
        motor_setpoint_t tracker_target = TrackingController::get_motors_target();
        response.command = frame_command::OK;
        response.data = (uint32_t)tracker_target.target_position_L;
        break;
    }
    case frame_command::GET_MOTORS_MOVING:

    {
        uint8_t tracker_motors_moving_flags = TrackingController::get_motors_moving();
        response.command = frame_command::OK;
        response.data = (uint32_t)tracker_motors_moving_flags;
        break;
    }
    case frame_command::GET_IS_HOMING:
    {
        response.command = frame_command::ERROR;
        response.data = (uint32_t)uart_errors::NOT_IMPLEMENTED;
        break;
    }
    case frame_command::GET_READING_1:
    {
        response.command = frame_command::ERROR;
        response.data = (uint32_t)uart_errors::NOT_IMPLEMENTED;
        break;
    }
    case frame_command::GET_READING_2:
    {
        response.command = frame_command::ERROR;
        response.data = (uint32_t)uart_errors::NOT_IMPLEMENTED;
        break;
    }
    case frame_command::GET_READING_3:
    {
        response.command = frame_command::ERROR;
        response.data = (uint32_t)uart_errors::NOT_IMPLEMENTED;
        break;
    }
    case frame_command::GET_READING_4:
    {
        response.command = frame_command::ERROR;
        response.data = (uint32_t)uart_errors::NOT_IMPLEMENTED;
        break;
    }

    default:
        response.command = frame_command::ERROR;
        response.data = (uint32_t)uart_errors::UNKNOWN;
        break;
    }

    return response;
}

bool SerialService::get_homing_flag()
{
    return atomic_load(&start_homing);
}

void SerialService::reset_homing_flag()
{
    atomic_store(&start_homing, false);
}