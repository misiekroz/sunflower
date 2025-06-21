/*
    The file consts of definitions that alter code behaviour.
    Relate to descriptions below for further information
*/

/*
 ***
 ***  LOGGING  ***
 ***
 */

/*** general ***/

/*
Sets the global logging verbosity. Available options:
    ESP_LOG_NONE - No log output
    ESP_LOG_ERROR - Error messages
    ESP_LOG_WARN - Warnings and above
    ESP_LOG_INFO - Informational messages and above
    ESP_LOG_DEBUG - Debug messages and above
    ESP_LOG_VERBOSE - All log messages, including verbose ones


*/
// frequency of verbose logs (current position, readings and such)
#ifndef LOG_VERBOSE_FREQUENCY
#define LOG_VERBOSE_FREQUENCY 0.2
#endif

#ifndef LOG_SD_DLY_MS
#define LOG_SD_DLY_MS 60000 // 1 minute
#endif

/*** tags ***/

// main program loop
#ifndef LOG_TAG_MAIN
#define LOG_TAG_MAIN "MAIN"
#endif

// init
#ifndef LOG_TAG_INIT
#define LOG_TAG_INIT "INIT"
#endif

// homing
#ifndef LOG_TAG_HOMING
#define LOG_TAG_HOMING "HOMING"
#endif

// safety related events
#ifndef LOG_TAG_SAFETY
#define LOG_TAG_SAFETY "SAFETY"
#endif

// errors during normal operation
#ifndef LOG_TAG_ERROR
#define LOG_TAG_ERROR "ERROR"
#endif

// events related to motors
#ifndef LOG_TAG_MOTORS
#define LOG_TAG_MOTORS "MOTORS"
#endif

// some runtime logs such as current position, readings and such
#ifndef LOG_TAG_RUNTIME
#define LOG_TAG_RUNTIME "RUNTIME"
#endif

/*
 ***
 ***  MONITORING  ***
 ***
 */

/*** physical output ***/

#ifndef OUTPUT_STATUS_LED
#define OUTPUT_STATUS_LED
// pin which has WS2812 status LED connected NOTE: pin number is temporary
#define OUTPUT_STATUS_LED_PIN 38

/*
*** A little note on Tracker States ***
IDLE - tracker is doing nothing, no expected movement
TRACKING - in some way, the tracker will move, which is not a homing sequenmce
HOMING - long range movement to the HOME position
STOP - sasfety STOP button has been pressed, or any other input signalled the device to STOP
        can be resumed only via user interaction.
WARNING - a state in which the tracker will continue working, but will undertake a
        unpredictable action (start to home, move to night pos, reset something etc.).
        Warning should be held for a short periods to notify surroundings about an issue.
        Another type of warning could be MAX TRAVEL reached during positioning move,
        or eg. SD card connection failed.
        NOTE: warning will lock all other device tasks for its duration.
        NOTE 2: any task waiting for mutex release, should abandon and retry after time
            if WARNING state is being held
ERROR - safety related issue that would result in charm for the tracker ot its surroundings.
        Requires physical device restart. State that cannot be resumed
*/
#ifndef TRACKER_STATES
#define TRACKER_STATES
typedef enum tracker_states
{
    IDLE,
    TRACKING,
    HOMING,
    NIGHT,
    STOP,
    WARNING,
    ERROR,
    UNKNOWN,
} tracker_states;
#endif

#define NUM_STATUS_PATTERNS 8

#ifndef STATUS_PATTERNS_HSV_DEF
#define STATUS_PATTERNS_HSV_DEF
// in HSV (easier to modify the brightness)
extern const float STATUS_PATTERNS_HSV[NUM_STATUS_PATTERNS][4];
#endif

/**
 * @brief Defines timeout of xSemaphoreTake in set_state_helper
 *
 */
#ifndef TRACKER_STATE_TIMEOUT_MS
#define TRACKER_STATE_TIMEOUT_MS 100
#endif

/**
 * @brief Defines lock time of WARNING state
 *
 */
#ifndef WARNING_LOCK_TIME_MS
#define WARNING_LOCK_TIME_MS 5000
#endif

#endif

/*
 ***
 ***  SAFETY  ***
 ***
 */

/*** physical input ***/
// input of physical E-STOP signal
#ifndef INPUT_ESTOP_SIGNAL
#define INPUT_ESTOP_SIGNAL GPIO_NUM_6
#endif

// input of safety reset signal
#ifndef INPUT_SAFETY_RESET_SIGNAL
#define INPUT_SAFETY_RESET_SIGNAL GPIO_NUM_7
#endif

#ifndef SAFETY_LOOP_DELAY_MS
#define SAFETY_LOOP_DELAY_MS 50
#endif

#ifndef SAFETY_WAIT_LOOP_DELAY
#define SAFETY_WAIT_LOOP_DELAY 50
#endif

/*
 ***
 ***  MOTORS  ***
 ***
 */

// Motor R
#ifndef MOTOR_R_CW_PWM
#define MOTOR_R_CW_PWM GPIO_NUM_21
#endif
#ifndef MOTOR_R_CCW_PWM
#define MOTOR_R_CCW_PWM GPIO_NUM_47
#endif
#ifndef MOTOR_R_ENC
#define MOTOR_R_ENC GPIO_NUM_39
#endif
#ifndef MOTOR_R_MAX_TRAVEL
#define MOTOR_R_MAX_TRAVEL 900
#endif
#ifndef MOTOR_R_CW_LEDC_CHANNEL
#define MOTOR_R_CW_LEDC_CHANNEL LEDC_CHANNEL_0
#endif
#ifndef MOTOR_R_CCW_LEDC_CHANNEL
#define MOTOR_R_CCW_LEDC_CHANNEL LEDC_CHANNEL_1
#endif

// Motor L
#ifndef MOTOR_L_CW_PWM
#define MOTOR_L_CW_PWM GPIO_NUM_48
#endif
#ifndef MOTOR_L_CCW_PWM
#define MOTOR_L_CCW_PWM GPIO_NUM_45
#endif
#ifndef MOTOR_L_ENC
#define MOTOR_L_ENC GPIO_NUM_40
#endif
#ifndef MOTOR_L_MAX_TRAVEL
#define MOTOR_L_MAX_TRAVEL 900
#endif
#ifndef MOTOR_L_CW_LEDC_CHANNEL
#define MOTOR_L_CW_LEDC_CHANNEL LEDC_CHANNEL_2
#endif
#ifndef MOTOR_L_CCW_LEDC_CHANNEL
#define MOTOR_L_CCW_LEDC_CHANNEL LEDC_CHANNEL_3
#endif

// PWM output config (LEDC)
#ifndef LEDC_TIMER
#define LEDC_TIMER LEDC_TIMER_0
#endif
#ifndef LEDC_MODE
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#endif
#ifndef LEDC_DUTY_RES
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#endif
#ifndef LEDC_FREQUENCY
#define LEDC_FREQUENCY 500
#endif

// PI's config
// NOTE: exact same drivetrain is used for R and L motors, so same PI values are assumed
#ifndef MOTOR_PI_Kp
#define MOTOR_PI_Kp 50
#endif
#ifndef MOTOR_PI_Ki
#define MOTOR_PI_Ki 1
#endif

/*
 ***
 ***  SERIAL  ***
 ***
 */
#ifndef UART_BAUD
#define UART_BAUD 115200
#endif
#ifndef UART_DATA_BITS
#define UART_DATA_BITS UART_DATA_8_BITS
#endif
#ifndef UART_PAIRITY
#define UART_PAIRITY UART_PARITY_DISABLE
#endif
#ifndef UART_STOP_BITS
#define UART_STOP_BITS UART_STOP_BITS_1
#endif
#ifndef UART_FLOWCTRL
#define UART_FLOWCTRL UART_HW_FLOWCTRL_DISABLE
#endif
#ifndef UART_CLOCK
#define UART_CLOCK UART_SCLK_DEFAULT
#endif

#ifndef UART_NUM
#define UART_NUM UART_NUM_1
#endif
#ifndef UART_TX_PIN
#define UART_TX_PIN 14
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN 13
#endif
#ifndef UART_BUF_SIZE
#define UART_BUF_SIZE 1024
#endif

/**
 ***
 *** SDCard
 ***
 */

#ifndef SD_MOUNT_POINT
#define SD_MOUNT_POINT "/sdcard"
#endif

// TODO: correct pin assignments

#ifndef SD_SPI_MISO
#define SD_SPI_MISO GPIO_NUM_36
#endif

#ifndef SD_SPI_MOSI
#define SD_SPI_MOSI GPIO_NUM_35
#endif

#ifndef SD_SPI_CLK
#define SD_SPI_CLK GPIO_NUM_37
#endif

#ifndef SD_SPI_CS
#define SD_SPI_CS GPIO_NUM_41
#endif

/*
 ***
 ***  ADC input  ***
 ***
 */
#define ADC_UNIT_USED ADC_UNIT_2
#define ADC_WIDTH ADC_BITWIDTH_DEFAULT
#define ADC_ATTEN ADC_ATTEN_DB_11

#ifndef ADC_GPIO_CONF
#define ADC_GPIO_CONF
#define ADC_FOTO_TL ADC_CHANNEL_4
#define ADC_FOTO_TR ADC_CHANNEL_5
#define ADC_FOTO_BL ADC_CHANNEL_6
#define ADC_FOTO_BR ADC_CHANNEL_7
#define ADC_FOTO_NEUTRAL ADC_CHANNEL_1
#endif

#ifndef ADC_NUM_CHANNELS
#define ADC_NUM_CHANNELS 5
#endif

#ifndef ADC_FREQUENCY
#define ADC_FREQUENCY 10
#endif

#ifndef ADC_AVERAGE_SAMPLES
#define ADC_AVERAGE_SAMPLES 4
#endif

// I2C humiture sensor SHT3x
#ifndef I2C_MASTER_NUM
#define I2C_MASTER_NUM I2C_NUM_0
#endif

#ifndef I2C_MASTER_SDA_IO
#define I2C_MASTER_SDA_IO GPIO_NUM_5
#endif

#ifndef I2C_MASTER_SCL_IO
#define I2C_MASTER_SCL_IO GPIO_NUM_4
#endif

#ifndef I2C_MASTER_FREQ_HZ
#define I2C_MASTER_FREQ_HZ 10000
#endif

#ifndef SHT3X_I2C_ADDR_GND
#define SHT3X_I2C_ADDR_GND 0x44
#endif

/*
 ***
 ***  Tracking  ***
 ***
 */

#ifndef TRACKING_NIGHT_TRSH
#define TRACKING_NIGHT_TRSH 0.5
#endif

#ifndef TRACKING_DEADZONE_VALUE
#define TRACKING_DEADZONE_VALUE 0.05
#endif

// in pulses
#ifndef TRACKING_STEP
#define TRACKING_STEP 75
#endif

#ifndef TRACKING_STEP_REDUCED
#define TRACKING_STEP_REDUCED 25
#endif