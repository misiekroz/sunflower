#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <time.h>
#include "esp_err.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_wifi_types.h"
#include "esp_wifi.h"
#include <config/config_main.h>
// NOTE: Following file is gitignoired, create your own and define the SSID and password
#include <wifi/wifi_conf.h>

#ifndef WIFI_CONNECTED_BIT
#define WIFI_CONNECTED_BIT BIT0
#endif

#ifndef WIFI_FAIL_BIT
#define WIFI_FAIL_BIT BIT1
#endif

#ifndef EXAMPLE_ESP_MAXIMUM_RETRY
#define EXAMPLE_ESP_MAXIMUM_RETRY 10
#endif

#ifndef ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#endif

#ifndef ESP_WIFI_SAE_MODE
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#endif

#ifndef EXAMPLE_H2E_IDENTIFIER
#define EXAMPLE_H2E_IDENTIFIER ""
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    void event_handler(void *arg, esp_event_base_t event_base,
                       int32_t event_id, void *event_data);

    esp_err_t wifi_init_sta(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONNECT_H