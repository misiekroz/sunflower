#include "sdcard/sd_card_manager.h"

SDCardManager::SDCardManager(gpio_num_t MOSI, gpio_num_t MISO, gpio_num_t CLK, gpio_num_t CS)
{
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 32 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = SD_MOUNT_POINT;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1000,
    };

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG_INIT, "Failed to mount filesystem");
        return;
    }
    ESP_LOGI(LOG_TAG_INIT, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    //// get date for file header ////
    char date_buf[11];
    get_current_date_str(date_buf, 11);
    // parse filename
    snprintf(log_file_path, 64, "%s/Log.csv", SD_MOUNT_POINT);
}

esp_err_t SDCardManager::log_line(int R_pos,
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
                                  int motors_moving)
{

    char date_buf[20],
        log_line[255];

    FILE *pFile;

    // get datetime
    get_current_datetime_str(date_buf, 20);
    // create the log string
    snprintf(log_line, 255, "%s;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i",
             date_buf,
             R_pos,
             L_pos,
             A1,
             A2,
             A3,
             A4,
             A5,
             T0,
             T1,
             T2,
             humidity,
             errors,
             motors_homing,
             motors_moving);

    ESP_LOGI(LOG_TAG_RUNTIME, "Opening file %s", log_file_path);
    pFile = fopen(log_file_path, "a");
    if (pFile != nullptr)
    {
        // write to opened file
        fputs(log_line, pFile);
        fputs("\n", pFile);
        fclose(pFile);
        return ESP_OK;
    }
    // or return an error
    return ESP_FAIL;
}

void SDCardManager::get_current_date_str(char *out_buf, size_t len)
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(out_buf, len, "%F", &timeinfo);
}

void SDCardManager::get_current_datetime_str(char *out_buf, size_t len)
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(out_buf, len, "%F %X", &timeinfo);
}