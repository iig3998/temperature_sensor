#include "esp_log.h"
#include "esp_system.h"
#include "chip_info.h"

#define TAG_CHIP_INFO "CHIP_INFO"

/* Print chip info version */
void print_chip_info_version() {

    ESP_LOGI(TAG_CHIP_INFO, "Chip info library version: %u.%u.%u", CHIP_INFO_MAJOR, CHIP_INFO_MINOR, CHIP_INFO_PATCH);

    return;
}

/* Print chip info */
void esp_print_chip_info(esp_chip_info_t *chip_info) {

    if (chip_info->model == CHIP_ESP8266) {
        ESP_LOGI(TAG_CHIP_INFO, "Model is ESP8266");
    } else {
        ESP_LOGE(TAG_CHIP_INFO, "Error, model not available");
        return;
    }

    ESP_LOGI(TAG_CHIP_INFO, "Number of CPU cores: %d", chip_info->cores);
    ESP_LOGI(TAG_CHIP_INFO, "Revision: %d", chip_info->revision);

    /* Features implemented */
    if (chip_info->features & CHIP_FEATURE_EMB_FLASH) {
        ESP_LOGI(TAG_CHIP_INFO, "Chip has embedded flash memory");
    } else if (chip_info->features & CHIP_FEATURE_WIFI_BGN) {
        ESP_LOGI(TAG_CHIP_INFO, "Chip has 2.4GHz WiFi");
    } else if (chip_info->features & CHIP_FEATURE_BLE) {
        ESP_LOGI(TAG_CHIP_INFO, "Chip has Bluetooth LE");
    } else if (chip_info->features & CHIP_FEATURE_BT) {
        ESP_LOGI(TAG_CHIP_INFO, "Chip has Bluetooth Classic");
    } else {
        ESP_LOGI(TAG_CHIP_INFO, "No features available");
    }

    return;
}