#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_now.h"

#include "driver/adc.h"
#include "driver/i2c.h"

#include "nvs_flash.h"

#include "chip_info.h"
#include "request.h"
#include "wifi.h"

#include "bmp280.h"
#include "sht21.h"
#include "aht10.h"
#include "aht20.h"

/**/
#define KEY_CLOUD                   "key_cloud"
#define URL                         "url"
#define PATH                        "/api/v2/domotichouse/feeds/"
#define OUTPUT_REQUEST_BUFFER_SIZE  1024
#define PAYLOAD_SIZE                512
#define CLK_FREQ_I2C                100000

#define TAG_MAIN "MAIN"

/* Aht10 task */
void aht10_task(void *param) {

    char payload[PAYLOAD_SIZE] = {'\0'};
    char output[256] = {'\0'};
    uint8_t data[6] = {0};
    uint8_t status = 0;
    uint16_t dbv = 0;
    esp_err_t err = ESP_FAIL;
    i2c_config_t conf;

    /* Configure I2C */
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.clk_stretch_tick = CLK_FREQ_I2C;

    /* Install driver for i2c master mode and I2C_NUM_0 channel */
    err = i2c_driver_install(I2C_NUM_0, conf.mode);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Error to install i2c driver, exit");
        return;
    }

    /* Configure i2c driver */
    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Error to configure i2c driver, exit");
        return;
    }

    /* Reset AHT10 sensor */
    reset_aht10();
    vTaskDelay(pdMS_TO_TICKS(80));

    /* Init AHT10 sensor */
    init_aht10();
    vTaskDelay(pdMS_TO_TICKS(100));

    while(1) {

        /* Trigger measurement aht10 */
        trigger_measurement_aht10();

        /* Waits 100 ms after trigger measurement */
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Read temperature and humidity value */
        err = read_measurement_aht10(data, sizeof(data), &status);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, measure not available");
        } else {
            ESP_LOGI(TAG_MAIN, "Status: 0x%2x", status);

            /* Read battery value */
            err = adc_read(&dbv);
            if (err != ESP_OK) {
                dbv = -1;
                ESP_LOGE(TAG_MAIN, "Error, battery value not available");
            }

            ESP_LOGI(TAG_MAIN, "Temperature: %f", get_temperature_aht10(data));
            ESP_LOGI(TAG_MAIN, "Humidity: %f", get_humidity_aht10(data));

            memset(payload, '\0', sizeof(payload));
            memset(output, '\0', sizeof(output));
            snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f, \"battery\": %.1f}", get_temperature_aht10(data), get_humidity_aht10(data), (float)((float)(dbv / 1023) * 3.3));

            ESP_LOGI(TAG_MAIN, "Payload: %s", payload);

            //http_client_post_request(URL, (const char*)"/api/v1/temperature_sensor/0/data", NULL, payload, output, sizeof(output));
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    return;
}

/* Main program */
void app_main() {

    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer;
    esp_chip_info_t chip_info;
    adc_config_t adc_config;
    static uint8_t example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    /* Init flash */
    nvs_flash_init();

    /* Read and print chip information */
    esp_chip_info(&chip_info);
    esp_print_chip_info(&chip_info);

    /* Init tcp adapter */
    tcpip_adapter_init();

    /* Initialize ESPNOW and register sending and receiving callback function. */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Error, ESP NOW not initializeted");
        return;
    }

    /* Configure peer */
    peer.channel = 0;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;

    memcpy(peer.peer_addr, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, peer address not configurated");
        return;
    }

    /* Init ADC */
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 8;
    err = adc_init(&adc_config);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, ADC not initialized");
        return;        
    }

    /* Init wifi station */
    err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, WiFi not initialized");
        return;
    }

    /* Init http resource */
    err = init_http();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, HTTP service not initialized");
        return;
    } 

    //ESP_LOGI(TAG_MAIN, "Deep sleep mode for 5 seconds");

    /* Start task bmp280 */
    //xTaskCreate(&bmp280_task, "bmp280", 4096, NULL, 1, NULL);

    /* Start task sht21 */
    //xTaskCreate(&sht21_task, "sht21", 4096, NULL, 1, NULL);

    /* Check wifi connection */
    check_wifi_connection();

    /* Start task aht10 */
    xTaskCreate(&aht10_task, "aht10", 4096, NULL, 1, NULL);

    /**/
    //wifi_stop_sta();

    //i2c_driver_delete();

    //esp_deep_sleep_set_rf_option(2);

    /* Enter in deep sleep mode for 5 seconds */
    //esp_deep_sleep(5 * 1000000);

}
