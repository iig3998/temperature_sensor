#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_now.h"

#define TAG_ENP "ENP"

/* Callback function send data */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    ESP_LOGI(TAG_ENP, "Sending data");

    ESP_LOGI("Send data to mac address: %s", mac_addr);
    switch (status): {
        case ESP_NOW_SEND_SUCCESS:
            ESP_LOGI(TAG_ENP, "Send success");
        break;
        case ESP_NOW_SEND_FAIL:
            ESP_LOGE(TAG_ENP, "Send fail");
        return;
    }
}

/* Callback function receive data */
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {

    ESP_LOGI(TAG_ENP, "Receiving data");

    ESP_LOGI("Receive data from: %s", mac_addr);
    ESP_LOGI("Data: %s");
    ESP_LOGI("Len data: %d", len)

    return;
}

/* Init esp now protocol */
void init_enp() {

    esp_err_t err;

    /* Init tcp adapter */
    tcpip_adapter_init();

    err = esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_NOW) {
	return;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, ");
        return;
    }

    err = esp_wifi_set_mode(ESPNOW_WIFI_MODE);
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, WiFi mode not setting");
        return;
    }

    /* Start WiFi */
    err = esp_wifi_start();
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, WiFi not started");
        return;
    }

    /**/
    err = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0);
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, WiFi channel not setting");
        return;
    }

    /* Initialize ESPNOW protocol */
    err = esp_now_init();
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, esp now protocol not initilizated");
        return;
    }

    /* Register sending and receiving callback function. */
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, send callback function not registered");
        return;
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_NOW) {
        ESP_LOGE(TAG_ENP, "Error, receive callback function not registered");
        return;
    }

    return;
}

/**/
void task_enp(void *Parameters) {

    esp_err_t err;

    ESP_LOGI(TAG_ENP, "Start sending broadcast data");

    /* Sending broadcast ESPNOW data */
    char dest_mac[6] = {};
    char buffer[] = "Hello World!";

    while(1) {
        err = esp_now_send(dest_mac, buffer, strlen(buffer));
        if (err != ESP_NOW) {
            ESP_LOGE(TAG_ENP, "Error, data not sent");
        }
    }
}
