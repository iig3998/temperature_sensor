#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "enp.h"
#include "sensor.h"

#define WIFI_CHANNEL 1

#define TAG_ENP "ENP"

#ifdef REGISTER_SEND_CB
/* Callback function send data */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    ESP_LOGI(TAG_ENP, "Send data to mac address: %x:%x:%x:%x:%x:%x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    switch (status) {
        case ESP_NOW_SEND_SUCCESS:
            ESP_LOGI(TAG_ENP, "Send success");
        break;
        case ESP_NOW_SEND_FAIL:
            ESP_LOGE(TAG_ENP, "Send fail");
        return;
    }

    return;
}
#endif

/* Init esp now protocol */
static esp_err_t init_espnow_protocol() {

    esp_err_t err = ESP_FAIL;

    /* Init tcp adapter */
    tcpip_adapter_init();

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, event loop default not created");
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, WiFi not init");
	    return err;
    }

    /* Set WiFi storage */
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, WiFi storage not setting");
        return err;
    }

    /* Set WiFi mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, WiFi mode not setting");
        return err;
    }

    /* Start WiFi module */
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, WiFi not started");
        return err;
    }

    /* Set WiFi channel */
    err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_ABOVE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, WiFi channel not setting");
        return err;
    }

    /* Initialize ESPNOW protocol */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, esp now protocol not initilizated");
        return err;
    }

    /* Register sending and receiving callback function */
    #ifdef REGISTER_SEND_CB
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, send callback function not registered");
        return err;
    }
    #endif

    return err;
}

/**/
esp_err_t conf_esp_now_protocol(esp_now_peer_info_t *peer) {

    esp_err_t err = ESP_FAIL;
    uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    ESP_LOGI(TAG_ENP, "Start ESPNOW protocol task");

    /* Init ESPNOW protocol */
    err = init_espnow_protocol();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, ESPNOW protcol not init");
        return err;
    }

    /* Configure peer ESPNOW */
    peer->channel = WIFI_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    err = esp_now_add_peer(peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, peer address destination not configurated");
        return err;
    }

    return err;
}

/**/
esp_err_t send_espnow_protocol(esp_now_peer_info_t *peer, float temperature, float humidity) {

    esp_err_t err = ESP_FAIL;
    node_id node;

    /* Build message */
    memset(&node, 0, sizeof(node_id));

    node.id = 0x01;
    node.type = TEMPERATURE_HUMIDITY_SENSOR;
    node.select.ms.temp = temperature;
    node.select.ms.hum = humidity;

    err = esp_now_send(peer->peer_addr, (uint8_t *)&node, sizeof(node));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_ENP, "Error, data not sent");
        return err;
    }

    return err;
}