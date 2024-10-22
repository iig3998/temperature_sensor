#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "wifi.h"

#define TAG_WIFI "WIFI"

#define WIFI_SSID "DomoticHouse"
#define WIFI_PASS "D0m0t1cH0use"

static EventGroupHandle_t s_wifi_event_group;

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

    switch(event_id) {
        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG_WIFI, "Station got IP from connected AP");
            ESP_LOGI(TAG_WIFI, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
            ESP_LOGI(TAG_WIFI, "Got Netmask: %s", ip4addr_ntoa(&event->ip_info.netmask));
            ESP_LOGI(TAG_WIFI, "Got Gateway: %s", ip4addr_ntoa(&event->ip_info.gw));
            xEventGroupSetBits(s_wifi_event_group, BIT(0));
        break;
        case IP_EVENT_STA_LOST_IP:
            ESP_LOGI(TAG_WIFI, "Station lost IP and the IP is reset to 0");
            xEventGroupClearBits(s_wifi_event_group, BIT(0));
        break;
        case IP_EVENT_AP_STAIPASSIGNED:
            ESP_LOGI(TAG_WIFI, "Soft-AP assign an IP to a connected station");
        break;
        case IP_EVENT_GOT_IP6:
            ESP_LOGI(TAG_WIFI, "Station or AP or ethernet interface IPv6 addr is preferred");
        break;
        default:
            ESP_LOGW(TAG_WIFI, "Network event not managed");
        break;
    }

    return;
}

/* Wifi event handler function */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    ESP_LOGI(TAG_WIFI, "Network event handler");

    switch(event_id) {

        case WIFI_EVENT_WIFI_READY:           
            ESP_LOGI(TAG_WIFI, "ESP8266 WiFi ready");
            break;
        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG_WIFI, "ESP8266 finish scanning AP");

            /* Connect to network */
            print_wifi_network();

            break;
        case WIFI_EVENT_STA_START: 
            ESP_LOGI(TAG_WIFI, "ESP8266 station start");

            /* Start scan */
            start_scan_networks();

            break;
        case WIFI_EVENT_STA_STOP:               
            ESP_LOGI(TAG_WIFI, "ESP8266 station stop");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG_WIFI, "ESP8266 station connected to AP");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG_WIFI, "ESP8266 station disconnected from AP");

            /* Start new scan */
            start_scan_networks();
            break;
        case WIFI_EVENT_AP_STACONNECTED:          
            ESP_LOGI(TAG_WIFI, "A station connected to ESP8266 soft-AP");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG_WIFI, "A station disconnected from ESP8266 soft-AP");
            break;
        case WIFI_EVENT_AP_PROBEREQRECVED:
            ESP_LOGI(TAG_WIFI, "Receive probe request packet in soft-AP interface");
            break;
        default:
            ESP_LOGI(TAG_WIFI, "Wifi event not supported");
            break;
    }

    return;
}

/* Print wifi library version */
void print_wifi_version() {

    ESP_LOGI(TAG_WIFI, "Version wifi library: %u.%u.%u", WIFI_MAJOR, WIFI_MINOR, WIFI_PATCH);

    return;
}

/* Find wifi network */
void find_wifi_network(wifi_ap_record_t *ap_list, uint16_t ap_count, const char *network_wifi) {

    if (!ap_list) {
        ESP_LOGE(TAG_WIFI, "Error, memory not allocated");
        return;
    }

    for (uint16_t i = 0; i < ap_count; i++) {
        if (!(strcmp((const char *)ap_list[i].ssid, network_wifi))) {
            ESP_LOGI(TAG_WIFI, "Network %s found. Wait for connect", network_wifi);
            esp_wifi_connect();
            return;
        }
    }

    ESP_LOGW(TAG_WIFI, "Network %s not found ... retry", network_wifi);

    return;
}

/* Start scan */
void start_scan_networks() {

    esp_err_t err;
    wifi_scan_config_t scan_config = {
		.ssid = NULL,
		.bssid = NULL,
		.channel = 0,
		.show_hidden = 1,
		.scan_type = WIFI_SCAN_TYPE_ACTIVE,
		.scan_time.active.min = 120,
		.scan_time.active.max = 250,
	};

    /* Start wifi scan */
    err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, scan wifi not started");
        return;
    }

    return;
}

/* Print wifi netwrok */
void print_wifi_network() {

    esp_err_t err;
    uint16_t ap_count;
    wifi_ap_record_t *ap_list = NULL;

    err = esp_wifi_scan_get_ap_num(&ap_count);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_WIFI, "Error: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG_WIFI, "--------Scan count of AP is %d-------", ap_count);
    if (ap_count == 0) {
        ESP_LOGW(TAG_WIFI, "No AP found. List is empty");
        return;
    }

    ap_list = (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!ap_list) {
        ESP_LOGE(TAG_WIFI, "No memory allocated");
        return;
    }

    err = esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_WIFI, "Error, list AP not available");
        return;
    }

    ESP_LOGI(TAG_WIFI, "==========================================");
	ESP_LOGI(TAG_WIFI, "            SSID             |    RSSI    ");
	ESP_LOGI(TAG_WIFI, "==========================================");
    for (uint16_t i = 0; i < ap_count; i++) {
        ESP_LOGI(TAG_WIFI, "|  %s    |      %d", ap_list[i].ssid, ap_list[i].rssi);
    }

    find_wifi_network(ap_list, ap_count, WIFI_SSID);

    free(ap_list);
}

/* Check wifi connection */
void check_wifi_connection() {

    xEventGroupWaitBits(s_wifi_event_group, BIT(0), pdFALSE, pdTRUE, portMAX_DELAY);

    /* Wait 1 seconds */
    vTaskDelay(pdMS_TO_TICKS(1000));

    return;
}

/* Get Wifi status connection */
bool get_wifi_status_connection() {

    if (xEventGroupGetBits(s_wifi_event_group) == 0x01)
        return true;

    return false;
}

/* Init wifi sta */
esp_err_t wifi_init_sta() {
   
    esp_err_t err = ESP_FAIL;
    wifi_config_t wifi_config = {};

    ESP_LOGI(TAG_WIFI, "WiFi init start");

    esp_event_loop_create_default();

    s_wifi_event_group = xEventGroupCreate();
    if(!s_wifi_event_group) {
        ESP_LOGE(TAG_WIFI, "The s_wifi_event_group was not created because there was insufficient FreeRTOS heap available");
        return err;
    }

    /* Create event handler function */
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL);

    /* Initi wifi station */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    memset(wifi_config.sta.ssid, '\0', sizeof(wifi_config.sta.ssid)/sizeof(wifi_config.sta.ssid[0]));
    memset(wifi_config.sta.password, '\0', sizeof(wifi_config.sta.password)/sizeof(wifi_config.sta.password[0]));

    /* Set ssid and password */
    memcpy(wifi_config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(wifi_config.sta.password, WIFI_PASS, strlen(WIFI_PASS));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /* Set the WiFi operating mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error: %s", esp_err_to_name(err));
        return err;
    } 
    
    /* Set wifi configuration */
    ESP_LOGI(TAG_WIFI, "SSID: %s", wifi_config.sta.ssid);
    ESP_LOGI(TAG_WIFI, "Password: %s", wifi_config.sta.password);

    err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi not configured");
        return err;
    } 

    /* Start wifi */
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi not started");
        return err;
    } 

    return err;
}

/**/
esp_err_t wifi_stop_sta() {

    esp_err_t err = ESP_FAIL;

    /* Stop Wifi */
    err = esp_wifi_stop();
    if (err != ESP_FAIL) {
        ESP_LOGE(TAG_WIFI, "Error, WiFi not stopped");
        return err;
    }

    /* Deinit Wifi and I2C */
    err = esp_wifi_deinit();
    if (err != ESP_FAIL) {
        ESP_LOGE(TAG_WIFI, "Error, WiFi not deinitialization");
        return err;
    }

    return err;
}