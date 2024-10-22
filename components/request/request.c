#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_http_client.h"

#include "request.h"

#define TAG_HTTP "HTTP"
#define HTTP_TIMEOUT 3000

/* HTTP event handler */
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG_HTTP, "Generic HTTP error");
        break;

        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG_HTTP, "Open HTTP connection");
        break;

        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG_HTTP, "All the header has been sent");
        break;

        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG_HTTP, "Receive header from the server");
        break;

        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG_HTTP, "Receive data from the server");
            if (!esp_http_client_is_chunked_response(evt->client))
                ESP_LOGD(TAG_HTTP, "%.*s", evt->data_len, (char*)evt->data);
        break;

        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG_HTTP, "Finish HTTP session");
        break;

        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG_HTTP, "Close HTTP connection");
        break;
    }

    return ESP_OK;
}

/* Print http version library*/
void print_http_request_version() {

    ESP_LOGI(TAG_HTTP, "Version http library: %u.%u.%u", HTTP_MAJOR, HTTP_MINOR, HTTP_PATCH);

    return;
}

/* Init http resource */
esp_err_t init_http() {

    return ESP_OK;
}

/* HTTP get request */
uint16_t http_client_get_request(const char *host, const char *path, const char *auth, char *response, uint16_t len) {

    esp_err_t err;
    int64_t content_length = 0;
    uint16_t status_code = 500;

    esp_http_client_config_t config = {
        .host = host,
        .path = path,
        .is_async = false,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .event_handler = http_event_handler,
        .disable_auto_redirect = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (auth != NULL)
        esp_http_client_set_header(client, "X-AIO-Key", auth);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "Error, failed to open HTTP connection");
        return status_code;
    }

    /* Run HTTP request */
    content_length = esp_http_client_fetch_headers(client);
    if (content_length >= 0 && content_length < len) {
        status_code = esp_http_client_get_status_code(client);

        /* Clear dynamic array */
        if (esp_http_client_read_response(client, response, len) >= 0) {
            ESP_LOGI(TAG_HTTP, "Status code: %u", status_code);
            ESP_LOGI(TAG_HTTP, "Response: %s", response);
        } else {
            ESP_LOGE(TAG_HTTP, "HTTP request failed");
        }
    } else {
        ESP_LOGW(TAG_HTTP, "Error, buffer is not big enough");
    }

    /* Close HTTP connection */
    esp_http_client_close(client);

    /* Cleanup HTTP connection */
    esp_http_client_cleanup(client);

    return status_code;
}

/* HTTP post request */
uint16_t http_client_post_request(const char *host, const char *path, const char *auth, const char *post_data, char *response, uint16_t len) {

    esp_err_t err;
    int64_t content_length = 0;
    uint16_t status_code = 500;

    esp_http_client_config_t config = {
        .host = host,
        .path = path,
        .is_async = false,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .event_handler = http_event_handler,
        .disable_auto_redirect = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (auth != NULL)
        esp_http_client_set_header(client, "X-AIO-Key", auth);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    err = esp_http_client_set_method(client, HTTP_METHOD_POST);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "Error to set HTTP POST Method");
        return status_code;
    }
    
    err = esp_http_client_open(client, strlen(post_data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "Error, failed to open HTTP connection");
        return status_code;
    }

    /* Set payload for POST request */
    if (esp_http_client_write(client, post_data, strlen(post_data)) < 0) {
        ESP_LOGW(TAG_HTTP, "Warning, nothing payload to send");
    }

    /* Run HTTP request */
    content_length = esp_http_client_fetch_headers(client);
    if (content_length >= 0) {
        status_code = esp_http_client_get_status_code(client);

        /* Clear dynamic array */
        if (esp_http_client_read_response(client, response, len) >= 0) {
            ESP_LOGI(TAG_HTTP, "Status code: %u", status_code);
            ESP_LOGI(TAG_HTTP, "Response: %s", response);
        } else {
            ESP_LOGE(TAG_HTTP, "Error, HTTP request failed");
        }
    } else {
        ESP_LOGW(TAG_HTTP, "Error, buffer is not big enough");
    }

    /* Close HTTP connection */
    esp_http_client_close(client);

    /* Cleanup HTTP connection */
    esp_http_client_cleanup(client);

    return status_code;
}

/* HTTP put request */
uint16_t http_client_put_request(const char *host, const char *path, const char *auth, char *response, uint16_t len) {

    esp_err_t err;
    int64_t content_length = 0;
    uint16_t status_code = 500;

    esp_http_client_config_t config = {
        .host = host,
        .path = path,
        .is_async = false,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .event_handler = http_event_handler,
        .disable_auto_redirect = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (auth != NULL)
        esp_http_client_set_header(client, "X-AIO-Key", auth);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_method(client, HTTP_METHOD_PUT);
    
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "Error, failed to open HTTP connection");
        return status_code;
    }

    /* Run HTTP request */
    content_length = esp_http_client_fetch_headers(client);
    if (content_length >= 0) {
        status_code = esp_http_client_get_status_code(client);

        /* Clear dynamic array */
        if (esp_http_client_read_response(client, response, len) >= 0) {
            ESP_LOGI(TAG_HTTP, "Status code: %u", status_code);
            ESP_LOGI(TAG_HTTP, "Response: %s", response);
        } else {
            ESP_LOGE(TAG_HTTP, "HTTP request failed");
        }
    } else {
        ESP_LOGW(TAG_HTTP, "Error, buffer is not big enough");
    }

    /* Close HTTP connection */
    esp_http_client_close(client);

    /* Cleanup HTTP connection */
    esp_http_client_cleanup(client);

    return status_code;
}

/* HTTP delete request */
uint16_t http_client_delete_request(const char *host, const char *path, const char *auth, char *response, uint16_t len) {

    esp_err_t err;
    int64_t content_length = 0;
    uint16_t status_code = 500;

    esp_http_client_config_t config = {
        .host = host,
        .path = path,
        .is_async = false,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .event_handler = http_event_handler,
        .disable_auto_redirect = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (auth != NULL)
        esp_http_client_set_header(client, "X-AIO-Key", auth);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_method(client, HTTP_METHOD_DELETE);

    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "Error, failed to open HTTP connection");
        return status_code;
    }

    /* Run HTTP request */
    content_length = esp_http_client_fetch_headers(client);
    if (content_length >= 0 && content_length < len) {
        status_code = esp_http_client_get_status_code(client);

        /* Clear dynamic array */
        if (esp_http_client_read_response(client, response, len) >= 0) {
            ESP_LOGI(TAG_HTTP, "Status code: %u", status_code);
            ESP_LOGI(TAG_HTTP, "Response: %s", response);
        } else {
            ESP_LOGE(TAG_HTTP, "HTTP request failed");
        }
    } else {
        ESP_LOGW(TAG_HTTP, "Error, buffer is not big enough");
    }

    /* Close HTTP connection */
    esp_http_client_close(client);

    /* Cleanup HTTP connection */
    esp_http_client_cleanup(client);

    return status_code;
}
