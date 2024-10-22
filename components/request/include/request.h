#ifndef REQUEST_H
#define REQUEST_H

#pragma once

#include "esp_http_client.h"

#define HTTP_MAJOR 0
#define HTTP_MINOR 1
#define HTTP_PATCH 0

/**/
void print_http_request_version();

/* Init http resource */
esp_err_t init_http();

/* */
uint16_t http_client_get_request(const char *host, const char *path, const char *auth, char *response, uint16_t len);

/* */
uint16_t http_client_post_request(const char *host, const char *path, const char *auth, const char *post_data, char *response, uint16_t len);

/* */
uint16_t http_client_put_request(const char *host, const char *path, const char *auth, char *response, uint16_t len);

/* */
uint16_t http_client_delete_request(const char *host, const char *path, const char *auth, char *response, uint16_t len);

#endif
