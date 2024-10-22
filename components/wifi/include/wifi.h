#ifndef WIFI_H
#define WIFI_H

#pragma once

#define WIFI_MAJOR 0
#define WIFI_MINOR 1
#define WIFI_PATCH 0

/**/
void print_wifi_version();

/**/
esp_err_t wifi_init_sta();

/**/
void print_wifi_network();

/**/
void start_scan_networks();

/**/
void check_wifi_connection();

/**/
bool get_wifi_status_connection();

/**/
esp_err_t wifi_stop_sta();

#endif