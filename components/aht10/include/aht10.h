#ifndef AHT10_H
#define AHT10_H

#pragma once

#include <stdio.h>
#include <stdint.h>

#define AHT10_MAJOR 0
#define AHT10_MINOR 1
#define AHT10_PATCH 0

/**/
void print_wifi_version();

/**/
void print_status_aht10(uint8_t status);

/**/
esp_err_t init_aht10();

/**/
esp_err_t trigger_measurement_aht10();

/**/
esp_err_t read_measurement_aht10(uint8_t *data, uint8_t size_data, uint8_t *status);

/**/
float get_temperature_aht10(uint8_t *data);

/**/
float get_humidity_aht10(uint8_t *data);

/**/
esp_err_t reset_aht10();

#endif
