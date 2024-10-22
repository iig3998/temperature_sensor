#ifndef AHT20_H
#define AHT20_H

#pragma once

#define AHT20_MAJOR 0
#define AHT20_MINOR 1
#define AHT20_PATCH 0

#define AHT20_ADDRESS             0x38
#define AHT20_INITIALIZATION      0xBE
#define AHT20_TRIGGER_MEASUREMENT 0xAC
#define AHT20_DATA_0              0x33
#define AHT20_DATA_1              0x00
#define AHT20_RESET               0xBA

void print_status_aht20(uint8_t status);

esp_err_t reset_aht20();

esp_err_t init_aht20();

esp_err_t trigger_measurement_aht20();

esp_err_t read_measurement_aht20(uint8_t *data, uint8_t size_data, uint8_t *status);

float get_temperature_aht20(uint8_t *data);

float get_humidity_aht20(uint8_t *data);

#endif
