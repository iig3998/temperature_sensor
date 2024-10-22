#ifndef SHT21_H
#define SHT21_H

#pragma once

#define SHT21_MAJOR 0
#define SHT21_MINOR 1
#define SHT21_PATCH 0

void print_sht21_version();

esp_err_t read_firmware_revision_sht21();

esp_err_t read_id_version_sht21();

esp_err_t soft_reset_sht21();

esp_err_t read_user_register_sht21();

esp_err_t set_user_register_sht21(uint8_t resolution, bool otp_reload, bool onchip_heater);

esp_err_t get_temperature_sht21(uint16_t *temperature);

esp_err_t get_humidity_sht21(uint16_t *humidity);

float get_temperature_float_sht21(uint16_t temperature);

float get_humidity_float_sht21(uint16_t humidity);

#endif
