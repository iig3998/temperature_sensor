#ifndef BME280_H
#define BME280_H

#pragma once

#include <stdint.h>

typedef struct  {
  uint16_t T1; 
  int16_t T2;
  int16_t T3;
  uint16_t P1;
  int16_t P2;
  int16_t P3;
  int16_t P4;
  int16_t P5;
  int16_t P6;
  int16_t P7;
  int16_t P8;
  int16_t P9;
  /* TODO: add humidity coefficent */
} calibration_param_t;

void set_configuration_bme280(uint8_t t_sb, uint8_t filter);

void set_oversampling_mode_bme280(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p);

uint8_t get_configuration_bme280();

uint8_t get_oversampling_mode_bme280();

void reset_bme280();

uint8_t read_id_sensor_bme280();

uint8_t get_status_bme280();

double get_compensate_temperature_double_bme280(int32_t adc_T, calibration_param_t *dig);

int32_t get_compensate_temperature_integer_bme280(int32_t adc_T, calibration_param_t *dig);

double get_compensate_pressure_double_bme280(int32_t adc_P, calibration_param_t *dig);

int64_t get_compensate_pressure_integer_bme280(int32_t adc_P, calibration_param_t *dig);

void get_calibration_temperature_parameter_bme280(calibration_param_t *dig);

void get_calibration_pressure_parameter_bme280(calibration_param_t *dig);

int32_t get_temperature_bme280();

int32_t get_pressure_bme280();

#endif