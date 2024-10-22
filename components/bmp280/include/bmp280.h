#ifndef BMP280_H
#define BMP280_H

#pragma once

#define BMP280_MAJOR 0
#define BMP280_MINOR 0
#define BMP280_PATCH 1

/* BMP280 registers */
#define ADDRESS_BMP280       0x76

/* BMP280 commands register */
#define CONFIGURE_REG_BMP280 0xF5
#define CTRL_MEAS_REG_BMP280 0XF4
#define STATUS_REG_BMP280    0xF3
#define RESET_REG_BMP280     0xE0
#define ID_REG_BMP280        0xD0

#define PRESSURE_H           0xF7 
#define PRESSURE_M           0xF8
#define PRESSURE_L           0xF9
#define TEMPERATURE_H        0xFA
#define TEMPERATURE_M        0xFB
#define TEMPERATURE_L        0xFC

#define T1_ADDRESS           0X88
#define T2_ADDRESS           0X8A
#define T3_ADDRESS           0X8C

#define P1_ADDRESS           0X8E
#define P2_ADDRESS           0X90
#define P3_ADDRESS           0X92
#define P4_ADDRESS           0X94
#define P5_ADDRESS           0X96
#define P6_ADDRESS           0X98
#define P7_ADDRESS           0X9A
#define P8_ADDRESS           0X9C
#define P9_ADDRESS           0X9E

#define H1_ADDRESS           0xA1
#define H2_ADDRESS           0xE1
#define H3_ADDRESS           0xE2
#define H4_ADDRESS           0xE3
#define H5_ADDRESS           0xE4
#define H6_ADDRESS           0xE5
#define H7_ADDRESS           0xE6
#define H8_ADDRESS           0xE7

/* Mode functionality */
#define SLEEP_MODE           0x00
#define FORCE_MODE           0x01
#define NORMAL_MODE          0x03

/* Standby time */
#define STANDBY_MS_0_5  0
#define STANDBY_MS_62_5 1
#define STANDBY_MS_125  2
#define STANDBY_MS_250  3
#define STANDBY_MS_500  4
#define STANDBY_MS_1000 5
#define STANDBY_MS_10   6
#define STANDBY_MS_20   7

/* Filter coefficent */
#define IIR_FILTER_OFF  1
#define IIR_FILTER_2    2
#define IIR_FILTER_4    5
#define IIR_FILTER_8    11
#define IIR_FILTER_16   22

/* Oversampling temperature */
#define TEMPERATURE_OVERSAMPLING_SKIPPED  0
#define TEMPERATURE_OVERSAMPLING_1        1
#define TEMPERATURE_OVERSAMPLING_2        2
#define TEMPERATURE_OVERSAMPLING_4        3 
#define TEMPERATURE_OVERSAMPLING_8        5
#define TEMPERATURE_OVERSAMPLING_16       6

/* Oversampling pressure */
#define PRESSURE_OVERSAMPLING_SKIPPED     0
#define PRESSURE_OVERSAMPLING_1           1
#define PRESSURE_OVERSAMPLING_2           2
#define PRESSURE_OVERSAMPLING_4           3
#define PRESSURE_OVERSAMPLING_8           5
#define PRESSURE_OVERSAMPLING_16          6

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
} calibration_param_t;

void print_bmp280_version();

void set_configuration_bmp280(uint8_t t_sb, uint8_t filter);

void set_oversampling_mode_bmp280(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p);

uint8_t get_configuration_bmp280();

uint8_t get_oversampling_mode_bmp280();

void reset_bmp280();

uint8_t read_id_sensor_bmp280();

uint8_t get_status_bmp280();

double get_compensate_temperature_double_bmp280(int32_t adc_T, calibration_param_t *dig);

int32_t get_compensate_temperature_integer_bmp280(int32_t adc_T, calibration_param_t *dig);

double get_compensate_pressure_double_bmp280(int32_t adc_P, calibration_param_t *dig);

int64_t get_compensate_pressure_integer_bmp280(int32_t adc_P, calibration_param_t *dig);

void get_calibration_temperature_parameter_bmp280(calibration_param_t *dig);

void get_calibration_pressure_parameter_bmp280(calibration_param_t *dig);

int32_t get_temperature_bmp280();

int32_t get_pressure_bmp280();

#endif