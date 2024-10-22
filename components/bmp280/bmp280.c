#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmp280.h"

#define TAG_BMP280 "BMP280"

static int32_t t_fine;

/* Print bmp280 version */
void print_bmp280_version() {

    ESP_LOGI(TAG_BMP280, "BMP280 library version: %u.%u.%u", BMP280_MAJOR, BMP280_MINOR, BMP280_MINOR);

    return;
}

/**/
double get_compensate_temperature_double_bmp280(int32_t adc_T, calibration_param_t *dig) {

    double var1, var2, T;

    var1 = ((double)adc_T / 16384.0 - ((double)dig->T1) / 1024.0);
    var1 = var1 * ((double)dig->T2);
    var2 = (((double)adc_T) / 131072.0 - ((double)dig->T1) / 8192.0);
    var2 = (var2 * var2) * ((double)dig->T3);
    t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0;
    
    return T;
}

/**/
int32_t get_compensate_temperature_integer_bmp280(int32_t adc_T, calibration_param_t *dig) {
  
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig->T1 << 1))) * ((int32_t)dig->T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig->T1)) * ((adc_T >> 4) - ((int32_t)dig->T1))) >> 12) * ((int32_t)dig->T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
 
    return T;
}

/**/
double get_compensate_pressure_double_bmp280(int32_t adc_P, calibration_param_t *dig) {

    double var1, var2, p;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig->P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig->P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dig->P4) * 65536.0);
    var1 = (((double)dig->P3) * var1 * var1 / 524288.0 + ((double)dig->P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0)*((double)dig->P1);
    
    if (var1 == 0.0) {
        return 0; /* Avoid exception caused by division by zero */
    }
    
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig->P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig->P8) / 32768.0;
    p = p + (var1 + var2 + ((double)dig->P7)) / 16.0;
    
    return p;
}

/**/
int64_t get_compensate_pressure_integer_bmp280(int32_t adc_P, calibration_param_t *dig) {

    int64_t var1, var2, p;

    var1 = (((int32_t) t_fine) / 2) - (int32_t) 64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) dig->P6);
    var2 = var2 + ((var1 * ((int32_t) dig->P5)) * 2);
    var2 = (var2 / 4) + (((int32_t) dig->P4) * 65536);
    var1 = (((dig->P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8) + ((((int32_t) dig->P2) * var1) / 2)) / 262144;
    var1 = ((((32768 + var1)) * ((int32_t) dig->P1)) / 32768);
    p = (uint32_t)(((int32_t)(1048576 - adc_P) - (var2 / 4096)) * 3125);

    /* Avoid exception caused by division with zero */
    if (var1) {
        /* Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1 */
        if (p < 0x80000000) {
            p = (p << 1) / ((uint32_t) var1);
        } else {
            p = (p / (uint32_t) var1) * 2;
        }

        var1 = (((int32_t) dig->P9) * ((int32_t) (((p / 8) * (p / 8)) / 8192))) / 4096;
        var2 = (((int32_t) (p / 4)) * ((int32_t) dig->P8)) / 8192;
        p = (uint32_t) ((int32_t)p + ((var1 + var2 + dig->P7) / 16));
    } else {
        p = 0;
    }

    return p;
}

/* Get pressure value from sensor */
int32_t get_pressure_bmp280() {

    uint8_t pressure[3] = {0};
    int32_t value = 0;
    esp_err_t err;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, PRESSURE_H, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, pressure, sizeof(pressure), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    value = (int32_t)((((uint32_t)pressure[0]) << 12) | (((uint32_t)pressure[1]) << 4) | (((uint32_t)pressure[2]) >> 4));

    ESP_LOGI(TAG_BMP280, "Pressure value: %d", value);

    return value;
}

/* Get temperature value from sensor */
int32_t get_temperature_bmp280() {

    uint8_t temperature[3] = {0};
    int32_t value = 0;
    esp_err_t err;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, TEMPERATURE_H, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, temperature, sizeof(temperature), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    value = (int32_t)((((uint32_t)temperature[0]) << 12) | (((uint32_t)temperature[1]) << 4) | (((uint32_t)temperature[2]) >> 4));

    ESP_LOGI(TAG_BMP280, "Temperature value: %d", value);

    return value;
}

/* Read ID sensor BMP280 */
uint8_t read_id_sensor_bmp280() {

    uint8_t id_sensor = 0;
    esp_err_t error;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return 0;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, ID_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &id_sensor, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    error = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (error != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, command read id sensor not send");
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_BMP280, "ID: %d", id_sensor);

    return id_sensor;
}

/* Set time sample and coefficent filter */
void set_configuration_bmp280(uint8_t t_sb, uint8_t filter) {

    esp_err_t err;
    i2c_cmd_handle_t cmd;

    switch(t_sb) {
        case 0:
            ESP_LOGI(TAG_BMP280, "Set t standby to 0.5 ms");
            break;
        case 1:
            ESP_LOGI(TAG_BMP280, "Set t standby to 62.5 ms");
            break;
        case 2:
            ESP_LOGI(TAG_BMP280, "Set t standby to 125 ms");
            break;
        case 3:
            ESP_LOGI(TAG_BMP280, "Set t standby to 250 ms");
            break;
        case 4:
            ESP_LOGI(TAG_BMP280, "Set t standby to 500 ms");
            break;
        case 5:
            ESP_LOGI(TAG_BMP280, "Set t standby to 1000 ms");
            break;
        case 6:
            ESP_LOGI(TAG_BMP280, "Set t standby to 10 ms");
            break;
        case 7:
            ESP_LOGI(TAG_BMP280, "Set t standby to 20 ms");
            break;
        default:
            ESP_LOGI(TAG_BMP280, "Value t standby not supported");
        break;
    }

    switch (filter) {
        case 0:
            ESP_LOGI(TAG_BMP280, "Set filter off");
            break;
        case 1:
            ESP_LOGI(TAG_BMP280, "Set coefficent filter 2");
            break;
        case 2:
            ESP_LOGI(TAG_BMP280, "Set coefficent filter 4");
            break;
        case 3:
            ESP_LOGI(TAG_BMP280, "Set coefficent filter 8");
            break;
        default:
            ESP_LOGI(TAG_BMP280, "Set coefficent filter 16");
            filter = 4;
            break;
    }

    ESP_LOGI(TAG_BMP280, "Configuration: %d", (t_sb << 5) | (filter << 2));

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, CONFIGURE_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, ((t_sb << 5) | (filter << 2)), I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, command set configuration not send");
    }
    i2c_cmd_link_delete(cmd);

    return;
}

/* Set oversampling for temperature, pressure and set mode functionality */
void set_oversampling_mode_bmp280(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p) {
    
    esp_err_t err;
    i2c_cmd_handle_t cmd;
    uint8_t conf;

    switch(mode) {
        case 0:
            ESP_LOGI(TAG_BMP280, "Set sleep mode");
        break;
        case 1:
        case 2:
            ESP_LOGI(TAG_BMP280, "Set force mode");
        break;
        case 3:
            ESP_LOGI(TAG_BMP280, "Set normal mode");
        break;
    }

    switch(osrs_t) {
        case 0:
            ESP_LOGI(TAG_BMP280, "Skipped temperature (output set to 0x80000)");
        break;
        case 1:
            ESP_LOGI(TAG_BMP280, "Oversampling temperature x1");
        break;
        case 2:
            ESP_LOGI(TAG_BMP280, "Oversampling temperature x2");
        break;
        case 3:
            ESP_LOGI(TAG_BMP280, "Oversampling temperature x4");
        break;
        case 4:
            ESP_LOGI(TAG_BMP280, "Oversampling temperature x8");
        break;
        case 5:
            ESP_LOGI(TAG_BMP280, "Others oversampling temperature x16");
        break;
    }

    switch(osrs_p) {
        case 0:
            ESP_LOGI(TAG_BMP280, "Skipped pressure (output set to 0x80000)");
        break;
        case 1:
            ESP_LOGI(TAG_BMP280, "Oversampling pressure x1");
        break;
        case 2:
            ESP_LOGI(TAG_BMP280, "Oversampling pressure x2");
        break;
        case 3:
            ESP_LOGI(TAG_BMP280, "Oversampling pressure x4");
        break;
        case 4:
            ESP_LOGI(TAG_BMP280, "Oversampling pressure x8");
        break;
        case 5:
            ESP_LOGI(TAG_BMP280, "Others oversampling pressure x16");
        break;
    }

    conf = (osrs_t & 0x07) << 5 | (osrs_p & 0x07) << 2 | mode;

    ESP_LOGI(TAG_BMP280, "Oversampling temperature and pressure mode: %d", conf);

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    /* Set oversampling temperature and pressure */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, CTRL_MEAS_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, conf, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, command set oversampling tempearature and pressure mode not send");
    }
    i2c_cmd_link_delete(cmd);

    return;
}

/* Get calibration temperature */
void get_calibration_temperature_parameter_bmp280(calibration_param_t *dig) {

    esp_err_t err;
    i2c_cmd_handle_t cmd;
    uint8_t calib[6] = {0};

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, T1_ADDRESS, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, calib, sizeof(calib), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    /* TODO */
    dig->T1 = 28009; //(uint16_t)(calib[1] << 8 | calib[0]);
    dig->T2 = 25654; //(int16_t)(calib[3] << 8 | calib[2]);
    dig->T3 = 50;    //(int16_t)(calib[5] << 8 | calib[4]);

    ESP_LOGI(TAG_BMP280, "Coefficent T1: %d", dig->T1);
    ESP_LOGI(TAG_BMP280, "Coefficent T2: %d", dig->T2);
    ESP_LOGI(TAG_BMP280, "Coefficent T3: %d", dig->T3);

    return;
}

/* Get calibration pressure */
void get_calibration_pressure_parameter_bmp280(calibration_param_t *dig) {

    esp_err_t err;
    i2c_cmd_handle_t cmd;
    uint8_t calib[18] = {0};

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, P1_ADDRESS, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, calib, sizeof(calib), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    for (uint8_t i = 0; i < sizeof(calib); i++) {
        ESP_LOGD(TAG_BMP280, "P[%u] coefficent [%u]: %u", i, i, calib[i]);
    }

    dig->P1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dig->P2 = (int16_t)(calib[3] << 8 | calib[2]);
    dig->P3 = (int16_t)(calib[5] << 8 | calib[4]);
    dig->P4 = (int16_t)(calib[7] << 8 | calib[6]);
    dig->P5 = (int16_t)(calib[9] << 8 | calib[8]);
    dig->P6 = (int16_t)(calib[11] << 8 | calib[10]);
    dig->P7 = (int16_t)(calib[13] << 8 | calib[12]);
    dig->P8 = (int16_t)(calib[15] << 8 | calib[14]);
    dig->P9 = (int16_t)(calib[17] << 8 | calib[16]);

    ESP_LOGI(TAG_BMP280, "P1 coefficent: %d", dig->P1);
    ESP_LOGI(TAG_BMP280, "P2 coefficent: %d", dig->P2);
    ESP_LOGI(TAG_BMP280, "P3 coefficent: %d", dig->P3);
    ESP_LOGI(TAG_BMP280, "P4 coefficent: %d", dig->P4);
    ESP_LOGI(TAG_BMP280, "P5 coefficent: %d", dig->P5);
    ESP_LOGI(TAG_BMP280, "P6 coefficent: %d", dig->P6);
    ESP_LOGI(TAG_BMP280, "P7 coefficent: %d", dig->P7);
    ESP_LOGI(TAG_BMP280, "P8 coefficent: %d", dig->P8);
    ESP_LOGI(TAG_BMP280, "P9 coefficent: %d", dig->P9);

    return;
}

/* Get status BMP280 */
uint8_t get_status_bmp280() {

    esp_err_t err;
    uint8_t status = 0;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, STATUS_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, &status, sizeof(status), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_BMP280, "Status BMP280: %u", status);

    return status;
}

/* Reset BMP280 sensor */
void reset_bmp280() {

    esp_err_t err;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, RESET_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, 0xB6, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, command reset not send");
        return;
    }

    i2c_cmd_link_delete(cmd);

    return;
}

/* Get configuartion BMP280 */
uint8_t get_configuration_bmp280() {

    esp_err_t err;
    i2c_cmd_handle_t cmd;
    uint8_t config = 0;

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, CONFIGURE_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, &config, sizeof(config), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_BMP280, "Config: %u", config);

    ESP_LOGI(TAG_BMP280, "Filter: %u", (config & 0x1C) >> 2);
    ESP_LOGI(TAG_BMP280, "Standby time: %u", (config & 0xE0) >> 5);

    return config;
}

/* Get oversampling temperature, pressure and mode */
uint8_t get_oversampling_mode_bmp280() {

    esp_err_t err;
    i2c_cmd_handle_t cmd;
    uint8_t config = 0;

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_BMP280, "Error, I2C link not created");
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, CTRL_MEAS_REG_BMP280, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_BMP280 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, &config, sizeof(config), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BMP280, "Error I2C, commands not send");
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_BMP280, "Oversampling mode: %u", config & 0x03);
    ESP_LOGI(TAG_BMP280, "Oversampling temp: %u", config & 0x1C);
    ESP_LOGI(TAG_BMP280, "Oversampling pressure: %u", config & 0xE0);

    return config;
}

/* BMP280 task */
void bmp280_task(void *param) {

    esp_err_t err;
    i2c_config_t conf;
    calibration_param_t dig;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.clk_stretch_tick = 100000;

    /* Install driver for i2c master mode and I2C_NUM_0 channel */
    err = i2c_driver_install(I2C_NUM_0, conf.mode);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_BMP280, "Error to install i2c driver, exit");
        return;
    }

    /* Configure i2c driver */
    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGI(TAG_BMP280, "Error to configure i2c driver, exit");
        return;
    }

    /* Reset device */
    reset_bmp280();

    /* Delay 50 ms after software reset */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Check sensor is alive */
    if (read_id_sensor_bmp280() != 0x58) {
        ESP_LOGE(TAG_BMP280, "Device BMP280 not found, exit");
        return;
    }

    /* Set configuration BMP280 */
    set_configuration_bmp280(1, 4);

    /* Set oversampling BMP280 for temperature and pressure */
    set_oversampling_mode_bmp280(NORMAL_MODE, 2, 5);

    /* Get calibration for calculate coefficent parameter temperature */
    get_calibration_temperature_parameter_bmp280(&dig);
    get_calibration_pressure_parameter_bmp280(&dig);

    while(1) {

        while(get_status_bmp280() & 0x08) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        ESP_LOGI(TAG_BMP280, "Temperature: %u", get_temperature_bmp280());
        ESP_LOGI(TAG_BMP280, "Pressure: %u", get_pressure_bmp280());

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}