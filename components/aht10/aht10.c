#include <stdio.h>
#include <ctype.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "aht10.h"

#define AHT10_ADDRESS          0x38
#define INITIALIZATION_COMMAND 0XE1
#define TRIGGER_MEASUREMENT    0xAC
#define INIT0_AHT10            0x08
#define INIT1_AHT10            0x00
#define DATA_0                 0x33
#define DATA_1                 0x00
#define AHT10_SOFT_RESET       0xBA
#define AHT10_BUSY             0x80
#define AHT10_MODE             0x60
#define AHT10_CALIBRATION      0x04

#define TIMEOUT 1000

#define TAG_AHT10 "AHT10"

/* Print version aht10 library */
void print_aht10_version() {

    ESP_LOGI(TAG_AHT10, "Version aht10 library: %u.%u.%u", AHT10_MAJOR, AHT10_MINOR, AHT10_PATCH);

    return;
}

/* Print status information aht10 sensor */
void print_status_aht10(uint8_t status) {

	if (status & 0x80)
		ESP_LOGI(TAG_AHT10, "The AHT10 device is busy");
	else
		ESP_LOGI(TAG_AHT10, "The AHT10 device is in the measurement state, and the device is idle");

	if ((status & 0x60) == 0x00)
		ESP_LOGI(TAG_AHT10, "AHT10 device is in NOR mode");
	else if((status & 0x60) == 0x40)
		ESP_LOGI(TAG_AHT10, "AHT10 device is in CYC code");
	else if ((status & 0x60) == 0x20)
		ESP_LOGI(TAG_AHT10, "AHT10 device is in CMD mode");

	if (status & 0x04)
		ESP_LOGI(TAG_AHT10, "AHT10 device is calibrate");
	else
		ESP_LOGI(TAG_AHT10, "AHT10 device is not calibrate");

	return;
}

/* Trigger measurement sensor aht10 */
esp_err_t trigger_measurement_aht10() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT10, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT10_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, TRIGGER_MEASUREMENT, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, DATA_0, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, DATA_1, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT10, "Error, trigger measurement not performed");
        return err;
    }

    i2c_cmd_link_delete(cmd);

	return err;
}

/* Init sensor aht10 */
esp_err_t init_aht10() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT10, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT10_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, INITIALIZATION_COMMAND, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, INIT0_AHT10, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, INIT1_AHT10, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT10, "Error, trigger measurement not performed");
        return err;
    }

    i2c_cmd_link_delete(cmd);

	return err;
}

/* Read measurement form sensor aht10*/
esp_err_t read_measurement_aht10(uint8_t *data, uint8_t size_data, uint8_t *status) {

   	esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT10, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT10_ADDRESS << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read(cmd, data, size_data, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT10, "Error, data not read");
        return err;
    }

    i2c_cmd_link_delete(cmd);

    *status = data[0];

	return err;
}

/* Get temperature value */
float get_temperature_aht10(uint8_t *data) {

	uint32_t raw_temperature = (((uint32_t)data[3] & 0x0F) << 16) | ((uint16_t)data[4] << 8) | data[5];

    return ((float)raw_temperature * 200.0 / 1048576.0) - 50.0;
}

/* Get humidity value */
float get_humidity_aht10(uint8_t *data) {

	int32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);

    return (float)raw_humidity * 100.0 / 1048576.0;
}

/* Reset sensor aht10 */
esp_err_t reset_aht10() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT10, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT10_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, AHT10_SOFT_RESET, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT10, "Error, AHT10 sensor not reseted");
        return err;
    }

    i2c_cmd_link_delete(cmd);

	return err;
}
