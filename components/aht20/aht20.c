#include <stdio.h>
#include <ctype.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "aht20.h"

#define TIMEOUT 1000
#define TAG_AHT20 "AHT20"

/* Print version aht20 library */
void print_aht20_version() {

    ESP_LOGI(TAG_AHT20, "Version aht20 library: %u.%u.%u", AHT20_MAJOR, AHT20_MINOR, AHT20_PATCH);

    return;
}

/* Print status information AHT20 sensor */
void print_status_aht20(uint8_t status) {

	if (status & 0x80)
		ESP_LOGI(TAG_AHT20, "The AHT20 device is busy");
	else
		ESP_LOGI(TAG_AHT20, "The AHT20 device is in the measurement state, and the device is idle");

	if ((status & 0x60) == 0x00)
		ESP_LOGI(TAG_AHT20, "AHT20 device is in NOR mode");
	else if((status & 0x60) == 0x40)
		ESP_LOGI(TAG_AHT20, "AHT20 device is in CYC code");
	else if ((status & 0x60) == 0x20)
		ESP_LOGI(TAG_AHT20, "AHT20 device is in CMD mode");

	if (status & 0x04)
		ESP_LOGI(TAG_AHT20, "AHT20 device is calibrate");
	else
		ESP_LOGI(TAG_AHT20, "AHT20 device is not calibrate");

	return;
}

/* Trigger measurement sensor AHT20 */
esp_err_t trigger_measurement_aht20() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT20, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, AHT20_TRIGGER_MEASUREMENT, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, AHT20_DATA_0, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, AHT20_DATA_1, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT20, "Error, trigger measurement not performed");
        return err;
    }

    i2c_cmd_link_delete(cmd);

	return err;
}

/* Init sensor aht20 */
esp_err_t init_aht20() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT20, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, AHT20_INITIALIZATION, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT20, "Error, trigger measurement not performed");
        return err;
    }

    i2c_cmd_link_delete(cmd);

	return err;
}

/* Read measurement form sensor aht20 */
esp_err_t read_measurement_aht20(uint8_t *data, uint8_t size_data, uint8_t *status) {

   	esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT20, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDRESS << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read(cmd, data, 7, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT20, "Error, data not read");
        return err;
    }

    i2c_cmd_link_delete(cmd);

    *status = data[0];

	return err;
}

/* Get temperature value */
float get_temperature_aht20(uint8_t *data) {

	uint32_t raw_temperature = (((uint32_t)data[3] & 0x0F) << 16) | ((uint16_t)data[4] << 8) | data[5];

    return ((float)raw_temperature * 200.0 / 1048576.0) - 50.0;
}

/* Get humidity value */
float get_humidity_aht20(uint8_t *data) {

	int32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);

    return (float)raw_humidity * 100.0 / 1048576.0;
}

/* Reset sensor aht20 */
esp_err_t reset_aht20() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_AHT20, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AHT20_ADDRESS << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, AHT20_RESET, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_AHT20, "Error, aht20 sensor not reseted");
        return err;
    }

    i2c_cmd_link_delete(cmd);

	return err;
}
