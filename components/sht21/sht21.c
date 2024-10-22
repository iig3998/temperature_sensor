#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sht21.h"

#define TAG_SHT21 "SHT21"

#define ADDRESS_SHT21                         0x40
#define TRIGGER_T_MEASUREMENT_HOLD_MASTER     0xE3
#define TRIGGER_RH_MEASUREMENT_HOLD_MASTER    0xE5
#define TRIGGER_T_MEASUREMENT_NO_HOLD_MASTER  0xF3
#define TRIGGER_RH_MEASUREMENT_NO_HOLD_MASTER 0xF5
#define WRITE_USER_REGISTER                   0xE6
#define READ_USER_REGISTER                    0xE7
#define SOFT_RESET                            0xFE
#define WRITE_HEATER_CONTROL_REGISTER         0x51
#define READ_HEATER_CONTROL_REGISTER          0x11

#define RESOLUTION_12RH_14T                   ((0 << 7) | (0 << 0)) // [12 bit resolution humidity - 14 bit resolution temperature]
#define RESOLUTION_8RH_12T                    ((0 << 7) | (1 << 0)) // [08 bit resolution humidity - 12 bit resolution temperature]
#define RESOLUTION_10RH_13T                   ((1 << 7) | (0 << 0)) // [10 bit resolution humidity - 13 bit resolution temperature]
#define RESOLUTION_11RH_11T                   ((1 << 7) | (1 << 0)) // [11 bit resolution humidity - 11 bit resolution temperature]

#define BATTERY_ABOVE_2V25                    (0 << 6)
#define BATTERY_BELOW_2V25                    (1 << 6)

#define ONCHIP_HEATER_ENABLE                  (1 << 2)
#define ONCHIP_HEATER_DISABLE                 (0 << 2)

/**/
static void print_user_register_context(uint8_t user_reg) {

    uint8_t res = user_reg & 0x81;
    if (res == RESOLUTION_12RH_14T)
        ESP_LOGI(TAG_SHT21, "Resolution RH: 12 bits -- Resolution T: 14 bits");
    else if (res == RESOLUTION_8RH_12T)
        ESP_LOGI(TAG_SHT21, "Resolution RH: 8 bits -- Resolution T: 12 bits");
    else if (res == RESOLUTION_10RH_13T)
        ESP_LOGI(TAG_SHT21, "Resolution RH: 10 bits -- Resolution T: 10 bits");
    else if (res == RESOLUTION_11RH_11T)
        ESP_LOGI(TAG_SHT21, "Resolution RH: 11 bits -- Resolution T: 11 bits");

    if (user_reg & 0x40)
        ESP_LOGI(TAG_SHT21, "Battery voltage is below");
    else
        ESP_LOGI(TAG_SHT21, "Battery voltage is above");

    if (user_reg & 0x04)
        ESP_LOGI(TAG_SHT21, "Onchip heater is enable");
    else
        ESP_LOGI(TAG_SHT21, "Onchip heater is disable");

    return;
}

/* */
static uint16_t calc_crc16_sht21(uint8_t *data) {

    uint8_t crc = 0xFF;
    uint8_t len = sizeof(data);
    size_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }

    return crc;
}

/**/
void print_sht21_version() {

    ESP_LOGI(TAG_SHT21, "SHT21 library version: %u.%u.%u", SHT21_MAJOR, SHT21_MINOR, SHT21_PATCH);

    return;
}

/* Read id revision */
esp_err_t read_id_version_sht21() {

    uint8_t re_id_1st_cr[2] = {0xFA, 0x0F};
    uint8_t re_id_2st_cr[2] = {0xFC, 0xC9};
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);

    i2c_cmd_link_delete(cmd);

    return err;
}

/* Read user register */
esp_err_t set_user_register_sht21(uint8_t resolution, bool otp_reload, bool onchip_heater) {

    uint8_t conf = 0x00;
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    if (otp_reload)
        conf |= 0x20;

    if (onchip_heater)
        conf |= 0x40;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, WRITE_USER_REGISTER, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &conf, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error I2C");
        return err;
    }

    i2c_cmd_link_delete(cmd);

    return err;
}

/* Read user register data */
esp_err_t read_user_register_sht21() {

    uint8_t user_reg = 0x00;
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, READ_USER_REGISTER, I2C_MASTER_ACK);

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &user_reg, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error, I2C write failed");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_SHT21, "User register value: %u", user_reg);

    /* Print user register context */
    print_user_register_context(user_reg);

    return err;
}

/* Software reset SHT21 */
esp_err_t soft_reset_sht21() {

    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, SOFT_RESET, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error, I2C write failed");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(20));

    return err;
}

/* Get real humidity value */
float get_humidity_float_sht21(uint16_t humidity) {

    return (float)((125 * humidity / 65536) - 6);
}

/* Get real temperature value */
float get_temperature_float_sht21(uint16_t temperature) {

    return (float)((175.72 * ((float)((float)temperature / 65536.0))) - 46.85);
}

/* Read humidity value */
esp_err_t get_humidity_sht21(uint16_t *humidity) {

    uint8_t msb_humidity = 0;
    uint8_t lsb_humidity = 0;
    uint8_t checksum = 0;
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    /* Send command for read HR value */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, TRIGGER_RH_MEASUREMENT_HOLD_MASTER, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error, I2C write failed");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    /* Wait measurement */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Read humidity value (16 bits) */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &msb_humidity, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &lsb_humidity, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &checksum, I2C_MASTER_LAST_NACK);
    
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error I2C");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_SHT21, "Humidity value [16 bits]: %u", ((msb_humidity << 8) | lsb_humidity));

    *humidity = ((msb_humidity << 8) | lsb_humidity) & 0xFFFC;

    return err;
}

/* Read temperature value */
esp_err_t get_temperature_sht21(uint16_t *temperature) {

    uint8_t msb_temperature = 0;
    uint8_t lsb_temperature = 0;
    uint8_t checksum = 0;
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    /* Send command for read HR value */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, TRIGGER_T_MEASUREMENT_HOLD_MASTER, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error, I2C cwrite failed");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    /* Wait measurement */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }
    
    /* Read temperature value (16 bits) */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &msb_temperature, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &lsb_temperature, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &checksum, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error, I2C write failed");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG_SHT21, "Temperature value [16 bits]: %u", ((msb_temperature << 8) | lsb_temperature));
    ESP_LOGI(TAG_SHT21, "Checksum %u", checksum);

    *temperature = ((msb_temperature << 8) | lsb_temperature) & 0xFFFC;

    return err;
}

/* Read Firmware Revision */
esp_err_t read_firmware_revision_sht21() {

    uint8_t firmware_revision[2] = {0x84, 0xB8};
    uint8_t fir_rev = 0x00;
    esp_err_t err = ESP_FAIL;
    i2c_cmd_handle_t cmd;

    /* Create I2C link */
    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_SHT21, "Error, I2C command link not created");
        return err;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write(cmd, firmware_revision, sizeof(firmware_revision), I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRESS_SHT21 << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &fir_rev, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error I2C");
        return err;
    }
    i2c_cmd_link_delete(cmd);

    /* Print firmware revision */
    if (fir_rev == 0xFF)
        ESP_LOGI(TAG_SHT21, "Firmware version 1.0");
    else if (fir_rev == 0x20)
        ESP_LOGI(TAG_SHT21, "Firmware version 2.0");

    return err;
}

/* SHT21 task */
void sht21_task(void *param) {

    uint16_t temperature = 0;
    uint16_t humidity = 0;
    esp_err_t err = ESP_FAIL;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.clk_stretch_tick = 100000;

    /* Install driver for i2c master mode and I2C_NUM_0 channel */
    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error to install i2c driver, exit");
        return;
    }

    /* Configure i2c driver */
    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SHT21, "Error to configure i2c driver, exit");
        return;
    }

    /* Delay 20ms */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Reset sht21 sensor */
    soft_reset_sht21();

    /* Wait 200ms after reset */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Read firmware version */
    read_firmware_revision_sht21();

    /* Read user register */
    read_user_register_sht21();

    while(1) {

        get_temperature_sht21(&temperature);
        ESP_LOGI(TAG_SHT21, "Temperature %.1f", get_temperature_float_sht21(temperature));

        get_humidity_sht21(&humidity);
        ESP_LOGI(TAG_SHT21, "Humidity %.1f", get_humidity_float_sht21(humidity));

        /* Delay 5 seconds */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    return;
}
