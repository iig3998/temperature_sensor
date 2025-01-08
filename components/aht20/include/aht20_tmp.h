#ifndef AHT_20_TMP_H
#define AHT_20_TMP_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_types.h"

struct aht20_data{
    float temperature;
    float rel_humidity;
} aht20_data;

void aht20_read_measures(void *ignore);

#endif