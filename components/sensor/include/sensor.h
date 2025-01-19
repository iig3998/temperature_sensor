#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

enum sensor_type {
    TEMPERATURE_HUMIDITY_SENSOR = 0,
    ALARM_SENSOR
};

typedef struct {
	float temp;
	uint8_t hum;
} metric_sensor;

typedef struct {
	bool state;
} alarm_sensor;

typedef struct {
	uint8_t id;
	uint8_t mac[6];
	enum sensor_type type;
    union {
		metric_sensor ms;
		alarm_sensor as;
	} select;
} __attribute__((__packed__)) node_id;

#endif