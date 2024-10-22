#ifndef CHIP_INFO_H
#define CHIP_INFO_H

#pragma once

#define CHIP_INFO_MAJOR 0
#define CHIP_INFO_MINOR 1
#define CHIP_INFO_PATCH 0

void print_chip_info_version();

void esp_print_chip_info(esp_chip_info_t *chip_info);

#endif
