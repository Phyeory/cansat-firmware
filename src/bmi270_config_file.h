#pragma once
#include <stdint.h>

/* Mandatory BMI270 init blob, defined in bmi270_config_file.c (Bosch
   BSD-3-Clause). */
extern const uint8_t bmi270_config_file[];
#define BMI270_CONFIG_FILE_SIZE 8192U
