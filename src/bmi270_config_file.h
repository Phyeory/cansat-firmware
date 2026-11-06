#pragma once
#include <stdint.h>

/* Mandatory BMI270 initialisation blob, defined in bmi270_config_file.c
   (Bosch BSD-3-Clause, see that file). */
extern const uint8_t bmi270_config_file[];

/* The .c only carries the extern array, so the size is fixed here
   (verified against bmi270_config_file.c when it was vendored). */
#define BMI270_CONFIG_FILE_SIZE 8192U
