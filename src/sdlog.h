#pragma once
#include <stdint.h>

void sdlog_init(void);
void sdlog_write(const char *row);
uint8_t sdlog_ok(void);
