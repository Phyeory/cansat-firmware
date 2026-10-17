#pragma once
#include <stdint.h>

/* SD card flight logging (FatFs on SPI2).
 *
 * Requires FatFS: generate the CubeMX code from stm32g474.ioc with the FatFS
 * middleware enabled (set SD_SPI + a compatible SPI transport in FatFs
 * user_diskio), then build with -DHAS_FATFS. Without it the module compiles
 * as a no-op so the rest of the firmware still runs and flies. */

void sdlog_init(void);                     /* mounts card, opens FLxxxxxx.csv */
void sdlog_write(const char *row);         /* one CSV line; flushed at ~1 Hz */
uint8_t sdlog_ok(void);
