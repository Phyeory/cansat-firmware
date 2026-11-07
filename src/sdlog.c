#include "sdlog.h"
#include <stdio.h>
#include <string.h>

#ifdef HAS_FATFS

#include "fatfs.h"
#include "ff.h"

static FIL   file;
static uint8_t ok = 0;
static uint32_t last_flush = 0;

void sdlog_init(void)
{
    char name[24];
    snprintf(name, sizeof(name), "FL%08lu.csv", (unsigned long)HAL_GetTick());
    if (f_mount(&USERFatFS, "", 1) != FR_OK) return;
    if (f_open(&file, name, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;
    ok = 1;
}

void sdlog_write(const char *row)
{
    if (!ok) return;
    UINT bw;
    f_write(&file, row, strlen(row), &bw);
    f_write(&file, "\r\n", 2, &bw);
    if (HAL_GetTick() - last_flush >= 1000U) {
        f_sync(&file);
        last_flush = HAL_GetTick();
    }
}

uint8_t sdlog_ok(void) { return ok; }

#else

void sdlog_init(void) {}
void sdlog_write(const char *row) { (void)row; }
uint8_t sdlog_ok(void) { return 0; }

#endif
