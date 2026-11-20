/* Host replacement for src/sdlog.c.
 *
 * The real implementation needs FatFS and a card; the simulator only needs
 * the same three functions, writing the exact same rows to sim/out/sd.csv so
 * that analysis/cansat_data.py can parse the result as if it came off a card.
 */
#include "sdlog.h"

#include <stdio.h>
#include "world.h"

static FILE *file;

void sdlog_init(void)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/sd.csv", SIM_OUT_DIR);
    file = fopen(path, "w");
}

void sdlog_write(const char *row)
{
    if (!file) return;
    fputs(row, file);
    fputc('\n', file);
    sim_count_row();
}

uint8_t sdlog_ok(void)
{
    if (!file) return 0;
    fflush(file);
    return 1;
}
