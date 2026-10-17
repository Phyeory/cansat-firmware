#include "gps_sam_m8q.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* u-blox default: NMEA over UART @ 9600. We ask for RMC+GGA at 1 Hz via
   standard $PUBX commands (accepted by all u-blox M8 modules). */

#define RXBUF_LEN 512
#define LINE_LEN  96

static UART_HandleTypeDef *u;
static uint8_t  rxbuf[RXBUF_LEN];
static uint16_t rxhead = 0;
static char     line[LINE_LEN];
static uint16_t linelen = 0;
static gps_data_t latest;

void gps_init(UART_HandleTypeDef *uart)
{
    u = uart;
    memset(&latest, 0, sizeof(latest));
    const char *cfg =
        "$PUBX,40,RMC,0,1,0,0,0,0*42\r\n"   /* RMC on UART1 */
        "$PUBX,40,GGA,0,1,0,0,0,0*42\r\n";  /* GGA on UART1 */
    HAL_UART_Transmit(u, (const uint8_t *)cfg, (uint16_t)strlen(cfg), 100);
}

static void parse_line(char *s)
{
    /* $GPRMC / $GNRMC: time,status,lat,N/S,lon,E/W,...  */
    if (strstr(s, "RMC")) {
        char *tok[13] = {0};
        int n = 0;
        for (char *t = strtok(s, ","); t && n < 13; t = strtok(NULL, ","))
            tok[n++] = t;
        if (n >= 9 && tok[2] && tok[2][0] == 'A') {          /* status A = fix */
            double lat = atof(tok[3]); double lon = atof(tok[5]);
            latest.lat_deg = (int)(lat / 100.0) + (lat / 100.0 - (int)(lat / 100.0)) * 100.0 / 60.0;
            latest.lon_deg = (int)(lon / 100.0) + (lon / 100.0 - (int)(lon / 100.0)) * 100.0 / 60.0;
            if (tok[4] && tok[4][0] == 'S') latest.lat_deg = -latest.lat_deg;
            if (tok[6] && tok[6][0] == 'W') latest.lon_deg = -latest.lon_deg;
            latest.fix = true;
        } else if (n >= 3) {
            latest.fix = false;
        }
    }
    /* $GPGGA / $GNGGA: ...,sats,hdop,alt,M,...  */
    if (strstr(s, "GGA")) {
        char *tok[10] = {0};
        int n = 0;
        for (char *t = strtok(s, ","); t && n < 10; t = strtok(NULL, ","))
            tok[n++] = t;
        if (n >= 10) {
            latest.sats  = (uint8_t)atoi(tok[7] ? tok[7] : "0");
            latest.alt_m = (float)atof(tok[9] ? tok[9] : "0");
        }
    }
}

void gps_poll(gps_data_t *g)
{
    /* non-blocking drain of whatever arrived since last call */
    while (HAL_UART_GetState(u) != HAL_UART_STATE_RESET) {
        uint8_t b;
        if (HAL_UART_Receive(u, &b, 1, 0) != HAL_OK) break;
        rxbuf[rxhead++ % RXBUF_LEN] = b;

        if (b == '\n' || linelen >= LINE_LEN - 1) {
            line[linelen] = '\0';
            if (line[0] == '$') parse_line(line);
            linelen = 0;
        } else if (b != '\r') {
            line[linelen++] = (char)b;
        }
    }
    *g = latest;
}
