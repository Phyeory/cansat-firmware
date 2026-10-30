#include "gps_sam_m8q.h"
#include <string.h>
#include <stdlib.h>

#define RXBUF_LEN 1024            /* power of two */
#define LINE_LEN  100
#define MAX_FIELDS 20

static UART_HandleTypeDef *u;
static volatile uint8_t  rxbuf[RXBUF_LEN];
static volatile uint16_t rx_head = 0;     /* written by ISR */
static uint16_t rx_tail = 0;              /* read by gps_poll */
static char     line[LINE_LEN];
static uint16_t linelen = 0;
static gps_data_t latest;

void gps_init(UART_HandleTypeDef *uart)
{
    u = uart;
    memset(&latest, 0, sizeof(latest));

    /* RX interrupt: one byte -> ring buffer */
    __HAL_UART_CLEAR_FLAG(u, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    SET_BIT(u->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

void USART1_IRQHandler(void)
{
    uint32_t isr = USART1->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    if (isr & USART_ISR_RXNE_RXFNE) {
        uint8_t b = (uint8_t)USART1->RDR;
        uint16_t next = (uint16_t)((rx_head + 1) & (RXBUF_LEN - 1));
        if (next != rx_tail) {              /* drop byte if buffer full */
            rxbuf[rx_head] = b;
            rx_head = next;
        }
    }
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* $....*HH  XOR of everything between $ and * */
static bool checksum_ok(const char *s)
{
    uint8_t x = 0;
    const char *p = s + 1;
    while (*p && *p != '*') x ^= (uint8_t)*p++;
    if (*p != '*' || hexval(p[1]) < 0 || hexval(p[2]) < 0) return false;
    return x == (uint8_t)((hexval(p[1]) << 4) | hexval(p[2]));
}

/* split on commas but KEEP empty fields (strtok would skip them and shift
   every index when there's no fix) */
static int split(char *s, char *f[], int max)
{
    int n = 0;
    char *star = strchr(s, '*');
    if (star) *star = '\0';
    f[n++] = s;
    for (char *p = s; *p && n < max; p++) {
        if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
    }
    return n;
}

/* ddmm.mmmm -> decimal degrees */
static double nmea_deg(const char *v, const char *hemi)
{
    if (!v[0]) return 0.0;
    double raw = atof(v);
    int deg = (int)(raw / 100.0);
    double d = deg + (raw - deg * 100.0) / 60.0;
    return (hemi[0] == 'S' || hemi[0] == 'W') ? -d : d;
}

static void parse_line(char *s)
{
    if (s[0] != '$' || !checksum_ok(s)) return;

    char *f[MAX_FIELDS];
    int n = split(s, f, MAX_FIELDS);
    if (strlen(f[0]) < 6) return;
    const char *type = f[0] + 3;           /* skip "$GP" / "$GN" / "$GL" */

    if (strcmp(type, "RMC") == 0 && n >= 7) {
        /* 1 time, 2 status, 3 lat, 4 N/S, 5 lon, 6 E/W */
        latest.fix = (f[2][0] == 'A');
        if (latest.fix) {
            latest.lat_deg = nmea_deg(f[3], f[4]);
            latest.lon_deg = nmea_deg(f[5], f[6]);
        }
        latest.sentences++;
    } else if (strcmp(type, "GGA") == 0 && n >= 10) {
        /* 6 quality, 7 sats, 8 hdop, 9 alt */
        latest.sats = (uint8_t)atoi(f[7]);
        if (f[6][0] != '0' && f[9][0]) latest.alt_m = (float)atof(f[9]);
        latest.sentences++;
    }
}

void gps_poll(gps_data_t *g)
{
    while (rx_tail != rx_head) {
        char c = (char)rxbuf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) & (RXBUF_LEN - 1));

        if (c == '$') linelen = 0;         /* resync on sentence start */
        if (c == '\n') {
            line[linelen] = '\0';
            parse_line(line);
            linelen = 0;
        } else if (c != '\r') {
            if (linelen < LINE_LEN - 1) line[linelen++] = c;
            else linelen = 0;              /* garbage, drop */
        }
    }
    *g = latest;
}
