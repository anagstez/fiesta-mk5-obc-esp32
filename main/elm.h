#pragma once
/*
 * ELM327 v1.5 emulation for the SERVICE mode: lets FORScan (WiFi ELM327, TCP) talk to a CAN bus through the board.
 * It only translates; the buses themselves are reached through the callbacks in elm_io_t (11-bit CAN frames).
 */
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool (*send)(uint32_t id, const uint8_t *d, uint8_t dlc);                  /* queue one frame; false = could not be sent */
    bool (*recv)(uint32_t *id, uint8_t *d, uint8_t *dlc, uint32_t wait_ms);    /* next received frame that passed the filter */
    void (*set_filter)(uint32_t id, uint32_t mask);                            /* only frames with (id & mask) == filter are queued */
    void (*flush)(void);                                                       /* drop everything received so far */
    int (*volt10)(void);                                                       /* battery voltage, 0.1 V (0 = unknown) */
} elm_io_t;

/* serves one connected client until it disconnects or *stop becomes true */
void elm_session(int sock, const elm_io_t *io, volatile bool *stop);
