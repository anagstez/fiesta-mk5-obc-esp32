#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* MCP2515 (8 MHz crystal): listen-only (receives, never transmits or ACKs) unless cfg.tx is set */

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool ext, rtr;
} mcp_frame_t;

typedef struct {
    int host, sck, mosi, miso, cs, intr;
    uint32_t bitrate;                /* 125000, 250000, 500000 or 1000000 */
    bool tx;                         /* normal mode + one-shot: ACKs bus frames and can send mcp2515_send() */
} mcp_cfg_t;

/* brings up the SPI device and the chip; ESP_ERR_NOT_FOUND = chip not answering (wiring/power) */
esp_err_t mcp2515_init(const mcp_cfg_t *cfg);

/* given from the INT pin ISR (falling edge) whenever a frame is pending */
SemaphoreHandle_t mcp2515_rx_sem(void);

/* pops one frame from the chip, false if none pending; also clears overflow flags */
bool mcp2515_read(mcp_frame_t *f);

/* sends one standard-ID frame once (one-shot, no retries); only with cfg.tx; false if the buffer is still busy */
bool mcp2515_send(uint32_t id, const uint8_t *data, uint8_t dlc);

/* number of RX buffer overflows seen so far */
uint32_t mcp2515_overflows(void);
