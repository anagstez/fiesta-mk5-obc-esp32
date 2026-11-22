#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "mcp2515.h"

static const char *TAG = "mcp";

#define CMD_RESET     0xC0
#define CMD_READ      0x03
#define CMD_WRITE     0x02
#define CMD_BITMOD    0x05
#define CMD_READ_RX0  0x90           /* RXB0 from SIDH, clears RX0IF on CS rise */
#define CMD_READ_RX1  0x94           /* RXB1 from SIDH, clears RX1IF on CS rise */

#define REG_CANSTAT   0x0E
#define REG_CANCTRL   0x0F
#define REG_CNF3      0x28
#define REG_CNF2      0x29
#define REG_CNF1      0x2A
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
#define REG_EFLG      0x2D
#define REG_RXB0CTRL  0x60
#define REG_RXB1CTRL  0x70

#define MODE_CONFIG   0x80
#define MODE_LISTEN   0x60
#define INT_RX0       0x01
#define INT_RX1       0x02
#define INT_ERR       0x20
#define EFLG_RXOVR    0xC0           /* RX1OVR | RX0OVR */

static spi_device_handle_t dev;
static SemaphoreHandle_t rx_sem;
static volatile uint32_t overflows;

static void xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = { .length = len * 8, .tx_buffer = tx, .rx_buffer = rx };
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev, &t));
}

static uint8_t reg_read(uint8_t r)
{
    uint8_t tx[3] = { CMD_READ, r, 0 }, rx[3];
    xfer(tx, rx, 3);
    return rx[2];
}

static void reg_write(uint8_t r, uint8_t v)
{
    uint8_t tx[3] = { CMD_WRITE, r, v };
    xfer(tx, NULL, 3);
}

static void reg_bitmod(uint8_t r, uint8_t mask, uint8_t v)
{
    uint8_t tx[4] = { CMD_BITMOD, r, mask, v };
    xfer(tx, NULL, 4);
}

/* CNF1..3 for an 8 MHz crystal, sample point 62.5-75 % (values from the Microchip bit time calculator) */
static bool timing_8mhz(uint32_t bitrate, uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3)
{
    switch (bitrate) {
    case 1000000: *cnf1 = 0x00; *cnf2 = 0x80; *cnf3 = 0x00; return true; /* 4 TQ */
    case 500000:  *cnf1 = 0x00; *cnf2 = 0x90; *cnf3 = 0x02; return true; /* 8 TQ, 1+1+3+3 */
    case 250000:  *cnf1 = 0x01; *cnf2 = 0x90; *cnf3 = 0x02; return true;
    case 125000:  *cnf1 = 0x03; *cnf2 = 0x90; *cnf3 = 0x02; return true;
    default:      return false;
    }
}

static void IRAM_ATTR on_int(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(rx_sem, &woken);
    if (woken) portYIELD_FROM_ISR();
}

esp_err_t mcp2515_init(const mcp_cfg_t *cfg)
{
    uint8_t cnf1, cnf2, cnf3;
    if (!timing_8mhz(cfg->bitrate, &cnf1, &cnf2, &cnf3)) return ESP_ERR_INVALID_ARG;

    spi_bus_config_t bus = { .sclk_io_num = cfg->sck, .mosi_io_num = cfg->mosi, .miso_io_num = cfg->miso,
                             .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 16 };
    ESP_ERROR_CHECK(spi_bus_initialize(cfg->host, &bus, SPI_DMA_DISABLED));
    spi_device_interface_config_t d = { .clock_speed_hz = 4 * 1000 * 1000, .mode = 0,
                                        .spics_io_num = cfg->cs, .queue_size = 1 };
    ESP_ERROR_CHECK(spi_bus_add_device(cfg->host, &d, &dev));

    rx_sem = xSemaphoreCreateBinary();

    uint8_t rst = CMD_RESET;
    xfer(&rst, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* after reset the chip is in configuration mode (CANSTAT 0x80) */
    if ((reg_read(REG_CANSTAT) & 0xE0) != MODE_CONFIG) {
        ESP_LOGE(TAG, "no answer from MCP2515 (CANSTAT=0x%02x), check wiring/power", reg_read(REG_CANSTAT));
        return ESP_ERR_NOT_FOUND;
    }

    reg_write(REG_CNF1, cnf1);
    reg_write(REG_CNF2, cnf2);
    reg_write(REG_CNF3, cnf3);
    if (reg_read(REG_CNF1) != cnf1 || reg_read(REG_CNF2) != cnf2 || reg_read(REG_CNF3) != cnf3) {
        ESP_LOGE(TAG, "CNF readback mismatch, SPI wiring?");
        return ESP_ERR_NOT_FOUND;
    }

    reg_write(REG_RXB0CTRL, 0x64);   /* RXM=11: accept everything, filters off; BUKT: roll over into RXB1 */
    reg_write(REG_RXB1CTRL, 0x60);
    reg_write(REG_CANINTE, INT_RX0 | INT_RX1);
    reg_write(REG_CANINTF, 0);

    reg_write(REG_CANCTRL, MODE_LISTEN);
    if ((reg_read(REG_CANSTAT) & 0xE0) != MODE_LISTEN) {
        ESP_LOGE(TAG, "failed to enter listen-only mode");
        return ESP_FAIL;
    }

    gpio_config_t io = { .pin_bit_mask = 1ULL << cfg->intr, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_NEGEDGE };
    ESP_ERROR_CHECK(gpio_config(&io));
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(isr_err);   /* already installed by the encoder */
    ESP_ERROR_CHECK(gpio_isr_handler_add(cfg->intr, on_int, NULL));
    xSemaphoreGive(rx_sem);          /* drain anything that arrived before the edge handler existed */

    ESP_LOGI(TAG, "listen-only %lu bit/s, CNF=%02x/%02x/%02x", (unsigned long)cfg->bitrate, cnf1, cnf2, cnf3);
    return ESP_OK;
}

SemaphoreHandle_t mcp2515_rx_sem(void) { return rx_sem; }
uint32_t mcp2515_overflows(void) { return overflows; }

bool mcp2515_read(mcp_frame_t *f)
{
    uint8_t flags = reg_read(REG_CANINTF);
    if (reg_read(REG_EFLG) & EFLG_RXOVR) {
        overflows++;
        reg_bitmod(REG_EFLG, EFLG_RXOVR, 0);
    }
    if (!(flags & (INT_RX0 | INT_RX1))) return false;

    uint8_t tx[14] = { (flags & INT_RX0) ? CMD_READ_RX0 : CMD_READ_RX1 }, rx[14];
    xfer(tx, rx, sizeof(tx));
    const uint8_t *b = rx + 1;       /* SIDH SIDL EID8 EID0 DLC D0..D7 */

    memset(f, 0, sizeof(*f));
    f->ext = b[1] & 0x08;
    f->id = (uint32_t)b[0] << 3 | b[1] >> 5;
    if (f->ext) f->id = f->id << 18 | (uint32_t)(b[1] & 0x03) << 16 | (uint32_t)b[2] << 8 | b[3];
    f->rtr = b[4] & 0x40;
    f->dlc = (b[4] & 0x0F) > 8 ? 8 : (b[4] & 0x0F);
    memcpy(f->data, b + 5, 8);
    return true;
}
