#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "service.h"

static const char *TAG = "service";

#define SVC_PORT 35000                       /* the usual port of WiFi ELM327 adapters; address 192.168.0.10 */
#define SVC_SSID "OBC-SERVICE"

static bool wifi_ready;
static esp_netif_t *ap_netif;
static char pass[9];
static volatile bool stopping, running, connected;
static const elm_io_t *svc_io;

static bool wifi_up(void)
{
    if (!wifi_ready) {
        esp_netif_init();
        esp_event_loop_create_default();         /* ESP_ERR_INVALID_STATE when it exists already: fine */
        ap_netif = esp_netif_create_default_wifi_ap();
        wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
        if (esp_wifi_init(&ic) != ESP_OK) { ESP_LOGE(TAG, "esp_wifi_init failed"); return false; }
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        wifi_ready = true;
    }
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(pass, sizeof(pass), "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_ip_info_t ip = { 0 };
    ip.ip.addr = ESP_IP4TOADDR(192, 168, 0, 10);
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    esp_netif_set_ip_info(ap_netif, &ip);
    esp_netif_dhcps_start(ap_netif);
    wifi_config_t wc = { 0 };
    memcpy(wc.ap.ssid, SVC_SSID, strlen(SVC_SSID));
    wc.ap.ssid_len = strlen(SVC_SSID);
    memcpy(wc.ap.password, pass, 8);
    wc.ap.channel = 6;
    wc.ap.max_connection = 2;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK || esp_wifi_set_config(WIFI_IF_AP, &wc) != ESP_OK || esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP start failed");
        return false;
    }
    return true;
}

static void server_task(void *arg)
{
    const int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(SVC_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    const int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (ls < 0 || bind(ls, (struct sockaddr *)&a, sizeof(a)) != 0 || listen(ls, 1) != 0) {
        ESP_LOGE(TAG, "TCP server start failed");
        if (ls >= 0) close(ls);
        running = false;
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "ELM327 server on port %d", SVC_PORT);
    while (!stopping) {
        fd_set rs;
        FD_ZERO(&rs);
        FD_SET(ls, &rs);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
        if (select(ls + 1, &rs, NULL, NULL, &tv) <= 0) continue;
        const int c = accept(ls, NULL, NULL);
        if (c < 0) continue;
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        struct timeval rt = { .tv_sec = 0, .tv_usec = 100000 };
        setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &rt, sizeof(rt));
        ESP_LOGI(TAG, "client connected");
        connected = true;
        elm_session(c, svc_io, &stopping);
        connected = false;
        close(c);
        ESP_LOGI(TAG, "client gone");
    }
    close(ls);
    running = false;
    vTaskDelete(NULL);
}

bool service_start(const elm_io_t *io)
{
    if (running) return true;
    svc_io = io;
    stopping = false;
    if (!wifi_up()) return false;
    running = true;
    if (xTaskCreate(server_task, "elm_srv", 6144, NULL, 5, NULL) != pdPASS) {
        running = false;
        esp_wifi_stop();
        return false;
    }
    return true;
}

void service_stop(void)
{
    if (!running && !wifi_ready) return;
    stopping = true;
    for (int i = 0; i < 40 && running; i++) vTaskDelay(pdMS_TO_TICKS(50));
    if (wifi_ready) esp_wifi_stop();
    connected = false;
}

bool service_client(void) { return connected; }
const char *service_ssid(void) { return SVC_SSID; }
const char *service_pass(void) { return pass; }
