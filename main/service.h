#pragma once
/* SERVICE mode network side: a WiFi access point plus a TCP server on port 35000 that speaks ELM327 (see elm.h) */
#include <stdbool.h>
#include "elm.h"

bool service_start(const elm_io_t *io);      /* AP + server; false if the radio could not be started */
void service_stop(void);
bool service_client(void);                   /* a client (FORScan) is connected */
const char *service_ssid(void);
const char *service_pass(void);              /* WPA2 password, 8 characters, derived from the chip's MAC */
