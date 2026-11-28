#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "elm.h"

static const char *TAG = "elm";

#define ST_DEFAULT_MS 200            /* answer timeout of a genuine ELM327 after ATZ */
#define QUIET_MS      40             /* after a complete answer: how long to wait for more (other modules) */
#define PENDING_MS    5000           /* after 7F xx 78 (response pending): wait this long for the real answer */
#define MAX_MSG       4095           /* ISO-TP limit */
#define REQ_MAX       260            /* longest request we accept (bytes), the line buffer is twice that */

typedef struct {
    int sock;
    const elm_io_t *io;
    volatile bool *stop;
    char ob[600];
    int on;
    bool echo, lf, spaces, hdr, caf, cfc, auto_proto;
    char proto;                      /* ATSP value, only for ATDP / ATDPN */
    uint32_t tx_id;                  /* ATSH, 11-bit */
    int cra;                         /* ATCRA, -1 = not set */
    bool use_cf;
    uint32_t cf, cm;                 /* ATCF / ATCM */
    int fcsh, fcsm, fcsd_n;          /* ATFCSH (-1 = default), ATFCSM, ATFCSD length */
    uint8_t fcsd[5];
    uint32_t st_ms;
    int want;                        /* trailing digit of a request: how many answers to wait for (0 = until the timeout) */
    uint8_t pad;
    char last[2 * REQ_MAX + 4];
    bool asm_on;                     /* ISO-TP answer in progress */
    uint32_t asm_id;
    int asm_len, asm_got, asm_sn;
    uint8_t rb[MAX_MSG + 8];
} elm_t;

/* ---- output ---- */

static void out_flush(elm_t *e)
{
    int off = 0;
    while (off < e->on) {
        const int n = send(e->sock, e->ob + off, e->on - off, 0);
        if (n <= 0) break;
        off += n;
    }
    e->on = 0;
}

static void put(elm_t *e, const char *s)
{
    for (; *s; s++) {
        if (e->on >= (int)sizeof(e->ob)) out_flush(e);
        e->ob[e->on++] = *s;
    }
}

static void eol(elm_t *e) { put(e, e->lf ? "\r\n" : "\r"); }
static void reply(elm_t *e, const char *s) { put(e, s); eol(e); }

static void end_cmd(elm_t *e)
{
    eol(e);
    put(e, ">");
    out_flush(e);
}

static void put_bytes(elm_t *e, const uint8_t *d, int n)
{
    char b[4];
    for (int i = 0; i < n; i++) {
        snprintf(b, sizeof(b), e->spaces && i ? " %02X" : "%02X", d[i]);
        put(e, b);
    }
}

static void print_frame(elm_t *e, uint32_t id, const uint8_t *d, int dlc)
{
    if (e->hdr) {
        char b[8];
        snprintf(b, sizeof(b), "%03X", (unsigned)id);
        put(e, b);
        if (e->spaces) put(e, " ");
    }
    put_bytes(e, d, dlc);
    eol(e);
}

/* ---- state ---- */

static void apply_filter(elm_t *e)
{
    if (e->cra >= 0) e->io->set_filter((uint32_t)e->cra, 0x7FF);
    else if (e->use_cf) e->io->set_filter(e->cf, e->cm);
    else if (e->tx_id != 0x7DF && (e->tx_id & 0x7F8) != 0x7E0) e->io->set_filter(e->tx_id + 8, 0x7FF);
    else e->io->set_filter(0x7E8, 0x7F8);
}

static bool accepts(const elm_t *e, uint32_t id)
{
    if (e->cra >= 0) return id == (uint32_t)e->cra;
    if (e->use_cf) return (id & e->cm) == e->cf;
    return (id & 0x7F8) == 0x7E8 || (e->tx_id != 0x7DF && id == e->tx_id + 8);
}

static void reset_state(elm_t *e)
{
    e->echo = true; e->lf = true; e->spaces = true; e->hdr = false; e->caf = true; e->cfc = true;
    e->auto_proto = true; e->proto = '6';
    e->tx_id = 0x7DF; e->cra = -1; e->use_cf = false; e->cf = 0; e->cm = 0;
    e->fcsh = -1; e->fcsm = 0; e->fcsd_n = 0;
    e->st_ms = ST_DEFAULT_MS; e->pad = 0x00;
    e->asm_on = false;
    apply_filter(e);
}

/* ---- CAN transport ---- */

static bool send_retry(elm_t *e, uint32_t id, const uint8_t *d, uint8_t dlc)
{
    for (int i = 0; i < 5; i++) {
        if (e->io->send(id, d, dlc)) return true;
        vTaskDelay(1);
    }
    return false;
}

static bool next_accepted(elm_t *e, uint32_t *id, uint8_t *d, uint8_t *dlc, uint32_t wait_ms)
{
    const TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(wait_ms);
    for (;;) {
        const int32_t left = (int32_t)(end - xTaskGetTickCount());
        if (left <= 0 || *e->stop) return false;
        if (!e->io->recv(id, d, dlc, pdTICKS_TO_MS(left))) return false;
        if (accepts(e, *id)) return true;
    }
}

static void send_fc(elm_t *e, uint32_t resp_id)
{
    uint8_t f[8];
    memset(f, e->pad, sizeof(f));
    f[0] = 0x30;
    uint32_t id = e->tx_id == 0x7DF ? resp_id - 8 : e->tx_id;
    if (e->fcsm == 1 && e->fcsh >= 0) id = (uint32_t)e->fcsh;
    if (e->fcsm >= 1 && e->fcsd_n > 0) memcpy(f, e->fcsd, e->fcsd_n);
    send_retry(e, id, f, 8);
}

/* ISO 15765-2 request: single frame, or first frame + flow control + consecutive frames. false = CAN ERROR */
static bool isotp_send(elm_t *e, const uint8_t *r, int n)
{
    uint8_t f[8];
    memset(f, e->pad, sizeof(f));
    if (n <= 7) {
        f[0] = n;
        memcpy(f + 1, r, n);
        return send_retry(e, e->tx_id, f, 8);
    }
    f[0] = 0x10 | (n >> 8);
    f[1] = n & 0xFF;
    memcpy(f + 2, r, 6);
    if (!send_retry(e, e->tx_id, f, 8)) return false;
    int off = 6, sn = 1;
    while (off < n) {
        uint32_t id; uint8_t d[8], dlc;
        int waits = 0;
        for (;;) {                                       /* wait for the flow control frame of the module */
            if (!next_accepted(e, &id, d, &dlc, 1000) || waits > 20) return false;
            if (d[0] >> 4 != 3) continue;
            if ((d[0] & 0x0F) == 1) { waits++; continue; }                 /* wait */
            if ((d[0] & 0x0F) == 2) return false;                          /* overflow */
            break;
        }
        const int bs = d[1], st = d[2];
        int cnt = 0;
        while (off < n) {
            memset(f, e->pad, sizeof(f));
            const int c = n - off < 7 ? n - off : 7;
            f[0] = 0x20 | (sn & 0x0F);
            memcpy(f + 1, r + off, c);
            if (!send_retry(e, e->tx_id, f, 8)) return false;
            off += c; sn++; cnt++;
            if (off >= n) break;
            if (st > 0 && st <= 0x7F) vTaskDelay(pdMS_TO_TICKS(st) ? pdMS_TO_TICKS(st) : 1);
            if (bs && cnt == bs) break;
        }
    }
    return true;
}

/* a complete ISO-TP message (CAF1, headers off): false = "response pending", the real answer follows */
static bool msg_done(elm_t *e, const uint8_t *m, int n, bool multi)
{
    if (n >= 3 && m[0] == 0x7F && m[2] == 0x78) return false;
    if (!e->hdr) {
        if (!multi) {
            put_bytes(e, m, n);
            eol(e);
        } else {
            char b[8];
            snprintf(b, sizeof(b), "%03X", n);
            reply(e, b);
            int off = 0, idx = 0;
            while (off < n) {
                const int c = idx == 0 ? (n < 6 ? n : 6) : (n - off < 7 ? n - off : 7);
                snprintf(b, sizeof(b), "%X: ", idx & 0x0F);
                put(e, b);
                put_bytes(e, m + off, c);
                eol(e);
                off += c; idx++;
            }
        }
    }
    return true;
}

/* waits for the answer(s) of the request just sent and prints them; returns how many lines/messages */
static int collect(elm_t *e)
{
    int printed = 0;
    e->asm_on = false;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(e->st_ms);
    for (;;) {
        const int32_t left = (int32_t)(deadline - xTaskGetTickCount());
        if (left <= 0 || *e->stop) break;
        uint32_t id; uint8_t d[8], dlc;
        if (!e->io->recv(&id, d, &dlc, pdTICKS_TO_MS(left))) break;
        if (!accepts(e, id) || dlc == 0) continue;
        bool complete = false, pending = false;
        if (!e->caf) {
            print_frame(e, id, d, dlc);
            printed++;
            complete = true;
        } else {
            if (e->hdr) { print_frame(e, id, d, dlc); printed++; }
            const int pci = d[0] >> 4;
            if (pci == 0) {
                const int n = d[0] & 0x0F;
                if (n > 0 && n < dlc) {
                    if (msg_done(e, d + 1, n, false)) { complete = true; if (!e->hdr) printed++; }
                    else pending = true;
                }
            } else if (pci == 1 && dlc >= 3) {
                const int total = (d[0] & 0x0F) << 8 | d[1];
                if (total > 7 && total <= MAX_MSG) {
                    e->asm_on = true; e->asm_id = id; e->asm_len = total; e->asm_sn = 1;
                    e->asm_got = dlc - 2 < 6 ? dlc - 2 : 6;
                    memcpy(e->rb, d + 2, e->asm_got);
                    if (e->cfc) send_fc(e, id);
                }
            } else if (pci == 2 && e->asm_on && id == e->asm_id && (d[0] & 0x0F) == (e->asm_sn & 0x0F)) {
                int c = e->asm_len - e->asm_got;
                if (c > dlc - 1) c = dlc - 1;
                if (c > 7) c = 7;
                memcpy(e->rb + e->asm_got, d + 1, c);
                e->asm_got += c; e->asm_sn++;
                if (e->asm_got >= e->asm_len) {
                    e->asm_on = false;
                    if (msg_done(e, e->rb, e->asm_len, true)) { complete = true; if (!e->hdr) printed++; }
                    else pending = true;
                }
            }
        }
        if (pending) {
            deadline = xTaskGetTickCount() + pdMS_TO_TICKS(PENDING_MS);
        } else if (complete) {
            if (e->want && printed >= e->want) break;
            const TickType_t q = xTaskGetTickCount() + pdMS_TO_TICKS(e->st_ms < QUIET_MS ? e->st_ms : QUIET_MS);
            if ((int32_t)(q - deadline) < 0) deadline = q;
            else if (printed == 1) deadline = q;
        }
    }
    return printed;
}

static void transact(elm_t *e, const uint8_t *req, int n)
{
    e->io->flush();
    bool ok;
    if (!e->caf) ok = n <= 8 && send_retry(e, e->tx_id, req, n);
    else ok = isotp_send(e, req, n);
    if (!ok) { ESP_LOGI(TAG, "  CAN ERROR on send"); reply(e, n > 8 && !e->caf ? "?" : "CAN ERROR"); return; }
    if (!collect(e)) { ESP_LOGI(TAG, "  NO DATA (id %03X, filter %03X/%03X)", (unsigned)e->tx_id, (unsigned)e->cf, (unsigned)e->cm); reply(e, "NO DATA"); }
}

/* ---- commands ---- */

static int hexv(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex(const char *s, uint32_t *v)
{
    if (!*s) return false;
    *v = 0;
    for (; *s; s++) {
        const int h = hexv(*s);
        if (h < 0) return false;
        *v = *v << 4 | h;
    }
    return true;
}

static const char *proto_name(char p)
{
    switch (p) {
    case '6': return "ISO 15765-4 (CAN 11/500)";
    case '7': return "ISO 15765-4 (CAN 29/500)";
    case '8': return "ISO 15765-4 (CAN 11/250)";
    case '9': return "ISO 15765-4 (CAN 29/250)";
    case 'A': return "SAE J1939 (CAN 29/250)";
    case 'B': return "USER1 (CAN 11/125)";
    case 'C': return "USER2 (CAN 11/50)";
    default:  return "ISO 15765-4 (CAN 11/500)";
    }
}

static void monitor(elm_t *e)                        /* ATMA: print everything that passes, until a character arrives */
{
    e->io->set_filter(0, 0);
    e->io->flush();
    char c;
    while (!*e->stop && recv(e->sock, &c, 1, MSG_DONTWAIT) <= 0) {
        uint32_t id; uint8_t d[8], dlc;
        if (e->io->recv(&id, d, &dlc, 50) && dlc) { print_frame(e, id, d, dlc); out_flush(e); }
    }
    reply(e, "STOPPED");
    apply_filter(e);
}

static void at_cmd(elm_t *e, const char *c)
{
#define IS(x)  (strcmp(c, x) == 0)
#define PFX(x) (strncmp(c, x, strlen(x)) == 0)
    uint32_t v;
    if (IS("Z") || IS("WS") || IS("D") || IS("PC")) {
        const bool zero = IS("Z");
        if (!IS("PC")) reset_state(e);
        if (zero) { eol(e); reply(e, "ELM327 v1.5"); } else reply(e, "OK");
    } else if (IS("I")) reply(e, "ELM327 v1.5");
    else if (IS("@1")) reply(e, "OBDII to RS232 Interpreter");
    else if (IS("@2")) reply(e, "?");
    else if (IS("E0") || IS("E1")) { e->echo = c[1] == '1'; reply(e, "OK"); }
    else if (IS("L0") || IS("L1")) { e->lf = c[1] == '1'; reply(e, "OK"); }
    else if (IS("S0") || IS("S1")) { e->spaces = c[1] == '1'; reply(e, "OK"); }
    else if (IS("H0") || IS("H1")) { e->hdr = c[1] == '1'; reply(e, "OK"); }
    else if (IS("CAF0") || IS("CAF1")) { e->caf = c[3] == '1'; reply(e, "OK"); }
    else if (IS("CFC0") || IS("CFC1")) { e->cfc = c[3] == '1'; reply(e, "OK"); }
    else if (IS("RV")) {
        const int v10 = e->io->volt10();
        char b[16];
        snprintf(b, sizeof(b), "%d.%dV", (v10 ? v10 : 120) / 10, (v10 ? v10 : 120) % 10);
        reply(e, b);
    } else if (IS("DPN")) {
        char b[4] = { e->auto_proto ? 'A' : e->proto, e->auto_proto ? e->proto : 0, 0 };
        reply(e, b);
    } else if (IS("DP")) {
        char b[64];
        snprintf(b, sizeof(b), "%s%s", e->auto_proto ? "AUTO, " : "", proto_name(e->proto));
        reply(e, b);
    } else if (PFX("SP") || PFX("TP")) {
        const char *p = c + 2;
        if (*p == 'A' && p[1]) { e->auto_proto = true; p++; }
        else e->auto_proto = false;
        e->proto = *p && *p != '0' ? *p : '6';
        if (*p == '0') e->auto_proto = true;
        reply(e, "OK");
    } else if (PFX("SH")) {
        if (parse_hex(c + 2, &v) && v <= 0x7FF) { e->tx_id = v; apply_filter(e); reply(e, "OK"); }
        else reply(e, "?");
    } else if (PFX("CRA")) {
        if (!c[3]) e->cra = -1;
        else if (parse_hex(c + 3, &v) && v <= 0x7FF) e->cra = (int)v;
        else { reply(e, "?"); return; }
        e->use_cf = false;
        apply_filter(e);
        reply(e, "OK");
    } else if (PFX("CF") || PFX("CM")) {
        if (!parse_hex(c + 2, &v)) { reply(e, "?"); return; }
        if (c[1] == 'F') e->cf = v & 0x7FF; else e->cm = v & 0x7FF;
        e->use_cf = e->cm != 0;
        e->cra = -1;
        apply_filter(e);
        reply(e, "OK");
    } else if (PFX("FCSH")) {
        if (parse_hex(c + 4, &v)) { e->fcsh = (int)(v & 0x7FF); reply(e, "OK"); } else reply(e, "?");
    } else if (PFX("FCSD")) {
        const int n = (int)strlen(c + 4);
        if (n < 2 || n > 10 || (n & 1)) { reply(e, "?"); return; }
        e->fcsd_n = n / 2;
        for (int i = 0; i < e->fcsd_n; i++) e->fcsd[i] = hexv(c[4 + 2 * i]) << 4 | hexv(c[5 + 2 * i]);
        reply(e, "OK");
    } else if (PFX("FCSM")) {
        e->fcsm = c[4] >= '0' && c[4] <= '2' ? c[4] - '0' : 0;
        reply(e, "OK");
    } else if (PFX("ST")) {
        if (parse_hex(c + 2, &v)) { e->st_ms = v ? v * 4 : 4; reply(e, "OK"); } else reply(e, "?");
    } else if (IS("AR")) {                           /* automatic receive: the answer address follows ATSH again */
        e->cra = -1; e->use_cf = false;
        apply_filter(e);
        reply(e, "OK");
    } else if (IS("MA")) monitor(e);
    else reply(e, "OK");                             /* AL, NL, AT0..2, M0/1, R0/1, D0/1, SS, IB.., PP.., BRD ...: accepted, no effect */
#undef IS
#undef PFX
}

static void handle(elm_t *e, char *line)
{
    if (!line[0]) {
        if (!e->last[0]) { end_cmd(e); return; }
        snprintf(line, 2 * REQ_MAX + 4, "%s", e->last);               /* an empty line repeats the previous command */
    } else {
        snprintf(e->last, sizeof(e->last), "%s", line);
    }
    ESP_LOGI(TAG, "> %s", line);
    if (line[0] == 'A' && line[1] == 'T') {
        at_cmd(e, line + 2);
    } else {
        int len = (int)strlen(line);
        e->want = 0;
        if ((len & 1) && len >= 3 && hexv(line[len - 1]) >= 0) {      /* ELM327 v1.3+: "0100" + "1" */
            e->want = hexv(line[len - 1]);
            line[--len] = 0;
        }
        bool ok = len >= 2 && !(len & 1);
        for (int i = 0; i < len && ok; i++) ok = hexv(line[i]) >= 0;
        if (!ok) {
            reply(e, "?");
        } else {
            uint8_t req[REQ_MAX];
            const int n = len / 2;
            if (n > (int)sizeof(req)) { reply(e, "?"); end_cmd(e); return; }
            for (int i = 0; i < n; i++) req[i] = hexv(line[2 * i]) << 4 | hexv(line[2 * i + 1]);
            ESP_LOGD(TAG, "request %s", line);
            transact(e, req, n);
        }
    }
    end_cmd(e);
}

void elm_session(int sock, const elm_io_t *io, volatile bool *stop)
{
    elm_t *e = calloc(1, sizeof(*e));
    if (!e) return;
    e->sock = sock; e->io = io; e->stop = stop;
    reset_state(e);
    char line[2 * REQ_MAX + 4];
    int n = 0;
    while (!*stop) {
        char buf[64];
        const int r = recv(sock, buf, sizeof(buf), 0);
        if (r == 0) break;
        if (r < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) continue;
            break;
        }
        for (int i = 0; i < r; i++) {
            const char c = buf[i];
            if (c == '\n') continue;
            if (e->echo) { char s[2] = { c, 0 }; put(e, s); }
            if (c == '\r') {
                line[n] = 0;
                handle(e, line);
                n = 0;
            } else if (n < (int)sizeof(line) - 1 && c != ' ') {
                line[n++] = toupper((unsigned char)c);
            }
        }
        out_flush(e);
    }
    free(e);
}
