// emuplay — proof of concept: stream audio to E-MU 0202 USB from user space via libusb.
// Default: streams SILENCE and reports the device's feedback (actual clock rate).
// --tone: plays a quiet 440 Hz sine (-30 dBFS).
//
// Build: clang -O2 -o emuplay emuplay.c -I/opt/homebrew/include/libusb-1.0 -L/opt/homebrew/lib -lusb-1.0
// Usage: ./emuplay [--rate 44100|48000|88200|96000] [--seconds N] [--tone]

#include <libusb.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define VID 0x041e
#define PID 0x3f02
#define IF_AC 0
#define IF_OUT 1
#define XU_UNIT 0x0c       // Extension Unit, wExtensionCode 0xE301 (clock rate)
#define XU_CS_RATE 0x03    // control selector: clock rate
#define EP_OUT 0x01        // iso OUT, async
#define EP_FB 0x81         // iso IN, explicit feedback
#define FRAME_BYTES 6      // 2 channels * 24-bit

#define OUT_XFERS 4
#define OUT_PKTS 8         // packets per transfer; one packet = 1 ms (bInterval 4 at HS)
#define FB_XFERS 2
#define FB_PKTS 8

static const struct { int rate, alt, xu_id, maxpkt; } RATES[] = {
    {44100, 1, 0, 0x112},
    {48000, 3, 1, 0x12a},
    {88200, 5, 2, 0x21a},
    {96000, 7, 3, 0x24a},
};

static volatile int stop_flag = 0;
static int g_rate, g_maxpkt, g_tone;
static double g_spp;          // samples per 1 ms packet, from feedback
static double g_acc = 0, g_phase = 0;
static int g_inflight = 0;
static long g_out_pkts = 0, g_out_errs = 0, g_fb_pkts = 0, g_fb_errs = 0;
static unsigned g_fb_raw_min = 0xffffffff, g_fb_raw_max = 0, g_fb_raw_last = 0;
static double g_fb_rate_min = 1e9, g_fb_rate_max = 0;
static int g_fb_mode = 0;     // 0 unknown, 8 = per microframe, 1 = per frame

static void on_sigint(int s) { (void)s; stop_flag = 1; }

static void put24(unsigned char *p, int v) {
    p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff;
}

static int fill_packet(unsigned char *buf) {
    g_acc += g_spp;
    int n = (int)g_acc;
    g_acc -= n;
    int maxn = g_maxpkt / FRAME_BYTES;
    if (n > maxn) n = maxn;
    for (int i = 0; i < n; i++) {
        int s = 0;
        if (g_tone) {
            s = (int)(sin(g_phase) * 0.0316 * 8388607.0);  // -30 dBFS
            g_phase += 2 * M_PI * 440.0 / g_rate;
            if (g_phase > 2 * M_PI) g_phase -= 2 * M_PI;
        }
        put24(buf + i * FRAME_BYTES, s);
        put24(buf + i * FRAME_BYTES + 3, s);
    }
    return n * FRAME_BYTES;
}

static void LIBUSB_CALL out_cb(struct libusb_transfer *t) {
    if (t->status != LIBUSB_TRANSFER_COMPLETED) g_out_errs++;
    for (int i = 0; i < t->num_iso_packets; i++) {
        g_out_pkts++;
        if (t->iso_packet_desc[i].status != LIBUSB_TRANSFER_COMPLETED) g_out_errs++;
    }
    if (stop_flag || t->status == LIBUSB_TRANSFER_NO_DEVICE) { g_inflight--; return; }
    int off = 0;
    for (int i = 0; i < t->num_iso_packets; i++) {
        int len = fill_packet(t->buffer + off);
        t->iso_packet_desc[i].length = len;
        off += len;
    }
    t->length = off;
    if (libusb_submit_transfer(t)) { g_out_errs++; g_inflight--; }
}

static void LIBUSB_CALL fb_cb(struct libusb_transfer *t) {
    if (t->status != LIBUSB_TRANSFER_COMPLETED) g_fb_errs++;
    for (int i = 0; i < t->num_iso_packets; i++) {
        struct libusb_iso_packet_descriptor *d = &t->iso_packet_desc[i];
        if (d->status != LIBUSB_TRANSFER_COMPLETED || d->actual_length < 3) continue;
        unsigned char *p = libusb_get_iso_packet_buffer_simple(t, i);
        unsigned raw = p[0] | (p[1] << 8) | (p[2] << 16) | (d->actual_length >= 4 ? (unsigned)p[3] << 24 : 0);
        if (!raw) continue;
        g_fb_pkts++;
        g_fb_raw_last = raw;
        if (raw < g_fb_raw_min) g_fb_raw_min = raw;
        if (raw > g_fb_raw_max) g_fb_raw_max = raw;
        double v = raw / 65536.0, nominal = g_rate / 1000.0;
        if (!g_fb_mode) g_fb_mode = (fabs(v * 8 - nominal) < fabs(v - nominal)) ? 8 : 1;
        double spp = v * g_fb_mode;
        if (fabs(spp - nominal) < nominal * 0.02) {  // sanity: within 2 %
            g_spp = spp;
            double r = spp * 1000.0;
            if (r < g_fb_rate_min) g_fb_rate_min = r;
            if (r > g_fb_rate_max) g_fb_rate_max = r;
        }
    }
    if (stop_flag || t->status == LIBUSB_TRANSFER_NO_DEVICE) { g_inflight--; return; }
    if (libusb_submit_transfer(t)) { g_fb_errs++; g_inflight--; }
}

static int xu_get(libusb_device_handle *h) {
    unsigned char v = 0xff;
    int r = libusb_control_transfer(h, 0xA1, 0x81 /*GET_CUR*/, XU_CS_RATE << 8,
                                    (XU_UNIT << 8) | IF_AC, &v, 1, 1000);
    return r == 1 ? v : -1000 + r;
}

static int xu_set(libusb_device_handle *h, unsigned char id) {
    return libusb_control_transfer(h, 0x21, 0x01 /*SET_CUR*/, XU_CS_RATE << 8,
                                   (XU_UNIT << 8) | IF_AC, &id, 1, 1000);
}

int main(int argc, char **argv) {
    int rate = 48000, seconds = 3;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tone")) g_tone = 1;
        else { fprintf(stderr, "usage: %s [--rate R] [--seconds N] [--tone]\n", argv[0]); return 2; }
    }
    int ri = -1;
    for (int i = 0; i < (int)(sizeof RATES / sizeof RATES[0]); i++) if (RATES[i].rate == rate) ri = i;
    if (ri < 0) { fprintf(stderr, "unsupported rate %d\n", rate); return 2; }
    g_rate = rate; g_maxpkt = RATES[ri].maxpkt; g_spp = rate / 1000.0;
    signal(SIGINT, on_sigint);

    libusb_context *ctx;
    if (libusb_init(&ctx)) { fprintf(stderr, "libusb_init failed\n"); return 1; }
    libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!h) { fprintf(stderr, "cannot open E-MU 0202\n"); return 1; }
    int r;
    if ((r = libusb_claim_interface(h, IF_AC))) fprintf(stderr, "claim IF0: %s (continuing)\n", libusb_error_name(r));
    if ((r = libusb_claim_interface(h, IF_OUT))) { fprintf(stderr, "claim IF1: %s\n", libusb_error_name(r)); return 1; }

    printf("clock-rate XU before: %d\n", xu_get(h));
    r = xu_set(h, RATES[ri].xu_id);
    printf("set clock-rate XU to %d (%d Hz): %s\n", RATES[ri].xu_id, rate, r == 1 ? "ok" : libusb_error_name(r));
    printf("clock-rate XU after:  %d\n", xu_get(h));

    if ((r = libusb_set_interface_alt_setting(h, IF_OUT, RATES[ri].alt))) {
        fprintf(stderr, "set alt %d: %s\n", RATES[ri].alt, libusb_error_name(r)); return 1;
    }
    printf("IF1 alt %d selected, streaming %s for %d s...\n", RATES[ri].alt, g_tone ? "440 Hz tone (-30 dBFS)" : "silence", seconds);

    struct libusb_transfer *fb[FB_XFERS], *out[OUT_XFERS];
    for (int i = 0; i < FB_XFERS; i++) {
        fb[i] = libusb_alloc_transfer(FB_PKTS);
        unsigned char *b = calloc(FB_PKTS, 4);
        libusb_fill_iso_transfer(fb[i], h, EP_FB, b, FB_PKTS * 4, FB_PKTS, fb_cb, NULL, 1000);
        libusb_set_iso_packet_lengths(fb[i], 4);
        if ((r = libusb_submit_transfer(fb[i]))) fprintf(stderr, "submit fb: %s\n", libusb_error_name(r));
        else g_inflight++;
    }
    for (int i = 0; i < OUT_XFERS; i++) {
        out[i] = libusb_alloc_transfer(OUT_PKTS);
        unsigned char *b = calloc(OUT_PKTS, g_maxpkt);
        libusb_fill_iso_transfer(out[i], h, EP_OUT, b, 0, OUT_PKTS, out_cb, NULL, 1000);
        int off = 0;
        for (int k = 0; k < OUT_PKTS; k++) { int len = fill_packet(b + off); out[i]->iso_packet_desc[k].length = len; off += len; }
        out[i]->length = off;
        if ((r = libusb_submit_transfer(out[i]))) fprintf(stderr, "submit out: %s\n", libusb_error_name(r));
        else g_inflight++;
    }

    struct timespec t0, now;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    struct timeval tv = {0, 100000};
    while (!stop_flag) {
        libusb_handle_events_timeout(ctx, &tv);
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec - t0.tv_sec >= seconds) stop_flag = 1;
    }
    for (int i = 0; i < 50 && g_inflight > 0; i++) libusb_handle_events_timeout(ctx, &tv);

    printf("out packets: %ld, out errors: %ld\n", g_out_pkts, g_out_errs);
    printf("feedback packets: %ld, errors: %ld, format: %s\n", g_fb_pkts, g_fb_errs,
           g_fb_mode == 8 ? "16.16 samples/microframe" : g_fb_mode == 1 ? "16.16 samples/frame" : "unknown");
    if (g_fb_pkts)
        printf("feedback raw: min 0x%08x max 0x%08x last 0x%08x -> device clock %.2f..%.2f Hz\n",
               g_fb_raw_min, g_fb_raw_max, g_fb_raw_last, g_fb_rate_min, g_fb_rate_max);

    libusb_set_interface_alt_setting(h, IF_OUT, 0);
    libusb_release_interface(h, IF_OUT);
    libusb_release_interface(h, IF_AC);
    libusb_close(h);
    libusb_exit(ctx);
    return 0;
}
