// Dump USB configuration descriptors of E-MU 0202 (041e:3f02) via libusb.
// Read-only: does not open or claim the device.
#include <stdio.h>
#include <libusb.h>

static void hex(const unsigned char *p, int n) {
    // split concatenated class-specific descriptors, one per line
    int i = 0;
    while (i < n) {
        int len = p[i];
        if (len == 0 || i + len > n) len = n - i;
        printf("  X");
        for (int j = 0; j < len; j++) printf(" %02x", p[i + j]);
        printf("\n");
        i += len;
    }
}

int main(void) {
    libusb_context *ctx;
    if (libusb_init(&ctx)) { fprintf(stderr, "libusb_init failed\n"); return 1; }
    libusb_device **list;
    ssize_t n = libusb_get_device_list(ctx, &list);
    int found = 0;
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        libusb_get_device_descriptor(list[i], &dd);
        if (dd.idVendor != 0x041e || (dd.idProduct & 0xff00) != 0x3f00) continue;
        found = 1;
        printf("DEV %04x:%04x bcdUSB=%04x class=%d/%d/%d maxpkt0=%d nconf=%d\n",
               dd.idVendor, dd.idProduct, dd.bcdUSB, dd.bDeviceClass,
               dd.bDeviceSubClass, dd.bDeviceProtocol, dd.bMaxPacketSize0,
               dd.bNumConfigurations);
        struct libusb_config_descriptor *cfg;
        if (libusb_get_active_config_descriptor(list[i], &cfg)) {
            printf("cannot get config\n"); continue;
        }
        printf("CFG value=%d attr=%02x maxpower=%d numif=%d\n",
               cfg->bConfigurationValue, cfg->bmAttributes, cfg->MaxPower, cfg->bNumInterfaces);
        if (cfg->extra_length) hex(cfg->extra, cfg->extra_length);
        for (int k = 0; k < cfg->bNumInterfaces; k++) {
            const struct libusb_interface *itf = &cfg->interface[k];
            for (int a = 0; a < itf->num_altsetting; a++) {
                const struct libusb_interface_descriptor *d = &itf->altsetting[a];
                printf("IF %d alt=%d class=%d/%d/%d neps=%d\n", d->bInterfaceNumber,
                       d->bAlternateSetting, d->bInterfaceClass, d->bInterfaceSubClass,
                       d->bInterfaceProtocol, d->bNumEndpoints);
                if (d->extra_length) hex(d->extra, d->extra_length);
                for (int e = 0; e < d->bNumEndpoints; e++) {
                    const struct libusb_endpoint_descriptor *ep = &d->endpoint[e];
                    printf(" EP addr=%02x attr=%02x maxpkt=%04x interval=%d refresh=%d synch=%02x\n",
                           ep->bEndpointAddress, ep->bmAttributes, ep->wMaxPacketSize,
                           ep->bInterval, ep->bRefresh, ep->bSynchAddress);
                    if (ep->extra_length) hex(ep->extra, ep->extra_length);
                }
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    if (!found) printf("E-MU device not found\n");
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return 0;
}
