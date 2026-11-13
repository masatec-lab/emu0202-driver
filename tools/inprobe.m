// inprobe — capture from E-MU 0202 (interface 2, EP 0x82) and measure how early completed
// isochronous transactions become visible:
//   * by polling a KernelUserShared transaction list from a real-time thread,
//   * via the transaction's kernel timestamp,
//   * via the completion handler.
// Also reports input peak levels per channel.
// Build: clang -fobjc-arc -O2 -o inprobe inprobe.m -framework Foundation -framework IOKit -framework IOUSBHost
#import <Foundation/Foundation.h>
#import <IOUSBHost/IOUSBHost.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <math.h>
#include <stdatomic.h>

#define NX 8
#define KF 4
#define MAXPKT 0x12a

static double tpms;
static uint64_t anchorF, anchorT, nextFrame;
static IOUSBHostInterface *gIn;
static IOUSBHostPipe *gPipe;
static NSMutableData *gData[NX], *gTxd[NX];
static IOUSBHostIsochronousTransaction *gTx[NX];
static uint64_t gXferFrame[NX];
static _Atomic int gDone[NX];
static _Atomic int gCbCount;
static double gCbSum, gCbMax;

static uint64_t frameTime(uint64_t f) { return anchorT + (uint64_t)((int64_t)(f - anchorF) * tpms); }

static IOUSBHostInterface *openItf(int n, dispatch_queue_t q) {
    CFMutableDictionaryRef m = [IOUSBHostInterface createMatchingDictionaryWithVendorID:@0x041e productID:@0x3f02
        bcdDevice:nil interfaceNumber:@(n) configurationValue:@1 interfaceClass:nil interfaceSubclass:nil
        interfaceProtocol:nil speed:nil productIDArray:nil];
    io_service_t s = IOServiceGetMatchingService(kIOMainPortDefault, m);
    if (!s) return nil;
    NSError *e = nil;
    IOUSBHostInterface *i = [[IOUSBHostInterface alloc] initWithIOService:s options:0 queue:q error:&e interestHandler:nil];
    IOObjectRelease(s);
    if (!i) printf("open %d: %s\n", n, e.localizedDescription.UTF8String);
    return i;
}

static void submit(int i) {
    uint64_t cur = [gIn frameNumberWithTime:NULL];
    if (nextFrame <= cur + 2) nextFrame = cur + 4;
    uint64_t xf = nextFrame;
    gXferFrame[i] = xf;
    nextFrame += KF;
    atomic_store(&gDone[i], 0);
    for (int k = 0; k < KF; k++) {
        gTx[i][k].status = kIOReturnInvalid;
        gTx[i][k].requestCount = MAXPKT;
        gTx[i][k].offset = k * MAXPKT;
        gTx[i][k].completeCount = 0;
        gTx[i][k].timeStamp = 0;
        gTx[i][k].options = 0;
    }
    NSError *e = nil;
    if (![gPipe enqueueIORequestWithData:gData[i] transactionList:gTx[i] transactionListCount:KF firstFrameNumber:xf
                                 options:0 error:&e completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction *list) {
            double d = (mach_absolute_time() - frameTime(xf + KF)) / tpms;
            gCbSum += d;
            if (d > gCbMax) gCbMax = d;
            atomic_fetch_add(&gCbCount, 1);
            atomic_store(&gDone[i], 1);
        }])
        printf("enqueue: %s\n", e.localizedDescription.UTF8String);
}

int main(int argc, char **argv) {
    @autoreleasepool {
        int seconds = argc > 1 ? atoi(argv[1]) : 5;
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        tpms = 1e6 * tb.denom / tb.numer;
        dispatch_queue_t q = dispatch_queue_create("inprobe", DISPATCH_QUEUE_SERIAL);

        IOUSBHostInterface *ac = openItf(0, q);
        gIn = openItf(2, q);
        if (!ac || !gIn) return 1;
        NSError *err = nil;
        uint8_t id = 1;  // 48 kHz
        IOUSBDeviceRequest req = {0x21, 0x01, 0x0300, 0x0c00, 1};
        if (![ac sendDeviceRequest:req data:[NSMutableData dataWithBytes:&id length:1] bytesTransferred:NULL completionTimeout:1 error:&err])
            printf("set rate: %s\n", err.localizedDescription.UTF8String);
        if (![gIn selectAlternateSetting:3 error:&err]) { printf("alt: %s\n", err.localizedDescription.UTF8String); return 1; }
        gPipe = [gIn copyPipeWithAddress:0x82 error:&err];
        if (!gPipe) { printf("pipe: %s\n", err.localizedDescription.UTF8String); return 1; }
        for (int i = 0; i < NX; i++) {
            gData[i] = [gIn ioDataWithCapacity:KF * MAXPKT error:&err];
            gTxd[i] = [gIn dataWithCapacity:KF * sizeof(IOUSBHostIsochronousTransaction) options:IOUSBHostObjectDataOptionsKernelUserShared error:&err];
            if (!gData[i] || !gTxd[i]) { printf("alloc: %s\n", err.localizedDescription.UTF8String); return 1; }
            gTx[i] = (IOUSBHostIsochronousTransaction *)gTxd[i].mutableBytes;
        }

        uint64_t p = [gIn frameNumberWithTime:NULL], c = p, t = 0;
        while (c == p) { c = [gIn frameNumberWithTime:NULL]; t = mach_absolute_time(); }
        anchorF = c;
        anchorT = t;
        nextFrame = c + 8;

        thread_time_constraint_policy_data_t pol = {(uint32_t)(tpms / 4), (uint32_t)(tpms * 0.05), (uint32_t)(tpms * 0.15), 1};
        thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol, THREAD_TIME_CONSTRAINT_POLICY_COUNT);

        for (int i = 0; i < NX; i++) submit(i);
        int seen[NX][KF];
        memset(seen, 0, sizeof seen);
        double pollSum = 0, pollMax = 0, tsSum = 0, tsMax = 0;
        long pollN = 0, bytes = 0, badPkts = 0;
        int sizes[400] = {0};
        int32_t peakL = 0, peakR = 0;
        uint64_t end = mach_absolute_time() + (uint64_t)(seconds * 1000 * tpms);
        while (mach_absolute_time() < end) {
            uint64_t now = mach_absolute_time();
            for (int i = 0; i < NX; i++) {
                for (int k = 0; k < KF; k++) {
                    if (seen[i][k] || gTx[i][k].status == kIOReturnInvalid) continue;
                    seen[i][k] = 1;
                    double frameEnd = frameTime(gXferFrame[i] + k + 1);
                    double d = (now - frameEnd) / tpms;
                    pollSum += d;
                    if (d > pollMax) pollMax = d;
                    if (gTx[i][k].timeStamp) {
                        double ts = (gTx[i][k].timeStamp - frameEnd) / tpms;
                        tsSum += ts;
                        if (ts > tsMax) tsMax = ts;
                    }
                    pollN++;
                    if (gTx[i][k].status != kIOReturnSuccess) badPkts++;
                    uint32_t n = gTx[i][k].completeCount;
                    bytes += n;
                    if (n < 400) sizes[n]++;
                    const uint8_t *b = (const uint8_t *)gData[i].bytes + k * MAXPKT;
                    for (uint32_t s = 0; s + 6 <= n; s += 6) {
                        int32_t l = (int32_t)((b[s] << 8) | (b[s + 1] << 16) | ((uint32_t)b[s + 2] << 24)) >> 8;
                        int32_t r = (int32_t)((b[s + 3] << 8) | (b[s + 4] << 16) | ((uint32_t)b[s + 5] << 24)) >> 8;
                        if (abs(l) > peakL) peakL = abs(l);
                        if (abs(r) > peakR) peakR = abs(r);
                    }
                }
                int all = 1;
                for (int k = 0; k < KF; k++) all &= seen[i][k];
                if (all && atomic_load(&gDone[i])) {
                    memset(seen[i], 0, sizeof seen[i]);
                    submit(i);
                }
            }
            mach_wait_until(now + (uint64_t)(tpms / 4));  // poll every 250 us
        }
        [gPipe abortWithOption:IOUSBHostAbortOptionSynchronous error:&err];
        [gIn selectAlternateSetting:0 error:&err];
        usleep(50000);
        printf("packets seen by polling: %ld (errors %ld), %.2f samples/packet\n", pollN, badPkts, bytes / 6.0 / (pollN ? pollN : 1));
        printf("packet sizes:");
        for (int s = 0; s < 400; s++) if (sizes[s]) printf(" %dB x%d", s, sizes[s]);
        printf("\n");
        printf("visible by polling: avg %.2f ms, max %.2f ms after frame end\n", pollSum / (pollN ? pollN : 1), pollMax);
        printf("kernel timestamp:   avg %.2f ms, max %.2f ms after frame end\n", tsSum / (pollN ? pollN : 1), tsMax);
        int n = atomic_load(&gCbCount);
        printf("completion handler: avg %.2f ms, max %.2f ms after transfer end (%d handlers)\n", n ? gCbSum / n : 0, gCbMax, n);
        printf("input peak: L %.1f dBFS, R %.1f dBFS\n", 20 * log10((peakL + 1) / 8388608.0), 20 * log10((peakR + 1) / 8388608.0));
        [gIn destroy];
        [ac destroy];
    }
    return 0;
}
