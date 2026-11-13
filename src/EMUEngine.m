// EMUEngine.m — IOUSBHost implementation of the E-MU 0202 USB streaming engine (full duplex).
//
// Protocol (see docs/descriptors.txt): USB Audio Class 1.0 at high speed, interfaces masked as
// vendor class 0xFF. Interfaces 1 (playback) and 2 (capture) share alternate setting numbers:
// 1/3/5/7 = 44.1/48/88.2/96 kHz, 1 packet per 1 ms frame, 24-bit stereo.
//   playback: iso OUT EP 0x01 (async) + explicit feedback EP 0x81 (16.16 samples per frame)
//   capture:  iso IN  EP 0x82 (async, implicit feedback)
// The card's clock is set through Extension Unit 12 (code 0xE301), control selector 3.
//
// Threads:
//   * enqueuer (high priority): keeps OUT and IN transfers enqueued ENQ_AHEAD frames ahead. Only
//     this thread makes IOUSBHost calls during streaming; they can block for ~10 ms, which the
//     long lead absorbs. Packet sizes and timeline positions are decided here.
//   * pump (real time, no system calls): wakes once per frame, writes audio into OUT packets
//     FILL_AHEAD frames before they go on the wire (the buffers are mapped for DMA, so the
//     controller reads them at transmission time), publishes zero timestamps, and polls the IN
//     transaction lists, which live in memory shared with the kernel: a finished IN packet is
//     visible ~0.2 ms after its frame.
//   * ioQueue: completion handlers (late by up to ~15 ms) only return buffers to their pools;
//     feedback values; hot-plug notifications.
//
// Timing model: frames have exact host times t(F) (the USB frame clock runs off the same crystal
// as mach_absolute_time; a transaction for frame F crosses the wire during [t(F) - 1 ms, t(F))).
// Timeline sample S sent in frame F gets host time t(F) - LEAD. Captured packets are placed on the
// same timeline at the moment they were sampled.

#import <Foundation/Foundation.h>
#import <IOUSBHost/IOUSBHost.h>
#include <IOKit/IOKitLib.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <math.h>
#include <os/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include "EMUEngine.h"

#define EMU_VID 0x041e
#define EMU_PID 0x3f02
#define XU_UNIT 0x0c
#define XU_CS_RATE 0x03
#define EP_OUT 0x01
#define EP_FB 0x81
#define EP_IN 0x82
#define FRAME_BYTES 6         // 2 ch * 24 bit
#define NSLOT 32              // OUT transfers in the pool
#define KPKT 2                // 1 ms packets per OUT transfer
#define NIN 32                // IN transfers in the pool
#define KIN 2                 // 1 ms packets per IN transfer
#define ENQ_AHEAD 24          // keep transfers enqueued this many frames ahead
#define ENQ_PERIOD_MS 2       // enqueuer wake period
#define FILL_AHEAD 3          // the pump writes OUT audio this many frames before the frame
#define NFB 4                 // feedback transfers in flight
#define KFB 8                 // frames per feedback transfer
#define START_LEAD 8          // first feedback frame, relative to now
#define WARMUP_FRAMES 16      // first OUT/IN frame, relative to now
#define LEAD_MS (FILL_AHEAD + 1)
#define WIRE_EARLY_MS 1       // frame F is on the wire during [t(F) - 1 ms, t(F))
#define OUT_SAFETY_MS 2
#define IN_SAFETY_MS 2.75      // a buffer may end on the first sample of a packet that is visible ~2.34 ms later
#define FT_SIZE 128           // frame table entries (power of two, > ENQ_AHEAD + KPKT)
#define PRIME_MS 300          // silence streamed before the restart that follows a clock rate change
#define RING_MASK (EMU_RING_FRAMES - 1)

typedef struct { double rate; int alt; int xu; int maxPacket; } RateInfo;
static const RateInfo kRates[] = {
    {44100.0, 1, 0, 0x112},
    {48000.0, 3, 1, 0x12a},
    {88200.0, 5, 2, 0x21a},
    {96000.0, 7, 3, 0x24a},
};

static const RateInfo *findRate(double r) {
    for (size_t i = 0; i < sizeof kRates / sizeof kRates[0]; i++)
        if (fabs(kRates[i].rate - r) < 1.0) return &kRates[i];
    return NULL;
}

static os_log_t elog(void) {
    static os_log_t l;
    static dispatch_once_t once;
    dispatch_once(&once, ^{ l = os_log_create("ru.masatec.emu0202", "engine"); });
    return l;
}

typedef struct {
    uint64_t frame;   // validity tag
    uint64_t pos;     // timeline sample of the packet's first sample
    uint32_t n;       // samples in the packet
    int32_t slot;     // OUT slot holding the packet, -1 when the frame was skipped
    uint32_t offset;  // byte offset of the packet in the slot buffer
} FrameEntry;

enum { IN_FREE = 0, IN_QUEUED = 1, IN_CONSUMED = 2 };

struct EMUEngine {
    float *ring;                      // output: EMU_RING_FRAMES * 2 floats, indexed by timeline sample
    float *inRing;                    // input:  same layout
    _Atomic uint32_t ztsSeq;          // seqlock for the zero timestamp
    double ztsSample;
    uint64_t ztsHost;
    uint64_t ztsSeed;
    _Atomic uint64_t devPos;          // output consumed (copied to packets) up to this timeline sample
    _Atomic uint64_t writtenEnd;      // end of the highest range Core Audio has written
    _Atomic uint64_t inAvail;         // captured samples are valid up to (excluding) this timeline sample
    _Atomic uint64_t stPackets, stErrors, stResyncs, stFb, stUnderruns, stLateWrites, stSlow, stLateFills;
    _Atomic uint64_t stInPackets, stInErrors, stInRealigns, stInUnderruns, stInShortMax;
    _Atomic double stRate, stMaxDelay, stInPickupMax;
    _Atomic bool present, running;
    _Atomic float outGainTarget;      // output gain set by the volume/mute controls (linear)
    float outGainApplied;             // gain at the end of the last written buffer; Core Audio IO thread only
    void *impl;
};

static void ztsWrite(EMUEngine *rt, double sample, uint64_t host, bool newSeed) {
    atomic_fetch_add_explicit(&rt->ztsSeq, 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    rt->ztsSample = sample;
    rt->ztsHost = host;
    if (newSeed) rt->ztsSeed++;
    atomic_fetch_add_explicit(&rt->ztsSeq, 1, memory_order_release);
}

static inline int32_t f2i24(float x) {
    if (!(x == x)) x = 0.f;
    if (x > 1.f) x = 1.f;
    else if (x < -1.f) x = -1.f;
    return (int32_t)lrintf(x * 8388607.0f);
}

static inline void put24(uint8_t *p, int32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

static inline float get24(const uint8_t *p) {
    int32_t v = (int32_t)(((uint32_t)p[0] << 8) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 24)) >> 8;
    return (float)v * (1.0f / 8388608.0f);
}

static bool isFatal(IOReturn s) {
    return s == kIOReturnNoDevice || s == kIOReturnNotAttached || s == kIOReturnOffline || s == kIOReturnNotOpen;
}

static void setRealtime(double ticksPerMs, double periodMs, double computationMs, double constraintMs) {
    thread_time_constraint_policy_data_t pol;
    pol.period = (uint32_t)(periodMs * ticksPerMs);
    pol.computation = (uint32_t)(computationMs * ticksPerMs);
    pol.constraint = (uint32_t)(constraintMs * ticksPerMs);
    pol.preemptible = 1;
    kern_return_t kr = thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol,
                                         THREAD_TIME_CONSTRAINT_POLICY_COUNT);
    if (kr) os_log_error(elog(), "real-time policy failed: 0x%x", kr);
}

@interface EMUEngineImpl : NSObject {
@public
    EMUEngine *rt;
    dispatch_queue_t ioQueue;
    pthread_mutex_t ctlMutex;       // serializes start/stop
    IONotificationPortRef notifyPort;
    io_iterator_t addIter, remIter;
    EMUPresenceCallback presenceCb;
    void *presenceCtx;
    double ticksPerSec, ticksPerFrame;

    IOUSBHostInterface *acItf, *outItf, *inItf;
    IOUSBHostPipe *outPipe, *fbPipe, *inPipe;
    const RateInfo *ri;
    const RateInfo *primedRate;      // the card has streamed at this rate since its clock was last set (ctlMutex)
    int primedConnection;
    _Atomic int connectionId;        // incremented every time the card is connected

    // OUT pool
    NSMutableData *outData[NSLOT];
    uint8_t *outBase[NSLOT];
    IOUSBHostIsochronousTransaction outTx[NSLOT][KPKT];
    uint64_t outFrame[NSLOT];
    _Atomic int slotBusy[NSLOT];
    FrameEntry ft[FT_SIZE];
    _Atomic uint64_t ftEnd;          // frames below this have valid entries
    // IN pool (transaction lists shared with the kernel)
    NSMutableData *inData[NIN];
    const uint8_t *inBase[NIN];
    NSMutableData *inTxData[NIN];
    IOUSBHostIsochronousTransaction *inTx[NIN];
    IOUSBHostIsochronousTransaction inTxLocal[NIN][KIN];
    uint64_t inFrame[NIN];
    _Atomic int inState[NIN];
    _Atomic int inPending[NIN];      // 1 until the completion handler has run
    int inFifo[NIN];
    _Atomic uint32_t inTail;         // written by the enqueuer
    uint32_t inHead;                 // pump
    int inConsumed;                  // pump: transactions consumed in the head slot
    // feedback
    NSMutableData *fbData[NFB];
    IOUSBHostIsochronousTransaction fbTx[NFB][KFB];

    pthread_t pumpThread, enqThread;
    _Atomic bool threadsRun, stopping, dead;
    _Atomic int inflight;
    _Atomic double sppFeedback;      // samples per frame from the card, 0 until known
    uint64_t anchorFrame;
    _Atomic uint64_t anchorTime;

    // enqueuer state
    uint64_t nextOutFrame, nextInFrame, schedPos, lastStats;
    double accEnq;
    int slotCursor;
    // pump state
    uint64_t fillFrame, nextZts, leadTicks, expectedInFrame, inPos;
    BOOL inPosValid;
    // ioQueue state
    uint64_t nextFbFrame;
}
@end

static void onDeviceAdded(void *refcon, io_iterator_t it);
static void onDeviceRemoved(void *refcon, io_iterator_t it);
static void *pumpMain(void *arg);
static void *enqMain(void *arg);

@implementation EMUEngineImpl

- (instancetype)init {
    if ((self = [super init])) {
        dispatch_queue_attr_t attr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, -1);
        ioQueue = dispatch_queue_create("ru.masatec.emu0202.io", attr);
        pthread_mutex_init(&ctlMutex, NULL);
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        ticksPerSec = 1e9 * tb.denom / tb.numer;
        ticksPerFrame = ticksPerSec / 1000.0;
    }
    return self;
}

#pragma mark Presence

- (CFMutableDictionaryRef)newDeviceMatching {
    CFMutableDictionaryRef m = IOServiceMatching("IOUSBHostDevice");
    int v = EMU_VID, p = EMU_PID;
    CFNumberRef nv = CFNumberCreate(NULL, kCFNumberIntType, &v);
    CFNumberRef np = CFNumberCreate(NULL, kCFNumberIntType, &p);
    CFDictionarySetValue(m, CFSTR("idVendor"), nv);
    CFDictionarySetValue(m, CFSTR("idProduct"), np);
    CFRelease(nv);
    CFRelease(np);
    return m;
}

- (void)startMonitoringWithCallback:(EMUPresenceCallback)cb ctx:(void *)ctx {
    presenceCb = cb;
    presenceCtx = ctx;
    notifyPort = IONotificationPortCreate(kIOMainPortDefault);
    IONotificationPortSetDispatchQueue(notifyPort, ioQueue);
    kern_return_t kr1 = IOServiceAddMatchingNotification(notifyPort, kIOFirstMatchNotification, [self newDeviceMatching],
                                                         onDeviceAdded, (__bridge void *)self, &addIter);
    kern_return_t kr2 = IOServiceAddMatchingNotification(notifyPort, kIOTerminatedNotification, [self newDeviceMatching],
                                                         onDeviceRemoved, (__bridge void *)self, &remIter);
    if (kr1 || kr2) os_log_error(elog(), "IOServiceAddMatchingNotification failed: 0x%x 0x%x", kr1, kr2);
    dispatch_async(ioQueue, ^{
        onDeviceAdded((__bridge void *)self, self->addIter);
        onDeviceRemoved((__bridge void *)self, self->remIter);
    });
}

- (void)setPresent:(BOOL)p {  // ioQueue
    if (atomic_load(&rt->present) == p) return;
    atomic_store(&rt->present, p);
    if (!p) atomic_store(&dead, true);
    else atomic_fetch_add(&connectionId, 1);
    os_log(elog(), "E-MU 0202 %{public}s", p ? "connected" : "disconnected");
    EMUPresenceCallback cb = presenceCb;
    void *ctx = presenceCtx;
    if (cb) dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{ cb(ctx, p); });
}

#pragma mark Start / stop

- (io_service_t)copyInterfaceService:(int)num {
    CFMutableDictionaryRef m = [IOUSBHostInterface createMatchingDictionaryWithVendorID:@(EMU_VID) productID:@(EMU_PID)
        bcdDevice:nil interfaceNumber:@(num) configurationValue:@1 interfaceClass:nil interfaceSubclass:nil
        interfaceProtocol:nil speed:nil productIDArray:nil];
    return IOServiceGetMatchingService(kIOMainPortDefault, m);  // consumes m
}

- (IOUSBHostInterface *)openInterface:(int)num error:(NSError **)err {
    io_service_t s = 0;
    for (int tries = 0; tries < 20 && !s; tries++) {  // interfaces appear shortly after the device
        s = [self copyInterfaceService:num];
        if (!s) usleep(100000);
    }
    if (!s) return nil;
    IOUSBHostInterface *itf = [[IOUSBHostInterface alloc] initWithIOService:s options:IOUSBHostObjectInitOptionsNone
                                                                       queue:ioQueue error:err interestHandler:nil];
    IOObjectRelease(s);
    return itf;
}

- (void)closeInterfaces {
    outPipe = nil;
    fbPipe = nil;
    inPipe = nil;
    for (int i = 0; i < NSLOT; i++) { outData[i] = nil; outBase[i] = NULL; }
    for (int i = 0; i < NIN; i++) { inData[i] = nil; inBase[i] = NULL; inTxData[i] = nil; inTx[i] = NULL; }
    for (int i = 0; i < NFB; i++) fbData[i] = nil;
    [inItf destroy];
    [outItf destroy];
    [acItf destroy];
    inItf = nil;
    outItf = nil;
    acItf = nil;
}

- (int)fail:(int)code what:(const char *)what error:(NSError *)err {
    os_log_error(elog(), "start: %{public}s: %{public}@", what, err);
    [self closeInterfaces];
    return code;
}

- (int)startLocked:(double)rate {
    const RateInfo *r = findRate(rate);
    if (!r) return -2;
    if (atomic_load(&rt->running)) {
        if (ri == r && !atomic_load(&dead)) return 0;
        [self stopLocked];
    }
    // The first stream after the card's clock rate changed (or after it was connected) can settle into a
    // different buffering state inside the card: 1-4 ms more or 1 ms less round trip than every later start
    // at that rate (tools/engloop --switch). A short stream of silence and a restart puts it into the usual
    // state, which the reported latency is calibrated for.
    int connection = atomic_load(&connectionId);
    BOOL needPrime = primedRate != r || primedConnection != connection;
    int res = [self openAndStream:r];
    if (res || !needPrime) return res;
    usleep(PRIME_MS * 1000);
    [self stopLocked];
    res = [self openAndStream:r];
    if (res) return res;
    primedRate = r;
    primedConnection = connection;
    os_log(elog(), "first stream at %.0f Hz since the clock changed: restarted after %d ms of silence", r->rate, PRIME_MS);
    return 0;
}

- (int)openAndStream:(const RateInfo *)r {
    NSError *err = nil;
    acItf = [self openInterface:0 error:&err];
    if (acItf) outItf = [self openInterface:1 error:&err];
    if (outItf) inItf = [self openInterface:2 error:&err];
    if (!inItf) return [self fail:-3 what:"cannot open interfaces" error:err];

    uint8_t rateId = (uint8_t)r->xu;
    IOUSBDeviceRequest req = {.bmRequestType = 0x21, .bRequest = 0x01, .wValue = XU_CS_RATE << 8, .wIndex = (XU_UNIT << 8) | 0, .wLength = 1};
    NSMutableData *d = [NSMutableData dataWithBytes:&rateId length:1];
    NSUInteger done = 0;
    if (![acItf sendDeviceRequest:req data:d bytesTransferred:&done completionTimeout:1.0 error:&err])
        return [self fail:-5 what:"set clock rate" error:err];
    if (primedRate != r) primedRate = NULL;  // the card's clock may have just changed
    if (![outItf selectAlternateSetting:r->alt error:&err]) return [self fail:-6 what:"select playback alt" error:err];
    if (![inItf selectAlternateSetting:r->alt error:&err]) return [self fail:-6 what:"select capture alt" error:err];
    outPipe = [outItf copyPipeWithAddress:EP_OUT error:&err];
    if (outPipe) fbPipe = [outItf copyPipeWithAddress:EP_FB error:&err];
    if (fbPipe) inPipe = [inItf copyPipeWithAddress:EP_IN error:&err];
    if (!inPipe) return [self fail:-7 what:"pipes not available" error:err];

    for (int i = 0; i < NSLOT && !err; i++) {
        outData[i] = [outItf ioDataWithCapacity:KPKT * r->maxPacket error:&err];
        outBase[i] = (uint8_t *)outData[i].mutableBytes;
    }
    for (int i = 0; i < NFB && !err; i++) fbData[i] = [outItf ioDataWithCapacity:KFB * 4 error:&err];
    for (int i = 0; i < NIN && !err; i++) {
        inData[i] = [inItf ioDataWithCapacity:KIN * r->maxPacket error:&err];
        inBase[i] = (const uint8_t *)inData[i].bytes;
        inTx[i] = inTxLocal[i];
        if (@available(macOS 26.4, *)) {
            NSError *e2 = nil;
            inTxData[i] = [inItf dataWithCapacity:KIN * sizeof(IOUSBHostIsochronousTransaction)
                                          options:IOUSBHostObjectDataOptionsKernelUserShared error:&e2];
            if (inTxData[i]) inTx[i] = (IOUSBHostIsochronousTransaction *)inTxData[i].mutableBytes;
        }
    }
    if (err) return [self fail:-8 what:"buffer allocation" error:err];
    if (!inTxData[0]) os_log(elog(), "shared transaction lists unavailable: capture latency will be higher");

    ri = r;
    leadTicks = (uint64_t)(LEAD_MS * ticksPerFrame);
    [self prime];
    atomic_store(&rt->running, true);
    os_log(elog(), "streaming started at %.0f Hz (alt %d)", r->rate, r->alt);
    return 0;
}

- (void)prime {
    atomic_store(&stopping, false);
    atomic_store(&dead, false);
    atomic_store(&inflight, 0);
    for (int i = 0; i < NSLOT; i++) atomic_store(&slotBusy[i], 0);
    for (int i = 0; i < NIN; i++) { atomic_store(&inState[i], IN_FREE); atomic_store(&inPending[i], 0); }
    for (int i = 0; i < FT_SIZE; i++) ft[i].frame = UINT64_MAX;
    atomic_store(&inTail, 0);
    inHead = 0;
    inConsumed = 0;
    memset(rt->ring, 0, EMU_RING_FRAMES * EMU_CHANNELS * sizeof(float));
    memset(rt->inRing, 0, EMU_RING_FRAMES * EMU_CHANNELS * sizeof(float));
    atomic_store(&rt->writtenEnd, 0);
    atomic_store(&rt->inAvail, 0);
    atomic_store(&sppFeedback, 0.0);
    accEnq = 0;
    slotCursor = 0;
    inPosValid = NO;
    lastStats = mach_absolute_time();

    dispatch_sync(ioQueue, ^{
        self->nextFbFrame = [self->outItf frameNumberWithTime:NULL] + START_LEAD;
        for (int i = 0; i < NFB; i++) [self submitFb:i];
    });

    // Catch a frame boundary to get an exact (frame, host time) pair.
    uint64_t prev = [outItf frameNumberWithTime:NULL], cur = prev, t = mach_absolute_time();
    for (int guard = 0; guard < 200000 && cur == prev; guard++) {
        cur = [outItf frameNumberWithTime:NULL];
        t = mach_absolute_time();
    }
    anchorFrame = cur;
    atomic_store(&anchorTime, t);

    // The first packet carries timeline sample P0 rather than 0, so that sample 0 (the first zero
    // timestamp) lies about 1 ms in the past: Core Audio expects zero timestamps <= now.
    uint64_t startFrame = cur + WARMUP_FRAMES;
    double spp0 = ri->rate / 1000.0;
    uint64_t P0 = (uint64_t)llround((WARMUP_FRAMES - LEAD_MS + 1) * spp0);
    nextOutFrame = nextInFrame = startFrame;
    fillFrame = startFrame;
    expectedInFrame = startFrame;
    atomic_store(&ftEnd, startFrame);
    schedPos = P0;
    atomic_store(&rt->devPos, P0);
    nextZts = EMU_RING_FRAMES;
    ztsWrite(rt, 0, [self frameTime:startFrame] - leadTicks - (uint64_t)(P0 / spp0 * ticksPerFrame), true);

    atomic_store(&threadsRun, true);
    pthread_create(&enqThread, NULL, enqMain, (__bridge void *)self);
    pthread_create(&pumpThread, NULL, pumpMain, (__bridge void *)self);
}

- (void)stopLocked {
    if (!acItf && !outItf && !inItf) {
        atomic_store(&rt->running, false);
        return;
    }
    if (atomic_load(&threadsRun)) {
        atomic_store(&threadsRun, false);
        pthread_join(pumpThread, NULL);
        pthread_join(enqThread, NULL);
    }
    atomic_store(&stopping, true);
    NSError *err = nil;
    [outPipe abortWithOption:IOUSBHostAbortOptionSynchronous error:&err];
    [fbPipe abortWithOption:IOUSBHostAbortOptionSynchronous error:&err];
    [inPipe abortWithOption:IOUSBHostAbortOptionSynchronous error:&err];
    for (int k = 0; k < 200 && atomic_load(&inflight) > 0; k++) usleep(5000);
    dispatch_sync(ioQueue, ^{});  // let queued completion handlers finish
    if (!atomic_load(&dead)) {
        [outItf selectAlternateSetting:0 error:&err];
        [inItf selectAlternateSetting:0 error:&err];
    }
    [self closeInterfaces];
    atomic_store(&rt->running, false);
    os_log(elog(), "streaming stopped");
}

#pragma mark Clock

- (uint64_t)frameTime:(uint64_t)frame {
    uint64_t at = atomic_load_explicit(&anchorTime, memory_order_relaxed);
    return (uint64_t)((int64_t)at + (int64_t)llround((double)((int64_t)frame - (int64_t)anchorFrame) * ticksPerFrame));
}

- (uint64_t)frameAt:(uint64_t)host {
    uint64_t at = atomic_load_explicit(&anchorTime, memory_order_relaxed);
    double d = ((double)(int64_t)(host - at)) / ticksPerFrame;
    return (uint64_t)((int64_t)anchorFrame + (int64_t)floor(d));
}

- (double)currentSpp {
    double fb = atomic_load_explicit(&sppFeedback, memory_order_relaxed);
    return fb > 0 ? fb : ri->rate / 1000.0;
}

// Enqueuer: keep the frame->time mapping honest (the frame the controller reports must contain now).
- (uint64_t)currentFrameChecked {
    IOUSBHostTime now = 0;
    uint64_t f = [outItf frameNumberWithTime:&now];
    uint64_t start = [self frameTime:f];
    uint64_t tol = (uint64_t)(0.2 * ticksPerFrame);
    if (now + tol < start) {
        atomic_fetch_sub(&anchorTime, start - now);
        os_log(elog(), "clock: frame mapping corrected by -%llu ticks", start - now);
    } else if (now > start + (uint64_t)ticksPerFrame + tol) {
        uint64_t d = now - start - (uint64_t)ticksPerFrame;
        atomic_fetch_add(&anchorTime, d);
        os_log(elog(), "clock: frame mapping corrected by +%llu ticks", d);
    }
    return f;
}

#pragma mark Enqueuer thread

- (void)enqLoop {
    pthread_setname_np("ru.masatec.emu0202.enqueue");
    setRealtime(ticksPerFrame, ENQ_PERIOD_MS, 0.5, ENQ_PERIOD_MS);
    while (atomic_load_explicit(&threadsRun, memory_order_relaxed)) {
        if (!atomic_load_explicit(&dead, memory_order_relaxed)) {
            uint64_t cur = [self currentFrameChecked];
            [self enqueueOut:cur];
            [self enqueueIn:cur];
            [self maybeLogStats];
        }
        mach_wait_until(mach_absolute_time() + (uint64_t)(ENQ_PERIOD_MS * ticksPerFrame));
    }
}

- (uint32_t)nextPacketSize:(double)spp {
    accEnq += spp;
    uint32_t n = (uint32_t)accEnq;
    accEnq -= n;
    uint32_t maxn = (uint32_t)(ri->maxPacket / FRAME_BYTES);
    return n > maxn ? maxn : n;
}

- (void)addEntry:(uint64_t)frame slot:(int)slot offset:(uint32_t)offset n:(uint32_t)n {
    FrameEntry *e = &ft[frame & (FT_SIZE - 1)];
    e->pos = schedPos;
    e->n = n;
    e->slot = slot;
    e->offset = offset;
    e->frame = frame;
    schedPos += n;
}

- (void)enqueueOut:(uint64_t)cur {
    double spp = [self currentSpp];
    if (nextOutFrame <= cur + FILL_AHEAD) {  // fell behind: the skipped frames keep the timeline continuous
        uint64_t from = nextOutFrame, to = cur + FILL_AHEAD + 2;
        while (nextOutFrame < to) {
            [self addEntry:nextOutFrame slot:-1 offset:0 n:[self nextPacketSize:spp]];
            nextOutFrame++;
        }
        atomic_store_explicit(&ftEnd, nextOutFrame, memory_order_release);
        atomic_fetch_add(&rt->stResyncs, 1);
        os_log(elog(), "out schedule late: skipped %llu frames", to - from);
    }
    while (nextOutFrame < cur + ENQ_AHEAD) {
        int slot = -1;
        for (int n = 0; n < NSLOT; n++) {
            int i = (slotCursor + n) % NSLOT;
            if (atomic_load_explicit(&slotBusy[i], memory_order_acquire) == 0) { slot = i; break; }
        }
        if (slot < 0) {
            os_log_error(elog(), "no free OUT transfer slots");
            break;
        }
        slotCursor = (slot + 1) % NSLOT;
        atomic_store(&slotBusy[slot], 1);
        uint64_t frame = nextOutFrame;
        outFrame[slot] = frame;
        memset(outBase[slot], 0, KPKT * ri->maxPacket);
        uint64_t posBefore = schedPos;
        double accBefore = accEnq;
        for (int k = 0; k < KPKT; k++) {
            uint32_t n = [self nextPacketSize:spp];
            IOUSBHostIsochronousTransaction *t = &outTx[slot][k];
            t->status = kIOReturnInvalid;
            t->requestCount = n * FRAME_BYTES;
            t->offset = (uint32_t)(k * ri->maxPacket);
            t->completeCount = 0;
            t->timeStamp = 0;
            t->options = IOUSBHostIsochronousTransactionOptionsNone;
            [self addEntry:frame + k slot:slot offset:t->offset n:n];
        }
        NSError *err = nil;
        atomic_fetch_add(&inflight, 1);
        BOOL ok = [outPipe enqueueIORequestWithData:outData[slot] transactionList:outTx[slot] transactionListCount:KPKT
                                   firstFrameNumber:frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                                  completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction *list) {
                                      [self outDone:slot status:status list:list];
                                  }];
        if (!ok) {
            atomic_fetch_sub(&inflight, 1);
            atomic_store(&slotBusy[slot], 0);
            atomic_fetch_add(&rt->stErrors, 1);
            os_log_error(elog(), "out enqueue failed (frame %llu): %{public}@", frame, err);
            if (isFatal((IOReturn)err.code)) atomic_store(&dead, true);
            // keep the frames on the timeline, without a packet
            schedPos = posBefore;
            accEnq = accBefore;
            for (int k = 0; k < KPKT; k++) [self addEntry:frame + k slot:-1 offset:0 n:[self nextPacketSize:spp]];
        }
        nextOutFrame += KPKT;
        atomic_store_explicit(&ftEnd, nextOutFrame, memory_order_release);
    }
}

- (void)outDone:(int)slot status:(IOReturn)status list:(IOUSBHostIsochronousTransaction *)list {  // ioQueue
    if (!atomic_load(&stopping)) {
        uint64_t now = mach_absolute_time(), end = [self frameTime:outFrame[slot] + KPKT];
        double delayMs = now > end ? (now - end) / ticksPerFrame : 0;
        if (delayMs > atomic_load_explicit(&rt->stMaxDelay, memory_order_relaxed))
            atomic_store_explicit(&rt->stMaxDelay, delayMs, memory_order_relaxed);
        if (delayMs > 4.0) atomic_fetch_add_explicit(&rt->stSlow, 1, memory_order_relaxed);
        atomic_fetch_add_explicit(&rt->stPackets, KPKT, memory_order_relaxed);
        for (int k = 0; k < KPKT; k++)
            if (list[k].status != kIOReturnSuccess) {
                atomic_fetch_add_explicit(&rt->stErrors, 1, memory_order_relaxed);
                os_log_error(elog(), "out packet error 0x%x (frame %llu)", list[k].status, outFrame[slot] + k);
            }
        if (isFatal(status)) {
            atomic_store(&dead, true);
            os_log_error(elog(), "out pipe stopped: 0x%x", status);
        }
    }
    atomic_store_explicit(&slotBusy[slot], 0, memory_order_release);
    atomic_fetch_sub(&inflight, 1);
}

- (void)enqueueIn:(uint64_t)cur {
    if (nextInFrame <= cur + 2) nextInFrame = cur + 4;  // fell behind: the pump notices the gap and realigns
    uint32_t tail = atomic_load_explicit(&inTail, memory_order_relaxed);
    while (nextInFrame < cur + ENQ_AHEAD) {
        int slot = -1;
        for (int i = 0; i < NIN; i++) {
            int st = atomic_load_explicit(&inState[i], memory_order_acquire);
            if ((st == IN_FREE || st == IN_CONSUMED) && !atomic_load_explicit(&inPending[i], memory_order_acquire)) {
                slot = i;
                break;
            }
        }
        if (slot < 0) break;
        uint64_t frame = nextInFrame;
        nextInFrame += KIN;
        inFrame[slot] = frame;
        IOUSBHostIsochronousTransaction *tx = inTx[slot];
        for (int k = 0; k < KIN; k++) {
            tx[k].status = kIOReturnInvalid;
            tx[k].requestCount = (uint32_t)ri->maxPacket;
            tx[k].offset = (uint32_t)(k * ri->maxPacket);
            tx[k].completeCount = 0;
            tx[k].timeStamp = 0;
            tx[k].options = IOUSBHostIsochronousTransactionOptionsNone;
        }
        NSError *err = nil;
        atomic_store(&inState[slot], IN_QUEUED);
        atomic_store(&inPending[slot], 1);
        atomic_fetch_add(&inflight, 1);
        BOOL ok = [inPipe enqueueIORequestWithData:inData[slot] transactionList:tx transactionListCount:KIN
                                  firstFrameNumber:frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                                 completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction *list) {
                                     if (!self->inTxData[slot]) memcpy(self->inTxLocal[slot], list, sizeof self->inTxLocal[slot]);
                                     if (isFatal(status) && !atomic_load(&self->stopping)) atomic_store(&self->dead, true);
                                     atomic_store_explicit(&self->inPending[slot], 0, memory_order_release);
                                     atomic_fetch_sub(&self->inflight, 1);
                                 }];
        if (!ok) {
            atomic_store(&inState[slot], IN_FREE);
            atomic_store(&inPending[slot], 0);
            atomic_fetch_sub(&inflight, 1);
            atomic_fetch_add(&rt->stInErrors, 1);
            os_log_error(elog(), "in enqueue failed (frame %llu): %{public}@", frame, err);
            continue;
        }
        inFifo[tail % NIN] = slot;
        tail++;
        atomic_store_explicit(&inTail, tail, memory_order_release);
    }
}

#pragma mark Pump thread (no system calls)

- (void)pumpLoop {
    pthread_setname_np("ru.masatec.emu0202.pump");
    setRealtime(ticksPerFrame, 0.5, 0.15, 0.3);
    while (atomic_load_explicit(&threadsRun, memory_order_relaxed)) {
        uint64_t now = mach_absolute_time();
        uint64_t cur = [self frameAt:now];
        if (!atomic_load_explicit(&dead, memory_order_relaxed)) {
            [self pollInput];
            [self fillOutputUpTo:cur + FILL_AHEAD now:now];
        }
        // wake twice per frame: 0.25 ms and 0.75 ms into it
        uint64_t f0 = [self frameTime:cur];
        uint64_t next = f0 + (uint64_t)(0.25 * ticksPerFrame);
        if (next <= now) next = f0 + (uint64_t)(0.75 * ticksPerFrame);
        if (next <= now) next = [self frameTime:cur + 1] + (uint64_t)(0.25 * ticksPerFrame);
        mach_wait_until(next);
    }
}

- (void)fillOutputUpTo:(uint64_t)lastFrame now:(uint64_t)now {
    uint64_t end = atomic_load_explicit(&ftEnd, memory_order_acquire);
    float *ring = rt->ring;
    while (fillFrame <= lastFrame && fillFrame < end) {
        FrameEntry *e = &ft[fillFrame & (FT_SIZE - 1)];
        if (e->frame != fillFrame) { fillFrame++; continue; }
        uint64_t P = e->pos;
        uint32_t n = e->n;
        uint64_t ft0 = [self frameTime:fillFrame];
        if (nextZts < P) nextZts = (P / EMU_RING_FRAMES + 1) * EMU_RING_FRAMES;  // never expected
        if (nextZts >= P && nextZts < P + n) {
            double frac = (double)(nextZts - P) / n;
            ztsWrite(rt, (double)nextZts, ft0 + (uint64_t)(frac * ticksPerFrame) - leadTicks, false);
            nextZts += EMU_RING_FRAMES;
        }
        if (e->slot >= 0 && now + (uint64_t)(0.25 * ticksPerFrame) > ft0 - (uint64_t)(WIRE_EARLY_MS * ticksPerFrame))
            atomic_fetch_add_explicit(&rt->stLateFills, 1, memory_order_relaxed);
        uint64_t wEnd = atomic_load_explicit(&rt->writtenEnd, memory_order_relaxed);
        if (wEnd && P + n > wEnd) atomic_fetch_add_explicit(&rt->stUnderruns, 1, memory_order_relaxed);
        uint8_t *dst = e->slot >= 0 ? outBase[e->slot] + e->offset : NULL;
        for (uint32_t j = 0; j < n; j++) {
            uint32_t idx = (uint32_t)((P + j) & RING_MASK);
            float l = ring[2 * idx], r = ring[2 * idx + 1];
            ring[2 * idx] = 0;
            ring[2 * idx + 1] = 0;
            if (dst) {
                put24(dst + j * FRAME_BYTES, f2i24(l));
                put24(dst + j * FRAME_BYTES + 3, f2i24(r));
            }
        }
        atomic_thread_fence(memory_order_release);
        atomic_store_explicit(&rt->devPos, P + n, memory_order_relaxed);
        fillFrame++;
    }
}

- (void)storeCaptured:(const uint8_t *)p samples:(uint32_t)n frame:(uint64_t)frame {
    if (!inPosValid) {
        // The packet for frame F carries audio sampled during [t(F) - 2 ms, t(F) - 1 ms).
        uint64_t captureHost = [self frameTime:frame] - (uint64_t)((WIRE_EARLY_MS + 1) * ticksPerFrame);
        double s = rt->ztsSample + ((double)(int64_t)(captureHost - rt->ztsHost)) / ticksPerFrame * [self currentSpp];
        inPos = s > 0 ? (uint64_t)llround(s) : 0;
        inPosValid = YES;
        atomic_fetch_add_explicit(&rt->stInRealigns, 1, memory_order_relaxed);
        atomic_store_explicit(&rt->inAvail, inPos, memory_order_release);
    }
    float *ring = rt->inRing;
    // Standard order: left jack (L, Mic/Hi-Z/Line) first, right jack (R, Hi-Z/Line) second.
    // Checked with a loopback cable from output R into the R input jack: it arrives in the second slot.
    for (uint32_t j = 0; j < n; j++) {
        uint32_t idx = (uint32_t)((inPos + j) & RING_MASK);
        ring[2 * idx] = p ? get24(p + j * FRAME_BYTES) : 0.f;
        ring[2 * idx + 1] = p ? get24(p + j * FRAME_BYTES + 3) : 0.f;
    }
    inPos += n;
    atomic_store_explicit(&rt->inAvail, inPos, memory_order_release);
}

- (void)pollInput {
    uint32_t tail = atomic_load_explicit(&inTail, memory_order_acquire);
    while (inHead != tail) {
        int slot = inFifo[inHead % NIN];
        IOUSBHostIsochronousTransaction *tx = inTx[slot];
        while (inConsumed < KIN) {
            volatile IOUSBHostIsochronousTransaction *t = &tx[inConsumed];
            IOReturn st = t->status;
            if (st == kIOReturnInvalid) return;  // not finished yet
            atomic_thread_fence(memory_order_acquire);
            uint64_t frame = inFrame[slot] + inConsumed;
            if (frame != expectedInFrame) inPosValid = NO;  // the enqueuer skipped frames
            expectedInFrame = frame + 1;
            double pickup = ((double)(int64_t)(mach_absolute_time() - [self frameTime:frame])) / ticksPerFrame;
            if (pickup > atomic_load_explicit(&rt->stInPickupMax, memory_order_relaxed) && atomic_load_explicit(&rt->stInPackets, memory_order_relaxed) > 100)
                atomic_store_explicit(&rt->stInPickupMax, pickup, memory_order_relaxed);
            if (st == kIOReturnSuccess) {
                [self storeCaptured:inBase[slot] + inConsumed * ri->maxPacket samples:t->completeCount / FRAME_BYTES frame:frame];
                atomic_fetch_add_explicit(&rt->stInPackets, 1, memory_order_relaxed);
            } else {
                atomic_fetch_add_explicit(&rt->stInErrors, 1, memory_order_relaxed);
                if (inPosValid) [self storeCaptured:NULL samples:(uint32_t)llround([self currentSpp]) frame:frame];
            }
            inConsumed++;
        }
        atomic_store_explicit(&inState[slot], IN_CONSUMED, memory_order_release);
        inHead++;
        inConsumed = 0;
    }
}

#pragma mark Feedback (ioQueue)

- (void)submitFb:(int)i {
    uint64_t cur = [outItf frameNumberWithTime:NULL];
    if (nextFbFrame <= cur + 1) nextFbFrame = cur + 2;
    uint64_t frame = nextFbFrame;
    nextFbFrame += KFB;
    memset(fbData[i].mutableBytes, 0, KFB * 4);
    for (int k = 0; k < KFB; k++) {
        IOUSBHostIsochronousTransaction *t = &fbTx[i][k];
        t->status = kIOReturnInvalid;
        t->requestCount = 4;
        t->offset = (uint32_t)(k * 4);
        t->completeCount = 0;
        t->timeStamp = 0;
        t->options = IOUSBHostIsochronousTransactionOptionsNone;
    }
    NSError *err = nil;
    atomic_fetch_add(&inflight, 1);
    BOOL ok = [fbPipe enqueueIORequestWithData:fbData[i] transactionList:fbTx[i] transactionListCount:KFB
                              firstFrameNumber:frame options:IOUSBHostIsochronousTransferOptionsNone error:&err
                             completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction *list) {
                                 [self fbDone:i status:status list:list];
                             }];
    if (!ok) {
        atomic_fetch_sub(&inflight, 1);
        os_log_error(elog(), "feedback enqueue failed (frame %llu): %{public}@", frame, err);
    }
}

- (void)fbDone:(int)i status:(IOReturn)status list:(IOUSBHostIsochronousTransaction *)list {
    const uint8_t *base = (const uint8_t *)fbData[i].bytes;
    double nominal = ri->rate / 1000.0;
    for (int k = 0; k < KFB; k++) {
        if (list[k].status != kIOReturnSuccess || list[k].completeCount < 3) continue;
        const uint8_t *p = base + k * 4;
        uint32_t raw = p[0] | (p[1] << 8) | (p[2] << 16) | (list[k].completeCount >= 4 ? ((uint32_t)p[3] << 24) : 0);
        double v = raw / 65536.0;
        if (fabs(v - nominal) < nominal * 0.02) {
            atomic_store_explicit(&sppFeedback, v, memory_order_relaxed);
            atomic_store_explicit(&rt->stRate, v * 1000.0, memory_order_relaxed);
            atomic_fetch_add_explicit(&rt->stFb, 1, memory_order_relaxed);
        }
    }
    atomic_fetch_sub(&inflight, 1);
    if (atomic_load(&stopping) || atomic_load(&dead)) return;
    if (isFatal(status)) {
        atomic_store(&dead, true);
        os_log_error(elog(), "feedback pipe stopped: 0x%x", status);
        return;
    }
    [self submitFb:i];
}

- (void)maybeLogStats {
    uint64_t now = mach_absolute_time();
    if (now - lastStats < (uint64_t)(10 * ticksPerSec)) return;
    lastStats = now;
    os_log(elog(), "stats: out %llu err %llu resync %llu underrun %llu late %llu latefill %llu | in %llu err %llu realign %llu underrun %llu short %llu | rate %.2f",
           atomic_load(&rt->stPackets), atomic_load(&rt->stErrors), atomic_load(&rt->stResyncs),
           atomic_load(&rt->stUnderruns), atomic_load(&rt->stLateWrites), atomic_load(&rt->stLateFills),
           atomic_load(&rt->stInPackets), atomic_load(&rt->stInErrors), atomic_load(&rt->stInRealigns),
           atomic_load(&rt->stInUnderruns), atomic_load(&rt->stInShortMax), atomic_load(&rt->stRate));
}

@end

static void *pumpMain(void *arg) {
    @autoreleasepool {
        [(__bridge EMUEngineImpl *)arg pumpLoop];
    }
    return NULL;
}

static void *enqMain(void *arg) {
    @autoreleasepool {
        [(__bridge EMUEngineImpl *)arg enqLoop];
    }
    return NULL;
}

static void drain(void *refcon, io_iterator_t it, BOOL present) {
    BOOL any = NO;
    io_object_t o;
    while ((o = IOIteratorNext(it))) {
        any = YES;
        IOObjectRelease(o);
    }
    if (any) [(__bridge EMUEngineImpl *)refcon setPresent:present];
}
static void onDeviceAdded(void *refcon, io_iterator_t it) { drain(refcon, it, YES); }
static void onDeviceRemoved(void *refcon, io_iterator_t it) { drain(refcon, it, NO); }

#pragma mark C API

#define IMPL(e) ((__bridge EMUEngineImpl *)(e)->impl)

EMUEngine *EMUEngineCreate(void) {
    EMUEngine *rt = calloc(1, sizeof *rt);
    rt->ring = calloc(EMU_RING_FRAMES * EMU_CHANNELS, sizeof(float));
    rt->inRing = calloc(EMU_RING_FRAMES * EMU_CHANNELS, sizeof(float));
    atomic_init(&rt->outGainTarget, 1.0f);
    rt->outGainApplied = -1.0f;  // no buffer written yet: the first one starts at the target gain
    EMUEngineImpl *impl = [[EMUEngineImpl alloc] init];
    impl->rt = rt;
    rt->impl = (__bridge_retained void *)impl;
    return rt;
}

void EMUEngineStartMonitoring(EMUEngine *e, EMUPresenceCallback cb, void *ctx) {
    [IMPL(e) startMonitoringWithCallback:cb ctx:ctx];
}

bool EMUEngineIsPresent(EMUEngine *e) { return atomic_load(&e->present); }
bool EMUEngineIsSupportedRate(double rate) { return findRate(rate) != NULL; }
bool EMUEngineIsRunning(EMUEngine *e) { return atomic_load(&e->running); }

int EMUEngineStart(EMUEngine *e, double sampleRate) {
    EMUEngineImpl *impl = IMPL(e);
    pthread_mutex_lock(&impl->ctlMutex);
    int r = [impl startLocked:sampleRate];
    pthread_mutex_unlock(&impl->ctlMutex);
    return r;
}

void EMUEngineStop(EMUEngine *e) {
    EMUEngineImpl *impl = IMPL(e);
    pthread_mutex_lock(&impl->ctlMutex);
    [impl stopLocked];
    pthread_mutex_unlock(&impl->ctlMutex);
}

// Output: timeline -> wire is LEAD ms, minus the frame being on the wire 1 ms before t(F).
uint32_t EMUEngineOutputLatencyFrames(double rate) { return (uint32_t)((LEAD_MS - WIRE_EARLY_MS) * rate / 1000.0); }
uint32_t EMUEngineOutputSafetyOffsetFrames(double rate) { return (uint32_t)(OUT_SAFETY_MS * rate / 1000.0); }
// Measured with a loopback cable (tools/looptest, output R -> input R): the real round trip is longer than the
// output latency above by 2.95 ms + 67 frames at 44.1, 48, 88.2 and 96 kHz, within 1-2 frames. The frames are
// most likely the converters' filters, the milliseconds buffering inside the card. Reported as input latency,
// so that DAWs, which add both latencies, place recordings where they were played.
#define IN_EXTRA_MS 2.95
#define IN_EXTRA_FRAMES 67
uint32_t EMUEngineInputLatencyFrames(double rate) { return (uint32_t)lround(IN_EXTRA_MS * rate / 1000.0 + IN_EXTRA_FRAMES); }
uint32_t EMUEngineInputSafetyOffsetFrames(double rate) { return (uint32_t)(IN_SAFETY_MS * rate / 1000.0); }

void EMUEngineWriteOutput(EMUEngine *e, double sampleTime, const float *frames, uint32_t n) {
    if (sampleTime < 0) return;
    uint64_t S = (uint64_t)sampleTime;
    uint64_t consumed = atomic_load_explicit(&e->devPos, memory_order_relaxed);
    uint32_t first = 0;
    if (S < consumed) {  // too late for (part of) this buffer: never write slots already sent
        atomic_fetch_add_explicit(&e->stLateWrites, 1, memory_order_relaxed);
        if (S + n <= consumed) return;
        first = (uint32_t)(consumed - S);
    }
    // Volume: ramp linearly from the previous buffer's gain to the target across this buffer,
    // so that volume steps do not click.
    float g1 = atomic_load_explicit(&e->outGainTarget, memory_order_relaxed);
    float g0 = e->outGainApplied < 0.f ? g1 : e->outGainApplied;
    float dg = n ? (g1 - g0) / (float)n : 0.f;
    e->outGainApplied = g1;
    float *ring = e->ring;
    for (uint32_t i = first; i < n; i++) {
        uint32_t idx = (uint32_t)((S + i) & RING_MASK);
        float g = g0 + dg * (float)(i + 1);
        ring[2 * idx] = frames[2 * i] * g;
        ring[2 * idx + 1] = frames[2 * i + 1] * g;
    }
    uint64_t end = S + n;
    if (end > atomic_load_explicit(&e->writtenEnd, memory_order_relaxed))
        atomic_store_explicit(&e->writtenEnd, end, memory_order_relaxed);
}

void EMUEngineSetOutputGain(EMUEngine *e, float gain) {
    if (!(gain >= 0.f)) gain = 0.f;  // also catches NaN
    if (gain > 1.f) gain = 1.f;
    atomic_store_explicit(&e->outGainTarget, gain, memory_order_relaxed);
}

void EMUEngineReadInput(EMUEngine *e, double sampleTime, float *frames, uint32_t n) {
    uint64_t avail = atomic_load_explicit(&e->inAvail, memory_order_acquire);
    int64_t S = (int64_t)llround(sampleTime);
    bool missing = false;
    const float *ring = e->inRing;
    for (uint32_t i = 0; i < n; i++) {
        int64_t s = S + i;
        if (s >= 0 && (uint64_t)s < avail && (uint64_t)s + EMU_RING_FRAMES > avail) {
            uint32_t idx = (uint32_t)((uint64_t)s & RING_MASK);
            frames[2 * i] = ring[2 * idx];
            frames[2 * i + 1] = ring[2 * idx + 1];
        } else {
            frames[2 * i] = 0;
            frames[2 * i + 1] = 0;
            if (avail && (uint64_t)s >= avail) missing = true;
        }
    }
    if (missing) {
        atomic_fetch_add_explicit(&e->stInUnderruns, 1, memory_order_relaxed);
        uint64_t shortBy = (uint64_t)(S + n) - avail;
        if (shortBy > atomic_load_explicit(&e->stInShortMax, memory_order_relaxed))
            atomic_store_explicit(&e->stInShortMax, shortBy, memory_order_relaxed);
    }
}

void EMUEngineGetZeroTimeStamp(EMUEngine *e, double *sampleTime, uint64_t *hostTime, uint64_t *seed) {
    uint32_t s1, s2;
    do {
        s1 = atomic_load_explicit(&e->ztsSeq, memory_order_acquire);
        *sampleTime = e->ztsSample;
        *hostTime = e->ztsHost;
        *seed = e->ztsSeed;
        atomic_thread_fence(memory_order_acquire);
        s2 = atomic_load_explicit(&e->ztsSeq, memory_order_relaxed);
    } while ((s1 & 1) || s1 != s2);
}

void EMUEngineGetStats(EMUEngine *e, EMUEngineStats *s) {
    s->packets = atomic_load(&e->stPackets);
    s->errors = atomic_load(&e->stErrors);
    s->resyncs = atomic_load(&e->stResyncs);
    s->feedback = atomic_load(&e->stFb);
    s->underruns = atomic_load(&e->stUnderruns);
    s->lateWrites = atomic_load(&e->stLateWrites);
    s->lateFills = atomic_load(&e->stLateFills);
    s->inPackets = atomic_load(&e->stInPackets);
    s->inErrors = atomic_load(&e->stInErrors);
    s->inRealigns = atomic_load(&e->stInRealigns);
    s->inUnderruns = atomic_load(&e->stInUnderruns);
    s->deviceRate = atomic_load(&e->stRate);
    s->inPickupMaxMs = atomic_load(&e->stInPickupMax);
    s->maxCallbackDelayMs = atomic_load(&e->stMaxDelay);
    s->slowCallbacks = atomic_load(&e->stSlow);
}
