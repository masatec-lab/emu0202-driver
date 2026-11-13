// clockprobe — measure USB frame clock vs mach_absolute_time via IOUSBHost.
// Opens E-MU 0202 interface 1 (no streaming), samples frame/microframe numbers with times.
// Build: clang -fobjc-arc -O2 -o clockprobe clockprobe.m -framework Foundation -framework IOKit -framework IOUSBHost
#import <Foundation/Foundation.h>
#import <IOUSBHost/IOUSBHost.h>
#include <mach/mach_time.h>

int main(void) {
    @autoreleasepool {
        CFMutableDictionaryRef match = [IOUSBHostInterface createMatchingDictionaryWithVendorID:@0x041e
            productID:@0x3f02 bcdDevice:nil interfaceNumber:@1 configurationValue:@1
            interfaceClass:nil interfaceSubclass:nil interfaceProtocol:nil speed:nil productIDArray:nil];
        io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, match);
        if (!svc) { printf("interface 1 not found\n"); return 1; }
        NSError *err = nil;
        dispatch_queue_t q = dispatch_queue_create("probe", DISPATCH_QUEUE_SERIAL);
        IOUSBHostInterface *itf = [[IOUSBHostInterface alloc] initWithIOService:svc options:0 queue:q error:&err interestHandler:nil];
        IOObjectRelease(svc);
        if (!itf) { printf("open failed: %s\n", err.description.UTF8String); return 1; }

        mach_timebase_info_data_t tb; mach_timebase_info(&tb);
        double ticksPerMs = 1e6 * tb.denom / tb.numer;
        printf("timebase %u/%u, ticks per ms %.1f\n", tb.numer, tb.denom, ticksPerMs);

        // 1) Is IOUSBHostTime in mach ticks?
        IOUSBHostTime t = 0;
        uint64_t m0 = mach_absolute_time();
        uint64_t f = [itf frameNumberWithTime:&t];
        uint64_t m1 = mach_absolute_time();
        printf("frameNumberWithTime: frame %llu time %llu, mach before %llu after %llu\n", f, t, m0, m1);
        NSError *e2 = nil;
        IOUSBHostTime tr = 0, tc = 0;
        uint64_t rmf = [itf referenceMicroframeWithTime:&tr error:&e2];
        uint64_t cmf = [itf currentMicroframeWithTime:&tc error:&e2];
        printf("referenceMicroframe %llu @ %llu (now-%.3f ms), currentMicroframe %llu @ %llu (now-%.3f ms)\n",
               rmf, tr, (mach_absolute_time() - tr) / ticksPerMs, cmf, tc, (mach_absolute_time() - tc) / ticksPerMs);

        // 2) Catch frame boundaries for precise (frame, time) pairs; measure drift over ~6 s
        uint64_t firstF = 0, firstT = 0, lastF = 0, lastT = 0;
        double maxResid = 0;
        for (int round = 0; round < 7; round++) {
            uint64_t prev = [itf frameNumberWithTime:NULL], cur, tt = 0;
            do { cur = [itf frameNumberWithTime:NULL]; tt = mach_absolute_time(); } while (cur == prev);
            if (!firstF) { firstF = cur; firstT = tt; }
            else {
                double expect = firstT + (cur - firstF) * ticksPerMs;
                double resid = (tt - expect) / ticksPerMs * 1000.0;
                printf("frame %llu: boundary at +%.1f ms, deviation from nominal 1 ms/frame: %+.1f us\n",
                       cur, (tt - firstT) / ticksPerMs, resid);
                if (fabs(resid) > maxResid) maxResid = fabs(resid);
            }
            lastF = cur; lastT = tt;
            if (round < 6) usleep(1000000);
        }
        double ppm = ((lastT - firstT) / ticksPerMs - (double)(lastF - firstF)) / (lastF - firstF) * 1e6;
        printf("USB frame clock vs mach clock: %+.1f ppm over %llu frames\n", ppm, lastF - firstF);

        // 3) referenceMicroframe consistency
        printf("referenceMicroframe samples:\n");
        uint64_t r0 = 0, rt0 = 0;
        for (int i = 0; i < 5; i++) {
            IOUSBHostTime rt = 0; NSError *e = nil;
            uint64_t r = [itf referenceMicroframeWithTime:&rt error:&e];
            if (!r0) { r0 = r; rt0 = rt; }
            printf("  mf %llu @ %llu  -> mf delta %llu, time delta %.3f ms, implied %.3f us/mf\n", r, rt, r - r0,
                   (rt - rt0) / ticksPerMs, r > r0 ? (rt - rt0) / ticksPerMs * 1000.0 / (r - r0) : 0.0);
            usleep(300000);
        }
        [itf destroy];
    }
    return 0;
}
