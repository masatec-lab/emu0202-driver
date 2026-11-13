// engloop — loopback latency test on the engine itself, without Core Audio (cable: output R -> input R).
// Like looptest, but drives EMUEngine directly the way emutest does, so engine changes can be tried
// without installing the plug-in. The card must not be in use by the installed driver.
//
// Usage: engloop [--rate R] [--runs N] [--buffer F] [--switch R2]
//   --switch R2  before every run, stream briefly at R2: each run is then the first start after a
//                sample rate change.
// Prints, per run, the measured round trip minus the latency the engine reports (should be ~0).
#import <Foundation/Foundation.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <math.h>
#include "../src/EMUEngine.h"

#define BURST_LEN 1024
#define BURSTS 6
#define SPACING_S 0.2
#define SETTLE_S 0.5

static double gTicksPerSec;
static float gBurst[BURST_LEN];

static void makeRealtime(double periodSec) {
    thread_time_constraint_policy_data_t p;
    p.period = (uint32_t)(periodSec * gTicksPerSec);
    p.computation = (uint32_t)(0.0003 * gTicksPerSec);
    p.constraint = (uint32_t)(0.0008 * gTicksPerSec);
    p.preemptible = 1;
    thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&p, THREAD_TIME_CONSTRAINT_POLICY_COUNT);
}

// Streams for `seconds` like the HAL does. With rec != NULL plays the bursts on output R and records
// input R; returns the timeline position of the first burst through *firstBurst, rec is indexed from *recBase.
static int stream(EMUEngine *e, double rate, uint32_t buf, double seconds, float *rec, long recLen, uint64_t *firstBurst,
                  uint64_t *recBase) {
    uint32_t outSafety = EMUEngineOutputSafetyOffsetFrames(rate), inSafety = EMUEngineInputSafetyOffsetFrames(rate);
    float *out = calloc(buf * 2, sizeof(float)), *in = calloc(buf * 2, sizeof(float));
    double ztsS;
    uint64_t ztsH, seed;
    EMUEngineGetZeroTimeStamp(e, &ztsS, &ztsH, &seed);
    double nowS = ztsS + (double)(int64_t)(mach_absolute_time() - ztsH) / gTicksPerSec * rate;
    uint64_t X = ((uint64_t)(nowS > 0 ? nowS : 0) / buf + 1) * buf;
    uint64_t fb = X + outSafety + (uint64_t)(SETTLE_S * rate), spacing = (uint64_t)(SPACING_S * rate);
    uint64_t base = X > inSafety + buf ? X - inSafety - buf : 0;
    if (firstBurst) *firstBurst = fb;
    if (recBase) *recBase = base;
    uint64_t endHost = mach_absolute_time() + (uint64_t)(seconds * gTicksPerSec);
    while (mach_absolute_time() < endHost) {
        EMUEngineGetZeroTimeStamp(e, &ztsS, &ztsH, &seed);
        mach_wait_until((uint64_t)((int64_t)ztsH + (int64_t)(((double)X - ztsS) / rate * gTicksPerSec)));
        double inStart = (double)X - inSafety - buf;
        if (inStart >= 0) {
            EMUEngineReadInput(e, inStart, in, buf);
            for (uint32_t i = 0; rec && i < buf; i++) {
                long idx = (long)((uint64_t)inStart + i - base);
                if (idx >= 0 && idx < recLen) rec[idx] = in[2 * i + 1];
            }
        }
        for (uint32_t i = 0; i < buf; i++) {
            uint64_t t = X + outSafety + i;
            float v = 0;
            if (rec && t >= fb) {
                uint64_t k = (t - fb) / spacing, off = (t - fb) % spacing;
                if (k < BURSTS && off < BURST_LEN) v = gBurst[off];
            }
            out[2 * i] = 0;
            out[2 * i + 1] = v;
        }
        EMUEngineWriteOutput(e, (double)(X + outSafety), out, buf);
        X += buf;
    }
    free(out);
    free(in);
    return 0;
}

static int cmpLong(const void *a, const void *b) { long x = *(const long *)a, y = *(const long *)b; return (x > y) - (x < y); }

int main(int argc, char **argv) {
    @autoreleasepool {
        double rate = 44100, switchRate = 0;
        int runs = 4;
        uint32_t buf = 128;
        for (int i = 1; i < argc; i++) {
            if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = atof(argv[++i]);
            else if (!strcmp(argv[i], "--runs") && i + 1 < argc) runs = atoi(argv[++i]);
            else if (!strcmp(argv[i], "--buffer") && i + 1 < argc) buf = (uint32_t)atoi(argv[++i]);
            else if (!strcmp(argv[i], "--switch") && i + 1 < argc) switchRate = atof(argv[++i]);
            else { fprintf(stderr, "usage: %s [--rate R] [--runs N] [--buffer F] [--switch R2]\n", argv[0]); return 2; }
        }
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        gTicksPerSec = 1e9 * tb.denom / tb.numer;
        unsigned seed = 12345;
        for (int j = 0; j < BURST_LEN; j++) {
            seed = seed * 1103515245u + 12345u;
            gBurst[j] = ((seed >> 16) & 1 ? 0.25f : -0.25f);
        }

        EMUEngine *e = EMUEngineCreate();
        long reported = (long)(EMUEngineOutputLatencyFrames(rate) + EMUEngineInputLatencyFrames(rate));
        printf("%.0f Hz, buffer %u, engine reports %ld frames (%.2f ms)%s\n", rate, buf, reported, reported / rate * 1000,
               switchRate > 0 ? ", each run after a rate change" : "");
        double secs = SETTLE_S + BURSTS * SPACING_S + 0.3;
        long recLen = (long)((secs + 0.5) * rate);
        float *rec = calloc((size_t)recLen, sizeof(float));
        makeRealtime(buf / rate);
        for (int r = 0; r < runs; r++) {
            if (switchRate > 0) {
                if (EMUEngineStart(e, switchRate)) { printf("cannot start at %.0f Hz (card busy?)\n", switchRate); return 1; }
                stream(e, switchRate, buf, 0.3, NULL, 0, NULL, NULL);
                EMUEngineStop(e);
            }
            uint64_t t0 = mach_absolute_time();
            if (EMUEngineStart(e, rate)) { printf("cannot start at %.0f Hz (card busy?)\n", rate); return 1; }
            double startMs = (mach_absolute_time() - t0) / gTicksPerSec * 1000;
            memset(rec, 0, recLen * sizeof(float));
            uint64_t fb, base;
            stream(e, rate, buf, secs, rec, recLen, &fb, &base);
            EMUEngineStop(e);

            long lags[BURSTS];
            int found = 0;
            double be = 0;
            for (int j = 0; j < BURST_LEN; j++) be += gBurst[j] * gBurst[j];
            long maxLag = (long)(0.05 * rate);
            for (int k = 0; k < BURSTS; k++) {
                long start = (long)(fb + (uint64_t)k * (uint64_t)(SPACING_S * rate) - base);
                double best = 0;
                long bestLag = -1;
                for (long lag = 0; lag <= maxLag; lag++) {
                    long s0 = start + lag;
                    if (s0 < 0 || s0 + BURST_LEN > recLen) continue;
                    double c = 0, en = 0;
                    for (int j = 0; j < BURST_LEN; j++) { c += gBurst[j] * rec[s0 + j]; en += rec[s0 + j] * rec[s0 + j]; }
                    double m = en > 0 ? fabs(c) / sqrt(en * be) : 0;
                    if (m > best) { best = m; bestLag = lag; }
                }
                if (best > 0.5) lags[found++] = bestLag;
            }
            if (found < BURSTS / 2) { printf("run %d: bursts not found (cable in output R -> input R?)\n", r + 1); continue; }
            qsort(lags, (size_t)found, sizeof(long), cmpLong);
            long lag = lags[found / 2];
            printf("run %d: start took %.0f ms, measured %ld frames -> unreported %+ld (%+.2f ms)%s\n", r + 1, startMs, lag,
                   lag - reported, (lag - reported) / rate * 1000, lags[0] != lags[found - 1] ? " (varied within run)" : "");
        }
    }
    return 0;
}
