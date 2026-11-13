// looptest — measure the real round-trip latency with a loopback cable (output R -> input R) and
// compare it with the latency the driver reports to Core Audio.
//
// Plays short noise bursts on output channel 2 through the regular HAL (like a DAW), records both
// inputs, takes the one where the bursts arrive (the louder one), and finds each burst in the recording by cross-correlation. On the device timeline a
// sample written at output time X is heard at X + output latency, and a sound at that moment gets
// input time X + output latency + input latency. So the measured offset should equal the sum of the
// reported latencies; the difference is what the driver does not report yet (converters etc.).
//
// Usage: looptest [rate|0 = current] [runs] [buffer]
#include <CoreAudio/CoreAudio.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BURST_LEN 1024
#define BURSTS 8
#define SPACING_S 0.25
#define SETTLE_S 0.5
#define MAX_LAG_S 0.05

static float gBurst[BURST_LEN];
static double gRate;
static double gFirstOut = -1, gFirstIn = -1, gFirstBurst;
static long gSpacing;
static float *gRecCh[2];  // both input channels, indexed by input sample time - gFirstIn
static float *gRec;       // the channel the bursts arrive on
static long gRecLen;
static _Atomic long gRecMax;
static float gPeakCh[2], gPeakIn;

static OSStatus ioProc(AudioObjectID dev, const AudioTimeStamp *now, const AudioBufferList *in, const AudioTimeStamp *inTime,
                       AudioBufferList *out, const AudioTimeStamp *outTime, void *ctx) {
    if (in && in->mNumberBuffers == 1 && in->mBuffers[0].mNumberChannels == 2) {
        const float *p = in->mBuffers[0].mData;
        UInt32 frames = in->mBuffers[0].mDataByteSize / (2 * sizeof(float));
        if (gFirstIn < 0) gFirstIn = inTime->mSampleTime;
        long base = lround(inTime->mSampleTime - gFirstIn);
        for (UInt32 i = 0; i < frames; i++) {
            long idx = base + (long)i;
            for (int ch = 0; ch < 2; ch++) {
                if (idx >= 0 && idx < gRecLen) gRecCh[ch][idx] = p[2 * i + ch];
                if (fabsf(p[2 * i + ch]) > gPeakCh[ch]) gPeakCh[ch] = fabsf(p[2 * i + ch]);
            }
        }
        if (base + (long)frames > atomic_load(&gRecMax)) atomic_store(&gRecMax, base + (long)frames);
    }
    if (out && out->mNumberBuffers == 1 && out->mBuffers[0].mNumberChannels == 2) {
        float *p = out->mBuffers[0].mData;
        UInt32 frames = out->mBuffers[0].mDataByteSize / (2 * sizeof(float));
        if (gFirstOut < 0) {
            gFirstOut = outTime->mSampleTime;
            gFirstBurst = gFirstOut + floor(SETTLE_S * gRate);
        }
        for (UInt32 i = 0; i < frames; i++) {
            double t = outTime->mSampleTime + i - gFirstBurst;
            float v = 0.f;
            if (t >= 0) {
                long k = (long)t / gSpacing, off = (long)t - k * gSpacing;
                if (k < BURSTS && off < BURST_LEN) v = gBurst[off];
            }
            p[2 * i] = 0.f;
            p[2 * i + 1] = v;
        }
    }
    return noErr;
}

static UInt32 getU32(AudioObjectID obj, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope) {
    AudioObjectPropertyAddress a = {sel, scope, kAudioObjectPropertyElementMain};
    UInt32 v = 0, size = sizeof v;
    AudioObjectGetPropertyData(obj, &a, 0, NULL, &size, &v);
    return v;
}

static Float64 getRate(AudioObjectID dev) {
    AudioObjectPropertyAddress a = {kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    Float64 r = 0;
    UInt32 size = sizeof r;
    AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &r);
    return r;
}

static bool setRate(AudioObjectID dev, Float64 r) {
    if (getRate(dev) == r) return true;
    AudioObjectPropertyAddress a = {kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    if (AudioObjectSetPropertyData(dev, &a, 0, NULL, sizeof r, &r)) return false;
    for (int i = 0; i < 40 && getRate(dev) != r; i++) usleep(50000);
    usleep(300000);
    return getRate(dev) == r;
}

// Latency the HAL adds up for a scope: device latency + the stream's own latency.
static UInt32 reportedLatency(AudioObjectID dev, AudioObjectPropertyScope scope) {
    UInt32 lat = getU32(dev, kAudioDevicePropertyLatency, scope);
    AudioObjectPropertyAddress a = {kAudioDevicePropertyStreams, scope, kAudioObjectPropertyElementMain};
    AudioObjectID s[4];
    UInt32 size = sizeof s;
    if (!AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, s) && size >= sizeof(AudioObjectID))
        lat += getU32(s[0], kAudioStreamPropertyLatency, kAudioObjectPropertyScopeGlobal);
    return lat;
}

static int cmpLong(const void *a, const void *b) { long x = *(const long *)a, y = *(const long *)b; return (x > y) - (x < y); }

// One run: start IO, play the bursts, stop, return the median lag (frames) or -1.
static long run(AudioObjectID dev, UInt32 buffer, int verbose) {
    AudioObjectPropertyAddress ba = {kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    AudioObjectSetPropertyData(dev, &ba, 0, NULL, sizeof buffer, &buffer);
    gRate = getRate(dev);
    gSpacing = lround(SPACING_S * gRate);
    gRecLen = lround((SETTLE_S + BURSTS * SPACING_S + 1.0) * gRate);
    for (int ch = 0; ch < 2; ch++) {
        free(gRecCh[ch]);
        gRecCh[ch] = calloc((size_t)gRecLen, sizeof(float));
    }
    gFirstOut = gFirstIn = -1;
    atomic_store(&gRecMax, 0);
    gPeakCh[0] = gPeakCh[1] = 0;

    AudioDeviceIOProcID proc = NULL;
    if (AudioDeviceCreateIOProcID(dev, ioProc, NULL, &proc) || AudioDeviceStart(dev, proc)) {
        printf("cannot start IO\n");
        return -1;
    }
    usleep((useconds_t)((SETTLE_S + BURSTS * SPACING_S + 0.3) * 1e6));
    AudioDeviceStop(dev, proc);
    AudioDeviceDestroyIOProcID(dev, proc);

    int inCh = gPeakCh[1] > gPeakCh[0] ? 1 : 0;
    gRec = gRecCh[inCh];
    gPeakIn = gPeakCh[inCh];
    long maxLag = lround(MAX_LAG_S * gRate), lags[BURSTS];
    int found = 0;
    double burstEnergy = 0;
    for (int j = 0; j < BURST_LEN; j++) burstEnergy += gBurst[j] * gBurst[j];
    for (int k = 0; k < BURSTS; k++) {
        long start = lround(gFirstBurst + (double)k * gSpacing - gFirstIn);  // burst k on the input index scale
        double best = 0;
        long bestLag = -1;
        for (long lag = -maxLag; lag <= maxLag; lag++) {
            long s0 = start + lag;
            if (s0 < 0 || s0 + BURST_LEN > atomic_load(&gRecMax)) continue;
            double c = 0, e = 0;
            for (int j = 0; j < BURST_LEN; j++) {
                c += gBurst[j] * gRec[s0 + j];
                e += gRec[s0 + j] * gRec[s0 + j];
            }
            double norm = e > 0 ? fabs(c) / sqrt(e * burstEnergy) : 0;  // 1 = perfect copy (any gain/polarity)
            if (norm > best) { best = norm; bestLag = lag; }
        }
        if (verbose) printf("    burst %d: lag %ld frames, match %.2f\n", k, bestLag, best);
        if (best > 0.5) lags[found++] = bestLag;
    }
    if (found < BURSTS / 2) {
        printf("  bursts not found (input peaks %.1f / %.1f dBFS) - is the cable in output R -> input R?\n",
               20 * log10(gPeakCh[0] + 1e-9), 20 * log10(gPeakCh[1] + 1e-9));
        return -1;
    }
    qsort(lags, (size_t)found, sizeof(long), cmpLong);
    if (lags[0] != lags[found - 1]) printf("  note: lags within the run vary %ld..%ld\n", lags[0], lags[found - 1]);
    printf("  bursts arrive on input %d (peak %.1f dBFS; other input %.1f dBFS), %d/%d found\n", inCh + 1,
           20 * log10(gPeakIn + 1e-9), 20 * log10(gPeakCh[1 - inCh] + 1e-9), found, BURSTS);
    return lags[found / 2];
}

int main(int argc, char **argv) {
    double wantRate = argc > 1 ? atof(argv[1]) : 0;
    int runs = argc > 2 ? atoi(argv[2]) : 3;
    UInt32 buffer = argc > 3 ? (UInt32)atoi(argv[3]) : 128;

    unsigned seed = 12345;  // fixed pseudo-random noise, about -12 dBFS
    for (int j = 0; j < BURST_LEN; j++) {
        seed = seed * 1103515245u + 12345u;
        gBurst[j] = ((seed >> 16) & 1 ? 0.25f : -0.25f);
    }

    CFStringRef uid = CFSTR("EMU0202USB:UserSpace");
    AudioObjectID dev = kAudioObjectUnknown;
    AudioObjectPropertyAddress ta = {kAudioHardwarePropertyTranslateUIDToDevice, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    UInt32 size = sizeof dev;
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &ta, sizeof uid, &uid, &size, &dev);
    if (dev == kAudioObjectUnknown) { printf("E-MU device not found in Core Audio\n"); return 1; }

    if (wantRate > 0 && !setRate(dev, wantRate)) { printf("cannot set %.0f Hz\n", wantRate); return 1; }
    double rate = getRate(dev);
    UInt32 outLat = reportedLatency(dev, kAudioObjectPropertyScopeOutput), inLat = reportedLatency(dev, kAudioObjectPropertyScopeInput);
    printf("%.0f Hz, buffer %u: driver reports latency out %u + in %u = %u frames (%.2f ms)\n", rate, buffer, outLat, inLat,
           outLat + inLat, (outLat + inLat) / rate * 1000);
    if (getU32(dev, kAudioDevicePropertyDeviceIsRunningSomewhere, kAudioObjectPropertyScopeGlobal))
        printf("note: another app is using the card, so the stream is not restarted between runs\n");

    int bad = 0;
    for (int r = 0; r < runs; r++) {
        printf("run %d:\n", r + 1);
        long lag = run(dev, buffer, getenv("V") != NULL);
        if (lag < 0) { bad++; continue; }
        long extra = lag - (long)(outLat + inLat);
        printf("  measured %ld frames (%.2f ms) -> unreported %+ld frames (%+.2f ms)\n", lag, lag / rate * 1000, extra,
               extra / rate * 1000);
        usleep(300000);
    }
    return bad ? 1 : 0;
}
