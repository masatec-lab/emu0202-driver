// haltest — open "E-MU 0202 | USB" through the regular Core Audio HAL (like a DAW does), run a
// full-duplex IOProc that outputs silence and measures the input, and report what the HAL sees:
// buffer size, latencies, the actual sample rate it measures from the driver's timestamps, overloads.
// Usage: haltest [seconds] [buffer]
#include <CoreAudio/CoreAudio.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <unistd.h>
#include <mach/mach_time.h>

static double gTicksPerMs, gDeltaMin = 1e9, gDeltaMax = -1e9, gWakeMin = 1e9, gWakeMax = -1e9;

static _Atomic uint64_t gCycles, gOverloads;
static float gPeakL, gPeakR;
static double gFirstIn = -1, gLastIn, gFirstOut = -1, gLastOut;

static OSStatus ioProc(AudioObjectID dev, const AudioTimeStamp *now, const AudioBufferList *in, const AudioTimeStamp *inTime,
                       AudioBufferList *out, const AudioTimeStamp *outTime, void *ctx) {
    atomic_fetch_add(&gCycles, 1);
    if (in && in->mNumberBuffers > 0) {
        const float *p = in->mBuffers[0].mData;
        UInt32 frames = in->mBuffers[0].mDataByteSize / (sizeof(float) * in->mBuffers[0].mNumberChannels);
        for (UInt32 i = 0; i < frames; i++) {
            if (fabsf(p[2 * i]) > gPeakL) gPeakL = fabsf(p[2 * i]);
            if (fabsf(p[2 * i + 1]) > gPeakR) gPeakR = fabsf(p[2 * i + 1]);
        }
        if (gFirstIn < 0) gFirstIn = inTime->mSampleTime;
        gLastIn = inTime->mSampleTime;
        // When is the HAL asking for the end of this input buffer, relative to that sample's
        // timeline time (per the HAL's own "now" mapping)?
        double h = (double)mach_absolute_time();
        double endTime = (double)now->mHostTime + (inTime->mSampleTime + frames - now->mSampleTime) / 48000.0 * 1000.0 * gTicksPerMs;
        double delta = (h - endTime) / gTicksPerMs;
        double wake = (h - (double)now->mHostTime) / gTicksPerMs;
        if (atomic_load(&gCycles) > 50) {
            if (delta < gDeltaMin) gDeltaMin = delta;
            if (delta > gDeltaMax) gDeltaMax = delta;
            if (wake < gWakeMin) gWakeMin = wake;
            if (wake > gWakeMax) gWakeMax = wake;
        }
    }
    for (UInt32 b = 0; out && b < out->mNumberBuffers; b++) memset(out->mBuffers[b].mData, 0, out->mBuffers[b].mDataByteSize);
    if (gFirstOut < 0) gFirstOut = outTime->mSampleTime;
    gLastOut = outTime->mSampleTime;
    return noErr;
}

static OSStatus overloadListener(AudioObjectID obj, UInt32 n, const AudioObjectPropertyAddress *a, void *ctx) {
    atomic_fetch_add(&gOverloads, 1);
    return noErr;
}

static UInt32 getU32(AudioObjectID dev, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope) {
    AudioObjectPropertyAddress a = {sel, scope, kAudioObjectPropertyElementMain};
    UInt32 v = 0, size = sizeof v;
    AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &v);
    return v;
}

int main(int argc, char **argv) {
    int seconds = argc > 1 ? atoi(argv[1]) : 10;
    mach_timebase_info_data_t tb; mach_timebase_info(&tb);
    gTicksPerMs = 1e6 * tb.denom / tb.numer;
    UInt32 buffer = argc > 2 ? (UInt32)atoi(argv[2]) : 128;

    CFStringRef uid = CFSTR("EMU0202USB:UserSpace");
    AudioObjectID dev = kAudioObjectUnknown;
    AudioObjectPropertyAddress ta = {kAudioHardwarePropertyTranslateUIDToDevice, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    UInt32 size = sizeof dev;
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &ta, sizeof uid, &uid, &size, &dev);
    if (dev == kAudioObjectUnknown) { printf("E-MU device not found in Core Audio\n"); return 1; }

    AudioObjectPropertyAddress ba = {kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    OSStatus e = AudioObjectSetPropertyData(dev, &ba, 0, NULL, sizeof buffer, &buffer);
    if (e) printf("set buffer size: error %d\n", (int)e);

    AudioObjectPropertyAddress oa = {kAudioDeviceProcessorOverload, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    AudioObjectAddPropertyListener(dev, &oa, overloadListener, NULL);

    AudioDeviceIOProcID procID = NULL;
    if ((e = AudioDeviceCreateIOProcID(dev, ioProc, NULL, &procID))) { printf("create IOProc: %d\n", (int)e); return 1; }
    if ((e = AudioDeviceStart(dev, procID))) { printf("AudioDeviceStart: %d\n", (int)e); return 1; }

    printf("running %d s at buffer %u...\n", seconds, getU32(dev, kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal));
    sleep(seconds > 2 ? 2 : seconds);
    Float64 actual = 0;
    AudioObjectPropertyAddress ra = {kAudioDevicePropertyActualSampleRate, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    size = sizeof actual;
    if (seconds > 2) sleep(seconds - 2);
    AudioObjectGetPropertyData(dev, &ra, 0, NULL, &size, &actual);
    AudioDeviceStop(dev, procID);
    AudioDeviceDestroyIOProcID(dev, procID);

    double nominalFrames = 48000.0 * seconds;
    printf("IO cycles: %llu, overloads reported by HAL: %llu\n", atomic_load(&gCycles), atomic_load(&gOverloads));
    printf("HAL actual sample rate (from driver timestamps): %.3f Hz\n", actual);
    printf("output sample time advanced %.0f, input %.0f (nominal ~%.0f for %d s)\n", gLastOut - gFirstOut, gLastIn - gFirstIn, nominalFrames, seconds);
    printf("latency out/in: %u/%u frames, safety offset out/in: %u/%u frames\n",
           getU32(dev, kAudioDevicePropertyLatency, kAudioObjectPropertyScopeOutput),
           getU32(dev, kAudioDevicePropertyLatency, kAudioObjectPropertyScopeInput),
           getU32(dev, kAudioDevicePropertySafetyOffset, kAudioObjectPropertyScopeOutput),
           getU32(dev, kAudioDevicePropertySafetyOffset, kAudioObjectPropertyScopeInput));
    printf("input read: buffer end requested %.2f..%.2f ms after its timeline time; callback %.2f..%.2f ms after HAL 'now'\n",
           gDeltaMin, gDeltaMax, gWakeMin, gWakeMax);
    printf("input peak: 1 (L) %.1f dBFS, 2 (R) %.1f dBFS\n", 20 * log10(gPeakL + 1e-9), 20 * log10(gPeakR + 1e-9));
    return 0;
}
