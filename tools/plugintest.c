// plugintest — load EMU0202.driver like coreaudiod does and exercise it with a fake host.
// Usage: plugintest build/EMU0202.driver
#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <mach/mach_time.h>
#include <stdio.h>
#include <unistd.h>

static AudioServerPlugInDriverRef drv;

static OSStatus hPropertiesChanged(AudioServerPlugInHostRef h, AudioObjectID obj, UInt32 n, const AudioObjectPropertyAddress *a) {
    for (UInt32 i = 0; i < n; i++) printf("  [host] PropertiesChanged obj %u '%.4s'\n", obj, (char *)&(UInt32){CFSwapInt32HostToBig(a[i].mSelector)});
    return 0;
}
static OSStatus hCopy(AudioServerPlugInHostRef h, CFStringRef k, CFPropertyListRef *d) { *d = NULL; return 0; }
static OSStatus hWrite(AudioServerPlugInHostRef h, CFStringRef k, CFPropertyListRef d) { return 0; }
static OSStatus hDelete(AudioServerPlugInHostRef h, CFStringRef k) { return 0; }
static OSStatus hRequest(AudioServerPlugInHostRef h, AudioObjectID dev, UInt64 action, void *info) {
    printf("  [host] RequestDeviceConfigurationChange %llu\n", action);
    return (*drv)->PerformDeviceConfigurationChange(drv, dev, action, info);
}
static AudioServerPlugInHostInterface host = {hPropertiesChanged, hCopy, hWrite, hDelete, hRequest};

static void getProp(AudioObjectID obj, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope, void *out, UInt32 *size) {
    AudioObjectPropertyAddress a = {sel, scope, kAudioObjectPropertyElementMain};
    UInt32 need = 0;
    OSStatus e = (*drv)->GetPropertyDataSize(drv, obj, 0, &a, 0, NULL, &need);
    if (e) { printf("  size('%.4s') error %d\n", (char *)&(UInt32){CFSwapInt32HostToBig(sel)}, (int)e); *size = 0; return; }
    e = (*drv)->GetPropertyData(drv, obj, 0, &a, 0, NULL, *size, size, out);
    if (e) printf("  get('%.4s') error %d\n", (char *)&(UInt32){CFSwapInt32HostToBig(sel)}, (int)e);
}

static void printString(const char *label, AudioObjectID obj, AudioObjectPropertySelector sel) {
    CFStringRef s = NULL;
    UInt32 size = sizeof s;
    getProp(obj, sel, kAudioObjectPropertyScopeGlobal, &s, &size);
    char buf[256] = "";
    if (s) CFStringGetCString(s, buf, sizeof buf, kCFStringEncodingUTF8);
    printf("%s: %s\n", label, buf);
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s path/to/EMU0202.driver\n", argv[0]); return 2; }
    CFURLRef url = CFURLCreateFromFileSystemRepresentation(NULL, (const UInt8 *)argv[1], (CFIndex)strlen(argv[1]), true);
    CFPlugInRef plugin = CFPlugInCreate(NULL, url);
    if (!plugin) { printf("cannot load bundle\n"); return 1; }
    CFArrayRef factories = CFPlugInFindFactoriesForPlugInType(kAudioServerPlugInTypeUUID);
    if (!factories || CFArrayGetCount(factories) == 0) { printf("no factories for AudioServerPlugIn type\n"); return 1; }
    void *unk = CFPlugInInstanceCreate(NULL, (CFUUIDRef)CFArrayGetValueAtIndex(factories, 0), kAudioServerPlugInTypeUUID);
    if (!unk) { printf("factory returned NULL\n"); return 1; }
    IUnknownVTbl **iu = unk;
    (*iu)->QueryInterface(unk, CFUUIDGetUUIDBytes(kAudioServerPlugInDriverInterfaceUUID), (LPVOID *)&drv);
    printf("plug-in loaded, initializing\n");
    if ((*drv)->Initialize(drv, &host) != 0) { printf("Initialize failed\n"); return 1; }
    usleep(500000);

    AudioObjectID devs[4];
    UInt32 size = sizeof devs;
    getProp(kAudioObjectPlugInObject, kAudioPlugInPropertyDeviceList, kAudioObjectPropertyScopeGlobal, devs, &size);
    printf("device list: %u device(s)\n", size / 4);
    if (!size) return 1;
    AudioObjectID dev = devs[0];
    printString("name", dev, kAudioObjectPropertyName);
    printString("uid", dev, kAudioDevicePropertyDeviceUID);

    AudioValueRange rates[8];
    size = sizeof rates;
    getProp(dev, kAudioDevicePropertyAvailableNominalSampleRates, kAudioObjectPropertyScopeGlobal, rates, &size);
    printf("rates:");
    for (UInt32 i = 0; i < size / sizeof(AudioValueRange); i++) printf(" %.0f", rates[i].mMinimum);
    printf("\n");
    UInt32 v = 0;
    size = 4; getProp(dev, kAudioDevicePropertyLatency, kAudioObjectPropertyScopeOutput, &v, &size); printf("latency: %u frames\n", v);
    size = 4; getProp(dev, kAudioDevicePropertySafetyOffset, kAudioObjectPropertyScopeOutput, &v, &size); printf("safety offset: %u frames\n", v);
    AudioObjectID streams[4];
    size = sizeof streams; getProp(dev, kAudioDevicePropertyStreams, kAudioObjectPropertyScopeOutput, streams, &size);
    printf("output streams: %u, input streams: ", size / 4);
    size = sizeof streams; getProp(dev, kAudioDevicePropertyStreams, kAudioObjectPropertyScopeInput, streams, &size);
    printf("%u\n", size / 4);

    // switch to 44.1 kHz through the stream format, then back to 48 kHz through the device
    Float64 r = 44100;
    AudioObjectPropertyAddress ra = {kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    (*drv)->SetPropertyData(drv, dev, 0, &ra, 0, NULL, sizeof r, &r);
    usleep(200000);
    Float64 cur = 0; size = sizeof cur; getProp(dev, kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, &cur, &size);
    printf("rate after change: %.0f\n", cur);
    r = 48000;
    (*drv)->SetPropertyData(drv, dev, 0, &ra, 0, NULL, sizeof r, &r);
    usleep(200000);

    // IO like the HAL: 512-frame cycles
    if ((*drv)->StartIO(drv, dev, 1) != 0) { printf("StartIO failed\n"); return 1; }
    mach_timebase_info_data_t tb; mach_timebase_info(&tb);
    double tps = 1e9 * tb.denom / tb.numer, rate = 48000;
    static float buf[512 * 2];
    Float64 zs; UInt64 zh, seed;
    (*drv)->GetZeroTimeStamp(drv, dev, 1, &zs, &zh, &seed);
    double now = zs + (mach_absolute_time() - zh) / tps * rate;
    double X = (floor(now / 512) + 1) * 512;
    uint64_t end = mach_absolute_time() + (uint64_t)(3 * tps);
    int cycles = 0;
    while (mach_absolute_time() < end) {
        (*drv)->GetZeroTimeStamp(drv, dev, 1, &zs, &zh, &seed);
        mach_wait_until(zh + (uint64_t)((X - zs) / rate * tps));
        AudioServerPlugInIOCycleInfo ci = {0};
        ci.mOutputTime.mSampleTime = X + 96;
        ci.mInputTime.mSampleTime = X - 96 - 512;
        static float inbuf[512 * 2];
        if (ci.mInputTime.mSampleTime >= 0) (*drv)->DoIOOperation(drv, dev, 4, 1, kAudioServerPlugInIOOperationReadInput, 512, &ci, inbuf, NULL);
        (*drv)->DoIOOperation(drv, dev, 3, 1, kAudioServerPlugInIOOperationWriteMix, 512, &ci, buf, NULL);
        X += 512;
        cycles++;
    }
    (*drv)->StopIO(drv, dev, 1);
    printf("IO: %d cycles completed, last zero timestamp sample %.0f\n", cycles, zs);
    return 0;
}
