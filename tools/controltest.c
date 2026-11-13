// controltest — load EMU0202.driver with a fake host and check the output volume and mute controls.
// Does not start IO, so it can run while the installed driver is playing.
// Usage: controltest build/EMU0202.driver
#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

static AudioServerPlugInDriverRef drv;
static CFMutableDictionaryRef storage;
static int failures;

#define CHECK(cond, ...)                         \
    do {                                         \
        printf("%s ", (cond) ? "ok  " : "FAIL"); \
        printf(__VA_ARGS__);                     \
        printf("\n");                            \
        if (!(cond)) failures++;                 \
    } while (0)

static OSStatus hPropertiesChanged(AudioServerPlugInHostRef h, AudioObjectID obj, UInt32 n, const AudioObjectPropertyAddress *a) {
    return 0;
}
static OSStatus hCopy(AudioServerPlugInHostRef h, CFStringRef k, CFPropertyListRef *d) {
    CFPropertyListRef v = CFDictionaryGetValue(storage, k);
    *d = v ? CFRetain(v) : NULL;
    return 0;
}
static OSStatus hWrite(AudioServerPlugInHostRef h, CFStringRef k, CFPropertyListRef d) {
    CFDictionarySetValue(storage, k, d);
    return 0;
}
static OSStatus hDelete(AudioServerPlugInHostRef h, CFStringRef k) {
    CFDictionaryRemoveValue(storage, k);
    return 0;
}
static OSStatus hRequest(AudioServerPlugInHostRef h, AudioObjectID dev, UInt64 action, void *info) { return 0; }
static AudioServerPlugInHostInterface host = {hPropertiesChanged, hCopy, hWrite, hDelete, hRequest};

static OSStatus get(AudioObjectID obj, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope, UInt32 qSize,
                    const void *q, void *out, UInt32 *size) {
    AudioObjectPropertyAddress a = {sel, scope, kAudioObjectPropertyElementMain};
    UInt32 need = 0;
    OSStatus e = (*drv)->GetPropertyDataSize(drv, obj, 0, &a, qSize, q, &need);
    if (e) return e;
    if (need > *size) return kAudioHardwareBadPropertySizeError;
    return (*drv)->GetPropertyData(drv, obj, 0, &a, qSize, q, *size, size, out);
}

static Float32 getF(AudioObjectID obj, AudioObjectPropertySelector sel) {
    Float32 v = NAN;
    UInt32 size = sizeof v;
    get(obj, sel, kAudioObjectPropertyScopeGlobal, 0, NULL, &v, &size);
    return v;
}

static UInt32 getU(AudioObjectID obj, AudioObjectPropertySelector sel) {
    UInt32 v = 0xFFFFFFFF;
    UInt32 size = sizeof v;
    get(obj, sel, kAudioObjectPropertyScopeGlobal, 0, NULL, &v, &size);
    return v;
}

static Float32 convert(AudioObjectPropertySelector sel, Float32 in) {
    UInt32 size = sizeof in;
    get(5, sel, kAudioObjectPropertyScopeGlobal, 0, NULL, &in, &size);
    return in;
}

static OSStatus set(AudioObjectID obj, AudioObjectPropertySelector sel, UInt32 size, const void *data) {
    AudioObjectPropertyAddress a = {sel, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    return (*drv)->SetPropertyData(drv, obj, 0, &a, 0, NULL, size, data);
}

static Boolean settable(AudioObjectID obj, AudioObjectPropertySelector sel) {
    AudioObjectPropertyAddress a = {sel, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    Boolean s = false;
    (*drv)->IsPropertySettable(drv, obj, 0, &a, &s);
    return s;
}

static void listIDs(const char *label, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope, AudioClassID qualifier,
                    const char *expected) {
    AudioObjectID ids[8];
    UInt32 size = sizeof ids;
    OSStatus e = get(2, sel, scope, qualifier ? sizeof qualifier : 0, qualifier ? &qualifier : NULL, ids, &size);
    char got[64] = "";
    for (UInt32 i = 0; !e && i < size / sizeof(AudioObjectID); i++)
        snprintf(got + strlen(got), sizeof got - strlen(got), "%s%u", i ? "," : "", ids[i]);
    CHECK(!e && strcmp(got, expected) == 0, "%s: [%s] (expected [%s])", label, got, expected);
}

static Float32 storedVolume(void) {
    CFNumberRef n = CFDictionaryGetValue(storage, CFSTR("OutputVolume"));
    Float32 v = NAN;
    if (n) CFNumberGetValue(n, kCFNumberFloat32Type, &v);
    return v;
}

static int storedMute(void) {
    CFBooleanRef b = CFDictionaryGetValue(storage, CFSTR("OutputMute"));
    return b ? CFBooleanGetValue(b) : -1;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s path/to/EMU0202.driver\n", argv[0]); return 2; }
    storage = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    Float32 half = 0.5f;
    CFNumberRef n = CFNumberCreate(NULL, kCFNumberFloat32Type, &half);
    CFDictionarySetValue(storage, CFSTR("OutputVolume"), n);  // as if saved by a previous run
    CFRelease(n);

    CFURLRef url = CFURLCreateFromFileSystemRepresentation(NULL, (const UInt8 *)argv[1], (CFIndex)strlen(argv[1]), true);
    CFPlugInRef plugin = CFPlugInCreate(NULL, url);
    if (!plugin) { printf("cannot load bundle\n"); return 1; }
    CFArrayRef factories = CFPlugInFindFactoriesForPlugInType(kAudioServerPlugInTypeUUID);
    if (!factories || CFArrayGetCount(factories) == 0) { printf("no factories\n"); return 1; }
    void *unk = CFPlugInInstanceCreate(NULL, (CFUUIDRef)CFArrayGetValueAtIndex(factories, 0), kAudioServerPlugInTypeUUID);
    IUnknownVTbl **iu = unk;
    (*iu)->QueryInterface(unk, CFUUIDGetUUIDBytes(kAudioServerPlugInDriverInterfaceUUID), (LPVOID *)&drv);
    if ((*drv)->Initialize(drv, &host) != 0) { printf("Initialize failed\n"); return 1; }

    // Objects
    listIDs("control list", kAudioObjectPropertyControlList, kAudioObjectPropertyScopeGlobal, 0, "5,6");
    listIDs("owned, global", kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, 0, "3,4,5,6");
    listIDs("owned, output", kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeOutput, 0, "3,5,6");
    listIDs("owned, input", kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeInput, 0, "4");
    listIDs("owned, controls only", kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, kAudioControlClassID, "5,6");
    listIDs("owned, volume only", kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, kAudioVolumeControlClassID, "5");
    listIDs("owned, streams only", kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, kAudioStreamClassID, "3,4");
    listIDs("streams, output", kAudioDevicePropertyStreams, kAudioObjectPropertyScopeOutput, 0, "3");

    // Volume control
    CHECK(getU(5, kAudioObjectPropertyClass) == kAudioVolumeControlClassID, "volume: class");
    CHECK(getU(5, kAudioObjectPropertyBaseClass) == kAudioLevelControlClassID, "volume: base class");
    CHECK(getU(5, kAudioObjectPropertyOwner) == 2, "volume: owner is the device");
    CHECK(getU(5, kAudioControlPropertyScope) == kAudioObjectPropertyScopeOutput, "volume: output scope");
    CHECK(getU(5, kAudioControlPropertyElement) == kAudioObjectPropertyElementMain, "volume: main element");
    CHECK(settable(5, kAudioLevelControlPropertyScalarValue) && settable(5, kAudioLevelControlPropertyDecibelValue),
          "volume: scalar and dB settable");
    CHECK(!settable(5, kAudioLevelControlPropertyDecibelRange), "volume: range read-only");
    AudioValueRange r = {0, 0};
    UInt32 size = sizeof r;
    get(5, kAudioLevelControlPropertyDecibelRange, kAudioObjectPropertyScopeGlobal, 0, NULL, &r, &size);
    CHECK(r.mMinimum == -96 && r.mMaximum == 0, "volume: range %.0f..%.0f dB", r.mMinimum, r.mMaximum);
    Float32 s = getF(5, kAudioLevelControlPropertyScalarValue), db = getF(5, kAudioLevelControlPropertyDecibelValue);
    CHECK(s == 0.5f, "volume: restored from storage, scalar %.3f", s);
    CHECK(fabsf(db - -18.06f) < 0.01f, "volume: half way is %.2f dB", db);
    db = convert(kAudioLevelControlPropertyConvertScalarToDecibels, 0.25f);
    CHECK(fabsf(db - -36.12f) < 0.01f, "convert: scalar 0.25 -> %.2f dB", db);
    db = convert(kAudioLevelControlPropertyConvertScalarToDecibels, 0.0f);
    CHECK(db == -96.f, "convert: scalar 0 -> %.1f dB", db);
    s = convert(kAudioLevelControlPropertyConvertDecibelsToScalar, -18.0618f);
    CHECK(fabsf(s - 0.5f) < 0.001f, "convert: -18.06 dB -> scalar %.4f", s);
    s = convert(kAudioLevelControlPropertyConvertDecibelsToScalar, 6.f);
    CHECK(s == 1.f, "convert: +6 dB clamps to scalar %.1f", s);

    Float32 v = 0.75f;
    CHECK(set(5, kAudioLevelControlPropertyScalarValue, sizeof v, &v) == 0, "volume: set scalar 0.75");
    s = getF(5, kAudioLevelControlPropertyScalarValue);
    CHECK(s == 0.75f, "volume: reads back %.3f (%.2f dB)", s, getF(5, kAudioLevelControlPropertyDecibelValue));
    v = -96.f;
    set(5, kAudioLevelControlPropertyDecibelValue, sizeof v, &v);
    CHECK(getF(5, kAudioLevelControlPropertyScalarValue) == 0.f, "volume: -96 dB is the bottom (scalar 0)");
    v = 1.7f;
    set(5, kAudioLevelControlPropertyScalarValue, sizeof v, &v);
    CHECK(getF(5, kAudioLevelControlPropertyScalarValue) == 1.f, "volume: scalar 1.7 clamps to 1");
    v = 0.625f;
    set(5, kAudioLevelControlPropertyScalarValue, sizeof v, &v);
    usleep(200000);  // storage is written on a background queue
    CHECK(storedVolume() == 0.625f, "volume: saved to storage (%.3f)", storedVolume());
    UInt16 wrong = 1;
    CHECK(set(5, kAudioLevelControlPropertyScalarValue, sizeof wrong, &wrong) == kAudioHardwareBadPropertySizeError,
          "volume: wrong data size rejected");

    // Mute control
    CHECK(getU(6, kAudioObjectPropertyClass) == kAudioMuteControlClassID, "mute: class");
    CHECK(getU(6, kAudioObjectPropertyBaseClass) == kAudioBooleanControlClassID, "mute: base class");
    CHECK(getU(6, kAudioControlPropertyScope) == kAudioObjectPropertyScopeOutput, "mute: output scope");
    CHECK(settable(6, kAudioBooleanControlPropertyValue), "mute: settable");
    CHECK(getU(6, kAudioBooleanControlPropertyValue) == 0, "mute: off by default");
    UInt32 m = 1;
    set(6, kAudioBooleanControlPropertyValue, sizeof m, &m);
    usleep(200000);
    CHECK(getU(6, kAudioBooleanControlPropertyValue) == 1 && storedMute() == 1, "mute: on, saved");
    CHECK(getF(5, kAudioLevelControlPropertyScalarValue) == 0.625f, "mute: volume position kept while muted");
    m = 0;
    set(6, kAudioBooleanControlPropertyValue, sizeof m, &m);
    usleep(200000);
    CHECK(getU(6, kAudioBooleanControlPropertyValue) == 0 && storedMute() == 0, "mute: off, saved");

    // Properties that belong to the other control
    AudioObjectPropertyAddress a = {kAudioBooleanControlPropertyValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    CHECK(!(*drv)->HasProperty(drv, 5, 0, &a), "volume has no mute value");
    a.mSelector = kAudioLevelControlPropertyScalarValue;
    CHECK(!(*drv)->HasProperty(drv, 6, 0, &a), "mute has no level value");

    printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
