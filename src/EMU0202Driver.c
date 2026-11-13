// EMU0202Driver.c — Core Audio AudioServerPlugIn for the E-MU 0202 USB.
//
// Publishes one device ("E-MU 0202 | USB") with a stereo output stream and a stereo input stream
// while the card is connected. Audio goes through EMUEngine, which talks to the card over IOUSBHost.
//
// Object IDs: 1 = plug-in, 2 = device, 3 = output stream, 4 = input stream,
//             5 = output volume control, 6 = output mute control.
//
// Volume and mute are applied in software (EMUEngineSetOutputGain) and remembered in the host's storage.

#include <CoreAudio/AudioServerPlugIn.h>
#include <dispatch/dispatch.h>
#include <math.h>
#include <os/log.h>
#include <pthread.h>
#include <stddef.h>
#include <string.h>
#include "EMUEngine.h"

enum {
    kObjectID_PlugIn = kAudioObjectPlugInObject,
    kObjectID_Device = 2,
    kObjectID_Stream_Output = 3,
    kObjectID_Stream_Input = 4,
    kObjectID_Volume_Output = 5,
    kObjectID_Mute_Output = 6,
};

#define kDeviceUID "EMU0202USB:UserSpace"
#define kModelUID "EMU0202USB:Model"

static const Float64 kSampleRates[] = {44100.0, 48000.0, 88200.0, 96000.0};
#define kNumSampleRates (sizeof kSampleRates / sizeof kSampleRates[0])

static pthread_mutex_t gStateMutex = PTHREAD_MUTEX_INITIALIZER;
static AudioServerPlugInHostRef gHost = NULL;
static UInt32 gRefCount = 0;
static EMUEngine *gEngine = NULL;
static Float64 gSampleRate = 48000.0;
static bool gPresent = false;
#define kMaxIOClients 64
static UInt32 gIOClientIDs[kMaxIOClients];  // clients that started IO on the current connection
static UInt32 gIOClients = 0;
static bool gOutputActive = true;
static bool gInputActive = true;
static Float32 gVolume = 1.0f;  // output volume slider position, 0..1
static bool gMute = false;
static dispatch_queue_t gStorageQueue;
static os_log_t gLog;

#define kStorageKeyVolume CFSTR("OutputVolume")
#define kStorageKeyMute CFSTR("OutputMute")

#pragma mark Prototypes

static HRESULT EMU_QueryInterface(void *inDriver, REFIID inUUID, LPVOID *outInterface);
static ULONG EMU_AddRef(void *inDriver);
static ULONG EMU_Release(void *inDriver);
static OSStatus EMU_Initialize(AudioServerPlugInDriverRef inDriver, AudioServerPlugInHostRef inHost);
static OSStatus EMU_CreateDevice(AudioServerPlugInDriverRef inDriver, CFDictionaryRef inDescription,
                                 const AudioServerPlugInClientInfo *inClientInfo, AudioObjectID *outDeviceObjectID);
static OSStatus EMU_DestroyDevice(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID);
static OSStatus EMU_AddDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                    const AudioServerPlugInClientInfo *inClientInfo);
static OSStatus EMU_RemoveDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                       const AudioServerPlugInClientInfo *inClientInfo);
static OSStatus EMU_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                                     UInt64 inChangeAction, void *inChangeInfo);
static OSStatus EMU_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                                   UInt64 inChangeAction, void *inChangeInfo);
static Boolean EMU_HasProperty(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                               const AudioObjectPropertyAddress *inAddress);
static OSStatus EMU_IsPropertySettable(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                       const AudioObjectPropertyAddress *inAddress, Boolean *outIsSettable);
static OSStatus EMU_GetPropertyDataSize(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                        const AudioObjectPropertyAddress *inAddress, UInt32 inQualifierDataSize,
                                        const void *inQualifierData, UInt32 *outDataSize);
static OSStatus EMU_GetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                    const AudioObjectPropertyAddress *inAddress, UInt32 inQualifierDataSize,
                                    const void *inQualifierData, UInt32 inDataSize, UInt32 *outDataSize, void *outData);
static OSStatus EMU_SetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                    const AudioObjectPropertyAddress *inAddress, UInt32 inQualifierDataSize,
                                    const void *inQualifierData, UInt32 inDataSize, const void *inData);
static OSStatus EMU_StartIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID);
static OSStatus EMU_StopIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID);
static OSStatus EMU_GetZeroTimeStamp(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                     Float64 *outSampleTime, UInt64 *outHostTime, UInt64 *outSeed);
static OSStatus EMU_WillDoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                      UInt32 inOperationID, Boolean *outWillDo, Boolean *outWillDoInPlace);
static OSStatus EMU_BeginIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                     UInt32 inOperationID, UInt32 inIOBufferFrameSize,
                                     const AudioServerPlugInIOCycleInfo *inIOCycleInfo);
static OSStatus EMU_DoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                  AudioObjectID inStreamObjectID, UInt32 inClientID, UInt32 inOperationID,
                                  UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo *inIOCycleInfo,
                                  void *ioMainBuffer, void *ioSecondaryBuffer);
static OSStatus EMU_EndIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                   UInt32 inOperationID, UInt32 inIOBufferFrameSize,
                                   const AudioServerPlugInIOCycleInfo *inIOCycleInfo);

static AudioServerPlugInDriverInterface gInterface = {
    NULL,
    EMU_QueryInterface,
    EMU_AddRef,
    EMU_Release,
    EMU_Initialize,
    EMU_CreateDevice,
    EMU_DestroyDevice,
    EMU_AddDeviceClient,
    EMU_RemoveDeviceClient,
    EMU_PerformDeviceConfigurationChange,
    EMU_AbortDeviceConfigurationChange,
    EMU_HasProperty,
    EMU_IsPropertySettable,
    EMU_GetPropertyDataSize,
    EMU_GetPropertyData,
    EMU_SetPropertyData,
    EMU_StartIO,
    EMU_StopIO,
    EMU_GetZeroTimeStamp,
    EMU_WillDoIOOperation,
    EMU_BeginIOOperation,
    EMU_DoIOOperation,
    EMU_EndIOOperation,
};
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;

#pragma mark Factory

__attribute__((visibility("default"))) void *EMU0202_Create(CFAllocatorRef inAllocator, CFUUIDRef inRequestedTypeUUID) {
    (void)inAllocator;
    if (CFEqual(inRequestedTypeUUID, kAudioServerPlugInTypeUUID)) return gDriverRef;
    return NULL;
}

#pragma mark Helpers

static bool isSupportedRate(Float64 r) {
    for (size_t i = 0; i < kNumSampleRates; i++)
        if (kSampleRates[i] == r) return true;
    return false;
}

static void fillFormat(AudioStreamBasicDescription *d, Float64 rate) {
    memset(d, 0, sizeof *d);
    d->mSampleRate = rate;
    d->mFormatID = kAudioFormatLinearPCM;
    d->mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
    d->mBytesPerPacket = 8;
    d->mFramesPerPacket = 1;
    d->mBytesPerFrame = 8;
    d->mChannelsPerFrame = 2;
    d->mBitsPerChannel = 32;
}

static bool isStream(AudioObjectID o) { return o == kObjectID_Stream_Output || o == kObjectID_Stream_Input; }

// Streams of the device visible in the given scope; returns the count written to out (max 2).
static UInt32 streamsForScope(AudioObjectPropertyScope scope, AudioObjectID *out) {
    UInt32 n = 0;
    if (scope == kAudioObjectPropertyScopeGlobal || scope == kAudioObjectPropertyScopeOutput) out[n++] = kObjectID_Stream_Output;
    if (scope == kAudioObjectPropertyScopeGlobal || scope == kAudioObjectPropertyScopeInput) out[n++] = kObjectID_Stream_Input;
    return n;
}

static void notify(AudioObjectID obj, UInt32 n, const AudioObjectPropertyAddress *addrs) {
    AudioServerPlugInHostRef host = gHost;
    if (!host) return;
    AudioObjectPropertyAddress *copy = malloc(n * sizeof *copy);
    memcpy(copy, addrs, n * sizeof *copy);
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_DEFAULT, 0), ^{
        host->PropertiesChanged(host, obj, n, copy);
        free(copy);
    });
}

// OwnedObjects may carry a qualifier: an array of class IDs the caller is interested in
// (e.g. only controls). Returns true when objects of class `cls` should be listed.
static bool qualifierAllows(UInt32 size, const void *q, AudioClassID cls) {
    if (!q || size < sizeof(AudioClassID)) return true;
    const AudioClassID *classes = q;
    for (UInt32 i = 0; i < size / sizeof(AudioClassID); i++)
        if (classes[i] == cls || classes[i] == kAudioObjectClassID) return true;
    return false;
}

static void presenceChanged(void *ctx, bool present) {
    (void)ctx;
    pthread_mutex_lock(&gStateMutex);
    gPresent = present;
    bool wasRunning = gIOClients > 0;
    if (!present) gIOClients = 0;  // the device object goes away; clients start again on the new one
    pthread_mutex_unlock(&gStateMutex);
    if (!present && wasRunning) EMUEngineStop(gEngine);
    os_log(gLog, "device %{public}s", present ? "published" : "removed");
    AudioServerPlugInHostRef host = gHost;
    if (!host) return;
    // In order: a departing device reports that it is no longer alive while it still exists,
    // then the plug-in's device list changes.
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_DEFAULT, 0), ^{
        if (!present) {
            AudioObjectPropertyAddress alive = {kAudioDevicePropertyDeviceIsAlive, kAudioObjectPropertyScopeGlobal,
                                                kAudioObjectPropertyElementMain};
            host->PropertiesChanged(host, kObjectID_Device, 1, &alive);
        }
        AudioObjectPropertyAddress list[] = {
            {kAudioPlugInPropertyDeviceList, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
            {kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
        };
        host->PropertiesChanged(host, kObjectID_PlugIn, 2, list);
    });
}

#pragma mark Volume

// Slider position s (0..1) maps to gain s^3, i.e. 60*log10(s) dB: half way is -18 dB, and the 16 steps of the
// volume keys go 0, -1.7, -3.5 ... -36 (at 1/4) ... -72 dB. The bottom of the slider is silence.
#define kVolumeMinDB -96.0f

static Float32 clamp01(Float32 v) { return v > 0.f ? (v < 1.f ? v : 1.f) : 0.f; }  // NaN -> 0

static Float32 volumeToDB(Float32 s) {
    s = clamp01(s);
    Float32 db = s > 0.f ? 60.f * log10f(s) : kVolumeMinDB;
    return db < kVolumeMinDB ? kVolumeMinDB : db;
}

static Float32 dbToVolume(Float32 db) {
    if (!(db > kVolumeMinDB)) return 0.f;
    if (db >= 0.f) return 1.f;
    return powf(10.f, db / 60.f);
}

// Call with gStateMutex held.
static void applyGainLocked(void) {
    Float32 s = clamp01(gVolume);
    EMUEngineSetOutputGain(gEngine, gMute ? 0.f : s * s * s);
}

static void loadControls(void) {
    AudioServerPlugInHostRef host = gHost;
    CFPropertyListRef v = NULL;
    if (host && host->CopyFromStorage(host, kStorageKeyVolume, &v) == 0 && v) {
        Float32 f;
        if (CFGetTypeID(v) == CFNumberGetTypeID() && CFNumberGetValue(v, kCFNumberFloat32Type, &f)) gVolume = clamp01(f);
        CFRelease(v);
    }
    v = NULL;
    if (host && host->CopyFromStorage(host, kStorageKeyMute, &v) == 0 && v) {
        if (CFGetTypeID(v) == CFBooleanGetTypeID()) gMute = CFBooleanGetValue(v);
        CFRelease(v);
    }
    pthread_mutex_lock(&gStateMutex);
    applyGainLocked();
    pthread_mutex_unlock(&gStateMutex);
    os_log(gLog, "output volume %.3f (%.1f dB)%{public}s", gVolume, volumeToDB(gVolume), gMute ? ", muted" : "");
}

// Writes the current values on a serial queue, so the latest value is always the one stored.
static void saveControls(void) {
    AudioServerPlugInHostRef host = gHost;
    if (!host) return;
    dispatch_async(gStorageQueue, ^{
        pthread_mutex_lock(&gStateMutex);
        Float32 vol = gVolume;
        bool mute = gMute;
        pthread_mutex_unlock(&gStateMutex);
        CFNumberRef n = CFNumberCreate(NULL, kCFNumberFloat32Type, &vol);
        host->WriteToStorage(host, kStorageKeyVolume, n);
        CFRelease(n);
        host->WriteToStorage(host, kStorageKeyMute, mute ? kCFBooleanTrue : kCFBooleanFalse);
    });
}

// Objects the device owns in `scope`, filtered by an OwnedObjects class qualifier; out holds up to 4.
static UInt32 deviceOwnedObjects(AudioObjectPropertyScope scope, UInt32 qSize, const void *q, AudioObjectID *out) {
    AudioObjectID streams[2];
    UInt32 ns = streamsForScope(scope, streams), n = 0;
    if (qualifierAllows(qSize, q, kAudioStreamClassID))
        for (UInt32 i = 0; i < ns; i++) out[n++] = streams[i];
    if (scope == kAudioObjectPropertyScopeGlobal || scope == kAudioObjectPropertyScopeOutput) {
        if (qualifierAllows(qSize, q, kAudioControlClassID) || qualifierAllows(qSize, q, kAudioLevelControlClassID) ||
            qualifierAllows(qSize, q, kAudioVolumeControlClassID))
            out[n++] = kObjectID_Volume_Output;
        if (qualifierAllows(qSize, q, kAudioControlClassID) || qualifierAllows(qSize, q, kAudioBooleanControlClassID) ||
            qualifierAllows(qSize, q, kAudioMuteControlClassID))
            out[n++] = kObjectID_Mute_Output;
    }
    return n;
}

static void requestRateChange(Float64 rate) {
    AudioServerPlugInHostRef host = gHost;
    if (!host) return;
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_DEFAULT, 0), ^{
        host->RequestDeviceConfigurationChange(host, kObjectID_Device, (UInt64)rate, NULL);
    });
}

#pragma mark IUnknown

static HRESULT EMU_QueryInterface(void *inDriver, REFIID inUUID, LPVOID *outInterface) {
    if (inDriver != gDriverRef || !outInterface) return kAudioHardwareIllegalOperationError;
    CFUUIDRef requested = CFUUIDCreateFromUUIDBytes(NULL, inUUID);
    HRESULT result = E_NOINTERFACE;
    if (CFEqual(requested, IUnknownUUID) || CFEqual(requested, kAudioServerPlugInDriverInterfaceUUID)) {
        pthread_mutex_lock(&gStateMutex);
        gRefCount++;
        pthread_mutex_unlock(&gStateMutex);
        *outInterface = gDriverRef;
        result = S_OK;
    }
    CFRelease(requested);
    return result;
}

static ULONG EMU_AddRef(void *inDriver) {
    if (inDriver != gDriverRef) return 0;
    pthread_mutex_lock(&gStateMutex);
    ULONG r = ++gRefCount;
    pthread_mutex_unlock(&gStateMutex);
    return r;
}

static ULONG EMU_Release(void *inDriver) {
    if (inDriver != gDriverRef) return 0;
    pthread_mutex_lock(&gStateMutex);
    if (gRefCount > 0) gRefCount--;
    ULONG r = gRefCount;
    pthread_mutex_unlock(&gStateMutex);
    return r;
}

#pragma mark Basic operations

static OSStatus EMU_Initialize(AudioServerPlugInDriverRef inDriver, AudioServerPlugInHostRef inHost) {
    if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
    gHost = inHost;
    gLog = os_log_create("ru.masatec.emu0202", "plugin");
    gStorageQueue = dispatch_queue_create("ru.masatec.emu0202.storage", DISPATCH_QUEUE_SERIAL);
    gEngine = EMUEngineCreate();
    loadControls();
    EMUEngineStartMonitoring(gEngine, presenceChanged, NULL);
    os_log(gLog, "E-MU 0202 user-space driver loaded");
    return kAudioHardwareNoError;
}

static OSStatus EMU_CreateDevice(AudioServerPlugInDriverRef inDriver, CFDictionaryRef inDescription,
                                 const AudioServerPlugInClientInfo *inClientInfo, AudioObjectID *outDeviceObjectID) {
    return kAudioHardwareUnsupportedOperationError;
}

static OSStatus EMU_DestroyDevice(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID) {
    return kAudioHardwareUnsupportedOperationError;
}

static OSStatus EMU_AddDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                    const AudioServerPlugInClientInfo *inClientInfo) {
    return inDeviceObjectID == kObjectID_Device ? kAudioHardwareNoError : kAudioHardwareBadObjectError;
}

static OSStatus EMU_RemoveDeviceClient(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                       const AudioServerPlugInClientInfo *inClientInfo) {
    return inDeviceObjectID == kObjectID_Device ? kAudioHardwareNoError : kAudioHardwareBadObjectError;
}

static OSStatus EMU_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                                     UInt64 inChangeAction, void *inChangeInfo) {
    if (inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
    Float64 rate = (Float64)inChangeAction;
    if (!isSupportedRate(rate)) return kAudioHardwareIllegalOperationError;
    pthread_mutex_lock(&gStateMutex);
    gSampleRate = rate;
    bool running = gIOClients > 0;
    pthread_mutex_unlock(&gStateMutex);
    if (running) EMUEngineStart(gEngine, rate);  // normally IO is stopped here; restart defensively
    os_log(gLog, "sample rate set to %.0f Hz", rate);
    return kAudioHardwareNoError;
}

static OSStatus EMU_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                                   UInt64 inChangeAction, void *inChangeInfo) {
    return kAudioHardwareNoError;
}

#pragma mark Properties

static Boolean EMU_HasProperty(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                               const AudioObjectPropertyAddress *a) {
    if (inDriver != gDriverRef || !a) return false;
    switch (inObjectID) {
    case kObjectID_PlugIn:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyManufacturer:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList:
        case kAudioPlugInPropertyTranslateUIDToDevice:
        case kAudioPlugInPropertyResourceBundle:
            return true;
        }
        return false;
    case kObjectID_Device:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyName:
        case kAudioObjectPropertyManufacturer:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioDevicePropertyDeviceUID:
        case kAudioDevicePropertyModelUID:
        case kAudioDevicePropertyTransportType:
        case kAudioDevicePropertyRelatedDevices:
        case kAudioDevicePropertyClockDomain:
        case kAudioDevicePropertyDeviceIsAlive:
        case kAudioDevicePropertyDeviceIsRunning:
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
        case kAudioDevicePropertyLatency:
        case kAudioDevicePropertyStreams:
        case kAudioObjectPropertyControlList:
        case kAudioDevicePropertySafetyOffset:
        case kAudioDevicePropertyNominalSampleRate:
        case kAudioDevicePropertyAvailableNominalSampleRates:
        case kAudioDevicePropertyIsHidden:
        case kAudioDevicePropertyZeroTimeStampPeriod:
        case kAudioDevicePropertyPreferredChannelsForStereo:
        case kAudioDevicePropertyPreferredChannelLayout:
            return true;
        }
        return false;
    case kObjectID_Stream_Output:
    case kObjectID_Stream_Input:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioStreamPropertyIsActive:
        case kAudioStreamPropertyDirection:
        case kAudioStreamPropertyTerminalType:
        case kAudioStreamPropertyStartingChannel:
        case kAudioStreamPropertyLatency:
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat:
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyAvailablePhysicalFormats:
            return true;
        }
        return false;
    case kObjectID_Volume_Output:
    case kObjectID_Mute_Output:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioControlPropertyScope:
        case kAudioControlPropertyElement:
            return true;
        case kAudioLevelControlPropertyScalarValue:
        case kAudioLevelControlPropertyDecibelValue:
        case kAudioLevelControlPropertyDecibelRange:
        case kAudioLevelControlPropertyConvertScalarToDecibels:
        case kAudioLevelControlPropertyConvertDecibelsToScalar:
            return inObjectID == kObjectID_Volume_Output;
        case kAudioBooleanControlPropertyValue:
            return inObjectID == kObjectID_Mute_Output;
        }
        return false;
    }
    return false;
}

static OSStatus EMU_IsPropertySettable(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                       const AudioObjectPropertyAddress *a, Boolean *outIsSettable) {
    if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
    if (!a || !outIsSettable) return kAudioHardwareIllegalOperationError;
    if (!EMU_HasProperty(inDriver, inObjectID, inClientProcessID, a)) return kAudioHardwareUnknownPropertyError;
    *outIsSettable = false;
    if (inObjectID == kObjectID_Device && a->mSelector == kAudioDevicePropertyNominalSampleRate) *outIsSettable = true;
    if (isStream(inObjectID) &&
        (a->mSelector == kAudioStreamPropertyIsActive || a->mSelector == kAudioStreamPropertyVirtualFormat ||
         a->mSelector == kAudioStreamPropertyPhysicalFormat))
        *outIsSettable = true;
    if (inObjectID == kObjectID_Volume_Output &&
        (a->mSelector == kAudioLevelControlPropertyScalarValue || a->mSelector == kAudioLevelControlPropertyDecibelValue))
        *outIsSettable = true;
    if (inObjectID == kObjectID_Mute_Output && a->mSelector == kAudioBooleanControlPropertyValue) *outIsSettable = true;
    return kAudioHardwareNoError;
}

static UInt32 deviceCount(void) {
    pthread_mutex_lock(&gStateMutex);
    UInt32 n = gPresent ? 1 : 0;
    pthread_mutex_unlock(&gStateMutex);
    return n;
}

static OSStatus EMU_GetPropertyDataSize(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                        const AudioObjectPropertyAddress *a, UInt32 inQualifierDataSize,
                                        const void *inQualifierData, UInt32 *outDataSize) {
    if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
    if (!a || !outDataSize) return kAudioHardwareIllegalOperationError;
    switch (inObjectID) {
    case kObjectID_PlugIn:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass: *outDataSize = sizeof(AudioClassID); return 0;
        case kAudioObjectPropertyOwner: *outDataSize = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyManufacturer:
        case kAudioPlugInPropertyResourceBundle: *outDataSize = sizeof(CFStringRef); return 0;
        case kAudioObjectPropertyOwnedObjects:
            *outDataSize = qualifierAllows(inQualifierDataSize, inQualifierData, kAudioDeviceClassID)
                               ? deviceCount() * sizeof(AudioObjectID) : 0;
            return 0;
        case kAudioPlugInPropertyDeviceList: *outDataSize = deviceCount() * sizeof(AudioObjectID); return 0;
        case kAudioPlugInPropertyTranslateUIDToDevice: *outDataSize = sizeof(AudioObjectID); return 0;
        }
        return kAudioHardwareUnknownPropertyError;
    case kObjectID_Device:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass: *outDataSize = sizeof(AudioClassID); return 0;
        case kAudioObjectPropertyOwner: *outDataSize = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyName:
        case kAudioObjectPropertyManufacturer:
        case kAudioDevicePropertyDeviceUID:
        case kAudioDevicePropertyModelUID: *outDataSize = sizeof(CFStringRef); return 0;
        case kAudioObjectPropertyOwnedObjects: {
            AudioObjectID ids[4];
            *outDataSize = deviceOwnedObjects(a->mScope, inQualifierDataSize, inQualifierData, ids) * sizeof(AudioObjectID);
            return 0;
        }
        case kAudioDevicePropertyStreams: {
            AudioObjectID ids[2];
            *outDataSize = streamsForScope(a->mScope, ids) * sizeof(AudioObjectID);
            return 0;
        }
        case kAudioObjectPropertyControlList: *outDataSize = 2 * sizeof(AudioObjectID); return 0;
        case kAudioDevicePropertyRelatedDevices: *outDataSize = sizeof(AudioObjectID); return 0;
        case kAudioDevicePropertyTransportType:
        case kAudioDevicePropertyClockDomain:
        case kAudioDevicePropertyDeviceIsAlive:
        case kAudioDevicePropertyDeviceIsRunning:
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
        case kAudioDevicePropertyLatency:
        case kAudioDevicePropertySafetyOffset:
        case kAudioDevicePropertyIsHidden:
        case kAudioDevicePropertyZeroTimeStampPeriod: *outDataSize = sizeof(UInt32); return 0;
        case kAudioDevicePropertyNominalSampleRate: *outDataSize = sizeof(Float64); return 0;
        case kAudioDevicePropertyAvailableNominalSampleRates: *outDataSize = kNumSampleRates * sizeof(AudioValueRange); return 0;
        case kAudioDevicePropertyPreferredChannelsForStereo: *outDataSize = 2 * sizeof(UInt32); return 0;
        case kAudioDevicePropertyPreferredChannelLayout:
            *outDataSize = (UInt32)(offsetof(AudioChannelLayout, mChannelDescriptions) + 2 * sizeof(AudioChannelDescription));
            return 0;
        }
        return kAudioHardwareUnknownPropertyError;
    case kObjectID_Stream_Output:
    case kObjectID_Stream_Input:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass: *outDataSize = sizeof(AudioClassID); return 0;
        case kAudioObjectPropertyOwner: *outDataSize = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyOwnedObjects: *outDataSize = 0; return 0;
        case kAudioStreamPropertyIsActive:
        case kAudioStreamPropertyDirection:
        case kAudioStreamPropertyTerminalType:
        case kAudioStreamPropertyStartingChannel:
        case kAudioStreamPropertyLatency: *outDataSize = sizeof(UInt32); return 0;
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat: *outDataSize = sizeof(AudioStreamBasicDescription); return 0;
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyAvailablePhysicalFormats:
            *outDataSize = kNumSampleRates * sizeof(AudioStreamRangedDescription);
            return 0;
        }
        return kAudioHardwareUnknownPropertyError;
    case kObjectID_Volume_Output:
    case kObjectID_Mute_Output:
        if (!EMU_HasProperty(inDriver, inObjectID, inClientProcessID, a)) return kAudioHardwareUnknownPropertyError;
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass: *outDataSize = sizeof(AudioClassID); return 0;
        case kAudioObjectPropertyOwner: *outDataSize = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyOwnedObjects: *outDataSize = 0; return 0;
        case kAudioControlPropertyScope: *outDataSize = sizeof(AudioObjectPropertyScope); return 0;
        case kAudioControlPropertyElement: *outDataSize = sizeof(AudioObjectPropertyElement); return 0;
        case kAudioLevelControlPropertyDecibelRange: *outDataSize = sizeof(AudioValueRange); return 0;
        case kAudioBooleanControlPropertyValue: *outDataSize = sizeof(UInt32); return 0;
        default: *outDataSize = sizeof(Float32); return 0;  // level values and conversions
        }
    }
    return kAudioHardwareBadObjectError;
}

#define NEED(sz)                                                         \
    do {                                                                 \
        if (inDataSize < (sz)) return kAudioHardwareBadPropertySizeError; \
        *outDataSize = (sz);                                             \
    } while (0)

static OSStatus EMU_GetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                    const AudioObjectPropertyAddress *a, UInt32 inQualifierDataSize,
                                    const void *inQualifierData, UInt32 inDataSize, UInt32 *outDataSize, void *outData) {
    if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
    if (!a || !outDataSize || !outData) return kAudioHardwareIllegalOperationError;

    pthread_mutex_lock(&gStateMutex);
    Float64 rate = gSampleRate;
    bool present = gPresent;
    bool running = gIOClients > 0;
    bool outActive = gOutputActive, inActive = gInputActive;
    Float32 volume = gVolume;
    bool mute = gMute;
    pthread_mutex_unlock(&gStateMutex);

    switch (inObjectID) {
    case kObjectID_PlugIn:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: NEED(sizeof(AudioClassID)); *(AudioClassID *)outData = kAudioObjectClassID; return 0;
        case kAudioObjectPropertyClass: NEED(sizeof(AudioClassID)); *(AudioClassID *)outData = kAudioPlugInClassID; return 0;
        case kAudioObjectPropertyOwner: NEED(sizeof(AudioObjectID)); *(AudioObjectID *)outData = kAudioObjectUnknown; return 0;
        case kAudioObjectPropertyManufacturer: NEED(sizeof(CFStringRef)); *(CFStringRef *)outData = CFSTR("masatec"); return 0;
        case kAudioPlugInPropertyResourceBundle: NEED(sizeof(CFStringRef)); *(CFStringRef *)outData = CFSTR(""); return 0;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList:
            if (a->mSelector == kAudioObjectPropertyOwnedObjects &&
                !qualifierAllows(inQualifierDataSize, inQualifierData, kAudioDeviceClassID)) {
                *outDataSize = 0;
                return 0;
            }
            if (present && inDataSize >= sizeof(AudioObjectID)) {
                *(AudioObjectID *)outData = kObjectID_Device;
                *outDataSize = sizeof(AudioObjectID);
            } else *outDataSize = 0;
            return 0;
        case kAudioPlugInPropertyTranslateUIDToDevice: {
            NEED(sizeof(AudioObjectID));
            AudioObjectID result = kAudioObjectUnknown;
            if (inQualifierDataSize == sizeof(CFStringRef) && inQualifierData) {
                CFStringRef uid = *(const CFStringRef *)inQualifierData;
                if (present && uid && CFStringCompare(uid, CFSTR(kDeviceUID), 0) == kCFCompareEqualTo) result = kObjectID_Device;
            }
            *(AudioObjectID *)outData = result;
            return 0;
        }
        }
        return kAudioHardwareUnknownPropertyError;

    case kObjectID_Device:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: NEED(sizeof(AudioClassID)); *(AudioClassID *)outData = kAudioObjectClassID; return 0;
        case kAudioObjectPropertyClass: NEED(sizeof(AudioClassID)); *(AudioClassID *)outData = kAudioDeviceClassID; return 0;
        case kAudioObjectPropertyOwner: NEED(sizeof(AudioObjectID)); *(AudioObjectID *)outData = kObjectID_PlugIn; return 0;
        case kAudioObjectPropertyName: NEED(sizeof(CFStringRef)); *(CFStringRef *)outData = CFSTR("E-MU 0202 | USB"); return 0;
        case kAudioObjectPropertyManufacturer: NEED(sizeof(CFStringRef)); *(CFStringRef *)outData = CFSTR("E-MU Systems"); return 0;
        case kAudioDevicePropertyDeviceUID: NEED(sizeof(CFStringRef)); *(CFStringRef *)outData = CFSTR(kDeviceUID); return 0;
        case kAudioDevicePropertyModelUID: NEED(sizeof(CFStringRef)); *(CFStringRef *)outData = CFSTR(kModelUID); return 0;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioDevicePropertyStreams:
        case kAudioObjectPropertyControlList: {
            AudioObjectID ids[4];
            UInt32 n;
            if (a->mSelector == kAudioObjectPropertyOwnedObjects)
                n = deviceOwnedObjects(a->mScope, inQualifierDataSize, inQualifierData, ids);
            else if (a->mSelector == kAudioDevicePropertyStreams) n = streamsForScope(a->mScope, ids);
            else {
                ids[0] = kObjectID_Volume_Output;
                ids[1] = kObjectID_Mute_Output;
                n = 2;
            }
            if (n > inDataSize / sizeof(AudioObjectID)) n = inDataSize / sizeof(AudioObjectID);
            memcpy(outData, ids, n * sizeof(AudioObjectID));
            *outDataSize = n * sizeof(AudioObjectID);
            return 0;
        }
        case kAudioDevicePropertyRelatedDevices: NEED(sizeof(AudioObjectID)); *(AudioObjectID *)outData = kObjectID_Device; return 0;
        case kAudioDevicePropertyTransportType: NEED(sizeof(UInt32)); *(UInt32 *)outData = kAudioDeviceTransportTypeUSB; return 0;
        case kAudioDevicePropertyClockDomain: NEED(sizeof(UInt32)); *(UInt32 *)outData = 0; return 0;
        case kAudioDevicePropertyDeviceIsAlive: NEED(sizeof(UInt32)); *(UInt32 *)outData = present ? 1 : 0; return 0;
        case kAudioDevicePropertyDeviceIsRunning: NEED(sizeof(UInt32)); *(UInt32 *)outData = running ? 1 : 0; return 0;
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
            NEED(sizeof(UInt32));
            *(UInt32 *)outData = 1;
            return 0;
        case kAudioDevicePropertyLatency:
            NEED(sizeof(UInt32));
            *(UInt32 *)outData = a->mScope == kAudioObjectPropertyScopeInput ? EMUEngineInputLatencyFrames(rate)
                                                                              : EMUEngineOutputLatencyFrames(rate);
            return 0;
        case kAudioDevicePropertySafetyOffset:
            NEED(sizeof(UInt32));
            *(UInt32 *)outData = a->mScope == kAudioObjectPropertyScopeInput ? EMUEngineInputSafetyOffsetFrames(rate)
                                                                              : EMUEngineOutputSafetyOffsetFrames(rate);
            return 0;
        case kAudioDevicePropertyIsHidden: NEED(sizeof(UInt32)); *(UInt32 *)outData = 0; return 0;
        case kAudioDevicePropertyZeroTimeStampPeriod: NEED(sizeof(UInt32)); *(UInt32 *)outData = EMU_RING_FRAMES; return 0;
        case kAudioDevicePropertyNominalSampleRate: NEED(sizeof(Float64)); *(Float64 *)outData = rate; return 0;
        case kAudioDevicePropertyAvailableNominalSampleRates: {
            UInt32 n = inDataSize / sizeof(AudioValueRange);
            if (n > kNumSampleRates) n = kNumSampleRates;
            AudioValueRange *r = outData;
            for (UInt32 i = 0; i < n; i++) r[i].mMinimum = r[i].mMaximum = kSampleRates[i];
            *outDataSize = n * sizeof(AudioValueRange);
            return 0;
        }
        case kAudioDevicePropertyPreferredChannelsForStereo:
            NEED(2 * sizeof(UInt32));
            ((UInt32 *)outData)[0] = 1;
            ((UInt32 *)outData)[1] = 2;
            return 0;
        case kAudioDevicePropertyPreferredChannelLayout: {
            UInt32 size = (UInt32)(offsetof(AudioChannelLayout, mChannelDescriptions) + 2 * sizeof(AudioChannelDescription));
            NEED(size);
            AudioChannelLayout *l = outData;
            memset(l, 0, size);
            l->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
            l->mNumberChannelDescriptions = 2;
            l->mChannelDescriptions[0].mChannelLabel = kAudioChannelLabel_Left;
            l->mChannelDescriptions[1].mChannelLabel = kAudioChannelLabel_Right;
            return 0;
        }
        }
        return kAudioHardwareUnknownPropertyError;

    case kObjectID_Stream_Output:
    case kObjectID_Stream_Input: {
        bool input = inObjectID == kObjectID_Stream_Input;
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: NEED(sizeof(AudioClassID)); *(AudioClassID *)outData = kAudioObjectClassID; return 0;
        case kAudioObjectPropertyClass: NEED(sizeof(AudioClassID)); *(AudioClassID *)outData = kAudioStreamClassID; return 0;
        case kAudioObjectPropertyOwner: NEED(sizeof(AudioObjectID)); *(AudioObjectID *)outData = kObjectID_Device; return 0;
        case kAudioObjectPropertyOwnedObjects: *outDataSize = 0; return 0;
        case kAudioStreamPropertyIsActive: NEED(sizeof(UInt32)); *(UInt32 *)outData = (input ? inActive : outActive) ? 1 : 0; return 0;
        case kAudioStreamPropertyDirection: NEED(sizeof(UInt32)); *(UInt32 *)outData = input ? 1 : 0; return 0;
        case kAudioStreamPropertyTerminalType: NEED(sizeof(UInt32)); *(UInt32 *)outData = kAudioStreamTerminalTypeLine; return 0;
        case kAudioStreamPropertyStartingChannel: NEED(sizeof(UInt32)); *(UInt32 *)outData = 1; return 0;
        case kAudioStreamPropertyLatency: NEED(sizeof(UInt32)); *(UInt32 *)outData = 0; return 0;
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat:
            NEED(sizeof(AudioStreamBasicDescription));
            fillFormat(outData, rate);
            return 0;
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyAvailablePhysicalFormats: {
            UInt32 n = inDataSize / sizeof(AudioStreamRangedDescription);
            if (n > kNumSampleRates) n = kNumSampleRates;
            AudioStreamRangedDescription *d = outData;
            for (UInt32 i = 0; i < n; i++) {
                fillFormat(&d[i].mFormat, kSampleRates[i]);
                d[i].mSampleRateRange.mMinimum = d[i].mSampleRateRange.mMaximum = kSampleRates[i];
            }
            *outDataSize = n * sizeof(AudioStreamRangedDescription);
            return 0;
        }
        }
        return kAudioHardwareUnknownPropertyError;
    }

    case kObjectID_Volume_Output:
    case kObjectID_Mute_Output: {
        if (!EMU_HasProperty(inDriver, inObjectID, inClientProcessID, a)) return kAudioHardwareUnknownPropertyError;
        bool isVolume = inObjectID == kObjectID_Volume_Output;
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
            NEED(sizeof(AudioClassID));
            *(AudioClassID *)outData = isVolume ? kAudioLevelControlClassID : kAudioBooleanControlClassID;
            return 0;
        case kAudioObjectPropertyClass:
            NEED(sizeof(AudioClassID));
            *(AudioClassID *)outData = isVolume ? kAudioVolumeControlClassID : kAudioMuteControlClassID;
            return 0;
        case kAudioObjectPropertyOwner: NEED(sizeof(AudioObjectID)); *(AudioObjectID *)outData = kObjectID_Device; return 0;
        case kAudioObjectPropertyOwnedObjects: *outDataSize = 0; return 0;
        case kAudioControlPropertyScope:
            NEED(sizeof(AudioObjectPropertyScope));
            *(AudioObjectPropertyScope *)outData = kAudioObjectPropertyScopeOutput;
            return 0;
        case kAudioControlPropertyElement:
            NEED(sizeof(AudioObjectPropertyElement));
            *(AudioObjectPropertyElement *)outData = kAudioObjectPropertyElementMain;
            return 0;
        case kAudioLevelControlPropertyScalarValue: NEED(sizeof(Float32)); *(Float32 *)outData = volume; return 0;
        case kAudioLevelControlPropertyDecibelValue: NEED(sizeof(Float32)); *(Float32 *)outData = volumeToDB(volume); return 0;
        case kAudioLevelControlPropertyDecibelRange:
            NEED(sizeof(AudioValueRange));
            ((AudioValueRange *)outData)->mMinimum = kVolumeMinDB;
            ((AudioValueRange *)outData)->mMaximum = 0.0;
            return 0;
        // Conversions: the value to convert comes in outData and is replaced with the result.
        case kAudioLevelControlPropertyConvertScalarToDecibels:
            NEED(sizeof(Float32));
            *(Float32 *)outData = volumeToDB(*(Float32 *)outData);
            return 0;
        case kAudioLevelControlPropertyConvertDecibelsToScalar:
            NEED(sizeof(Float32));
            *(Float32 *)outData = dbToVolume(*(Float32 *)outData);
            return 0;
        case kAudioBooleanControlPropertyValue: NEED(sizeof(UInt32)); *(UInt32 *)outData = mute ? 1 : 0; return 0;
        }
        return kAudioHardwareUnknownPropertyError;
    }
    }
    return kAudioHardwareBadObjectError;
}

static OSStatus EMU_SetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID,
                                    const AudioObjectPropertyAddress *a, UInt32 inQualifierDataSize,
                                    const void *inQualifierData, UInt32 inDataSize, const void *inData) {
    if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
    if (!a || !inData) return kAudioHardwareIllegalOperationError;

    if (inObjectID == kObjectID_Device && a->mSelector == kAudioDevicePropertyNominalSampleRate) {
        if (inDataSize != sizeof(Float64)) return kAudioHardwareBadPropertySizeError;
        Float64 r = *(const Float64 *)inData;
        if (!isSupportedRate(r)) return kAudioHardwareIllegalOperationError;
        pthread_mutex_lock(&gStateMutex);
        bool change = r != gSampleRate;
        pthread_mutex_unlock(&gStateMutex);
        if (change) requestRateChange(r);
        return kAudioHardwareNoError;
    }
    if (isStream(inObjectID)) {
        if (a->mSelector == kAudioStreamPropertyVirtualFormat || a->mSelector == kAudioStreamPropertyPhysicalFormat) {
            if (inDataSize != sizeof(AudioStreamBasicDescription)) return kAudioHardwareBadPropertySizeError;
            const AudioStreamBasicDescription *f = inData;
            if (f->mFormatID != kAudioFormatLinearPCM || f->mChannelsPerFrame != 2 || f->mBitsPerChannel != 32 ||
                !(f->mFormatFlags & kAudioFormatFlagIsFloat) || !isSupportedRate(f->mSampleRate))
                return kAudioDeviceUnsupportedFormatError;
            pthread_mutex_lock(&gStateMutex);
            bool change = f->mSampleRate != gSampleRate;
            pthread_mutex_unlock(&gStateMutex);
            if (change) requestRateChange(f->mSampleRate);
            return kAudioHardwareNoError;
        }
        if (a->mSelector == kAudioStreamPropertyIsActive) {
            if (inDataSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
            pthread_mutex_lock(&gStateMutex);
            if (inObjectID == kObjectID_Stream_Input) gInputActive = *(const UInt32 *)inData != 0;
            else gOutputActive = *(const UInt32 *)inData != 0;
            pthread_mutex_unlock(&gStateMutex);
            AudioObjectPropertyAddress addr = {kAudioStreamPropertyIsActive, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
            notify(inObjectID, 1, &addr);
            return kAudioHardwareNoError;
        }
    }
    if (inObjectID == kObjectID_Volume_Output &&
        (a->mSelector == kAudioLevelControlPropertyScalarValue || a->mSelector == kAudioLevelControlPropertyDecibelValue)) {
        if (inDataSize != sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
        Float32 v = *(const Float32 *)inData;
        Float32 s = a->mSelector == kAudioLevelControlPropertyScalarValue ? clamp01(v) : dbToVolume(v);
        pthread_mutex_lock(&gStateMutex);
        bool change = s != gVolume;
        gVolume = s;
        applyGainLocked();
        pthread_mutex_unlock(&gStateMutex);
        if (change) {
            AudioObjectPropertyAddress addrs[] = {
                {kAudioLevelControlPropertyScalarValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
                {kAudioLevelControlPropertyDecibelValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
            };
            notify(kObjectID_Volume_Output, 2, addrs);
            saveControls();
        }
        return kAudioHardwareNoError;
    }
    if (inObjectID == kObjectID_Mute_Output && a->mSelector == kAudioBooleanControlPropertyValue) {
        if (inDataSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
        bool m = *(const UInt32 *)inData != 0;
        pthread_mutex_lock(&gStateMutex);
        bool change = m != gMute;
        gMute = m;
        applyGainLocked();
        pthread_mutex_unlock(&gStateMutex);
        if (change) {
            AudioObjectPropertyAddress addr = {kAudioBooleanControlPropertyValue, kAudioObjectPropertyScopeGlobal,
                                               kAudioObjectPropertyElementMain};
            notify(kObjectID_Mute_Output, 1, &addr);
            saveControls();
        }
        return kAudioHardwareNoError;
    }
    return EMU_HasProperty(inDriver, inObjectID, inClientProcessID, a) ? kAudioHardwareUnsupportedOperationError
                                                                        : kAudioHardwareUnknownPropertyError;
}

#pragma mark IO

static OSStatus EMU_StartIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID) {
    if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&gStateMutex);
    OSStatus result = kAudioHardwareNoError;
    if (!gPresent) result = kAudioHardwareNotRunningError;
    else if (gIOClients >= kMaxIOClients) result = kAudioHardwareIllegalOperationError;
    else {
        // No-op while already streaming; restarts the engine after the card was reconnected.
        int r = EMUEngineStart(gEngine, gSampleRate);
        if (r) {
            os_log_error(gLog, "StartIO: engine start failed (%d)", r);
            result = kAudioHardwareUnspecifiedError;
        } else gIOClientIDs[gIOClients++] = inClientID;
    }
    pthread_mutex_unlock(&gStateMutex);
    return result;
}

static OSStatus EMU_StopIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID) {
    if (inDriver != gDriverRef || inDeviceObjectID != kObjectID_Device) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&gStateMutex);
    // Only clients that started IO on the current connection count; stale ones are ignored.
    for (UInt32 i = 0; i < gIOClients; i++) {
        if (gIOClientIDs[i] != inClientID) continue;
        gIOClientIDs[i] = gIOClientIDs[--gIOClients];
        if (gIOClients == 0) EMUEngineStop(gEngine);
        break;
    }
    pthread_mutex_unlock(&gStateMutex);
    return kAudioHardwareNoError;
}

static OSStatus EMU_GetZeroTimeStamp(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                     Float64 *outSampleTime, UInt64 *outHostTime, UInt64 *outSeed) {
    double s;
    uint64_t h, seed;
    EMUEngineGetZeroTimeStamp(gEngine, &s, &h, &seed);
    *outSampleTime = s;
    *outHostTime = h;
    *outSeed = seed;
    return kAudioHardwareNoError;
}

static OSStatus EMU_WillDoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                      UInt32 inOperationID, Boolean *outWillDo, Boolean *outWillDoInPlace) {
    *outWillDo = inOperationID == kAudioServerPlugInIOOperationWriteMix || inOperationID == kAudioServerPlugInIOOperationReadInput;
    *outWillDoInPlace = true;
    return kAudioHardwareNoError;
}

static OSStatus EMU_BeginIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                     UInt32 inOperationID, UInt32 inIOBufferFrameSize,
                                     const AudioServerPlugInIOCycleInfo *inIOCycleInfo) {
    return kAudioHardwareNoError;
}

static OSStatus EMU_DoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID,
                                  AudioObjectID inStreamObjectID, UInt32 inClientID, UInt32 inOperationID,
                                  UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo *inIOCycleInfo,
                                  void *ioMainBuffer, void *ioSecondaryBuffer) {
    if (!ioMainBuffer) return kAudioHardwareNoError;
    if (inOperationID == kAudioServerPlugInIOOperationWriteMix && inStreamObjectID == kObjectID_Stream_Output)
        EMUEngineWriteOutput(gEngine, inIOCycleInfo->mOutputTime.mSampleTime, (const float *)ioMainBuffer, inIOBufferFrameSize);
    else if (inOperationID == kAudioServerPlugInIOOperationReadInput && inStreamObjectID == kObjectID_Stream_Input)
        EMUEngineReadInput(gEngine, inIOCycleInfo->mInputTime.mSampleTime, (float *)ioMainBuffer, inIOBufferFrameSize);
    return kAudioHardwareNoError;
}

static OSStatus EMU_EndIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID,
                                   UInt32 inOperationID, UInt32 inIOBufferFrameSize,
                                   const AudioServerPlugInIOCycleInfo *inIOCycleInfo) {
    return kAudioHardwareNoError;
}
