// EMUEngine — user-space USB streaming engine for E-MU 0202 USB (041e:3f02).
// Full duplex: stereo playback and stereo capture on the card's clock, through IOUSBHost (no kext).
// Used by the Core Audio plug-in and by the test tools.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EMU_RING_FRAMES 16384u  // power of two; also the Core Audio zero-timestamp period
#define EMU_CHANNELS 2

typedef struct EMUEngine EMUEngine;
typedef void (*EMUPresenceCallback)(void *ctx, bool present);

typedef struct {
    uint64_t packets;         // iso OUT packets completed
    uint64_t errors;          // failed OUT packets / transfers
    uint64_t resyncs;         // times the OUT schedule fell behind and was re-anchored
    uint64_t feedback;        // valid feedback values received
    uint64_t underruns;       // OUT packets that needed samples the host had not written yet
    uint64_t lateWrites;      // host writes that arrived after the samples were sent
    uint64_t lateFills;       // OUT packets filled too close to their transmission time
    uint64_t inPackets;       // iso IN packets captured
    uint64_t inErrors;        // failed IN packets
    uint64_t inRealigns;      // times the capture position was re-derived from the clock
    uint64_t inUnderruns;     // host reads that asked for samples not captured yet
    double deviceRate;        // sample rate reported by the card's feedback endpoint
    double inPickupMaxMs;     // worst delay from t(F) until the pump picked up IN packet F
    double maxCallbackDelayMs;
    uint64_t slowCallbacks;
} EMUEngineStats;

EMUEngine *EMUEngineCreate(void);
// Hot-plug monitoring. The callback is invoked on a background queue.
void EMUEngineStartMonitoring(EMUEngine *e, EMUPresenceCallback cb, void *ctx);
bool EMUEngineIsPresent(EMUEngine *e);
bool EMUEngineIsSupportedRate(double rate);

int EMUEngineStart(EMUEngine *e, double sampleRate);  // 0 on success; streams both directions
void EMUEngineStop(EMUEngine *e);
bool EMUEngineIsRunning(EMUEngine *e);

uint32_t EMUEngineOutputLatencyFrames(double rate);
uint32_t EMUEngineOutputSafetyOffsetFrames(double rate);
uint32_t EMUEngineInputLatencyFrames(double rate);
uint32_t EMUEngineInputSafetyOffsetFrames(double rate);

// Real-time safe (called from the Core Audio IO thread). Buffers are interleaved stereo float.
void EMUEngineWriteOutput(EMUEngine *e, double sampleTime, const float *frames, uint32_t frameCount);
void EMUEngineReadInput(EMUEngine *e, double sampleTime, float *frames, uint32_t frameCount);
void EMUEngineGetZeroTimeStamp(EMUEngine *e, double *sampleTime, uint64_t *hostTime, uint64_t *seed);

// Linear gain applied to everything written with EMUEngineWriteOutput (1 = unchanged, 0 = silence).
// Any thread; changes are ramped over one IO buffer.
void EMUEngineSetOutputGain(EMUEngine *e, float gain);

void EMUEngineGetStats(EMUEngine *e, EMUEngineStats *s);

#ifdef __cplusplus
}
#endif
