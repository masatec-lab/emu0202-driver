// emutest — drive EMUEngine the way Core Audio does, without installing the plug-in.
// Every IO cycle (one buffer) on a real-time thread it reads input one safety offset + one buffer
// behind "now" and writes output one safety offset ahead, like the HAL does.
//
// Usage: emutest [--rate 48000] [--seconds 5] [--buffer 128] [--tone | --monitor]
//   --tone     play a quiet 440 Hz sine (-30 dBFS)
//   --monitor  send input 1 to both outputs (play guitar through the driver)
#import <Foundation/Foundation.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <math.h>
#include "../src/EMUEngine.h"

static void makeRealtime(double ticksPerSec, double periodSec) {
    thread_time_constraint_policy_data_t p;
    p.period = (uint32_t)(periodSec * ticksPerSec);
    p.computation = (uint32_t)(0.0003 * ticksPerSec);
    p.constraint = (uint32_t)(0.0008 * ticksPerSec);
    p.preemptible = 1;
    thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&p, THREAD_TIME_CONSTRAINT_POLICY_COUNT);
}

static double dbfs(float peak) { return 20 * log10(peak + 1e-9); }

int main(int argc, char **argv) {
    @autoreleasepool {
        double rate = 48000;
        int seconds = 5, tone = 0, monitor = 0;
        uint32_t buf = 128;
        for (int i = 1; i < argc; i++) {
            if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = atof(argv[++i]);
            else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atoi(argv[++i]);
            else if (!strcmp(argv[i], "--buffer") && i + 1 < argc) buf = (uint32_t)atoi(argv[++i]);
            else if (!strcmp(argv[i], "--tone")) tone = 1;
            else if (!strcmp(argv[i], "--monitor")) monitor = 1;
            else { fprintf(stderr, "usage: %s [--rate R] [--seconds N] [--buffer F] [--tone | --monitor]\n", argv[0]); return 2; }
        }
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        double ticksPerSec = 1e9 * tb.denom / tb.numer;

        EMUEngine *e = EMUEngineCreate();
        int r = EMUEngineStart(e, rate);
        if (r) { fprintf(stderr, "EMUEngineStart failed: %d\n", r); return 1; }
        uint32_t outSafety = EMUEngineOutputSafetyOffsetFrames(rate), inSafety = EMUEngineInputSafetyOffsetFrames(rate);
        uint32_t outLat = EMUEngineOutputLatencyFrames(rate), inLat = EMUEngineInputLatencyFrames(rate);
        double rtl = (inLat + inSafety + buf + buf + outSafety + outLat) / rate * 1000.0;
        printf("started: %.0f Hz, buffer %u, safety in/out %u/%u, latency in/out %u/%u frames -> round trip %.1f ms (+ converters)\n",
               rate, buf, inSafety, outSafety, inLat, outLat, rtl);
        printf("mode: %s\n", monitor ? "monitor (input 1 -> outputs)" : tone ? "440 Hz tone at -30 dBFS" : "silence");
        makeRealtime(ticksPerSec, buf / rate);

        float *out = calloc(buf * 2, sizeof(float)), *in = calloc(buf * 2, sizeof(float));
        double phase = 0, ztsS;
        uint64_t ztsH, seed;
        float peakL = 0, peakR = 0;
        EMUEngineGetZeroTimeStamp(e, &ztsS, &ztsH, &seed);
        double nowS = ztsS + (double)(int64_t)(mach_absolute_time() - ztsH) / ticksPerSec * rate;
        uint64_t X = ((uint64_t)(nowS > 0 ? nowS : 0) / buf + 1) * buf;
        uint64_t endHost = mach_absolute_time() + (uint64_t)(seconds * ticksPerSec);
        uint64_t cycles = 0, lateWakes = 0;

        while (mach_absolute_time() < endHost) {
            EMUEngineGetZeroTimeStamp(e, &ztsS, &ztsH, &seed);
            uint64_t wake = (uint64_t)((int64_t)ztsH + (int64_t)(((double)X - ztsS) / rate * ticksPerSec));
            mach_wait_until(wake);
            if (mach_absolute_time() > wake + (uint64_t)(0.0005 * ticksPerSec)) lateWakes++;

            double inStart = (double)X - inSafety - buf;
            if (inStart >= 0) EMUEngineReadInput(e, inStart, in, buf);
            for (uint32_t i = 0; i < buf; i++) {
                float l = in[2 * i], rr = in[2 * i + 1];
                if (fabsf(l) > peakL) peakL = fabsf(l);
                if (fabsf(rr) > peakR) peakR = fabsf(rr);
                float s = 0;
                if (monitor) s = l;
                else if (tone) {
                    s = (float)(sin(phase) * 0.0316);
                    phase += 2 * M_PI * 440.0 / rate;
                    if (phase > 2 * M_PI) phase -= 2 * M_PI;
                }
                out[2 * i] = s;
                out[2 * i + 1] = s;
            }
            EMUEngineWriteOutput(e, (double)(X + outSafety), out, buf);
            X += buf;
            cycles++;
        }

        EMUEngineStats st;
        EMUEngineGetStats(e, &st);
        EMUEngineStop(e);
        printf("output: packets %llu, errors %llu, resyncs %llu, underruns %llu, late writes %llu, late fills %llu\n",
               st.packets, st.errors, st.resyncs, st.underruns, st.lateWrites, st.lateFills);
        printf("input:  packets %llu, errors %llu, realigns %llu, underruns %llu, max pickup %.2f ms after t(F)\n", st.inPackets, st.inErrors,
               st.inRealigns, st.inUnderruns, st.inPickupMaxMs);
        printf("device rate %.2f Hz, IO cycles %llu (late wakes %llu)\n", st.deviceRate, cycles, lateWakes);
        printf("input peak: 1 (L) %.1f dBFS, 2 (R) %.1f dBFS\n", dbfs(peakL), dbfs(peakR));
    }
    return 0;
}
