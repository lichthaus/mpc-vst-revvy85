// Revvy85.cpp - MPC effect plugin around the REV7 emulator in ../../../core and ../../../frontend.
//
// SPDX-License-Identifier: MIT
//
// Signal flow:
//   in (44.1 kHz) -> Input Level -> [MONO: left only | STEREO: sum] -> HP -> LP -> SEM filter -> Engine: resample to
//   31.25 kHz -> emulated HD6303R + two YM3804 + YM3807 running the owner's firmware -> resample back
//   [Stages: D/A hold + fitted analogue sections] -> MUTE / BYPASS relays, OUT PHASE -> Mixing (dry
//   delayed to line up) -> Volume -> out
//
// Firmware: the EPROM image is NOT part of this plugin. It is read at run time from
// Content/roms/rev7_v1.0.bin (32 KB) inside the plugin folder, where the owner copies their own dump. Without it
// the plugin passes audio through and the display says what is missing.
//
// Threads: the firmware is booted (5 s of emulated time) on the UI thread in prepare() or
// loadExtraState(), in an engine of its own, then swapped in. process() only touches the engine it
// holds; a non-blocking flag keeps the two apart.
//
// Panel (skin from make_skin.py): every front-panel key is a switch parameter, and every change of
// value is one press of that key (held 80 ms). The display (32 "LCD n" cells), the two memory digits,
// the key LEDs (three LED columns) and the input meter are output parameters bound to film strips:
// the MPC redraws an image when its parameter moves, which it does reliably (it does not re-read
// value text). Program selects a memory with a MIDI program change into the emulated MIDI IN.
//
// State: the unit's battery-backed RAM (4 KB at 0x5000: user memories, the current program and
// edits, the MIDI table) is saved with the project. Loading it reboots the firmware with it.

#include "mpc_plugin.h"
#include "dsp/math.h"
#include "../../../frontend/engine.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

namespace {
struct KeyDef { const char* id; const char* name; int col, row; };
// The front-panel matrix (column = bit of 0x2002, row = bit of the 0x2004 read), names as printed on
// the panel; each was checked by pressing it in the emulator.
const KeyDef kKeys[] = {
    { "pk1", "REV1", 0, 0 }, { "pk2", "E/R1", 0, 1 }, { "pk3", "REV2", 0, 2 }, { "pk4", "E/R2", 0, 3 },
    { "pk5", "REV3", 0, 4 }, { "pk6", "OTHERS", 0, 5 }, { "pk7", "REV4", 0, 6 }, { "pk8", "USER MEMORY", 0, 7 },
    { "p1", "REV TIME", 1, 0 }, { "p2", "F1", 1, 1 }, { "p3", "INITIAL DELAY", 1, 2 }, { "p4", "F2", 1, 3 },
    { "p5", "1ST REF", 1, 4 }, { "p6", "F3", 1, 5 }, { "minus", "Data -", 1, 6 }, { "plus", "Data +", 1, 7 },
    { "d7", "7", 2, 0 }, { "d8", "8", 2, 1 }, { "d9", "9", 3, 0 }, { "memory", "MEMORY", 3, 1 },
    { "d4", "4", 2, 2 }, { "d5", "5", 2, 3 }, { "d6", "6", 3, 2 }, { "store", "STORE", 3, 3 },
    { "d1", "1", 2, 4 }, { "d2", "2", 2, 5 }, { "d3", "3", 3, 4 }, { "recall", "RECALL", 3, 5 },
    { "d0", "0", 2, 6 }, { "dot", ".", 2, 7 }, { "clear", "CLEAR", 3, 6 }, { "enter", "ENTER", 3, 7 },
    { "mute", "MUTE", 4, 0 }, { "outphase", "OUT PHASE", 4, 1 }, { "midikey", "MIDI CTRL", 4, 2 }, { "bypass", "BYPASS", 4, 3 },
};
constexpr int kNumKeys = (int)(sizeof kKeys / sizeof kKeys[0]);

// CRC-32 (IEEE) of the EPROM dump. Only the V1.0 dump this was built and measured with is known; the
// fitted constants are keyed to its DSP programs, so another version runs, with a warning.
constexpr uint32_t kV10Crc = 0x43E5DF55u;
uint32_t crc32(const uint8_t* p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) { c ^= p[i]; for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
    return ~c;
}
}

class Revvy85 : public mpc::Plugin
{
public:
    enum Params {
        kVolume, kProgram, kMix, kInTrim, kHeadroom, kStages, kDisplay1, kDisplay2,
        kMono, kSemBp, kHpFreq, kLpFreq, kSemFreq, kSemRes, kSemMode, kSemMix,
        kKey0, kCell0 = kKey0 + kNumKeys, kDigit0 = kCell0 + 32, kDigit1, kLed2, kLed3, kLed4, kMeter, kMemSelect, kRecallPush, kNumParams };

    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo i;
        i.name = "Revvy85";
        i.vendor = "Lichthaus";
        i.uniqueId = MPC_FOURCC('L', 'h', 'R', '7');   // == "uid" in plugin.json
        i.version = 400;
        i.isInstrument = false;
        i.numInputs = 2;
        i.numOutputs = 2;
        i.latencySamples = rev7::Engine::kDryDelay;    // the two rate converters
        i.tailSamples = 0;
        return i;
    }

    Revvy85() : mpc::Plugin(makeInfo())
    {
        using namespace mpc;
        addParam(floatParam("vol", "Volume", -24.0f, 12.0f, 0.0f, "dB", 1));        // parameter 0: harmless when nudged
        {
            std::vector<std::string> progs;
            for (int i = 1; i <= 90; ++i) progs.push_back(std::to_string(i));
            addParam(choiceParam("prog", "Program", std::move(progs), 0));
        }
        addParam(floatParam("mix", "Mixing", 0.0f, 100.0f, 100.0f, "%", 0));
        addParam(floatParam("intrim", "Input Level", -24.0f, 12.0f, 0.0f, "dB", 1));
        addParam(choiceParam("headroom", "Headroom", { "0 dB", "6 dB", "12 dB", "18 dB", "24 dB" }, 2));
        addParam(choiceParam("stages", "Stages", { "Off", "On" }, 1));
        addParam(floatParam("lcd1", "Display 1", 0.0f, 1.0f, 0.0f));
        addParam(floatParam("lcd2", "Display 2", 0.0f, 1.0f, 0.0f));
        addParam(boolParam("mono", "Mono", false));
        // Input filters (replace the unit's EQ): each is out of circuit at its neutral setting.
        addParam(boolParam("sembp", "SEM BP", false));
        addParam(floatParam("hpf", "HP Freq", 0.0f, 1.0f, 0.0f));                     // 0 = off, then 20 Hz - 2 kHz
        addParam(floatParam("lpf", "LP Freq", 0.0f, 1.0f, 1.0f));                     // 1 = off, else 500 Hz - 18 kHz
        addParam(floatParam("semf", "SEM Freq", 0.0f, 1.0f, 0.5f));                   // 30 Hz - 12 kHz
        addParam(floatParam("semres", "SEM Res", 0.0f, 100.0f, 0.0f, "%", 0));
        addParam(floatParam("semmode", "SEM Mode", 0.0f, 1.0f, 0.0f));                // LP -> notch -> HP
        addParam(floatParam("semmix", "SEM Mix", 0.0f, 100.0f, 0.0f, "%", 0));        // 0 = off
        for (int k = 0; k < kNumKeys; ++k) addParam(boolParam(kKeys[k].id, kKeys[k].name, false));
        for (int i = 0; i < 32; ++i)
        {
            char id[8], name[12];
            std::snprintf(id, sizeof id, "lcd%02d", i); std::snprintf(name, sizeof name, "LCD %d", i + 1);
            addParam(floatParam(id, name, 0.0f, 1.0f, frameValue(' ')));
        }
        addParam(floatParam("dig0", "Memory Tens", 0.0f, 1.0f, 0.0f));
        addParam(floatParam("dig1", "Memory Units", 0.0f, 1.0f, 0.0f));
        addParam(floatParam("led2", "LEDs 2", 0.0f, 1.0f, 0.0f));
        addParam(floatParam("led3", "LEDs 3", 0.0f, 1.0f, 0.0f));
        addParam(floatParam("led4", "LEDs 4", 0.0f, 1.0f, 0.0f));
        addParam(floatParam("meter", "Meter", 0.0f, 1.0f, 0.0f));
        {
            // Memory scroll on the MEMORY digits: the data wheel picks a number (shown blinking on the
            // digits), a push of the wheel (or a double tap) flips Recall Push, which recalls it.
            std::vector<std::string> mems;
            for (int i = 1; i <= 90; ++i) mems.push_back(std::to_string(i));
            addParam(choiceParam("memsel", "Memory Select", std::move(mems), 0));
        }
        addParam(boolParam("recallpush", "Recall Push", false));
        if (numParams() != kNumParams) mpc::log("Revvy85: parameter count mismatch");
        setLines("No firmware yet", "Content/roms/");
    }

    ~Revvy85() override { busy_.test_and_set(); }

    void prepare(double, int) override
    {
        if (engine_) return;
        std::unique_ptr<rev7::Engine> e = bootEngine(nullptr, 0);
        if (e) swapIn(std::move(e));
    }

    void saveExtraState(std::vector<uint8_t>& out) override
    {
        holdOff();
        if (engine_)
        {
            out.push_back('R'); out.push_back('7'); out.push_back(1); out.push_back(0);
            const uint8_t* r = engine_->m.sram + 0x1000; out.insert(out.end(), r, r + 0x1000);
        }
        release();
    }

    void loadExtraState(const uint8_t* data, size_t size) override
    {
        if (size != 4 + 0x1000 || data[0] != 'R' || data[1] != '7' || data[2] != 1) return;
        std::unique_ptr<rev7::Engine> e = bootEngine(data + 4, size - 4);
        if (e) swapIn(std::move(e));
    }

    void reset() override
    {
        for (int k = 0; k < kNumKeys; ++k) lastKey_[k] = getBool(kKey0 + k);
        lastProgram_ = getInt(kProgram); lastStages_ = -1; shownUpdates_ = ~0u; seqLen_ = seqPos_ = 0;
        for (int i = 0; i < kOutputs; ++i) outShown_[i] = -1;
        meterDb_ = -60.0f; sinceProgram_ = 1 << 30;
        lastMemSel_ = getInt(kMemSelect); lastPush_ = getBool(kRecallPush); selecting_ = 0;
    }

    bool formatValue(int index, float plain, std::string& out) override
    {
        char t[24];
        switch (index)
        {
        case kInTrim: std::snprintf(t, sizeof t, "%+.1f dB", plain); out = t; return true;
        case kMix: case kSemRes: case kSemMix:
            if (index == kSemMix && plain < 0.5f) { out = "Off"; return true; }
            std::snprintf(t, sizeof t, "%.0f%%", plain); out = t; return true;
        case kHpFreq: case kLpFreq: case kSemFreq:
        {
            const float hz = filterHz(index, plain);
            if (hz <= 0.0f) { out = "Off"; return true; }
            if (hz >= 1000.0f) std::snprintf(t, sizeof t, "%.2f kHz", hz / 1000.0f); else std::snprintf(t, sizeof t, "%.0f Hz", hz);
            out = t; return true;
        }
        case kSemMode:
            if (plain < 0.03f) out = "LP"; else if (plain > 0.97f) out = "HP"; else if (std::fabs(plain - 0.5f) < 0.03f) out = "Notch";
            else { std::snprintf(t, sizeof t, plain < 0.5f ? "LP %.0f%%" : "HP %.0f%%", plain < 0.5f ? 100.0f - 200.0f * plain : 200.0f * plain - 100.0f); out = t; }
            return true;
        case kDisplay1: case kDisplay2: out = line_[index == kDisplay1 ? 0 : 1]; return true;
        default: break;
        }
        if (index >= kCell0) { out = ""; return true; }
        return false;
    }

    void process(mpc::ProcessContext& ctx) override
    {
        const float* inL = ctx.inputs[0];
        const float* inR = ctx.numInputs > 1 ? ctx.inputs[1] : ctx.inputs[0];
        float* outL = ctx.outputs[0];
        float* outR = ctx.numOutputs > 1 ? ctx.outputs[1] : ctx.outputs[0];
        const int n = ctx.numFrames;
        auto passThrough = [&] {                      // no firmware, or a swap in progress: dry signal
            if (outL != inL) std::memcpy(outL, inL, sizeof(float) * (size_t)n);
            if (outR != inR && outR != outL) std::memcpy(outR, inR, sizeof(float) * (size_t)n);
        };
        if (busy_.test_and_set(std::memory_order_acquire)) { passThrough(); return; }
        rev7::Engine* e = engine_.get();
        if (!e) { busy_.clear(std::memory_order_release); passThrough(); return; }

        // Keys: one press per change of a switch.
        for (int k = 0; k < kNumKeys; ++k)
        {
            const bool v = getBool(kKey0 + k);
            if (v == lastKey_[k]) continue;
            lastKey_[k] = v;
            queuePress(kKeys[k].col, kKeys[k].row);
        }
        // Program: a MIDI program change (the firmware's default table maps change n-1 to memory n).
        const int prog = getInt(kProgram);
        if (prog != lastProgram_) { lastProgram_ = prog; e->programChange(prog + 1); sinceProgram_ = 0; }
        // Memory scroll: turning starts (or continues) a selection; the push recalls it.
        const int sel = getInt(kMemSelect);
        if (sel != lastMemSel_) { lastMemSel_ = sel; selecting_ = kSelectHold; }
        const bool push = getBool(kRecallPush);
        if (push != lastPush_)
        {
            lastPush_ = push;
            if (selecting_ > 0)
            {
                selecting_ = 0; lastProgram_ = sel; e->programChange(sel + 1); sinceProgram_ = 0;
                setNormalised(kProgram, (float)sel / 89.0f); notifyHostParameterChanged(kProgram);
            }
        }
        if (selecting_ > 0) selecting_ -= n < selecting_ ? n : selecting_;
        runPresses(e, n);

        const int stages = getInt(kStages);
        if (stages != lastStages_) { e->modelOutput.store(stages == 1 && hasStageFile_); lastStages_ = stages; }
        static const float headroomDb[] = { 0.0f, 6.0f, 12.0f, 18.0f, 24.0f };
        e->headroom.store(mpc::dsp::dbToGain(-headroomDb[mpc::dsp::clamp(getInt(kHeadroom), 0, 4)]));
        e->inGain.store(mpc::dsp::dbToGain(get(kInTrim)));
        e->outGain.store(mpc::dsp::dbToGain(get(kVolume)));
        e->wet.store(get(kMix) * 0.01f);
        e->monoInput.store(getBool(kMono));
        {
            rev7::Engine::Filters f;
            f.hpHz = filterHz(kHpFreq, get(kHpFreq)); f.lpHz = filterHz(kLpFreq, get(kLpFreq));
            f.semHz = filterHz(kSemFreq, get(kSemFreq)); f.semRes = get(kSemRes) * 0.01f;
            f.semMode = get(kSemMode); f.semMix = get(kSemMix) * 0.01f; f.semBp = getBool(kSemBp);
            e->setFilters(f);
        }

        for (int done = 0; done < n; )
        {
            const int k = n - done < kChunk ? n - done : kChunk;
            std::memcpy(tmpL_, inL + done, sizeof(float) * (size_t)k); std::memcpy(tmpR_, inR + done, sizeof(float) * (size_t)k);
            e->process(tmpL_, tmpR_, outL + done, outR + done, k);
            done += k;
        }

        // Panel outputs.
        sinceProgram_ += n; sinceOutputs_ += n;
        const float peak = e->peakIn.exchange(0.0f, std::memory_order_relaxed);
        const float db = peak > 1e-6f ? 20.0f * std::log10(peak) : -120.0f;
        meterDb_ = db > meterDb_ ? db : meterDb_ - 20.0f * (float)n / 44100.0f;     // falls 20 dB/s
        if (romUnknown_.exchange(false, std::memory_order_relaxed)) warnLeft_ = 4 * 44100;
        if (warnLeft_ > 0) warnLeft_ -= n;
        { rev7::DisplaySnapshot t; if (e->readDisplay(t)) snap_ = t; }
        const rev7::DisplaySnapshot& s = snap_;
        if (s.updates != shownUpdates_)
        {
            shownUpdates_ = s.updates;
            for (int r = 0; r < 2; ++r)
            {
                for (int i = 0; i < 16; ++i) { const uint8_t c = (uint8_t)s.lcd[r][i]; line_[r][i] = c >= 0x20 && c < 0x7F ? (char)c : '*'; }
                line_[r][16] = 0;
            }
        }
        if (sinceOutputs_ >= 1764)                                 // at most 25 times a second
        {
            sinceOutputs_ = 0;
            int want[kOutputs];
            for (int i = 0; i < 32; ++i) want[i] = (uint8_t)s.lcd[i / 16][i % 16];
            if (warnLeft_ > 0)                                      // an EPROM other than the known V1.0 dump
            {
                static const char kWarn[33] = "Unknown EPROM   V1.0 expected   ";
                for (int i = 0; i < 32; ++i) want[i] = (uint8_t)kWarn[i];
            }
            want[32] = s.led[0]; want[33] = s.led[1];
            if (selecting_ > 0)                                     // the number being picked, blinking
            {
                const bool on = (selecting_ / 11025) % 2 == 0 || selecting_ > kSelectHold - 22050;
                const int m = lastMemSel_ + 1;
                want[32] = on && m >= 10 ? kSeg[m / 10] : 0; want[33] = on ? kSeg[m % 10] : 0;
            } want[34] = s.led[2]; want[35] = s.led[3]; want[36] = s.led[4];
            want[37] = meterFrame(meterDb_);
            for (int i = 0; i < kOutputs; ++i)
                if (want[i] != outShown_[i])
                {
                    outShown_[i] = want[i];
                    const int p = kCell0 + i;
                    // frame strips: the meter has 9 frames, the memory digits 128 (bit 7 unused), the rest 256
                    setNormalised(p, i == 37 ? (float)want[i] / 8.0f
                                   : i == 32 || i == 33 ? ((float)(want[i] & 0x7F) + 0.25f) / 127.0f : frameValue(want[i]));
                    notifyHostParameterChanged(p);
                }
            // Program follows the unit when it changes from the keys (not while a change is on its way).
            const int unitProg = s.program;
            if (sinceProgram_ > 22050 && unitProg >= 1 && unitProg <= 90 && selecting_ == 0 && unitProg - 1 != lastMemSel_)
            {
                lastMemSel_ = unitProg - 1;                            // the scroll starts from the unit's memory
                setNormalised(kMemSelect, (float)lastMemSel_ / 89.0f); notifyHostParameterChanged(kMemSelect);
            }
            if (sinceProgram_ > 22050 && unitProg >= 1 && unitProg <= 90 && unitProg - 1 != lastProgram_)
            {
                lastProgram_ = unitProg - 1;
                setNormalised(kProgram, (float)lastProgram_ / 89.0f);
                notifyHostParameterChanged(kProgram);
            }
        }
        busy_.clear(std::memory_order_release);
    }

    // Frame of a 256-frame strip, as a parameter value.
    static float frameValue(int frame) { return (float)frame / 255.0f + 0.25f / 255.0f; }

private:
    static constexpr int kChunk = 128, kOutputs = 38;
    std::unique_ptr<rev7::Engine> engine_;
    std::atomic_flag busy_ = ATOMIC_FLAG_INIT;
    bool hasStageFile_ = false;
    std::atomic<bool> romUnknown_{false};   // set by bootEngine, read once by process()
    int warnLeft_ = 0;                      // samples left of the "Unknown EPROM" display
    bool lastKey_[kNumKeys] = {};
    int lastProgram_ = 0, lastStages_ = -1;
    float tmpL_[kChunk], tmpR_[kChunk];
    char line_[2][17] = {};
    uint32_t shownUpdates_ = ~0u;
    int outShown_[kOutputs];
    float meterDb_ = -60.0f;
    rev7::DisplaySnapshot snap_ = {};
    int sinceOutputs_ = 0, sinceProgram_ = 1 << 30;
    int lastMemSel_ = 0, selecting_ = 0; bool lastPush_ = false;
    static constexpr int kSelectHold = 4 * 44100;                // a selection lapses after 4 s without a push
    // seven-segment patterns as the firmware writes them (bit 0 g, 1 a, 2 f, 3 b, 4 d, 5 e, 6 c)
    static constexpr int kSeg[10] = { 0x7E, 0x48, 0x3B, 0x5B, 0x4D, 0x57, 0x77, 0x4A, 0x7F, 0x4F };

    // Filter frequency knobs, log scale; 0 Hz = out of circuit (HP fully left, LP fully right).
    static float filterHz(int index, float v)
    {
        v = mpc::dsp::clamp(v, 0.0f, 1.0f);
        if (index == kHpFreq) return v < 0.005f ? 0.0f : 20.0f * std::pow(100.0f, v);
        if (index == kLpFreq) return v > 0.995f ? 0.0f : 500.0f * std::pow(36.0f, v);
        return 30.0f * std::pow(400.0f, v);
    }
    // Input meter segments (dB re the A/D's full scale): -30 -20 -15 -10 -6 -3 0 CLIP.
    static int meterFrame(float db)
    {
        static const float steps[8] = { -30.0f, -20.0f, -15.0f, -10.0f, -6.0f, -3.0f, -0.5f, -0.01f };
        int f = 0; while (f < 8 && db >= steps[f]) ++f;
        return f;
    }

    // Key presses are held 80 ms and spaced 80 ms (the firmware scans and debounces at 4 ms ticks;
    // digit entry times out after 400 ms).
    static constexpr int kHold = 3528, kGap = 3528, kMaxSeq = 32;
    int seq_[kMaxSeq][2]; int seqLen_ = 0, seqPos_ = 0, seqClock_ = 0; bool seqDown_ = false;
    void queuePress(int col, int row) { if (seqLen_ < kMaxSeq) { seq_[seqLen_][0] = col; seq_[seqLen_][1] = row; ++seqLen_; } }
    void runPresses(rev7::Engine* e, int n)
    {
        if (seqPos_ >= seqLen_) { seqLen_ = seqPos_ = 0; return; }
        seqClock_ -= n;
        if (seqClock_ > 0) return;
        const int* k = seq_[seqPos_];
        if (!seqDown_) { e->key(k[0], k[1], true); seqDown_ = true; seqClock_ = kHold; }
        else { e->key(k[0], k[1], false); seqDown_ = false; seqClock_ = kGap; ++seqPos_; }
    }

    void setLines(const char* a, const char* b)
    {
        std::snprintf(line_[0], sizeof line_[0], "%-16s", a); std::snprintf(line_[1], sizeof line_[1], "%-16s", b);
        for (int i = 0; i < 32; ++i) setNormalised(kCell0 + i, frameValue((uint8_t)line_[i / 16][i % 16]));
    }

    void holdOff() { while (busy_.test_and_set(std::memory_order_acquire)) std::this_thread::sleep_for(std::chrono::microseconds(200)); }
    void release() { busy_.clear(std::memory_order_release); }
    void swapIn(std::unique_ptr<rev7::Engine> e)
    {
        holdOff();
        std::unique_ptr<rev7::Engine> old = std::move(engine_);
        engine_ = std::move(e);
        reset();
        release();
    }

    std::unique_ptr<rev7::Engine> bootEngine(const uint8_t* ram, size_t ramSize)
    {
        std::unique_ptr<rev7::Engine> e(new rev7::Engine);
        if (!e->loadRom(mpc::contentPath("roms/rev7_v1.0.bin").c_str()))
        {
            setLines("Firmware missing", "Content/roms/");
            mpc::log("Revvy85: firmware not found in %s", mpc::contentPath("roms").c_str());
            return nullptr;
        }
        const uint32_t crc = crc32(e->m.rom, sizeof e->m.rom);
        if (crc != kV10Crc) { mpc::log("Revvy85: EPROM CRC32 %08x is not the known V1.0 dump (%08x); running it anyway", crc, kV10Crc); romUnknown_.store(true); }
        hasStageFile_ = rev7::loadOutputStage(mpc::contentPath("output-stage.txt").c_str(), e->analogue, &e->inputChain) > 0;
        if (ram && ramSize == 0x1000) std::memcpy(e->m.sram + 0x1000, ram, ramSize);
        e->reset();
        e->runSilent(5.0);
        return e;
    }
};

MPC_PLUGIN_ENTRY(Revvy85)
