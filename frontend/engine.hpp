// Real-time engine for the REV7 emulation: resampling, the oversampled output stage and the stage loader.
//
// The host side runs at 44.1 kHz; the emulated REV7 at 31,250 Hz. The engine converts on the way in
// (mono sum, as the REV7's stereo input mode does) and out with windowed-sinc interpolation. With
// "Stages" on, the D/A hold and the fitted analogue sections (Content/output-stage.txt) shape the output.
//
// One thread at a time drives process() (normally the audio thread). Controls go in through a
// single-producer queue; the LCD comes out through a sequence-locked snapshot. process() does not
// allocate, lock or touch files. No firmware content lives here: the ROM is handed in at run time.
#pragma once
#include "../core/machine.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rev7 {

// Four floats or two doubles, as one SIMD register (NEON on the MPC and the Mac).
typedef float F4 __attribute__((vector_size(16)));
typedef double D2 __attribute__((vector_size(16)));
inline F4 load4(const float* p) { F4 v; std::memcpy(&v, p, sizeof v); return v; }

// Windowed-sinc fractional interpolator over a short history: one table row per
// sub-sample phase. read(f) returns the signal (kLatency + f) samples behind the newest.
// Each row is stored oldest sample first (h[p][k] weighs the sample Taps - 1 - k behind),
// the order the history window is read in.
template <int Taps> struct SincTableT
{
    static_assert(Taps % 8 == 0, "read() takes eight taps at a time");
    static constexpr int kTaps = Taps, kPhases = 256, kLatency = Taps / 2 - 1;
    float h[kPhases + 1][Taps];
    // cutoff in cycles per input sample (at most 0.5)
    void build(double cutoff)
    {
        const double beta = 8.0, half = kTaps / 2.0;
        for (int p = 0; p <= kPhases; ++p)
        {
            const double centre = kLatency + (double)p / kPhases; double sum = 0; double w[kTaps];
            for (int j = 0; j < kTaps; ++j)
            {
                const double x = j - centre, u = x / half;
                w[j] = 0;
                if (std::fabs(u) < 1.0)
                {
                    const double sn = std::fabs(x) < 1e-9 ? 1.0 : std::sin(2 * M_PI * cutoff * x) / (2 * M_PI * cutoff * x);
                    w[j] = 2 * cutoff * sn * bessel(beta * std::sqrt(1 - u * u)) / bessel(beta);
                }
                sum += w[j];
            }
            for (int j = 0; j < kTaps; ++j) h[p][kTaps - 1 - j] = (float)(w[j] / sum);   // unity gain at DC for every phase
        }
    }
    static double bessel(double x) { double sm = 1, t = 1; for (int k = 1; k < 30; ++k) { t *= (x / (2 * k)) * (x / (2 * k)); sm += t; } return sm; }
};
using SincTable = SincTableT<64>;

// Every sample is stored twice, Size apart, so that any window of up to Size samples lies in one
// piece and read() runs as plain SIMD dot products.
template <int Size> struct HistoryT                           // Size: a power of two, at least taps + whole-sample delay
{
    float buf[2 * Size] = {}; uint32_t pos = 0;               // sample j behind the newest: buf[(pos - 1 - j) & (Size - 1)] (and Size above)
    void push(float v) { const uint32_t i = pos++ & (Size - 1); buf[i] = v; buf[i + Size] = v; }
    // f in [0, 1]; extra = further whole samples of delay
    template <int Taps> float read(const SincTableT<Taps>& t, double f, int extra = 0) const
    {
        static_assert(Taps <= Size, "window longer than the history");
        const double x = f * SincTableT<Taps>::kPhases; int p = (int)x; if (p >= SincTableT<Taps>::kPhases) p = SincTableT<Taps>::kPhases - 1;
        const float a = (float)(x - p); const float* h0 = t.h[p]; const float* h1 = t.h[p + 1];
        const float* w = buf + ((pos - (uint32_t)(extra + Taps)) & (Size - 1));     // the window, oldest first
        if (a == 0.0f)                                         // on a table phase (the anti-alias stand-in always is)
        {
            F4 s0 = {}, s1 = {};
            for (int k = 0; k < Taps; k += 8) { s0 += load4(w + k) * load4(h0 + k); s1 += load4(w + k + 4) * load4(h0 + k + 4); }
            const F4 s = s0 + s1;
            return (s[0] + s[1]) + (s[2] + s[3]);
        }
        // The two neighbouring phases' dot products, then the blend between them: the same as
        // blending the coefficients, up to rounding.
        F4 s0 = {}, s1 = {}, u0 = {}, u1 = {};
        for (int k = 0; k < Taps; k += 8)
        {
            const F4 x0 = load4(w + k), x1 = load4(w + k + 4);
            s0 += x0 * load4(h0 + k); s1 += x1 * load4(h0 + k + 4);
            u0 += x0 * load4(h1 + k); u1 += x1 * load4(h1 + k + 4);
        }
        const F4 s = s0 + s1, u = u0 + u1;
        const float d0 = (s[0] + s[1]) + (s[2] + s[3]), d1 = (u[0] + u[1]) + (u[2] + u[3]);
        return d0 + a * (d1 - d0);
    }
};
using History = HistoryT<128>;

// The analogue side of the D/A converter, modelled at four times the unit's sample rate:
// the converter holds each sample for a whole period (a staircase, which droops towards
// the top of the band and leaves images above it), then a chain of filter sections stands
// in for the reconstruction filter and output amplifiers. Sections are given as analogue
// transfer functions (b0 + b1 s + b2 s^2) / (a0 + a1 s + a2 s^2) and turned into digital
// ones with the bilinear transform, matched at a frequency of the caller's choice.
struct OutputStage
{
    static constexpr int kOver = 4, kMaxSections = 8;
    struct Section { double b0, b1, b2, a1, a2; };
    Section sec[kMaxSections]; int count = 0;
    D2 z[kMaxSections][2] = {};                               // per section, both channels side by side
    double rate = 0;                                           // kOver x unit rate

    void clear() { count = 0; std::memset(z, 0, sizeof z); }
    void resetState() { std::memset(z, 0, sizeof z); }
    void addAnalogue(double b0, double b1, double b2, double a0, double a1, double a2, double matchHz)
    {
        if (count >= kMaxSections) return;
        // The digital section equals the analogue one exactly at the matching frequency. A corner
        // above 15 kHz is matched at 15 kHz instead: what matters is its effect inside the band the
        // engine carries, and for the peaked output filter this keeps the error under 0.05 dB.
        const double fm = matchHz > 0 && matchHz < 15000.0 ? matchHz : 15000.0;
        const double w = 2 * M_PI * fm, k = w / std::tan(w / (2 * rate));
        const double B0 = b0 + b1 * k + b2 * k * k, B1 = 2 * b0 - 2 * b2 * k * k, B2 = b0 - b1 * k + b2 * k * k;
        const double A0 = a0 + a1 * k + a2 * k * k, A1 = 2 * a0 - 2 * a2 * k * k, A2 = a0 - a1 * k + a2 * k * k;
        sec[count++] = Section{ B0 / A0, B1 / A0, B2 / A0, A1 / A0, A2 / A0 };
    }
    // Convenience forms.
    void lowPass2(double f0, double q, double gain = 1.0) { const double w = 2 * M_PI * f0; addAnalogue(gain * w * w, 0, 0, w * w, w / q, 1, f0); }
    void lowPass1(double f0) { const double w = 2 * M_PI * f0; addAnalogue(w, 0, 0, w, 1, 0, f0); }
    void highPass1(double f0) { const double w = 2 * M_PI * f0; addAnalogue(0, 1, 0, w, 1, 0, f0); }

    // Repeating a sample four times is the hold seen at four times the rate; the last quarter-
    // period of hold between those points is a further sin(x)/x of at most 0.34 dB at 22 kHz.
    // This three-tap filter supplies it to within 0.02 dB.
    D2 hold[2] = {};                                           // the last two inputs, both channels
    bool holdCorrection = true;                                // off for a chain that is not behind a converter (the input side)
    // Both channels at once, one in each half of a SIMD register.
    void run(float& l, float& r)
    {
        const double a = holdCorrection ? 0.0425 : 0.0;
        const D2 x = { l, r };
        D2 v = a * x + (1 - 2 * a) * hold[0] + a * hold[1];
        hold[1] = hold[0]; hold[0] = x;
        for (int i = 0; i < count; ++i)
        {
            const Section& c = sec[i]; D2* st = z[i];
            const D2 y = c.b0 * v + st[0];
            st[0] = c.b1 * v - c.a1 * y + st[1]; st[1] = c.b2 * v - c.a2 * y;
            v = y;
        }
        l = (float)v[0]; r = (float)v[1];
    }
    // Magnitude of the section chain alone at a frequency, in dB (for reports).
    double chainDb(double hz) const
    {
        const double w = 2 * M_PI * hz / rate; double re = 1, im = 0;
        for (int i = 0; i < count; ++i)
        {
            const Section& c = sec[i];
            const double c1 = std::cos(w), s1 = std::sin(w), c2 = std::cos(2 * w), s2 = std::sin(2 * w);
            const double nr = c.b0 + c.b1 * c1 + c.b2 * c2, ni = -(c.b1 * s1 + c.b2 * s2), dr = 1 + c.a1 * c1 + c.a2 * c2, di = -(c.a1 * s1 + c.a2 * s2);
            const double den = dr * dr + di * di, hr = (nr * dr + ni * di) / den, hi = (ni * dr - nr * di) / den;
            const double tr = re * hr - im * hi; im = re * hi + im * hr; re = tr;
        }
        return 10 * std::log10(re * re + im * im + 1e-30);
    }
};


// Input filters (2026-11-25, replacing the unit's EQ, whose MID and HI do nothing on the
// reference unit): a 12 dB/oct high-pass and low-pass and a SEM-style state-variable filter, before the A/D, so
// they shape the reverb only. Each is out of circuit at its neutral setting (HP at its lowest, LP
// at its highest, SEM mix 0).
struct Biquad
{
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    float run(float x) { const float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
    // 2-pole Butterworth (RBJ, Q 1/sqrt 2): high = true for high-pass
    void butter(bool high, double fs, double f)
    {
        const double w = 2 * M_PI * std::fmin(f, 0.45 * fs) / fs, c = std::cos(w), al = std::sin(w) / std::sqrt(2.0), a0 = 1 + al;
        const double bb = high ? (1 + c) / 2 : (1 - c) / 2;
        b0 = (float)(bb / a0); b1 = (float)((high ? -(1 + c) : (1 - c)) / a0); b2 = b0;
        a1 = (float)(-2 * c / a0); a2 = (float)((1 - al) / a0);
    }
};

// SEM-style 2-pole state-variable filter (topology-preserving form): MODE crossfades low-pass ->
// notch (low + high) -> high-pass as the SEM's mode knob does, BP selects the band-pass output.
// The band integrator is softly saturated, so high resonance stays bounded and gains some grit.
struct SemFilter
{
    float ic1 = 0, ic2 = 0, a1 = 1, a2 = 0, a3 = 0, k = 2;
    void set(double fs, double f, double res)                 // res 0..1
    {
        const double g = std::tan(M_PI * std::fmin(f, 0.45 * fs) / fs);
        k = (float)(2.0 - 1.96 * std::fmin(std::fmax(res, 0.0), 1.0));
        a1 = (float)(1.0 / (1.0 + g * (g + k))); a2 = (float)(g * a1); a3 = (float)(g * a2);
    }
    float run(float x, float mode, bool bp)
    {
        const float v3 = x - ic2;
        const float v1 = a1 * ic1 + a2 * v3, v2 = ic2 + a2 * ic1 + a3 * v3;
        const float n1 = 2 * v1 - ic1;
        ic1 = n1 / std::sqrt(1.0f + 0.25f * n1 * n1);           // soft saturation of the band state
        ic2 = 2 * v2 - ic2;
        if (bp) return k * v1;                                    // unity gain at the centre
        const float lp = v2, hp = x - k * v1 - v2;
        return lp + mode * (hp - lp);
    }
};

struct DisplaySnapshot
{
    char     lcd[2][16];        // the two LCD lines (raw codes)
    uint8_t  led[5];            // LED column buffer as the firmware keeps it (RAM 0x401E-0x4022)
    uint8_t  running;           // bit 0 = DSP A, bit 1 = DSP B
    uint8_t  program;           // current memory number (battery RAM 0x5008)
    uint8_t  relays;            // port 0x2002 bits 5-7: bit 0 MUTE, bit 1 BYPASS (relay drives)
    uint32_t updates;
    uint32_t pad2;
};
static_assert(sizeof(DisplaySnapshot) == 48, "snapshot is copied as 6 words");

class Engine
{
public:
    static constexpr int kRate = 44100;
    static constexpr double kUnitRate = 31250.0;
    // The unit's own converter/pipeline latency beyond the DSP model (measured: wet = address delay
    // + 12.4 samples; the model has 3), added on the unit side so wet timing matches the REV7.
    static constexpr int kConverterDelay = 9;
    static constexpr int kDryDelay = 63;                      // 31 frames in + 32 out of resampler latency

    Machine m;
    OutputStage analogue;                                     // fitted stages (loadOutputStage)
    OutputStage inputChain;                                   // empty: the fit puts the whole response on the output side
    std::atomic<bool> modelOutput{true};                      // "Stages": true = modelled analogue path, false = digital path only

    Engine()
    {
        down.build(0.47 * kUnitRate / kRate);
        up.build(0.47);
        upHi.build(21300.0 / (OutputStage::kOver * kUnitRate));
        analogue.rate = OutputStage::kOver * kUnitRate;
        inputChain.rate = OutputStage::kOver * kUnitRate; inputChain.holdCorrection = false;
        m.logTx = false;
    }
    bool loadRom(const char* path) { return m.load(path); }
    void reset()
    {
        m.reset(); acc = 0; lastOut[0] = lastOut[1] = 0; std::memset(convDelay, 0, sizeof convDelay); convPos = 0;
        for (auto& h : inHist) h = History{}; for (auto& h : outHist) h = History{}; for (auto& h : outHi) h = HistoryT<256>{};
        analogue.resetState(); std::memset(analogue.hold, 0, sizeof analogue.hold); inputChain.resetState();
        std::memset(dry, 0, sizeof dry); dryPos = 0; hp.z1 = hp.z2 = lp.z1 = lp.z2 = 0; sem.ic1 = sem.ic2 = 0; fFirst = true; wetGain = 1; bypassGain = 0; lastUpdates = ~0u; lastRelays = 0xFF; publish();
    }
    // Runs the machine with silent input, faster than real time (start-up, program loads).
    void runSilent(double seconds)
    {
        const long n = (long)(seconds * kUnitRate); int16_t l, r;
        for (long i = 0; i < n; ++i) { drainEvents(); m.runFrame(0, l, r); }
        publish();
    }

    // ---- controls (one producer thread) ----
    void key(int col, int row, bool down) { post(Event{down ? Event::Down : Event::Up, (int16_t)col, (int16_t)row}); }
    void midi(uint8_t b) { post(Event{Event::Midi, (int16_t)b, 0}); }
    void programChange(int memory) { midi(0xC0); midi((uint8_t)((memory - 1) & 0x7F)); }

    std::atomic<float> wet{1.0f}, inGain{1.0f}, outGain{1.0f};
    std::atomic<float> headroom{0.251189f};                   // -12 dB operating level into the converters
    std::atomic<bool>  mute{false};
    std::atomic<bool>  monoInput{false};                      // MONO: the left input only; STEREO: both summed [SM]
    // Fitted (2026-11-25): the model's left D/A word comes out exactly 4x (12.04 dB) the right one in
    // every program; the recordings could not check the absolute balance (the unit's left output
    // carried the dry signal), and the plugin sounded left-heavy. The right channel, which the
    // analogue fit was made on, keeps its level.
    static constexpr float kLeftTrim = 0.25f;
    // Input filters (audio thread only: call from the same thread as process()). hpHz <= 0 and
    // lpHz <= 0 switch those filters out; semMix 0 switches the SEM out. Changes glide over ~10 ms.
    struct Filters { float hpHz = 0, lpHz = 0, semHz = 1000, semRes = 0, semMode = 0, semMix = 0; bool semBp = false; };
    void setFilters(const Filters& f) { fTarget = f; }
    std::atomic<float> peakIn{0.0f}, peakOut[2] = {{0.0f}, {0.0f}};

    bool readDisplay(DisplaySnapshot& out) const
    {
        for (int attempt = 0; attempt < 8; ++attempt)
        {
            const uint32_t a = snapSeq.load(); if (a & 1) continue;
            uint64_t w[kSnapWords]; for (int i = 0; i < kSnapWords; ++i) w[i] = snapWords[i].load();
            if (snapSeq.load() == a) { std::memcpy(&out, w, sizeof out); return true; }
        }
        return false;
    }

    // ---- the audio path ----
    void process(const float* inL, const float* inR, float* outL, float* outR, int frames)
    {
        drainEvents();
        const float w = wet.load(std::memory_order_relaxed), d = 1.0f - w;
        const float gi = inGain.load(std::memory_order_relaxed), go = outGain.load(std::memory_order_relaxed);
        const float h = headroom.load(std::memory_order_relaxed), makeUp = 1.0f / h;
        const bool silent = mute.load(std::memory_order_relaxed), model = modelOutput.load(std::memory_order_relaxed);
        const bool monoIn = monoInput.load(std::memory_order_relaxed);
        // The unit's MUTE and BYPASS relays (port 0x2002 bits 5/6) and OUT PHASE (LED column 4 bit 2;
        // the DSP output does not change with it, so it is taken as an analogue polarity switch on the
        // reverb output). MUTE silences the reverb, BYPASS outputs the direct signal only. Ramped over
        // about 5 ms.
        const uint8_t relays = (uint8_t)(m.colSelect >> 5);
        const float wetTarget = (relays & 3) ? 0.0f : ((m.sram[0x22] & 0x04) ? -1.0f : 1.0f);
        const float bypassTarget = (relays & 2) ? 1.0f : 0.0f;
        const double ratio = kUnitRate / kRate;
        float pi = 0, po[2] = {0, 0};
        for (int i = 0; i < frames; ++i)
        {
            const float xl = (inL ? inL[i] : 0.0f) * gi, xr = (inR ? inR[i] : (inL ? inL[i] : 0.0f)) * gi;
            float mono = monoIn ? xl : 0.5f * (xl + xr);
            if ((i & 15) == 0) updateFilters();
            if (hpOn) mono = hp.run(mono);
            if (lpOn) mono = lp.run(mono);
            if (fNow.semMix > 1e-4f) mono += fNow.semMix * (sem.run(mono, fNow.semMode, fTarget.semBp) - mono);
            inHist[0].push(mono * h);
            acc += ratio;
            if (acc >= 1.0)
            {
                acc -= 1.0;
                const double f = acc / ratio;
                const float u = inHist[0].read(down, f);
                pi = std::fmax(pi, std::fabs(u));
                int16_t l16, r16; m.runFrame(toCode(u), l16, r16);
                // the unit's remaining converter latency
                convDelay[convPos][0] = l16; convDelay[convPos][1] = r16; convPos = (convPos + 1) % kConverterDelay;
                lastOut[0] = convDelay[convPos][0] * (kLeftTrim / 32768.0f); lastOut[1] = convDelay[convPos][1] * (1.0f / 32768.0f);
                outHist[0].push(lastOut[0]); outHist[1].push(lastOut[1]);
                for (int k = 0; k < OutputStage::kOver; ++k) { float l = lastOut[0], r = lastOut[1]; analogue.run(l, r); outHi[0].push(l); outHi[1].push(r); }
                if ((++now & 255) == 0) publish();
            }
            float el, er;
            if (model)
            {
                const double x = 2 * OutputStage::kOver - 1 - acc * OutputStage::kOver; const int whole = (int)x;
                el = outHi[0].read(upHi, x - whole, whole) * makeUp; er = outHi[1].read(upHi, x - whole, whole) * makeUp;
            }
            else
            {
                const double fo = 1.0 - acc;
                el = outHist[0].read(up, fo) * makeUp; er = outHist[1].read(up, fo) * makeUp;
            }
            const float dl = dry[0][dryPos], dr = dry[1][dryPos];
            dry[0][dryPos] = xl; dry[1][dryPos] = xr; dryPos = (dryPos + 1) % kDryDelay;
            wetGain += 0.005f * (wetTarget - wetGain); bypassGain += 0.005f * (bypassTarget - bypassGain);
            const float wl = d * dl + w * wetGain * el, wr = d * dr + w * wetGain * er;
            const float yl = (wl + bypassGain * (dl - wl)) * go, yr = (wr + bypassGain * (dr - wr)) * go;
            po[0] = std::fmax(po[0], std::fabs(yl)); po[1] = std::fmax(po[1], std::fabs(yr));
            outL[i] = silent ? 0.0f : yl; outR[i] = silent ? 0.0f : yr;
        }
        publish();
        raise(peakIn, pi); raise(peakOut[0], po[0]); raise(peakOut[1], po[1]);
    }

private:
    struct Event { enum Type : int16_t { Down, Up, Midi } type; int16_t a, b; };
    static constexpr int kQueue = 256;
    Event queue[kQueue];
    std::atomic<uint32_t> qHead{0}, qTail{0};
    void post(const Event& e)
    {
        const uint32_t hd = qHead.load(std::memory_order_relaxed);
        if (hd - qTail.load(std::memory_order_acquire) >= (uint32_t)kQueue) return;
        queue[hd & (kQueue - 1)] = e; qHead.store(hd + 1, std::memory_order_release);
    }
    void drainEvents()
    {
        uint32_t t = qTail.load(std::memory_order_relaxed); const uint32_t hd = qHead.load(std::memory_order_acquire);
        for (; t != hd; ++t)
        {
            const Event e = queue[t & (kQueue - 1)];
            if (e.type == Event::Midi) m.rxQueue.push_back((uint8_t)e.a);
            else if (e.a >= 0 && e.a < 5 && e.b >= 0 && e.b < 8)
            {
                if (e.type == Event::Down) m.keyRows[e.a] |= (uint8_t)(1 << e.b); else m.keyRows[e.a] &= (uint8_t)~(1 << e.b);
            }
        }
        qTail.store(t, std::memory_order_release);
    }
    SincTable down, up; SincTableT<128> upHi;
    History inHist[1], outHist[2]; HistoryT<256> outHi[2];
    double acc = 0; float lastOut[2] = {0, 0}; uint64_t now = 0;
    Biquad hp, lp; SemFilter sem; bool hpOn = false, lpOn = false;
    Filters fTarget, fNow; bool fFirst = true;
    float wetGain = 1.0f, bypassGain = 0.0f;
    // every 16 samples: glide the settings (frequencies in log) and recompute the coefficients
    void updateFilters()
    {
        const float c = fFirst ? 1.0f : 0.15f; fFirst = false;
        auto glideHz = [c](float now, float to) { return now <= 0 || to <= 0 ? to : now * std::pow(to / now, c); };
        fNow.hpHz = glideHz(fNow.hpHz, fTarget.hpHz); fNow.lpHz = glideHz(fNow.lpHz, fTarget.lpHz);
        fNow.semHz = glideHz(fNow.semHz, fTarget.semHz);
        fNow.semRes += c * (fTarget.semRes - fNow.semRes); fNow.semMode += c * (fTarget.semMode - fNow.semMode);
        fNow.semMix += c * (fTarget.semMix - fNow.semMix);
        if (!fTarget.semMix && fNow.semMix < 1e-4f) fNow.semMix = 0;
        hpOn = fNow.hpHz > 0; lpOn = fNow.lpHz > 0;
        if (hpOn) hp.butter(true, kRate, fNow.hpHz);
        if (lpOn) lp.butter(false, kRate, fNow.lpHz);
        if (!hpOn) hp.z1 = hp.z2 = 0;
        if (!lpOn) lp.z1 = lp.z2 = 0;
        sem.set(kRate, fNow.semHz, fNow.semRes);
    }
    int16_t convDelay[kConverterDelay][2] = {}; int convPos = 0;
    float dry[2][kDryDelay] = {}; int dryPos = 0;

    static int16_t toCode(float x)
    {
        const float v = x * 32767.0f;
        return (int16_t)(v >= 32767.0f ? 32767 : v <= -32768.0f ? -32768 : std::lrintf(v));
    }
    static void raise(std::atomic<float>& p, float v) { if (v > p.load(std::memory_order_relaxed)) p.store(v, std::memory_order_relaxed); }

    static constexpr int kSnapWords = (int)(sizeof(DisplaySnapshot) / 8);
    std::atomic<uint32_t> snapSeq{0};
    std::atomic<uint64_t> snapWords[kSnapWords] = {};
    uint32_t lastUpdates = ~0u; uint8_t lastRelays = 0xFF, lastLed[5] = {};
    void publish()
    {
        // republish when the display, an LED column or a relay changes
        const uint8_t relays = (uint8_t)(m.colSelect >> 5);
        if (m.lcd.writes == lastUpdates && relays == lastRelays && std::memcmp(lastLed, m.sram + 0x1e, 5) == 0) return;
        lastUpdates = m.lcd.writes; lastRelays = relays; std::memcpy(lastLed, m.sram + 0x1e, 5);
        DisplaySnapshot s; std::memset(&s, 0, sizeof s);
        for (int r = 0; r < 2; ++r) for (int i = 0; i < 16; ++i) s.lcd[r][i] = (char)m.lcd.ddram[(r ? 0x40 : 0) + i];
        for (int i = 0; i < 5; ++i) s.led[i] = m.sram[0x1e + i];
        s.running = (uint8_t)((m.dsp.A.running() ? 1 : 0) | (m.dsp.B.running() ? 2 : 0));
        s.program = m.sram[0x1008]; s.relays = (uint8_t)(m.colSelect >> 5); s.updates = m.lcd.writes;
        uint64_t w[kSnapWords]; std::memcpy(w, &s, sizeof s);
        const uint32_t q = snapSeq.load(); snapSeq.store(q + 1);
        for (int i = 0; i < kSnapWords; ++i) snapWords[i].store(w[i]);
        snapSeq.store(q + 2);
    }
};

// Reads an analogue-stage description: one section per line, '#' starts a comment. A line that
// starts with "in " describes the input side (ahead of the A/D converter); the rest the output side.
//   lp2 F0_HZ Q [GAIN]     second-order low-pass
//   lp1 F0_HZ              first-order low-pass
//   hp1 F0_HZ              first-order high-pass (coupling capacitor)
//   s2 b0 b1 b2 a0 a1 a2 MATCH_HZ   any analogue section, coefficients of s^0, s^1, s^2
// Not for the audio thread. Returns the number of sections, or -1 if the file cannot be read.
inline int loadOutputStage(const char* path, OutputStage& output, OutputStage* input = nullptr)
{
    FILE* f = std::fopen(path, "r"); if (!f) return -1;
    output.clear(); if (input) input->clear();
    char line[256];
    while (std::fgets(line, sizeof line, f))
    {
        const char* text = line; while (*text == ' ' || *text == '\t') ++text;
        const bool in = !std::strncmp(text, "in ", 3); if (in) text += 3;
        if (in && !input) continue;
        OutputStage& stage = in ? *input : output;
        char kind[16]; double v[7] = {0, 0, 1, 0, 0, 0, 0};
        const int n = std::sscanf(text, " %15s %lf %lf %lf %lf %lf %lf %lf", kind, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]);
        if (n < 2 || kind[0] == '#') continue;
        if (!std::strcmp(kind, "lp2") && n >= 3) stage.lowPass2(v[0], v[1], n >= 4 ? v[2] : 1.0);
        else if (!std::strcmp(kind, "lp1")) stage.lowPass1(v[0]);
        else if (!std::strcmp(kind, "hp1")) stage.highPass1(v[0]);
        else if (!std::strcmp(kind, "s2") && n >= 8) stage.addAnalogue(v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    }
    std::fclose(f); return output.count + (input ? input->count : 0);
}

} // namespace rev7
