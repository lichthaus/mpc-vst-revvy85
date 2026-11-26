// YM3807 "MOD", behavioural model.
// Decoded: 16 channels; register = CDI command 0x10-0xFF (low nibble channel, high nibble row);
// 20-bit phase accumulator per channel advanced by the rate (rows 1/2) each frame, f = v fs / 2^20;
// rate 0xFF = follow channel 2 (even) or 3 (odd, when 3 runs); row 3 phase offset (64ths); row 4 mode
// (bit 0 invert); row D depth. Fitted: triangle wave; delay 128 + 1.15 d w / 128 samples (q8);
// AM gain 1 - 1.9 d (1 + w)/2 / 127, AM channels (8-15) in phase.
#pragma once
#include "ym3804.hpp"
#include <cmath>

namespace rev7 {

struct YM3807
{
    uint8_t r[256];
    uint32_t ph[16] = {};
    double dmK = 1.15, amK = 1.9;
    YM3807() { std::memset(r, 0, sizeof r); for (int c = 0; c < 16; ++c) r[0x10 + c] = 0xFF; }
    void cdi(uint8_t cmd, uint8_t d)
    {
        if (cmd >= 0x10) { r[cmd] = d; if ((cmd >> 4) == 3) ph[cmd & 15] = (uint32_t)(d & 63) << 14; }
    }
    int rate(int c) const { const uint8_t lo = r[0x10 + c]; return lo == 0xFF ? -1 : (r[0x20 + c] << 8) | lo; }
    void step() { for (int c = 0; c < 16; ++c) { const int v = rate(c); if (v >= 0) ph[c] = (ph[c] + (uint32_t)v) & 0xFFFFF; } }
    uint32_t phase(int c) const
    {
        if (rate(c) >= 0) return ph[c];
        const int m = (c & 1) && rate(3) >= 0 ? 3 : 2;
        const uint32_t off = c >= 8 ? 0 : (uint32_t)(r[0x30 + c] & 63) << 14;
        return (ph[m] + off) & 0xFFFFF;
    }
    double wave(int c) const
    {
        const double p = phase(c) / double(1 << 20);
        double tri = 4 * std::fabs(p - 0.5) - 1;
        if (r[0x40 + c] & 1) tri = -tri;
        return tri * 127;
    }
    int32_t delayQ8(int c) const { return (int32_t)(128 * 256 + r[0xD0 + c] * wave(c) * 2 * dmK); }
    int32_t amGain(int c) const
    {
        double g = 1.0 - amK * r[0xD0 + c] * (1 + wave(c) / 127.0) / 2 / 127.0;
        return (int32_t)(4096 * std::min(1.0, std::max(0.0, g)));
    }
};

inline void YM3804::frame()
{
    if (dirty) recognise();
    for (int i = 0; i < 64; ++i)
    {
        if (i == 0) counter = (counter - 1) & 0x3FFF;
        const uint8_t p0 = plane[0][i], p1 = plane[1][i], p2 = plane[2][i], p3 = plane[3][i], p4 = plane[4][i];
        const int32_t store = q(acc);
        const int32_t tv = T[p1 >> 4];
        const int64_t src = (p2 & 2) ? tv : R[(p0 >> 4) & 7];
        int64_t c;
        if ((p2 & 8) && modImg && ym && i >= 17 && i <= 39) c = (ym->delayQ8(2 + (i - 17) / 4) & 0xFF) * 16;
        else if ((p2 & 8) && modImg && ym && (i == 41 || i == 43 || i == 45 || i == 53 || i == 55))
            c = ym->amGain(i == 41 ? 8 : i == 43 ? 9 : i == 45 ? 10 : i == 53 ? 11 : 12);
        else if (p2 & 8) c = (p2 & 2) ? 4096 : 0;
        else c = coefv(i);
        const int64_t prod = src * c;
        int64_t nacc;
        if (p2 & 0x40) nacc = prod;
        else if (p2 & 0x20) nacc = prod - acc;
        else if (p2 & 1) nacc = acc + prod;
        else nacc = prod + ((int64_t)tv << 12);
        if (p2 & 0x10) nacc = nacc >= 0 ? nacc / 2 : -((-nacc) / 2);
        int32_t busv = 0;
        if (p4 & 0x20)
        {
            uint32_t a = (addr[i] + counter) & 0x3FFF;
            if ((p4 & 0x04) && modImg && ym && i >= 13 && i <= 35)
                a = (a + (uint32_t)(ym->delayQ8(2 + (i - 13) / 4) >> 8) + ((p4 & 0x08) ? 1 : 0)) & 0x3FFF;
            if (p4 & 0x40) mem[a] = q(nacc);
            busv = mem[a];
        }
        bus[i] = busv;
        if (p0 & 8)
        {
            if (p3 & 0x80) R[p0 & 7] = si[(p3 & 0x40) ? 1 : 0][p4 & 1];
            else if (p3 & 0x40) R[p0 & 7] = bus[(i + 61) & 63];
            else R[p0 & 7] = 0;
        }
        if (p1 & 8)
        {
            int32_t st = store;
            if (reverbImg && (i == 20 || i == 30 || i == 40 || i == 50 || i == 60)) st = (int32_t)std::clamp<int64_t>((int64_t)st * 2, LO, HI);
            T[((p1 & 7) << 1) | (p0 >> 7)] = st;
        }
        if (p3 & 0x20)
            so[(p3 & 8) ? 0 : 1][(p3 >> 4) & 1] = (int32_t)std::clamp<int64_t>((int64_t)store * hopGain, LO, HI);
        acc = nacc;
    }
}

// The two DSPs, the YM3807 and the serial wiring [SM]: A/D -> B.SI0; B.SO0 -> A.SI0; A.SO1 -> B.SI1;
// B.SO1 -> D/A. Each chip-to-chip word arrives two frames after it was latched (fitted, M1).
struct DspSystem
{
    YM3804 A, B; YM3807 mod;
    int32_t qAB[2][2] = {}, qBA[2][2] = {}; int qi = 0; int32_t out24[2] = {};
    void attach(const uint8_t* rom) { A.isA = true; A.rom = B.rom = rom; A.ym = B.ym = &mod; }
    // CDI word 00 <select> <cmd> <data>: bit 7 = A, bit 6 = B, bit 5 = MOD (cascade A -> B -> MOD).
    void cdiWord(uint8_t sel, uint8_t cmd, uint8_t d)
    {
        if (sel & 0x80) A.cdi(cmd, d);
        if (sel & 0x40) B.cdi(cmd, d);
        if (sel & 0x20) mod.cdi(cmd, d);
    }
    // One 31.25 kHz frame: in = 16-bit A/D sample; out = top 16 bits of the D/A words.
    void frame(int16_t in, int16_t& outL, int16_t& outR)
    {
        const int32_t v24 = std::clamp<int32_t>((int32_t)in << 8, YM3804::LO, YM3804::HI);
        B.si[0][0] = B.si[0][1] = v24;
        std::memcpy(A.si[0], qAB[qi], sizeof A.si[0]);
        std::memcpy(B.si[1], qBA[qi], sizeof B.si[1]);
        mod.step();
        if (A.running()) A.frame();
        if (B.running()) B.frame();
        std::memcpy(qAB[qi], B.so[0], sizeof qAB[qi]);
        std::memcpy(qBA[qi], A.so[1], sizeof qBA[qi]);
        qi ^= 1;
        out24[0] = B.so[1][0]; out24[1] = B.so[1][1];
        outL = (int16_t)(B.so[1][0] >> 8); outR = (int16_t)(B.so[1][1] >> 8);
    }
};

} // namespace rev7
