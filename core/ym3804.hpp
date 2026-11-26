// YM3804 DSP, reference form: the model decoded from the chip running the V1.0 firmware. Loaded only through its serial control port (CDI words) from the firmware
// at run time; nothing firmware-derived is compiled in.
//
// Microword (five byte planes per step) as decoded:
//   plane0  b7 T write address bit 0 | b6-4 R read address | b3 R write enable | b2-0 R write address
//   plane1  b7-4 T read address | b3 T write enable | b2-0 T write address bits 3-1
//   plane2  b6 load (acc = product) | b5 reverse subtract (acc = product - acc) | b4 halve the result
//           | b3 coefficient from the YM3807 | b1 multiplier input T (else R) | b0 accumulate
//           | none of b6/b5/b0: acc = product + T
//   plane3  b7 R load from a serial input (b6 = SI1, else SI0) | b6 alone: R load from memory bus
//           (value of the access 3 steps earlier) | b5 output latch (b4 = right word, b3 = SO0 else SO1)
//   plane4  b5 memory access at addr[i] + counter | b6 write | b2 add the YM3807 delay offset
//           | b3 +1 sample | b0 right word for serial-input loads | b1 YM3807 data latch (implicit here)
//   coefficients: 16-bit two's complement, 0x1000 = 1.0; top nibble 7 = 13-bit signed (diffusion).
// Two fitted constants, applied only to the images they were fitted on:
//   - reverb image: comb one-pole state (T writes at steps 20/30/40/50/60) stored x2;
//   - modulation image: behavioural YM3807 fraction/offset on taps 13..39 and AM at 41/43/45/53/55.
#pragma once
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace rev7 {

struct YM3807;

struct YM3804
{
    // loaded program
    uint8_t  plane[5][64] = {};
    uint16_t addr[64] = {}, coef[64] = {};
    // CDI loader state
    uint8_t ctl = 2, width = 0, bank = 0, lo = 0; int ptr = 0; bool dirty = true;
    // datapath
    int32_t R[8] = {}, T[16] = {}; int64_t acc = 0; int32_t mem[16384] = {}; uint32_t counter = 0;
    int32_t bus[64] = {};
    int32_t si[2][2] = {}, so[2][2] = {};
    bool reverbImg = false, modImg = false; bool isA = false;
    YM3807* ym = nullptr;
    const uint8_t* rom = nullptr;          // the user's EPROM image (0x8000-based), for image recognition
    int hopGain = 4;
    static constexpr int32_t LO = -(1 << 23), HI = (1 << 23) - 1;

    void cdi(uint8_t cmd, uint8_t d)
    {
        switch (cmd)
        {
        case 0x00: ctl = d; break;
        case 0x03: width = d; break;
        case 0x04: ptr = d; break;
        case 0x05: bank = d; break;
        case 0x06:
            if (bank < 5) { plane[bank][ptr & 63] = d; ++ptr; dirty = true; }
            else lo = d;
            break;
        case 0x07:
        {
            const uint16_t w = (uint16_t)((d << 8) | lo);
            if (bank == 6) addr[ptr & 63] = w; else coef[ptr & 63] = w;
            ++ptr; break;
        }
        default: break;
        }
    }
    bool running() const { return ctl == 0; }

    void recognise()
    {
        dirty = false; reverbImg = modImg = false;
        if (!rom || !isA) return;
        auto same = [&](uint16_t base) {
            for (int b = 0; b < 5; ++b) for (int i = 0; i < 64; ++i) if (plane[b][i] != rom[base - 0x8000 + 64 * b + i]) return false;
            return true; };
        reverbImg = same(0x8240); modImg = same(0x86C0);
    }

    static int32_t q(int64_t a)
    {
        int64_t v = a >= 0 ? (a >> 12) : -((-a) >> 12);
        return (int32_t)std::clamp<int64_t>(v, LO, HI);
    }
    int32_t coefv(int i) const
    {
        const uint16_t c = coef[i];
        if ((c >> 12) == 7) return (int32_t)(c & 0x1FFF) - 0x2000;
        return c & 0x8000 ? (int32_t)c - 0x10000 : (int32_t)c;
    }
    void frame();                          // defined after YM3807
};

} // namespace rev7
