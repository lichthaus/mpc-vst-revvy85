// The REV7 as the V1.0 firmware sees it. Addresses are from the firmware and
// the service manual; anything marked HYP is a hypothesis for the trace to confirm.
#pragma once
#include "hd6303.hpp"
#include "ym3807.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <deque>
#include <vector>

namespace rev7 {

// HD44780-type character LCD, 16 x 2, instruction register at 0x6001, data at 0x6000.
struct Lcd
{
    uint8_t ddram[0x80] = {}, cgram[64] = {};
    uint8_t addr = 0; bool toCg = false, increment = true, displayOn = false;
    uint32_t writes = 0;
    Lcd() { std::memset(ddram, ' ', sizeof ddram); }
    void instruction(uint8_t v)
    {
        ++writes;
        if (v & 0x80) { addr = v & 0x7F; toCg = false; }
        else if (v & 0x40) { addr = v & 0x3F; toCg = true; }
        else if (v & 0x20) { /* function set */ }
        else if (v & 0x10) { /* cursor/display shift: not used by V1.0 as far as seen */ }
        else if (v & 0x08) displayOn = (v & 4) != 0;
        else if (v & 0x04) increment = (v & 2) != 0;
        else if (v & 0x02) { addr = 0; toCg = false; }
        else if (v & 0x01) { std::memset(ddram, ' ', sizeof ddram); addr = 0; toCg = false; increment = true; }
    }
    void data(uint8_t v)
    {
        ++writes;
        if (toCg) { cgram[addr & 63] = v; addr = (uint8_t)((addr + (increment ? 1 : -1)) & 63); }
        else      { ddram[addr & 0x7F] = v; addr = (uint8_t)((addr + (increment ? 1 : -1)) & 0x7F); }
    }
    std::string line(int r) const
    {
        std::string t;
        for (int i = 0; i < 16; ++i) { const uint8_t c = ddram[(r ? 0x40 : 0) + i]; t += c >= 0x20 && c < 0x7F ? (char)c : '#'; }
        return t;
    }
};

// One byte the 6303 sent out of its serial port, with the time it left.
struct TxByte { uint64_t cycle; uint8_t value; };

struct Machine
{
    HD6303 cpu;
    uint8_t rom[0x8000] = {};
    uint8_t sram[0x2000] = {};          // 0x4000-0x5FFF, TC5565, battery backed from 0x5000 (HYP for the split)
    Lcd lcd;
    uint8_t colSelect = 0;              // last value written to 0x2002 (bits 0-4 one-hot column, bits 5-7 extra LEDs)
    uint8_t ledCol[8] = {};             // segments last written to 0x2004 for each column
    uint8_t keyRows[8] = {};            // pressed keys per column, bit set = pressed (the firmware complements the read)
    std::vector<TxByte> tx;             // everything sent on the SCI
    bool logTx = true;
    std::vector<std::pair<uint16_t, uint8_t>>* ioTrace = nullptr;   // debug: panel port writes (0x2002/0x2004)
    bool traceLcd = false;              // debug: print the call chain of every LCD character write
    void traceLcdWrite(uint8_t v)
    {
        std::printf("lcd '%c' pc=%04x ret:", v >= 0x20 && v < 0x7F ? v : '?', cpu.lastPc);
        for (int k = 1; k <= 12; k += 2) { const uint16_t sp = (uint16_t)(cpu.sp + k); if (sp >= 0x4000 && sp < 0x5FFF) std::printf(" %02x%02x", sram[sp - 0x4000], sram[sp + 1 - 0x4000]); }
        std::printf("\n");
    }
    uint32_t unknownReads = 0, unknownWrites = 0;
    uint16_t lastUnknown = 0;

    DspSystem dsp;
    bool load(const char* path)
    {
        FILE* f = std::fopen(path, "rb"); if (!f) return false;
        const size_t n = std::fread(rom, 1, sizeof rom, f); std::fclose(f);
        dsp.attach(rom);
        return n == sizeof rom;
    }
    // Audio: one 31.25 kHz frame = 32 E cycles of the HD6303R (1 MHz) and one DSP frame.
    uint64_t frameCycles = 0;
    void runFrame(int16_t in, int16_t& l, int16_t& r)
    {
        frameCycles += 32; run(frameCycles > cpu.cycles ? frameCycles - cpu.cycles : 0);
        dsp.frame(in, l, r);
    }
    void reset() { cpu.cyclesPerSerialByte = 320; cpu.reset(*this); }   // 10 bits at 31,250 baud on a 1 MHz E clock (HYP: E = 1 MHz)

    int column() const { for (int c = 0; c < 5; ++c) if (colSelect & (1 << c)) return c; return -1; }

    // ---- bus interface for HD6303 ----
    uint8_t read(uint16_t a)
    {
        if (a >= 0x8000) return rom[a - 0x8000];
        if (a >= 0x4000 && a < 0x6000) return sram[a - 0x4000];
        if (a == 0x2004) { uint8_t v = 0; for (int c = 0; c < 5; ++c) if (colSelect & (1 << c)) v |= keyRows[c]; return (uint8_t)~v; }
        ++unknownReads; lastUnknown = a; return 0xFF;
    }
    void write(uint16_t a, uint8_t v)
    {
        if (a >= 0x8000) return;
        if (a >= 0x4000 && a < 0x6000) { sram[a - 0x4000] = v; return; }
        if (a == 0x6001) { lcd.instruction(v); return; }
        if (a == 0x6000) { if (traceLcd) traceLcdWrite(v); lcd.data(v); return; }
        if (ioTrace && (a == 0x2002 || a == 0x2004)) ioTrace->push_back({a, v});
        if (a == 0x2002) { colSelect = v; return; }
        if (a == 0x2004) { const int c = column(); if (c >= 0) ledCol[c] = v; return; }
        ++unknownWrites; lastUnknown = a;
    }
    uint8_t portIn(int) { return 0xFF; }
    // Port 1 bits 0-3 select the serial multiplexer IC143/IC144 (firmware routine 0xDE40):
    // 1 = normal (MIDI IN -> RXD, TXD -> CDI chain), 2 = loopback (chain's CDO -> RXD; power-on
    // "DSP and MOD control circuit check"), 0 = used before the DRAM tests (meaning HYP).
    uint8_t mux = 0;
    void portOut(int port, uint8_t v, uint8_t ddr) { if (port == 1) mux = (uint8_t)(v & ddr & 0x0F); }

    // CDI cascade A (select bit 7) -> B (bit 6) -> MOD (bit 5): each chip acts on a word whose
    // select byte has bit 7 set and passes the word on with that byte shifted left one place.
    // Fitted to the firmware's check: it sends 00 C0 04 22 and expects 00 00 04 22 back.
    uint8_t cdiWord[4] = {}; int cdiCount = 0;
    std::deque<uint8_t> rxQueue;        // bytes waiting for the 6303's receiver (MIDI IN or CDO)
    uint64_t rxReadyAt = 0;
    void sciTx(uint8_t b)
    {
        if (logTx) tx.push_back({cpu.cycles, b});
        cdiWord[cdiCount++] = b;
        if (cdiCount == 4)
        {
            cdiCount = 0;
            if (mux == 2) { rxQueue.push_back(cdiWord[0]); rxQueue.push_back((uint8_t)(cdiWord[1] << 3)); rxQueue.push_back(cdiWord[2]); rxQueue.push_back(cdiWord[3]); }
            else if (cdiWord[0] == 0) dsp.cdiWord(cdiWord[1], cdiWord[2], cdiWord[3]);
        }
    }

    void run(uint64_t cycles)
    {
        const uint64_t end = cpu.cycles + cycles;
        while (cpu.cycles < end)
        {
            cpu.step(*this);
            if (!rxQueue.empty() && cpu.cycles >= rxReadyAt && !(cpu.trcsr & HD6303::SCI_RDRF))
            { cpu.sciReceive(rxQueue.front()); rxQueue.pop_front(); rxReadyAt = cpu.cycles + (uint64_t)cpu.cyclesPerSerialByte; }
        }
    }
};

} // namespace rev7
