// HD6303R as used by the REV7. Motorola 6801/6803 interpreter with cycle counts, the on-chip timer, serial port,
// ports 1 and 2 and the 128 bytes of on-chip RAM. Written from the Motorola
// MC6801/6803 data sheet. No Hitachi 6301 extensions (the REV7 V1.0 code uses none); an
// undefined opcode takes the 6301 TRAP vector at 0xFFEE. Cycle counts are still the
// 6801 counts (HD6303 counts are mostly lower): UNVERIFIED for timing work.
//
// The Bus type supplies:
//   uint8_t read(uint16_t addr);            // everything outside 0x00-0x1F and 0x80-0xFF
//   void    write(uint16_t addr, uint8_t v);
//   uint8_t portIn(int port);               // pin levels of port 1 or 2
//   void    portOut(int port, uint8_t v, uint8_t ddr);
//   void    sciTx(uint8_t byte);            // a byte has left the serial port
#pragma once
#include <cstdint>
#include <cstring>

namespace rev7 {

struct HD6303
{
    enum : uint8_t { CC_C = 1, CC_V = 2, CC_Z = 4, CC_N = 8, CC_I = 16, CC_H = 32 };
    enum : uint8_t { TCSR_ICF = 0x80, TCSR_OCF = 0x40, TCSR_TOF = 0x20, TCSR_EICI = 0x10, TCSR_EOCI = 0x08, TCSR_ETOI = 0x04, TCSR_IEDG = 0x02 };
    enum : uint8_t { SCI_RDRF = 0x80, SCI_ORFE = 0x40, SCI_TDRE = 0x20, SCI_RIE = 0x10, SCI_RE = 0x08, SCI_TIE = 0x04, SCI_TE = 0x02 };

    uint8_t  a = 0, b = 0, cc = 0xC0 | CC_I;
    uint16_t x = 0, sp = 0, pc = 0;
    uint8_t  iram[128] = {};
    uint8_t  p1ddr = 0, p2ddr = 0, p1 = 0, p2 = 0, p2mode = 0x40;   // mode bits read back in port 2 bits 5-7
    uint8_t  tcsr = 0, tcsrSeen = 0, rmcr = 0, trcsr = SCI_TDRE, trcsrSeen = 0, rdr = 0, tdr = 0, ramctl = 0x40;
    uint16_t counter = 0, ocr = 0xFFFF, icr = 0;
    bool     irqLine = false, waiting = false;
    int      txCountdown = 0, cyclesPerSerialByte = 640;             // 10 bits at 31,250 baud on a 2 MHz bus clock
    uint8_t  txShift = 0;
    uint64_t cycles = 0;
    uint16_t lastPc = 0;
    uint32_t illegalCount = 0;

    uint16_t d() const { return (uint16_t)((a << 8) | b); }
    void setD(uint16_t v) { a = (uint8_t)(v >> 8); b = (uint8_t)v; }

    template <class Bus> void reset(Bus& bus)
    {
        cc = 0xC0 | CC_I; p1ddr = p2ddr = 0; tcsr = 0; trcsr = SCI_TDRE; rmcr = 0; ramctl = 0x40;
        counter = 0; ocr = 0xFFFF; waiting = false; txCountdown = 0;
        pc = rd16(bus, 0xFFFE);
    }

    // A byte arrives at the serial input.
    void sciReceive(uint8_t byte)
    {
        if (!(trcsr & SCI_RE)) return;
        if (trcsr & SCI_RDRF) { trcsr |= SCI_ORFE; return; }
        rdr = byte; trcsr |= SCI_RDRF;
    }

    // An active edge on the timer capture pin (port 2 bit 0).
    void inputCapture() { icr = counter; tcsr |= TCSR_ICF; }

    // Executes one instruction (or takes one interrupt); returns the cycles used.
    template <class Bus> int step(Bus& bus)
    {
        int c = serviceInterrupts(bus);
        if (c == 0) c = waiting ? 2 : execute(bus);
        advanceTimer(bus, c);
        cycles += (uint64_t)c;
        return c;
    }

    template <class Bus> uint8_t rd(Bus& bus, uint16_t addr)
    {
        if (addr >= 0x100) return bus.read(addr);
        if (addr >= 0x80) return iram[addr - 0x80];
        if (addr >= 0x20) return bus.read(addr);
        switch (addr)
        {
        case 0x00: return p1ddr;
        case 0x01: return p2ddr;
        case 0x02: return (uint8_t)((p1 & p1ddr) | (bus.portIn(1) & ~p1ddr));
        case 0x03: return (uint8_t)(p2mode << 0 | (((p2 & p2ddr) | (bus.portIn(2) & ~p2ddr)) & 0x1F));
        case 0x08: tcsrSeen = (uint8_t)(tcsr & 0xE0); return tcsr;
        case 0x09: if (tcsrSeen & TCSR_TOF) { tcsr &= (uint8_t)~TCSR_TOF; tcsrSeen &= (uint8_t)~TCSR_TOF; } return (uint8_t)(counter >> 8);
        case 0x0A: return (uint8_t)counter;
        case 0x0B: return (uint8_t)(ocr >> 8);
        case 0x0C: return (uint8_t)ocr;
        case 0x0D: if (tcsrSeen & TCSR_ICF) { tcsr &= (uint8_t)~TCSR_ICF; tcsrSeen &= (uint8_t)~TCSR_ICF; } return (uint8_t)(icr >> 8);
        case 0x0E: return (uint8_t)icr;
        case 0x10: return rmcr;
        case 0x11: trcsrSeen = (uint8_t)(trcsr & 0xE0); return trcsr;
        case 0x12:
            if (trcsrSeen & (SCI_RDRF | SCI_ORFE)) { trcsr &= (uint8_t)~(SCI_RDRF | SCI_ORFE); trcsrSeen &= (uint8_t)~(SCI_RDRF | SCI_ORFE); }
            return rdr;
        case 0x14: return ramctl;
        default: return 0xFF;
        }
    }

    template <class Bus> void wr(Bus& bus, uint16_t addr, uint8_t v)
    {
        if (addr >= 0x100) { bus.write(addr, v); return; }
        if (addr >= 0x80) { iram[addr - 0x80] = v; return; }
        if (addr >= 0x20) { bus.write(addr, v); return; }
        switch (addr)
        {
        case 0x00: p1ddr = v; bus.portOut(1, p1, p1ddr); break;
        case 0x01: p2ddr = (uint8_t)(v & 0x1F); bus.portOut(2, p2, p2ddr); break;
        case 0x02: p1 = v; bus.portOut(1, p1, p1ddr); break;
        case 0x03: p2 = (uint8_t)(v & 0x1F); bus.portOut(2, p2, p2ddr); break;
        case 0x08: tcsr = (uint8_t)((tcsr & 0xE0) | (v & 0x1F)); break;
        case 0x09: counter = 0xFFF8; break;                       // any write presets the counter
        case 0x0B: ocr = (uint16_t)((ocr & 0x00FF) | (v << 8)); clearOcf(); break;
        case 0x0C: ocr = (uint16_t)((ocr & 0xFF00) | v); clearOcf(); break;
        case 0x10: rmcr = (uint8_t)(v & 0x0F); break;
        case 0x11: trcsr = (uint8_t)((trcsr & 0xE0) | (v & 0x1F)); break;
        case 0x13:
            tdr = v;
            if (trcsrSeen & SCI_TDRE) trcsrSeen &= (uint8_t)~SCI_TDRE;
            trcsr &= (uint8_t)~SCI_TDRE; txShift = v; txCountdown = cyclesPerSerialByte;
            break;
        case 0x14: ramctl = v; break;
        default: break;
        }
    }

private:
    void clearOcf() { if (tcsrSeen & TCSR_OCF) { tcsr &= (uint8_t)~TCSR_OCF; tcsrSeen &= (uint8_t)~TCSR_OCF; } }

    template <class Bus> void advanceTimer(Bus& bus, int c)
    {
        const uint32_t before = counter, after = before + (uint32_t)c;
        const uint32_t target = ocr > before ? ocr : (uint32_t)ocr + 0x10000u;
        if (target <= after) tcsr |= TCSR_OCF;
        if (after > 0xFFFF) tcsr |= TCSR_TOF;
        counter = (uint16_t)after;
        if (txCountdown > 0 && (txCountdown -= c) <= 0)
        {
            txCountdown = 0; trcsr |= SCI_TDRE;
            if (trcsr & SCI_TE) bus.sciTx(txShift);
        }
    }

    template <class Bus> uint16_t rd16(Bus& bus, uint16_t addr)
    { const uint8_t h = rd(bus, addr); return (uint16_t)((h << 8) | rd(bus, (uint16_t)(addr + 1))); }
    template <class Bus> void wr16(Bus& bus, uint16_t addr, uint16_t v)
    { wr(bus, addr, (uint8_t)(v >> 8)); wr(bus, (uint16_t)(addr + 1), (uint8_t)v); }
    template <class Bus> uint8_t fetch(Bus& bus) { return rd(bus, pc++); }
    template <class Bus> uint16_t fetch16(Bus& bus) { const uint8_t h = rd(bus, pc++); return (uint16_t)((h << 8) | rd(bus, pc++)); }
    template <class Bus> void push(Bus& bus, uint8_t v) { wr(bus, sp--, v); }
    template <class Bus> uint8_t pull(Bus& bus) { return rd(bus, ++sp); }
    template <class Bus> void push16(Bus& bus, uint16_t v) { push(bus, (uint8_t)v); push(bus, (uint8_t)(v >> 8)); }
    template <class Bus> uint16_t pull16(Bus& bus) { const uint8_t h = pull(bus); return (uint16_t)((h << 8) | pull(bus)); }

    template <class Bus> void pushAll(Bus& bus)
    { push16(bus, pc); push16(bus, x); push(bus, a); push(bus, b); push(bus, cc); }

    template <class Bus> int serviceInterrupts(Bus& bus)
    {
        if (cc & CC_I) return 0;
        uint16_t vec = 0;
        if (irqLine) vec = 0xFFF8;
        else if ((tcsr & TCSR_ICF) && (tcsr & TCSR_EICI)) vec = 0xFFF6;
        else if ((tcsr & TCSR_OCF) && (tcsr & TCSR_EOCI)) vec = 0xFFF4;
        else if ((tcsr & TCSR_TOF) && (tcsr & TCSR_ETOI)) vec = 0xFFF2;
        else if (((trcsr & (SCI_RDRF | SCI_ORFE)) && (trcsr & SCI_RIE)) || ((trcsr & SCI_TDRE) && (trcsr & SCI_TIE))) vec = 0xFFF0;
        if (!vec) return 0;
        if (!waiting) pushAll(bus);
        waiting = false; cc |= CC_I; pc = rd16(bus, vec);
        return 12;
    }

    void nz8(uint8_t r) { cc = (uint8_t)((cc & ~(CC_N | CC_Z)) | (r & 0x80 ? CC_N : 0) | (r == 0 ? CC_Z : 0)); }
    void nz16(uint16_t r) { cc = (uint8_t)((cc & ~(CC_N | CC_Z)) | (r & 0x8000 ? CC_N : 0) | (r == 0 ? CC_Z : 0)); }
    void setFlag(uint8_t f, bool on) { cc = (uint8_t)(on ? (cc | f) : (cc & ~f)); }

    uint8_t add8(uint8_t p, uint8_t q, int carry)
    {
        const unsigned r = (unsigned)p + q + (unsigned)carry;
        setFlag(CC_H, ((p ^ q ^ r) & 0x10) != 0); setFlag(CC_V, ((p ^ r) & (q ^ r) & 0x80) != 0); setFlag(CC_C, r > 0xFF);
        nz8((uint8_t)r); return (uint8_t)r;
    }
    uint8_t sub8(uint8_t p, uint8_t q, int carry)
    {
        const unsigned r = (unsigned)p - q - (unsigned)carry;
        setFlag(CC_V, ((p ^ q) & (p ^ r) & 0x80) != 0); setFlag(CC_C, (r & 0x100) != 0);
        nz8((uint8_t)r); return (uint8_t)r;
    }
    uint16_t add16(uint16_t p, uint16_t q)
    {
        const uint32_t r = (uint32_t)p + q;
        setFlag(CC_V, ((p ^ r) & (q ^ r) & 0x8000) != 0); setFlag(CC_C, r > 0xFFFF); nz16((uint16_t)r); return (uint16_t)r;
    }
    uint16_t sub16(uint16_t p, uint16_t q)
    {
        const uint32_t r = (uint32_t)p - q;
        setFlag(CC_V, ((p ^ q) & (p ^ r) & 0x8000) != 0); setFlag(CC_C, (r & 0x10000) != 0); nz16((uint16_t)r); return (uint16_t)r;
    }
    void logic(uint8_t r) { nz8(r); setFlag(CC_V, false); }

    // Read-modify-write group shared by the 0x40, 0x50, 0x60 and 0x70 columns.
    uint8_t rmw(int op, uint8_t m, bool& store)
    {
        store = true; uint8_t r = m; const int c = cc & CC_C;
        switch (op)
        {
        case 0x0: r = (uint8_t)(0 - m); nz8(r); setFlag(CC_V, r == 0x80); setFlag(CC_C, r != 0); break;                 // NEG
        case 0x3: r = (uint8_t)~m; nz8(r); setFlag(CC_V, false); setFlag(CC_C, true); break;                           // COM
        case 0x4: r = (uint8_t)(m >> 1); setFlag(CC_C, m & 1); nz8(r); setFlag(CC_V, (m & 1) != 0); break;             // LSR
        case 0x6: r = (uint8_t)((m >> 1) | (c << 7)); setFlag(CC_C, m & 1); nz8(r); vFromNC(); break;                  // ROR
        case 0x7: r = (uint8_t)((m >> 1) | (m & 0x80)); setFlag(CC_C, m & 1); nz8(r); vFromNC(); break;                // ASR
        case 0x8: r = (uint8_t)(m << 1); setFlag(CC_C, (m & 0x80) != 0); nz8(r); vFromNC(); break;                     // ASL
        case 0x9: r = (uint8_t)((m << 1) | c); setFlag(CC_C, (m & 0x80) != 0); nz8(r); vFromNC(); break;               // ROL
        case 0xA: r = (uint8_t)(m - 1); nz8(r); setFlag(CC_V, m == 0x80); break;                                       // DEC
        case 0xC: r = (uint8_t)(m + 1); nz8(r); setFlag(CC_V, m == 0x7F); break;                                       // INC
        case 0xD: nz8(m); setFlag(CC_V, false); setFlag(CC_C, false); store = false; break;                            // TST
        case 0xF: r = 0; nz8(0); setFlag(CC_V, false); setFlag(CC_C, false); break;                                    // CLR
        default: store = false; ++illegalCount; break;
        }
        return r;
    }
    void vFromNC() { setFlag(CC_V, ((cc & CC_N) != 0) != ((cc & CC_C) != 0)); }

    bool cond(int n) const
    {
        const bool c = cc & CC_C, v = cc & CC_V, z = cc & CC_Z, ng = cc & CC_N;
        switch (n)
        {
        case 0x0: return true;        case 0x1: return false;
        case 0x2: return !(c || z);   case 0x3: return c || z;
        case 0x4: return !c;          case 0x5: return c;
        case 0x6: return !z;          case 0x7: return z;
        case 0x8: return !v;          case 0x9: return v;
        case 0xA: return !ng;         case 0xB: return ng;
        case 0xC: return ng == v;     case 0xD: return ng != v;
        case 0xE: return !z && ng == v; default: return z || ng != v;
        }
    }

    template <class Bus> int execute(Bus& bus)
    {
        lastPc = pc;
        const uint8_t op = fetch(bus);
        const int hi = op >> 4, lo = op & 15;

        if (hi == 2) { const int8_t off = (int8_t)fetch(bus); if (cond(lo)) pc = (uint16_t)(pc + off); return 3; }

        if (hi >= 4 && hi <= 7)
        {
            if (hi == 4 || hi == 5)
            {
                uint8_t& reg = hi == 4 ? a : b; bool store;
                const uint8_t r = rmw(lo, reg, store); if (store) reg = r;
                return 2;
            }
            const uint16_t ea = hi == 6 ? (uint16_t)(x + fetch(bus)) : fetch16(bus);
            if (lo == 0xE) { pc = ea; return 3; }                                                                      // JMP
            bool store;
            const uint8_t m = lo == 0xF ? 0 : rd(bus, ea);
            const uint8_t r = rmw(lo, m, store); if (store) wr(bus, ea, r);
            return 6;
        }

        if (hi >= 8)
        {
            const int mode = hi & 3;                 // 0 immediate, 1 direct, 2 indexed, 3 extended
            const bool isB = hi >= 0xC;
            uint8_t& acc = isB ? b : a;
            const bool wide = lo == 3 || lo == 0xC || lo == 0xD || lo == 0xE || lo == 0xF;
            const bool isStore = lo == 7 || (lo == 0xF) || (isB && lo == 0xD);
            // 0x8D is BSR, 0x9D/0xAD/0xBD JSR
            if (!isB && lo == 0xD)
            {
                if (mode == 0) { const int8_t off = (int8_t)fetch(bus); push16(bus, pc); pc = (uint16_t)(pc + off); return 6; }
                const uint16_t ea = effective(bus, mode); push16(bus, pc); pc = ea; return mode == 1 ? 5 : 6;
            }
            if (mode == 0 && isStore) { ++illegalCount; return 2; }
            uint16_t ea = 0; uint8_t m8 = 0; uint16_t m16 = 0;
            if (mode == 0) { if (wide) m16 = fetch16(bus); else m8 = fetch(bus); }
            else
            {
                ea = effective(bus, mode);
                if (!isStore) { if (wide) m16 = rd16(bus, ea); else m8 = rd(bus, ea); }
            }
            static const int base8[4] = { 2, 3, 4, 4 };
            static const int base16arith[4] = { 4, 5, 6, 6 };     // SUBD, ADDD, CPX
            static const int base16load[4] = { 3, 4, 5, 5 };      // LDS, LDX, LDD and the 16-bit stores
            switch (lo)
            {
            case 0x0: acc = sub8(acc, m8, 0); return base8[mode];
            case 0x1: sub8(acc, m8, 0); return base8[mode];
            case 0x2: acc = sub8(acc, m8, cc & CC_C); return base8[mode];
            case 0x3: if (isB) setD(add16(d(), m16)); else setD(sub16(d(), m16)); return base16arith[mode];
            case 0x4: acc &= m8; logic(acc); return base8[mode];
            case 0x5: logic((uint8_t)(acc & m8)); return base8[mode];
            case 0x6: acc = m8; logic(acc); return base8[mode];
            case 0x7: logic(acc); wr(bus, ea, acc); return base8[mode];
            case 0x8: acc ^= m8; logic(acc); return base8[mode];
            case 0x9: acc = add8(acc, m8, cc & CC_C); return base8[mode];
            case 0xA: acc |= m8; logic(acc); return base8[mode];
            case 0xB: acc = add8(acc, m8, 0); return base8[mode];
            case 0xC:
                if (isB) { setD(m16); nz16(m16); setFlag(CC_V, false); return base16load[mode]; }                      // LDD
                sub16(x, m16); return base16arith[mode];                                                               // CPX
            case 0xD: nz16(d()); setFlag(CC_V, false); wr16(bus, ea, d()); return base16load[mode];                    // STD
            case 0xE:
                if (isB) x = m16; else sp = m16;
                nz16(m16); setFlag(CC_V, false); return base16load[mode];                                              // LDX / LDS
            default:
            {
                const uint16_t v = isB ? x : sp;
                nz16(v); setFlag(CC_V, false); wr16(bus, ea, v); return base16load[mode];                              // STX / STS
            }
            }
        }

        switch (op)
        {
        case 0x01: return 2;                                                                                           // NOP
        case 0x04: { const uint16_t v = d(); const uint16_t r = (uint16_t)(v >> 1); setFlag(CC_C, v & 1); nz16(r); vFromNC(); setD(r); return 3; }      // LSRD
        case 0x05: { const uint16_t v = d(); const uint16_t r = (uint16_t)(v << 1); setFlag(CC_C, (v & 0x8000) != 0); nz16(r); vFromNC(); setD(r); return 3; } // ASLD
        case 0x06: cc = (uint8_t)(a | 0xC0); return 2;                                                                 // TAP
        case 0x07: a = (uint8_t)(cc | 0xC0); return 2;                                                                 // TPA
        case 0x08: ++x; setFlag(CC_Z, x == 0); return 3;                                                               // INX
        case 0x09: --x; setFlag(CC_Z, x == 0); return 3;                                                               // DEX
        case 0x0A: setFlag(CC_V, false); return 2;
        case 0x0B: setFlag(CC_V, true); return 2;
        case 0x0C: setFlag(CC_C, false); return 2;
        case 0x0D: setFlag(CC_C, true); return 2;
        case 0x0E: setFlag(CC_I, false); return 2;
        case 0x0F: setFlag(CC_I, true); return 2;
        case 0x10: a = sub8(a, b, 0); return 2;                                                                        // SBA
        case 0x11: sub8(a, b, 0); return 2;                                                                            // CBA
        case 0x16: b = a; logic(b); return 2;                                                                          // TAB
        case 0x17: a = b; logic(a); return 2;                                                                          // TBA
        case 0x19:                                                                                                     // DAA
        {
            unsigned adj = 0; const bool c = cc & CC_C;
            if ((cc & CC_H) || (a & 0x0F) > 9) adj |= 0x06;
            if (c || a > 0x99) adj |= 0x60;
            const unsigned r = a + adj; a = (uint8_t)r; nz8(a); setFlag(CC_C, c || r > 0xFF); return 2;
        }
        case 0x1B: a = add8(a, b, 0); return 2;                                                                        // ABA
        case 0x30: x = (uint16_t)(sp + 1); return 3;                                                                   // TSX
        case 0x31: ++sp; return 3;                                                                                     // INS
        case 0x32: a = pull(bus); return 4;
        case 0x33: b = pull(bus); return 4;
        case 0x34: --sp; return 3;                                                                                     // DES
        case 0x35: sp = (uint16_t)(x - 1); return 3;                                                                   // TXS
        case 0x36: push(bus, a); return 3;
        case 0x37: push(bus, b); return 3;
        case 0x38: x = pull16(bus); return 5;                                                                          // PULX
        case 0x39: pc = pull16(bus); return 5;                                                                         // RTS
        case 0x3A: x = (uint16_t)(x + b); return 3;                                                                    // ABX
        case 0x3B: cc = (uint8_t)(pull(bus) | 0xC0); b = pull(bus); a = pull(bus); x = pull16(bus); pc = pull16(bus); return 10; // RTI
        case 0x3C: push16(bus, x); return 4;                                                                           // PSHX
        case 0x3D: { const uint16_t r = (uint16_t)(a * b); setD(r); setFlag(CC_C, (r & 0x80) != 0); return 10; }        // MUL
        case 0x3E: pushAll(bus); waiting = true; return 9;                                                             // WAI
        case 0x3F: pushAll(bus); cc |= CC_I; pc = rd16(bus, 0xFFFA); return 12;                                        // SWI
        default: ++illegalCount; pc = lastPc; pushAll(bus); cc |= CC_I; pc = rd16(bus, 0xFFEE); return 12;   // 6301 TRAP
        }
    }

    template <class Bus> uint16_t effective(Bus& bus, int mode)
    {
        if (mode == 1) return fetch(bus);
        if (mode == 2) return (uint16_t)(x + fetch(bus));
        return fetch16(bus);
    }
};

} // namespace rev7
