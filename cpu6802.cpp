// ============================================================
//  MIT License
//
//  Copyright (c) 2026 Kamel Khelif
//
//  Permission is hereby granted, free of charge, to any person
//  obtaining a copy of this software and associated documentation
//  files (the "Software"), to deal in the Software without
//  restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following
//  conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
//  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//  OTHER DEALINGS IN THE SOFTWARE.
// ============================================================
//
//  MOTOROLA 6802 EMULATOR FOR ESP32
//  cpu6802.cpp — Full emulator core
//
//  Implements all 72 instructions of the Motorola 6802 / 6800
//  instruction set, covering:
//    - All addressing modes: inherent, immediate, direct,
//      extended, indexed, relative
//    - Accurate flag updates (H I N Z V C) per instruction
//    - Hardware interrupt handling: IRQ, NMI, SWI, RTI
//    - Memory-mapped I/O dispatch ($8000-$8FFF)
//    - Built-in serial monitor helpers: disassembler,
//      register dump, hex memory dump
//
// ============================================================

#include "cpu6802.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// ============================================================
//  INTERNAL MACROS
// ============================================================

// Memory read/write shortcuts
#define READ(addr)        cpu6802_read(cpu, addr)
#define WRITE(addr, val)  cpu6802_write(cpu, addr, (uint8_t)(val))
#define READ16(addr)      cpu6802_read16(cpu, addr)

// Condition-code flag helpers
#define SET_FLAG(f)       (cpu->CC |= (f))
#define CLR_FLAG(f)       (cpu->CC &= ~(f))
#define TST_FLAG(f)       ((cpu->CC & (f)) != 0)
#define SET_OR_CLR(f, c)  do { if(c) SET_FLAG(f); else CLR_FLAG(f); } while(0)

// Update N (Negative) and Z (Zero) flags after an 8-bit or 16-bit result
#define UPD_NZ8(val)  do { \
    SET_OR_CLR(CC_N, (val) & 0x80); \
    SET_OR_CLR(CC_Z, ((val) & 0xFF) == 0); \
} while(0)

#define UPD_NZ16(val) do { \
    SET_OR_CLR(CC_N, (val) & 0x8000); \
    SET_OR_CLR(CC_Z, ((val) & 0xFFFF) == 0); \
} while(0)

// Fetch next byte(s) from memory at PC, auto-increment PC
#define FETCH8()   (READ(cpu->PC++))
#define FETCH16()  (cpu->PC += 2, READ16(cpu->PC - 2))

// Stack push/pull helpers (SP grows downward)
#define PUSH8(v)   do { WRITE(cpu->SP--, (v) & 0xFF); } while(0)
#define PULL8()    (READ(++cpu->SP))
#define PUSH16(v)  do { PUSH8((v) & 0xFF); PUSH8(((v) >> 8) & 0xFF); } while(0)
#define PULL16()   ((uint16_t)(PULL8() << 8) | PULL8())

// ============================================================
//  MEMORY ACCESS
// ============================================================

uint8_t cpu6802_read(CPU6802 *cpu, uint16_t addr) {
    // Internal RAM $0000-$007F (128 bytes, built into the 6802 chip)
    if (addr < RAM_INTERNAL) {
        return cpu->ram_internal[addr];
    }
    // Memory-mapped I/O window $8000-$8FFF
    if (addr >= IO_BASE && addr <= IO_END) {
        if (cpu->io_read_cb) return cpu->io_read_cb(addr);
        // Default UART handler (if no callback is registered)
        if (addr == UART_DATA) {
            if (Serial.available()) return (uint8_t)Serial.read();
            return 0x00;
        }
        if (addr == UART_STATUS) {
            uint8_t st = 0x02; // TX always ready on ESP32
            if (Serial.available()) st |= 0x01;
            return st;
        }
        return 0xFF;
    }
    return cpu->memory[addr];
}

void cpu6802_write(CPU6802 *cpu, uint16_t addr, uint8_t val) {
    // Internal RAM $0000-$007F
    if (addr < RAM_INTERNAL) {
        cpu->ram_internal[addr] = val;
        return;
    }
    // Memory-mapped I/O window $8000-$8FFF
    if (addr >= IO_BASE && addr <= IO_END) {
        if (cpu->io_write_cb) { cpu->io_write_cb(addr, val); return; }
        if (addr == UART_DATA) {
            Serial.write(val);
        }
        return;
    }
    // High memory area ($F000-$FFFF) — writable so the user can
    // load programs and set interrupt vectors from the monitor.
    // Remove this block if you want to enforce read-only ROM.
    if (addr >= ROM_START) {
        // Allow writes (no protection in this build)
    }
    cpu->memory[addr] = val;
}

uint16_t cpu6802_read16(CPU6802 *cpu, uint16_t addr) {
    return ((uint16_t)cpu6802_read(cpu, addr) << 8) | cpu6802_read(cpu, addr + 1);
}

// ============================================================
//  INITIALISATION AND RESET
// ============================================================

void cpu6802_init(CPU6802 *cpu) {
    memset(cpu, 0, sizeof(CPU6802));
    cpu->CC = CC_BIT6 | CC_BIT7 | CC_I; // Bits 6,7 always 1; IRQ masked after reset
    cpu->SP = 0x00FF; // Default stack pointer (top of zero page)
    cpu->io_read_cb = NULL;
    cpu->io_write_cb = NULL;
    cpu->debug_mode = false;
}

void cpu6802_reset(CPU6802 *cpu) {
    cpu->CC |= CC_I;          // Masque IRQ
    cpu->halted = false;
    cpu->irq_pending = false;
    cpu->nmi_pending = false;
    cpu->cycles = 0;
    cpu->instr_count = 0;
    // Load PC from the RESET vector at $FFFE-$FFFF
    cpu->PC = READ16(VEC_RESET);
    if (cpu->debug_mode) {
        Serial.print(F("[RESET] PC=0x"));
        Serial.println(cpu->PC, HEX);
    }
}

void cpu6802_load(CPU6802 *cpu, uint16_t addr, const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        cpu->memory[(uint32_t)(addr + i) & 0xFFFF] = data[i];
    }
}

// ============================================================
//  INTERRUPT HANDLING
// ============================================================

void cpu6802_irq(CPU6802 *cpu) { cpu->irq_pending = true; }
void cpu6802_nmi(CPU6802 *cpu) { cpu->nmi_pending = true; }

static void do_interrupt(CPU6802 *cpu, uint16_t vector) {
    // Save full CPU context onto the stack (6800 order: PC, X, A, B, CC)
    PUSH16(cpu->PC);
    PUSH16(cpu->X);
    PUSH8(cpu->A);
    PUSH8(cpu->B);
    PUSH8(cpu->CC);
    SET_FLAG(CC_I);           // Mask further IRQs while in handler
    cpu->PC = READ16(vector);
    cpu->cycles += 12;
}

// RTI — Return from interrupt: restore full context from stack
static void do_rti(CPU6802 *cpu) {
    cpu->CC = PULL8();
    cpu->B  = PULL8();
    cpu->A  = PULL8();
    cpu->X  = PULL16();
    cpu->PC = PULL16();
    cpu->cycles += 10;
}

// ============================================================
//  ARITHMETIC HELPERS
// ============================================================

// 8-bit addition with optional carry; updates all relevant flags
static uint8_t add8(CPU6802 *cpu, uint8_t a, uint8_t b, uint8_t carry) {
    uint16_t res16 = (uint16_t)a + b + carry;
    uint8_t res = (uint8_t)res16;
    // Half-carry: carry from bit 3 into bit 4 (needed for BCD / DAA)
    SET_OR_CLR(CC_H, ((a & 0x0F) + (b & 0x0F) + carry) > 0x0F);
    SET_OR_CLR(CC_C, res16 > 0xFF);
    // Overflow: unexpected sign change (signed arithmetic overflow)
    SET_OR_CLR(CC_V, ((a ^ res) & (b ^ res)) & 0x80);
    UPD_NZ8(res);
    return res;
}

// SUB 8 bits avec borrow, met à jour tous les flags
static uint8_t sub8(CPU6802 *cpu, uint8_t a, uint8_t b, uint8_t borrow) {
    uint16_t res16 = (uint16_t)a - b - borrow;
    uint8_t res = (uint8_t)res16;
    SET_OR_CLR(CC_C, res16 > 0xFF);
    SET_OR_CLR(CC_V, ((a ^ b) & (a ^ res)) & 0x80);
    UPD_NZ8(res);
    return res;
}

// DAA — Decimal Adjust Accumulator A
static void do_daa(CPU6802 *cpu) {
    uint8_t cf = 0, hf = TST_FLAG(CC_H) ? 1 : 0;
    uint8_t a = cpu->A;
    if (!TST_FLAG(CC_C)) {
        if ((a & 0x0F) > 9 || hf) cf = 0x06;
        if (a > 0x99 || TST_FLAG(CC_C)) { cf |= 0x60; SET_FLAG(CC_C); }
    } else {
        cf = (hf || (a & 0x0F) > 9) ? 0x66 : 0x60;
        SET_FLAG(CC_C);
    }
    cpu->A += cf;
    UPD_NZ8(cpu->A);
    // V non défini après DAA (comportement 6802 réel)
}

// ============================================================
//  ADDRESSING MODES — compute and return the effective address
// ============================================================
static uint16_t addr_direct(CPU6802 *cpu)   { return FETCH8(); }                        // Direct (page 0)
static uint16_t addr_extended(CPU6802 *cpu) { return FETCH16(); }                       // Extended (16-bit)
static uint16_t addr_indexed(CPU6802 *cpu)  { return (cpu->X + FETCH8()) & 0xFFFF; }   // Indexed (X + offset)

// ============================================================
//  STEP PRINCIPAL
// ============================================================

int cpu6802_step(CPU6802 *cpu) {
    if (cpu->halted) return 0;

    // ---- NMI handling (non-maskable, always serviced) ----
    if (cpu->nmi_pending) {
        cpu->nmi_pending = false;
        do_interrupt(cpu, VEC_NMI);
        return 12;
    }
    // ---- IRQ handling (if not masked by I flag) ----
    if (cpu->irq_pending && !TST_FLAG(CC_I)) {
        cpu->irq_pending = false;
        do_interrupt(cpu, VEC_IRQ);
        return 12;
    }

    uint16_t pc_before = cpu->PC;
    uint8_t  opcode    = FETCH8();
    int      cycles    = 2; // minimum
    uint16_t ea;            // effective address
    uint8_t  tmp8;
    uint16_t tmp16;

    if (cpu->debug_mode) {
        char buf[64];
        cpu6802_disasm(cpu, pc_before, buf, sizeof(buf));
        Serial.print(F("PC:")); Serial.print(pc_before, HEX);
        Serial.print(F(" ")); Serial.print(buf);
        Serial.print(F("  A:")); Serial.print(cpu->A, HEX);
        Serial.print(F(" B:")); Serial.print(cpu->B, HEX);
        Serial.print(F(" X:")); Serial.print(cpu->X, HEX);
        Serial.print(F(" SP:")); Serial.print(cpu->SP, HEX);
        Serial.print(F(" CC:")); Serial.println(cpu->CC, BIN);
    }

    switch (opcode) {

    // --------------------------------------------------------
    //  NOP
    // --------------------------------------------------------
    case 0x01: cycles = 2; break; // NOP

    // --------------------------------------------------------
    //  TAP / TPA / INX / DEX / CLV / SEV / CLC / SEC / CLI / SEI
    // --------------------------------------------------------
    case 0x06: cpu->CC = cpu->A; cycles = 2; break;                          // TAP  A -> CC
    case 0x07: cpu->A  = cpu->CC; UPD_NZ8(cpu->A); cycles = 2; break;       // TPA  CC -> A
    case 0x08: cpu->X++; SET_OR_CLR(CC_Z, cpu->X == 0); cycles = 4; break;  // INX
    case 0x09: cpu->X--; SET_OR_CLR(CC_Z, cpu->X == 0); cycles = 4; break;  // DEX
    case 0x0A: CLR_FLAG(CC_V); cycles = 2; break;  // CLV
    case 0x0B: SET_FLAG(CC_V); cycles = 2; break;  // SEV
    case 0x0C: CLR_FLAG(CC_C); cycles = 2; break;  // CLC
    case 0x0D: SET_FLAG(CC_C); cycles = 2; break;  // SEC
    case 0x0E: CLR_FLAG(CC_I); cycles = 2; break;  // CLI
    case 0x0F: SET_FLAG(CC_I); cycles = 2; break;  // SEI

    // --------------------------------------------------------
    //  SBA / CBA / TAB / TBA / DAA / ABA
    // --------------------------------------------------------
    case 0x10: cpu->A = sub8(cpu, cpu->A, cpu->B, 0); cycles = 2; break; // SBA
    case 0x11: sub8(cpu, cpu->A, cpu->B, 0); cycles = 2; break;          // CBA (compare, no store)
    case 0x16: cpu->A = cpu->B; UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles = 2; break; // TAB
    case 0x17: cpu->B = cpu->A; UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles = 2; break; // TBA
    case 0x19: do_daa(cpu); cycles = 2; break;                            // DAA
    case 0x1B: cpu->A = add8(cpu, cpu->A, cpu->B, 0); cycles = 2; break; // ABA

    // --------------------------------------------------------
    //  Branchements relatifs
    // --------------------------------------------------------
    #define BRANCH(cond) do { \
        int8_t off = (int8_t)FETCH8(); \
        cycles = 4; \
        if (cond) cpu->PC = (uint16_t)(cpu->PC + off); \
    } while(0)

    case 0x20: BRANCH(1); break;                                              // BRA
    case 0x22: BRANCH(!TST_FLAG(CC_C) && !TST_FLAG(CC_Z)); break;            // BHI
    case 0x23: BRANCH(TST_FLAG(CC_C) || TST_FLAG(CC_Z)); break;              // BLS
    case 0x24: BRANCH(!TST_FLAG(CC_C)); break;                                // BCC (BHS)
    case 0x25: BRANCH(TST_FLAG(CC_C)); break;                                 // BCS (BLO)
    case 0x26: BRANCH(!TST_FLAG(CC_Z)); break;                                // BNE
    case 0x27: BRANCH(TST_FLAG(CC_Z)); break;                                 // BEQ
    case 0x28: BRANCH(!TST_FLAG(CC_V)); break;                                // BVC
    case 0x29: BRANCH(TST_FLAG(CC_V)); break;                                 // BVS
    case 0x2A: BRANCH(!TST_FLAG(CC_N)); break;                                // BPL
    case 0x2B: BRANCH(TST_FLAG(CC_N)); break;                                 // BMI
    case 0x2C: BRANCH(TST_FLAG(CC_N) == TST_FLAG(CC_V)); break;              // BGE
    case 0x2D: BRANCH(TST_FLAG(CC_N) != TST_FLAG(CC_V)); break;              // BLT
    case 0x2E: BRANCH(!TST_FLAG(CC_Z) && (TST_FLAG(CC_N)==TST_FLAG(CC_V))); break; // BGT
    case 0x2F: BRANCH(TST_FLAG(CC_Z) || (TST_FLAG(CC_N)!=TST_FLAG(CC_V))); break;  // BLE

    // --------------------------------------------------------
    //  TSX / INS / PULA / PULB / DES / TXS / PSHA / PSHB
    // --------------------------------------------------------
    case 0x30: cpu->X = cpu->SP + 1; cycles = 4; break;  // TSX
    case 0x31: cpu->SP++; cycles = 4; break;              // INS
    case 0x32: cpu->A = PULL8(); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles = 4; break; // PULA
    case 0x33: cpu->B = PULL8(); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles = 4; break; // PULB
    case 0x34: cpu->SP--; cycles = 4; break;              // DES
    case 0x35: cpu->SP = cpu->X - 1; cycles = 4; break;  // TXS
    case 0x36: PUSH8(cpu->A); cycles = 4; break;          // PSHA
    case 0x37: PUSH8(cpu->B); cycles = 4; break;          // PSHB
    case 0x39: // RTS
        cpu->PC = PULL16(); cycles = 5; break;
    case 0x3B: // RTI
        do_rti(cpu); break;
    case 0x3E: // WAI — attendre interruption
        // Bloquer le PC sur l'instruction WAI.
        // Le CPU reste ici jusqu'à réception d'un IRQ ou NMI.
        // do_interrupt() sauvegardera le contexte normalement.
        cpu->PC = pc_before;
        cycles = 9; break;
    case 0x3F: // SWI
        do_interrupt(cpu, VEC_SWI); break;

    // --------------------------------------------------------
    //  Opérations sur accumulateur A (inherent)
    // --------------------------------------------------------
    case 0x40: // NEGA
        tmp8 = sub8(cpu, 0, cpu->A, 0);
        SET_OR_CLR(CC_V, cpu->A == 0x80);
        cpu->A = tmp8; cycles = 2; break;
    case 0x43: // COMA
        cpu->A ^= 0xFF; UPD_NZ8(cpu->A); CLR_FLAG(CC_V); SET_FLAG(CC_C); cycles = 2; break;
    case 0x44: // LSRA
        SET_OR_CLR(CC_C, cpu->A & 0x01);
        cpu->A >>= 1; UPD_NZ8(cpu->A); CLR_FLAG(CC_N); cycles = 2; break;
    case 0x46: // RORA
        tmp8 = (cpu->A >> 1) | (TST_FLAG(CC_C) ? 0x80 : 0);
        SET_OR_CLR(CC_C, cpu->A & 0x01);
        cpu->A = tmp8; UPD_NZ8(cpu->A); cycles = 2; break;
    case 0x47: // ASRA
        SET_OR_CLR(CC_C, cpu->A & 0x01);
        cpu->A = (cpu->A >> 1) | (cpu->A & 0x80);
        UPD_NZ8(cpu->A); cycles = 2; break;
    case 0x48: // ASLA / LSLA
        SET_OR_CLR(CC_C, cpu->A & 0x80);
        cpu->A <<= 1; UPD_NZ8(cpu->A); cycles = 2; break;
    case 0x49: // ROLA
        tmp8 = (cpu->A << 1) | (TST_FLAG(CC_C) ? 0x01 : 0);
        SET_OR_CLR(CC_C, cpu->A & 0x80);
        SET_OR_CLR(CC_V, ((cpu->A ^ tmp8) & 0x80) != 0);
        cpu->A = tmp8; UPD_NZ8(cpu->A); cycles = 2; break;
    case 0x4A: // DECA
        SET_OR_CLR(CC_V, cpu->A == 0x80);
        cpu->A--; UPD_NZ8(cpu->A); cycles = 2; break;
    case 0x4C: // INCA
        SET_OR_CLR(CC_V, cpu->A == 0x7F);
        cpu->A++; UPD_NZ8(cpu->A); cycles = 2; break;
    case 0x4D: // TSTA
        UPD_NZ8(cpu->A); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles = 2; break;
    case 0x4F: // CLRA
        cpu->A = 0; CLR_FLAG(CC_N); SET_FLAG(CC_Z); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles = 2; break;

    // --------------------------------------------------------
    //  Opérations sur accumulateur B (inherent)
    // --------------------------------------------------------
    case 0x50: tmp8 = sub8(cpu, 0, cpu->B, 0); SET_OR_CLR(CC_V, cpu->B==0x80); cpu->B=tmp8; cycles=2; break; // NEGB
    case 0x53: cpu->B^=0xFF; UPD_NZ8(cpu->B); CLR_FLAG(CC_V); SET_FLAG(CC_C); cycles=2; break; // COMB
    case 0x54: SET_OR_CLR(CC_C,cpu->B&0x01); cpu->B>>=1; UPD_NZ8(cpu->B); CLR_FLAG(CC_N); cycles=2; break; // LSRB
    case 0x56: tmp8=(cpu->B>>1)|(TST_FLAG(CC_C)?0x80:0); SET_OR_CLR(CC_C,cpu->B&0x01); cpu->B=tmp8; UPD_NZ8(cpu->B); cycles=2; break; // RORB
    case 0x57: SET_OR_CLR(CC_C,cpu->B&0x01); cpu->B=(cpu->B>>1)|(cpu->B&0x80); UPD_NZ8(cpu->B); cycles=2; break; // ASRB
    case 0x58: SET_OR_CLR(CC_C,cpu->B&0x80); cpu->B<<=1; UPD_NZ8(cpu->B); cycles=2; break; // ASLB
    case 0x59: tmp8=(cpu->B<<1)|(TST_FLAG(CC_C)?0x01:0); SET_OR_CLR(CC_C,cpu->B&0x80); SET_OR_CLR(CC_V,((cpu->B^tmp8)&0x80)!=0); cpu->B=tmp8; UPD_NZ8(cpu->B); cycles=2; break; // ROLB
    case 0x5A: SET_OR_CLR(CC_V,cpu->B==0x80); cpu->B--; UPD_NZ8(cpu->B); cycles=2; break; // DECB
    case 0x5C: SET_OR_CLR(CC_V,cpu->B==0x7F); cpu->B++; UPD_NZ8(cpu->B); cycles=2; break; // INCB
    case 0x5D: UPD_NZ8(cpu->B); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles=2; break; // TSTB
    case 0x5F: cpu->B=0; CLR_FLAG(CC_N); SET_FLAG(CC_Z); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles=2; break; // CLRB

    // --------------------------------------------------------
    //  Memory operations — indexed / direct / extended addressing
    //  (NEG, COM, LSR, ROR, ASR, ASL, ROL, DEC, INC, TST, JMP, CLR)
    // --------------------------------------------------------
    #define MEM_OP(ea, op, cyc) do { \
        tmp8 = READ(ea); op; WRITE(ea, tmp8); cycles = cyc; \
    } while(0)

    // NEG
    case 0x60: ea=addr_indexed(cpu);  tmp8=READ(ea); tmp8=sub8(cpu,0,tmp8,0); WRITE(ea,tmp8); cycles=7; break;
    case 0x70: ea=addr_extended(cpu); tmp8=READ(ea); tmp8=sub8(cpu,0,tmp8,0); WRITE(ea,tmp8); cycles=6; break;

    // COM
    case 0x63: ea=addr_indexed(cpu);  tmp8=READ(ea)^0xFF; UPD_NZ8(tmp8); CLR_FLAG(CC_V); SET_FLAG(CC_C); WRITE(ea,tmp8); cycles=7; break;
    case 0x73: ea=addr_extended(cpu); tmp8=READ(ea)^0xFF; UPD_NZ8(tmp8); CLR_FLAG(CC_V); SET_FLAG(CC_C); WRITE(ea,tmp8); cycles=6; break;

    // LSR
    case 0x64: ea=addr_indexed(cpu);  tmp8=READ(ea); SET_OR_CLR(CC_C,tmp8&0x01); tmp8>>=1; UPD_NZ8(tmp8); CLR_FLAG(CC_N); WRITE(ea,tmp8); cycles=7; break;
    case 0x74: ea=addr_extended(cpu); tmp8=READ(ea); SET_OR_CLR(CC_C,tmp8&0x01); tmp8>>=1; UPD_NZ8(tmp8); CLR_FLAG(CC_N); WRITE(ea,tmp8); cycles=6; break;

    // ROR
    case 0x66: ea=addr_indexed(cpu);  { uint8_t r=READ(ea); uint8_t n=(r>>1)|(TST_FLAG(CC_C)?0x80:0); SET_OR_CLR(CC_C,r&0x01); UPD_NZ8(n); WRITE(ea,n); cycles=7; } break;
    case 0x76: ea=addr_extended(cpu); { uint8_t r=READ(ea); uint8_t n=(r>>1)|(TST_FLAG(CC_C)?0x80:0); SET_OR_CLR(CC_C,r&0x01); UPD_NZ8(n); WRITE(ea,n); cycles=6; } break;

    // ASR
    case 0x67: ea=addr_indexed(cpu);  tmp8=READ(ea); SET_OR_CLR(CC_C,tmp8&0x01); tmp8=(tmp8>>1)|(tmp8&0x80); UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=7; break;
    case 0x77: ea=addr_extended(cpu); tmp8=READ(ea); SET_OR_CLR(CC_C,tmp8&0x01); tmp8=(tmp8>>1)|(tmp8&0x80); UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=6; break;

    // ASL
    case 0x68: ea=addr_indexed(cpu);  tmp8=READ(ea); SET_OR_CLR(CC_C,tmp8&0x80); tmp8<<=1; UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=7; break;
    case 0x78: ea=addr_extended(cpu); tmp8=READ(ea); SET_OR_CLR(CC_C,tmp8&0x80); tmp8<<=1; UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=6; break;

    // ROL
    case 0x69: ea=addr_indexed(cpu);  { uint8_t r=READ(ea); uint8_t n=(r<<1)|(TST_FLAG(CC_C)?0x01:0); SET_OR_CLR(CC_C,r&0x80); SET_OR_CLR(CC_V,((r^n)&0x80)!=0); UPD_NZ8(n); WRITE(ea,n); cycles=7; } break;
    case 0x79: ea=addr_extended(cpu); { uint8_t r=READ(ea); uint8_t n=(r<<1)|(TST_FLAG(CC_C)?0x01:0); SET_OR_CLR(CC_C,r&0x80); SET_OR_CLR(CC_V,((r^n)&0x80)!=0); UPD_NZ8(n); WRITE(ea,n); cycles=6; } break;

    // DEC
    case 0x6A: ea=addr_indexed(cpu);  tmp8=READ(ea); SET_OR_CLR(CC_V,tmp8==0x80); tmp8--; UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=7; break;
    case 0x7A: ea=addr_extended(cpu); tmp8=READ(ea); SET_OR_CLR(CC_V,tmp8==0x80); tmp8--; UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=6; break;

    // INC
    case 0x6C: ea=addr_indexed(cpu);  tmp8=READ(ea); SET_OR_CLR(CC_V,tmp8==0x7F); tmp8++; UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=7; break;
    case 0x7C: ea=addr_extended(cpu); tmp8=READ(ea); SET_OR_CLR(CC_V,tmp8==0x7F); tmp8++; UPD_NZ8(tmp8); WRITE(ea,tmp8); cycles=6; break;

    // TST
    case 0x6D: ea=addr_indexed(cpu);  tmp8=READ(ea); UPD_NZ8(tmp8); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles=7; break;
    case 0x7D: ea=addr_extended(cpu); tmp8=READ(ea); UPD_NZ8(tmp8); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles=6; break;

    // JMP
    case 0x6E: cpu->PC = addr_indexed(cpu);  cycles=4; break;
    case 0x7E: cpu->PC = addr_extended(cpu); cycles=3; break;

    // CLR
    case 0x6F: ea=addr_indexed(cpu);  WRITE(ea,0); CLR_FLAG(CC_N); SET_FLAG(CC_Z); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles=7; break;
    case 0x7F: ea=addr_extended(cpu); WRITE(ea,0); CLR_FLAG(CC_N); SET_FLAG(CC_Z); CLR_FLAG(CC_V); CLR_FLAG(CC_C); cycles=6; break;

    // --------------------------------------------------------
    //  SUBA — Subtract from A
    // --------------------------------------------------------
    case 0x80: cpu->A = sub8(cpu,cpu->A,FETCH8(),0); cycles=2; break;           // IMM
    case 0x90: cpu->A = sub8(cpu,cpu->A,READ(addr_direct(cpu)),0); cycles=3; break;  // DIR
    case 0xA0: cpu->A = sub8(cpu,cpu->A,READ(addr_indexed(cpu)),0); cycles=5; break; // IDX
    case 0xB0: cpu->A = sub8(cpu,cpu->A,READ(addr_extended(cpu)),0); cycles=4; break;// EXT

    // CMPA
    case 0x81: sub8(cpu,cpu->A,FETCH8(),0); cycles=2; break;
    case 0x91: sub8(cpu,cpu->A,READ(addr_direct(cpu)),0); cycles=3; break;
    case 0xA1: sub8(cpu,cpu->A,READ(addr_indexed(cpu)),0); cycles=5; break;
    case 0xB1: sub8(cpu,cpu->A,READ(addr_extended(cpu)),0); cycles=4; break;

    // SBCA
    case 0x82: cpu->A = sub8(cpu,cpu->A,FETCH8(),TST_FLAG(CC_C)?1:0); cycles=2; break;
    case 0x92: cpu->A = sub8(cpu,cpu->A,READ(addr_direct(cpu)),TST_FLAG(CC_C)?1:0); cycles=3; break;
    case 0xA2: cpu->A = sub8(cpu,cpu->A,READ(addr_indexed(cpu)),TST_FLAG(CC_C)?1:0); cycles=5; break;
    case 0xB2: cpu->A = sub8(cpu,cpu->A,READ(addr_extended(cpu)),TST_FLAG(CC_C)?1:0); cycles=4; break;

    // ANDA
    case 0x84: cpu->A &= FETCH8(); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=2; break;
    case 0x94: cpu->A &= READ(addr_direct(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=3; break;
    case 0xA4: cpu->A &= READ(addr_indexed(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=5; break;
    case 0xB4: cpu->A &= READ(addr_extended(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=4; break;

    // BITA
    case 0x85: tmp8=cpu->A&FETCH8(); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=2; break;
    case 0x95: tmp8=cpu->A&READ(addr_direct(cpu)); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=3; break;
    case 0xA5: tmp8=cpu->A&READ(addr_indexed(cpu)); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=5; break;
    case 0xB5: tmp8=cpu->A&READ(addr_extended(cpu)); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=4; break;

    // LDAA
    case 0x86: cpu->A=FETCH8(); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=2; break;
    case 0x96: cpu->A=READ(addr_direct(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=3; break;
    case 0xA6: cpu->A=READ(addr_indexed(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=5; break;
    case 0xB6: cpu->A=READ(addr_extended(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=4; break;

    // STAA
    case 0x97: ea=addr_direct(cpu);   WRITE(ea,cpu->A); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=4; break;
    case 0xA7: ea=addr_indexed(cpu);  WRITE(ea,cpu->A); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=6; break;
    case 0xB7: ea=addr_extended(cpu); WRITE(ea,cpu->A); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=5; break;

    // EORA
    case 0x88: cpu->A ^= FETCH8(); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=2; break;
    case 0x98: cpu->A ^= READ(addr_direct(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=3; break;
    case 0xA8: cpu->A ^= READ(addr_indexed(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=5; break;
    case 0xB8: cpu->A ^= READ(addr_extended(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=4; break;

    // ADCA
    case 0x89: cpu->A=add8(cpu,cpu->A,FETCH8(),TST_FLAG(CC_C)?1:0); cycles=2; break;
    case 0x99: cpu->A=add8(cpu,cpu->A,READ(addr_direct(cpu)),TST_FLAG(CC_C)?1:0); cycles=3; break;
    case 0xA9: cpu->A=add8(cpu,cpu->A,READ(addr_indexed(cpu)),TST_FLAG(CC_C)?1:0); cycles=5; break;
    case 0xB9: cpu->A=add8(cpu,cpu->A,READ(addr_extended(cpu)),TST_FLAG(CC_C)?1:0); cycles=4; break;

    // ORAA
    case 0x8A: cpu->A |= FETCH8(); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=2; break;
    case 0x9A: cpu->A |= READ(addr_direct(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=3; break;
    case 0xAA: cpu->A |= READ(addr_indexed(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=5; break;
    case 0xBA: cpu->A |= READ(addr_extended(cpu)); UPD_NZ8(cpu->A); CLR_FLAG(CC_V); cycles=4; break;

    // ADDA
    case 0x8B: cpu->A=add8(cpu,cpu->A,FETCH8(),0); cycles=2; break;
    case 0x9B: cpu->A=add8(cpu,cpu->A,READ(addr_direct(cpu)),0); cycles=3; break;
    case 0xAB: cpu->A=add8(cpu,cpu->A,READ(addr_indexed(cpu)),0); cycles=5; break;
    case 0xBB: cpu->A=add8(cpu,cpu->A,READ(addr_extended(cpu)),0); cycles=4; break;

    // CPX — Compare Index Register
    case 0x8C: tmp16=FETCH16(); tmp16=(uint16_t)(cpu->X-tmp16); UPD_NZ16(tmp16); SET_OR_CLR(CC_V,((cpu->X^tmp16)&0x8000)!=0); cycles=3; break;
    case 0x9C: tmp16=READ16(addr_direct(cpu)); tmp16=(uint16_t)(cpu->X-tmp16); UPD_NZ16(tmp16); SET_OR_CLR(CC_V,((cpu->X^tmp16)&0x8000)!=0); cycles=4; break;
    case 0xAC: tmp16=READ16(addr_indexed(cpu)); tmp16=(uint16_t)(cpu->X-tmp16); UPD_NZ16(tmp16); SET_OR_CLR(CC_V,((cpu->X^tmp16)&0x8000)!=0); cycles=6; break;
    case 0xBC: tmp16=READ16(addr_extended(cpu)); tmp16=(uint16_t)(cpu->X-tmp16); UPD_NZ16(tmp16); SET_OR_CLR(CC_V,((cpu->X^tmp16)&0x8000)!=0); cycles=5; break;

    // BSR / JSR
    case 0x8D: { int8_t off=(int8_t)FETCH8(); PUSH16(cpu->PC); cpu->PC=(uint16_t)(cpu->PC+off); cycles=8; } break; // BSR
    case 0x9D: ea=addr_direct(cpu);   PUSH16(cpu->PC); cpu->PC=ea; cycles=8; break; // JSR DIR
    case 0xAD: ea=addr_indexed(cpu);  PUSH16(cpu->PC); cpu->PC=ea; cycles=8; break; // JSR IDX
    case 0xBD: ea=addr_extended(cpu); PUSH16(cpu->PC); cpu->PC=ea; cycles=9; break; // JSR EXT

    // LDS — Load Stack Pointer
    case 0x8E: cpu->SP=FETCH16(); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=3; break;
    case 0x9E: cpu->SP=READ16(addr_direct(cpu)); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=4; break;
    case 0xAE: cpu->SP=READ16(addr_indexed(cpu)); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=6; break;
    case 0xBE: cpu->SP=READ16(addr_extended(cpu)); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=5; break;

    // STS — Store Stack Pointer
    case 0x9F: ea=addr_direct(cpu);   WRITE(ea,cpu->SP>>8); WRITE(ea+1,cpu->SP&0xFF); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=5; break;
    case 0xAF: ea=addr_indexed(cpu);  WRITE(ea,cpu->SP>>8); WRITE(ea+1,cpu->SP&0xFF); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=7; break;
    case 0xBF: ea=addr_extended(cpu); WRITE(ea,cpu->SP>>8); WRITE(ea+1,cpu->SP&0xFF); UPD_NZ16(cpu->SP); CLR_FLAG(CC_V); cycles=6; break;

    // --------------------------------------------------------
    //  Accumulateur B (symétrique avec A, codes C0-FF)
    // --------------------------------------------------------
    case 0xC0: cpu->B=sub8(cpu,cpu->B,FETCH8(),0); cycles=2; break;           // SUBB IMM
    case 0xD0: cpu->B=sub8(cpu,cpu->B,READ(addr_direct(cpu)),0); cycles=3; break;
    case 0xE0: cpu->B=sub8(cpu,cpu->B,READ(addr_indexed(cpu)),0); cycles=5; break;
    case 0xF0: cpu->B=sub8(cpu,cpu->B,READ(addr_extended(cpu)),0); cycles=4; break;

    case 0xC1: sub8(cpu,cpu->B,FETCH8(),0); cycles=2; break;                  // CMPB IMM
    case 0xD1: sub8(cpu,cpu->B,READ(addr_direct(cpu)),0); cycles=3; break;
    case 0xE1: sub8(cpu,cpu->B,READ(addr_indexed(cpu)),0); cycles=5; break;
    case 0xF1: sub8(cpu,cpu->B,READ(addr_extended(cpu)),0); cycles=4; break;

    case 0xC2: cpu->B=sub8(cpu,cpu->B,FETCH8(),TST_FLAG(CC_C)?1:0); cycles=2; break; // SBCB
    case 0xD2: cpu->B=sub8(cpu,cpu->B,READ(addr_direct(cpu)),TST_FLAG(CC_C)?1:0); cycles=3; break;
    case 0xE2: cpu->B=sub8(cpu,cpu->B,READ(addr_indexed(cpu)),TST_FLAG(CC_C)?1:0); cycles=5; break;
    case 0xF2: cpu->B=sub8(cpu,cpu->B,READ(addr_extended(cpu)),TST_FLAG(CC_C)?1:0); cycles=4; break;

    case 0xC4: cpu->B&=FETCH8(); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=2; break; // ANDB
    case 0xD4: cpu->B&=READ(addr_direct(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=3; break;
    case 0xE4: cpu->B&=READ(addr_indexed(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=5; break;
    case 0xF4: cpu->B&=READ(addr_extended(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=4; break;

    case 0xC5: tmp8=cpu->B&FETCH8(); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=2; break; // BITB
    case 0xD5: tmp8=cpu->B&READ(addr_direct(cpu)); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=3; break;
    case 0xE5: tmp8=cpu->B&READ(addr_indexed(cpu)); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=5; break;
    case 0xF5: tmp8=cpu->B&READ(addr_extended(cpu)); UPD_NZ8(tmp8); CLR_FLAG(CC_V); cycles=4; break;

    case 0xC6: cpu->B=FETCH8(); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=2; break;   // LDAB IMM
    case 0xD6: cpu->B=READ(addr_direct(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=3; break;
    case 0xE6: cpu->B=READ(addr_indexed(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=5; break;
    case 0xF6: cpu->B=READ(addr_extended(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=4; break;

    case 0xD7: ea=addr_direct(cpu);   WRITE(ea,cpu->B); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=4; break; // STAB
    case 0xE7: ea=addr_indexed(cpu);  WRITE(ea,cpu->B); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=6; break;
    case 0xF7: ea=addr_extended(cpu); WRITE(ea,cpu->B); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=5; break;

    case 0xC8: cpu->B^=FETCH8(); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=2; break;  // EORB
    case 0xD8: cpu->B^=READ(addr_direct(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=3; break;
    case 0xE8: cpu->B^=READ(addr_indexed(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=5; break;
    case 0xF8: cpu->B^=READ(addr_extended(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=4; break;

    case 0xC9: cpu->B=add8(cpu,cpu->B,FETCH8(),TST_FLAG(CC_C)?1:0); cycles=2; break; // ADCB
    case 0xD9: cpu->B=add8(cpu,cpu->B,READ(addr_direct(cpu)),TST_FLAG(CC_C)?1:0); cycles=3; break;
    case 0xE9: cpu->B=add8(cpu,cpu->B,READ(addr_indexed(cpu)),TST_FLAG(CC_C)?1:0); cycles=5; break;
    case 0xF9: cpu->B=add8(cpu,cpu->B,READ(addr_extended(cpu)),TST_FLAG(CC_C)?1:0); cycles=4; break;

    case 0xCA: cpu->B|=FETCH8(); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=2; break;  // ORAB
    case 0xDA: cpu->B|=READ(addr_direct(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=3; break;
    case 0xEA: cpu->B|=READ(addr_indexed(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=5; break;
    case 0xFA: cpu->B|=READ(addr_extended(cpu)); UPD_NZ8(cpu->B); CLR_FLAG(CC_V); cycles=4; break;

    case 0xCB: cpu->B=add8(cpu,cpu->B,FETCH8(),0); cycles=2; break;          // ADDB IMM
    case 0xDB: cpu->B=add8(cpu,cpu->B,READ(addr_direct(cpu)),0); cycles=3; break;
    case 0xEB: cpu->B=add8(cpu,cpu->B,READ(addr_indexed(cpu)),0); cycles=5; break;
    case 0xFB: cpu->B=add8(cpu,cpu->B,READ(addr_extended(cpu)),0); cycles=4; break;

    // LDX — Load Index Register
    case 0xCE: cpu->X=FETCH16(); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=3; break;
    case 0xDE: cpu->X=READ16(addr_direct(cpu)); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=4; break;
    case 0xEE: cpu->X=READ16(addr_indexed(cpu)); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=6; break;
    case 0xFE: cpu->X=READ16(addr_extended(cpu)); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=5; break;

    // STX — Store Index Register
    case 0xDF: ea=addr_direct(cpu);   WRITE(ea,cpu->X>>8); WRITE(ea+1,cpu->X&0xFF); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=5; break;
    case 0xEF: ea=addr_indexed(cpu);  WRITE(ea,cpu->X>>8); WRITE(ea+1,cpu->X&0xFF); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=7; break;
    case 0xFF: ea=addr_extended(cpu); WRITE(ea,cpu->X>>8); WRITE(ea+1,cpu->X&0xFF); UPD_NZ16(cpu->X); CLR_FLAG(CC_V); cycles=6; break;

    // --------------------------------------------------------
    //  SUBD — Subtract Double (D = A:B)
    // --------------------------------------------------------
    case 0x83: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=FETCH16(); uint32_t r32=(uint32_t)d-op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((d^op)&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=4; } break;
    case 0x93: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=READ16(addr_direct(cpu)); uint32_t r32=(uint32_t)d-op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((d^op)&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=5; } break;
    case 0xA3: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=READ16(addr_indexed(cpu)); uint32_t r32=(uint32_t)d-op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((d^op)&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=7; } break;
    case 0xB3: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=READ16(addr_extended(cpu)); uint32_t r32=(uint32_t)d-op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((d^op)&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=6; } break;

    // ADDD — Add Double
    case 0xC3: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=FETCH16(); uint32_t r32=(uint32_t)d+op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((~(d^op))&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=4; } break;
    case 0xD3: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=READ16(addr_direct(cpu)); uint32_t r32=(uint32_t)d+op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((~(d^op))&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=5; } break;
    case 0xE3: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=READ16(addr_indexed(cpu)); uint32_t r32=(uint32_t)d+op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((~(d^op))&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=7; } break;
    case 0xF3: { uint16_t d=(cpu->A<<8)|cpu->B; uint16_t op=READ16(addr_extended(cpu)); uint32_t r32=(uint32_t)d+op; uint16_t r=(uint16_t)r32; SET_OR_CLR(CC_C,r32>0xFFFF); SET_OR_CLR(CC_V,((~(d^op))&(d^r))&0x8000); UPD_NZ16(r); cpu->A=r>>8; cpu->B=r&0xFF; cycles=6; } break;

    // LDD — Load Double
    case 0xCC: cpu->A=FETCH8(); cpu->B=FETCH8(); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=3; break;
    case 0xDC: ea=addr_direct(cpu);   cpu->A=READ(ea); cpu->B=READ(ea+1); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=4; break;
    case 0xEC: ea=addr_indexed(cpu);  cpu->A=READ(ea); cpu->B=READ(ea+1); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=6; break;
    case 0xFC: ea=addr_extended(cpu); cpu->A=READ(ea); cpu->B=READ(ea+1); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=5; break;

    // STD — Store Double
    case 0xDD: ea=addr_direct(cpu);   WRITE(ea,cpu->A); WRITE(ea+1,cpu->B); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=5; break;
    case 0xED: ea=addr_indexed(cpu);  WRITE(ea,cpu->A); WRITE(ea+1,cpu->B); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=7; break;
    case 0xFD: ea=addr_extended(cpu); WRITE(ea,cpu->A); WRITE(ea+1,cpu->B); UPD_NZ16((cpu->A<<8)|cpu->B); CLR_FLAG(CC_V); cycles=6; break;

    // --------------------------------------------------------
    //  OPCODE NON RECONNU
    // --------------------------------------------------------
    default:
        if (cpu->debug_mode) {
            Serial.print(F("[WARN] Unknown opcode: 0x"));
            Serial.print(opcode, HEX);
            Serial.print(F(" @ 0x"));
            Serial.println(pc_before, HEX);
        }
        cycles = 2; // consume 2 cycles for unknown opcodes
        break;
    }

    cpu->cycles += cycles;
    cpu->instr_count++;
    return cycles;
}

// ============================================================
//  EXECUTE N STEPS
// ============================================================

void cpu6802_run(CPU6802 *cpu, uint32_t n_steps) {
    for (uint32_t i = 0; i < n_steps && !cpu->halted; i++) {
        cpu6802_step(cpu);
    }
}

// ============================================================
//  DEBUG — Print CPU register state to Serial
// ============================================================

void cpu6802_print_state(CPU6802 *cpu) {
    Serial.println(F("=== CPU 6802 STATE ==="));
    Serial.print(F("PC=0x")); Serial.print(cpu->PC, HEX);
    Serial.print(F("  SP=0x")); Serial.print(cpu->SP, HEX);
    Serial.print(F("  X=0x")); Serial.println(cpu->X, HEX);
    Serial.print(F("A=0x")); Serial.print(cpu->A, HEX);
    Serial.print(F("  B=0x")); Serial.println(cpu->B, HEX);
    // CC flags: uppercase = set, lowercase = clear
    Serial.print(F("CC="));
    Serial.print(TST_FLAG(CC_H)?'H':'h');
    Serial.print(TST_FLAG(CC_I)?'I':'i');
    Serial.print(TST_FLAG(CC_N)?'N':'n');
    Serial.print(TST_FLAG(CC_Z)?'Z':'z');
    Serial.print(TST_FLAG(CC_V)?'V':'v');
    Serial.print(TST_FLAG(CC_C)?'C':'c');
    Serial.print(F("  (0x"));
    Serial.print(cpu->CC, HEX);
    Serial.print(F(")"));
    Serial.print(F("  Cycles=")); Serial.print(cpu->cycles);
    Serial.print(F("  Instrs=")); Serial.println(cpu->instr_count);
}

// ============================================================
//  DEBUG — Single-instruction disassembler
//  Decodes the instruction at addr and writes a human-readable
//  string into buf (max buf_len bytes).
// ============================================================

void cpu6802_disasm(CPU6802 *cpu, uint16_t addr, char *buf, int buf_len) {
    uint8_t op = cpu6802_read(cpu, addr);
    uint8_t b1 = cpu6802_read(cpu, addr+1);
    uint8_t b2 = cpu6802_read(cpu, addr+2);
    switch(op) {
        case 0x01: snprintf(buf,buf_len,"NOP"); break;
        case 0x06: snprintf(buf,buf_len,"TAP"); break;
        case 0x07: snprintf(buf,buf_len,"TPA"); break;
        case 0x08: snprintf(buf,buf_len,"INX"); break;
        case 0x09: snprintf(buf,buf_len,"DEX"); break;
        case 0x0C: snprintf(buf,buf_len,"CLC"); break;
        case 0x0D: snprintf(buf,buf_len,"SEC"); break;
        case 0x0E: snprintf(buf,buf_len,"CLI"); break;
        case 0x0F: snprintf(buf,buf_len,"SEI"); break;
        case 0x10: snprintf(buf,buf_len,"SBA"); break;
        case 0x16: snprintf(buf,buf_len,"TAB"); break;
        case 0x17: snprintf(buf,buf_len,"TBA"); break;
        case 0x19: snprintf(buf,buf_len,"DAA"); break;
        case 0x1B: snprintf(buf,buf_len,"ABA"); break;
        case 0x20: snprintf(buf,buf_len,"BRA $%02X (off=%d)", addr+2+(int8_t)b1, (int8_t)b1); break;
        case 0x27: snprintf(buf,buf_len,"BEQ $%02X", addr+2+(int8_t)b1); break;
        case 0x26: snprintf(buf,buf_len,"BNE $%02X", addr+2+(int8_t)b1); break;
        case 0x2A: snprintf(buf,buf_len,"BPL $%02X", addr+2+(int8_t)b1); break;
        case 0x2B: snprintf(buf,buf_len,"BMI $%02X", addr+2+(int8_t)b1); break;
        case 0x24: snprintf(buf,buf_len,"BCC $%02X", addr+2+(int8_t)b1); break;
        case 0x25: snprintf(buf,buf_len,"BCS $%02X", addr+2+(int8_t)b1); break;
        case 0x30: snprintf(buf,buf_len,"TSX"); break;
        case 0x35: snprintf(buf,buf_len,"TXS"); break;
        case 0x36: snprintf(buf,buf_len,"PSHA"); break;
        case 0x37: snprintf(buf,buf_len,"PSHB"); break;
        case 0x32: snprintf(buf,buf_len,"PULA"); break;
        case 0x33: snprintf(buf,buf_len,"PULB"); break;
        case 0x39: snprintf(buf,buf_len,"RTS"); break;
        case 0x3B: snprintf(buf,buf_len,"RTI"); break;
        case 0x3F: snprintf(buf,buf_len,"SWI"); break;
        case 0x48: snprintf(buf,buf_len,"WAI"); break;
        case 0x4C: snprintf(buf,buf_len,"INCA"); break;
        case 0x4A: snprintf(buf,buf_len,"DECA"); break;
        case 0x4F: snprintf(buf,buf_len,"CLRA"); break;
        case 0x5C: snprintf(buf,buf_len,"INCB"); break;
        case 0x5A: snprintf(buf,buf_len,"DECB"); break;
        case 0x5F: snprintf(buf,buf_len,"CLRB"); break;
        case 0x7E: snprintf(buf,buf_len,"JMP $%04X", (b1<<8)|b2); break;
        case 0x86: snprintf(buf,buf_len,"LDAA #$%02X", b1); break;
        case 0x96: snprintf(buf,buf_len,"LDAA $%02X", b1); break;
        case 0xB6: snprintf(buf,buf_len,"LDAA $%04X", (b1<<8)|b2); break;
        case 0xC6: snprintf(buf,buf_len,"LDAB #$%02X", b1); break;
        case 0xD6: snprintf(buf,buf_len,"LDAB $%02X", b1); break;
        case 0x97: snprintf(buf,buf_len,"STAA $%02X", b1); break;
        case 0xB7: snprintf(buf,buf_len,"STAA $%04X", (b1<<8)|b2); break;
        case 0xD7: snprintf(buf,buf_len,"STAB $%02X", b1); break;
        case 0x8B: snprintf(buf,buf_len,"ADDA #$%02X", b1); break;
        case 0x80: snprintf(buf,buf_len,"SUBA #$%02X", b1); break;
        case 0x81: snprintf(buf,buf_len,"CMPA #$%02X", b1); break;
        case 0xC1: snprintf(buf,buf_len,"CMPB #$%02X", b1); break;
        case 0x8C: snprintf(buf,buf_len,"CPX #$%04X", (b1<<8)|b2); break;
        case 0xCE: snprintf(buf,buf_len,"LDX #$%04X", (b1<<8)|b2); break;
        case 0x8E: snprintf(buf,buf_len,"LDS #$%04X", (b1<<8)|b2); break;
        case 0xBD: snprintf(buf,buf_len,"JSR $%04X", (b1<<8)|b2); break;
        case 0x8D: snprintf(buf,buf_len,"BSR $%04X", addr+2+(int8_t)b1); break;
        case 0xCC: snprintf(buf,buf_len,"LDD #$%04X", (b1<<8)|b2); break;
        default:   snprintf(buf,buf_len,"??? $%02X", op); break;
    }
}

// ============================================================
//  DEBUG — Hex memory dump to Serial
//  Prints len bytes starting at start in hex rows of 16.
// ============================================================

void cpu6802_dump_memory(CPU6802 *cpu, uint16_t start, uint16_t len) {
    Serial.println(F("--- Memory Dump ---")); // 16 bytes per row
    for (uint16_t i = 0; i < len; i += 16) {
        Serial.print(F("0x"));
        if ((start+i) < 0x1000) Serial.print('0');
        if ((start+i) < 0x100)  Serial.print('0');
        if ((start+i) < 0x10)   Serial.print('0');
        Serial.print(start+i, HEX);
        Serial.print(F(": "));
        for (int j = 0; j < 16 && (i+j) < len; j++) {
            uint8_t b = cpu6802_read(cpu, start+i+j);
            if (b < 0x10) Serial.print('0');
            Serial.print(b, HEX);
            Serial.print(' ');
        }
        Serial.println();
    }
}
