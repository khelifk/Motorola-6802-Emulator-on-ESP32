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
//  cpu6802.h — Register definitions, constants, public API
//
//  This file declares the CPU structure, memory-map constants,
//  condition-code flag masks, and all public function prototypes
//  used by the emulator core (cpu6802.cpp) and the Arduino
//  sketch (esp32_6802.ino).
//
// ============================================================

#ifndef CPU6802_H
#define CPU6802_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// ============================================================
//  MEMORY MAP CONSTANTS
// ============================================================

#define MEM_SIZE        0x10000   // Full 16-bit address space : 64 KB
#define RAM_INTERNAL    128       // Internal RAM : $0000-$007F (6802 built-in)
#define RAM_SIZE        0x8000    // External RAM upper limit
#define ROM_START       0xF000    // Start of high ROM area (4 KB)
#define ROM_SIZE        0x1000    // Size of ROM area

// ============================================================
//  INTERRUPT VECTORS  (stored at top of memory, big-endian)
// ============================================================

#define VEC_IRQ         0xFFF8    // IRQ  vector (maskable hardware interrupt)
#define VEC_SWI         0xFFFA    // SWI  vector (software interrupt, opcode $3F)
#define VEC_NMI         0xFFFC    // NMI  vector (non-maskable interrupt)
#define VEC_RESET       0xFFFE    // RESET vector — PC is loaded from here on reset

// ============================================================
//  CONDITION CODE REGISTER (CC) BIT MASKS
//
//  Bit layout:  7  6  5  4  3  2  1  0
//               1  1  H  I  N  Z  V  C
//
//  Bits 7 and 6 are always 1 (hardware fixed).
// ============================================================

#define CC_C    (1 << 0)   // Carry       : set when unsigned arithmetic overflows
#define CC_V    (1 << 1)   // Overflow    : set when signed arithmetic overflows
#define CC_Z    (1 << 2)   // Zero        : set when result is exactly zero
#define CC_N    (1 << 3)   // Negative    : set when bit 7 of result is 1
#define CC_I    (1 << 4)   // IRQ Mask    : 1 = IRQ disabled, 0 = IRQ enabled
#define CC_H    (1 << 5)   // Half-carry  : carry from bit 3 to bit 4 (BCD use)
#define CC_BIT6 (1 << 6)   // Always 1 (hardware)
#define CC_BIT7 (1 << 7)   // Always 1 (hardware)

// ============================================================
//  CPU STATE STRUCTURE
// ============================================================

typedef struct {
    // --- Registers ---
    uint8_t  A;     // Accumulator A  (8-bit primary working register)
    uint8_t  B;     // Accumulator B  (8-bit secondary working register)
    uint16_t X;     // Index register (16-bit, used for indexed addressing)
    uint16_t SP;    // Stack Pointer  (16-bit, stack grows downward)
    uint16_t PC;    // Program Counter (address of the next instruction)
    uint8_t  CC;    // Condition Codes register (flags H I N Z V C)

    // --- Memory ---
    uint8_t  ram_internal[RAM_INTERNAL]; // 128-byte internal RAM ($0000-$007F)
    uint8_t  memory[MEM_SIZE];           // Full 64 KB memory bus

    // --- Internal state flags ---
    bool     halted;       // CPU halted (illegal opcode or explicit halt)
    bool     irq_pending;  // IRQ interrupt is waiting to be serviced
    bool     nmi_pending;  // NMI interrupt is waiting to be serviced

    // --- Counters ---
    uint32_t cycles;       // Total clock cycles executed since last reset
    uint32_t instr_count;  // Total instructions executed since last reset

    // --- Debug ---
    bool     debug_mode;   // When true, prints each instruction to Serial

    // --- I/O callbacks ---
    // Called for every read/write in the $8000-$8FFF memory-mapped I/O range.
    // Set these in your .ino file to connect real ESP32 peripherals.
    uint8_t  (*io_read_cb)(uint16_t addr);
    void     (*io_write_cb)(uint16_t addr, uint8_t val);

} CPU6802;

// ============================================================
//  MEMORY-MAPPED I/O RANGE
// ============================================================

#define IO_BASE      0x8000   // Start of I/O window
#define IO_END       0x8FFF   // End   of I/O window

// Default UART mapping (Serial port of the ESP32)
#define UART_DATA    0x8000   // Read = receive byte,  Write = send byte
#define UART_STATUS  0x8001   // Bit 0 = RX ready,     Bit 1 = TX ready

// ============================================================
//  PUBLIC API
// ============================================================

// Initialise the CPU structure (zero memory, set default CC, SP)
void cpu6802_init(CPU6802 *cpu);

// Reset the CPU: re-reads RESET vector, clears pending interrupts
void cpu6802_reset(CPU6802 *cpu);

// Load a byte array into memory starting at addr
void cpu6802_load(CPU6802 *cpu, uint16_t addr,
                  const uint8_t *data, uint16_t len);

// Execute one instruction; returns number of clock cycles consumed
int  cpu6802_step(CPU6802 *cpu);

// Execute n_steps instructions (stops early if CPU halted)
void cpu6802_run(CPU6802 *cpu, uint32_t n_steps);

// Trigger a maskable IRQ (honours the I flag in CC)
void cpu6802_irq(CPU6802 *cpu);

// Trigger a non-maskable NMI (always serviced regardless of I flag)
void cpu6802_nmi(CPU6802 *cpu);

// Memory access with full I/O dispatch
uint8_t  cpu6802_read(CPU6802 *cpu, uint16_t addr);
void     cpu6802_write(CPU6802 *cpu, uint16_t addr, uint8_t val);
uint16_t cpu6802_read16(CPU6802 *cpu, uint16_t addr); // big-endian 16-bit read

// Debug / monitor helpers
void cpu6802_print_state(CPU6802 *cpu);                            // Print registers to Serial
void cpu6802_disasm(CPU6802 *cpu, uint16_t addr,                   // Disassemble one instruction
                    char *buf, int buf_len);
void cpu6802_dump_memory(CPU6802 *cpu, uint16_t start,             // Hex dump to Serial
                         uint16_t len);

#endif // CPU6802_H
